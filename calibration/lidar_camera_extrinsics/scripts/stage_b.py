#!/usr/bin/python3
"""Supervise the existing Stage B drivers. Preparation/dry-run never opens hardware."""
import argparse, datetime, fcntl, hashlib, json, os, signal, subprocess, sys, time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
STATE=ROOT/'logs/stage_b_state.json'
class StopRequested(Exception):pass

def identity(pid):
    try:
        # comm can contain spaces/parentheses; split after its final closing parenthesis.
        stat=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
        if stat[0]=='Z':return None
        return stat[19] # /proc stat field 22: starttime
    except (OSError,IndexError):return None

def active_state(d):
    return d.get('supervisor_starttime') is not None and identity(d.get('supervisor_pid'))==d['supervisor_starttime']

def conflicts():
    found=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            if p.stat().st_uid!=os.getuid():continue
            argv=(p/'cmdline').read_bytes().split(b'\0');names=[Path(x.decode(errors='replace')).name for x in argv[:2] if x]
            matches=[n for n in names if n in ('livox_ros_driver2_node','timeshare_writer_node') or ('hik' in n.lower() and 'camera' in n.lower() and 'node' in n.lower())]
            if matches:found.append({'pid':int(p.name),'executables':matches})
        except (OSError,ValueError):pass
    return found

def commands(a,run):
    return [
      ('lidar',['ros2','run','livox_ros_driver2','livox_ros_driver2_node','--ros-args','-p','xfer_format:=0','-p','multi_topic:=0','-p','data_src:=0','-p','publish_freq:=10.0','-p','output_data_type:=0','-p','frame_id:=livox_frame','-p','user_config_path:='+str(a.lidar_config),'-p','cmdline_input_bd_code:=livox0000000001','-r','__node:=livox_lidar_publisher']),
      ('writer',['ros2','launch','timeshare_writer','timeshare_launch.py','csv:='+str(run/'timeshare_writer.csv'),'timeshare_path:='+str(run/'timeshare')]),
      ('camera',['ros2','launch','hik_camera_trigger_driver','trigger_launch.py','exposure_us:='+str(float(a.exposure_us)),'sync_csv:='+str(run/'camera_sync.csv'),'timeshare_path:='+str(run/'timeshare')])]

def stop_children(children):
    # Signal only groups created by this supervisor, including remaining ROS launch descendants.
    for sig,seconds in [(signal.SIGINT,8),(signal.SIGTERM,3),(signal.SIGKILL,1)]:
        for _,p in reversed(children):
            try:os.killpg(p.pid,sig)
            except ProcessLookupError:pass
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            alive=False
            for _,p in children:
                p.poll()
                try:os.killpg(p.pid,0);alive=True
                except ProcessLookupError:pass
            if not alive:break
            time.sleep(.1)
    for _,p in children:
        try:p.wait(timeout=1)
        except subprocess.TimeoutExpired:pass

def start(a):
    cfg=Path(a.lidar_config).expanduser().resolve();a.lidar_config=cfg
    d=json.loads(cfg.read_text())
    if d.get('lidar_summary_info',{}).get('lidar_type')!=8 or 'Mid360s' not in d:raise ValueError('Expected verified Mid-360S config (type 8, Mid360s)')
    if not 1<=a.exposure_us<=40000:raise ValueError('Exposure must be 1..40000 us')
    for p in [ROOT.parent.parent/'ws_livox/install/setup.bash',ROOT.parent.parent/'ws_camera/install/setup.bash']:
        if not p.is_file():raise ValueError('Missing '+str(p))
    run=ROOT/'logs'/('stage_b_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    if a.dry_run or not a.execute:
        import shlex
        for name,cmd in commands(a,run):print(name+': '+shlex.join(cmd))
        print('DRY_RUN: no drivers launched; no camera access.');return 0
    (ROOT/'logs').mkdir(exist_ok=True)
    lock=(ROOT/'logs/stage_b.lock').open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise ValueError('This launcher is already running; use status or stop')
    if STATE.exists():
        old=json.loads(STATE.read_text())
        if active_state(old):raise ValueError('An active supervisor is already recorded')
    busy=conflicts()
    if busy:raise ValueError('Existing driver processes; stop/coordinate their owner before starting: '+str(busy))
    run.mkdir(exist_ok=False)
    cmds=commands(a,run)
    state={'supervisor_pid':os.getpid(),'supervisor_starttime':identity(os.getpid()),'log_dir':str(run),'config':str(cfg),'config_sha256':hashlib.sha256(cfg.read_bytes()).hexdigest(),'commands':dict(cmds),'children':[]}
    def persist():
        tmp=STATE.with_suffix('.tmp');tmp.write_text(json.dumps(state,indent=2)+'\n');tmp.replace(STATE)
    def interrupted(*_):raise StopRequested()
    children=[];logs=[];oldhandlers={}
    for sig in (signal.SIGINT,signal.SIGTERM):oldhandlers[sig]=signal.signal(sig,interrupted)
    persist()
    print('启动 Stage B；日志：'+str(run),flush=True)
    print('本终端保持运行；Ctrl+C 停止本脚本启动的全部节点。',flush=True)
    try:
        for name,cmd in cmds:
            f=(run/(name+'.log')).open('w');logs.append(f)
            proc=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,cwd=run,start_new_session=True)
            children.append((name,proc));state['children'].append({'name':name,'pid':proc.pid,'starttime':identity(proc.pid)});persist()
            # Stage B requires the timestamp writer to run before camera startup.
            deadline=time.monotonic()+(3 if name=='writer' else 1)
            while time.monotonic()<deadline:
                for n,p in children:
                    if p.poll() is not None:raise ValueError(n+' exited during startup; see '+str(run/(n+'.log')))
                time.sleep(.1)
        print('三个启动进程均存活；这不代表话题或同步已验收。采集脚本会检查可靠 QoS、消息和配对。',flush=True)
        while True:
            for name,p in children:
                if p.poll() is not None:raise ValueError(name+' exited; see '+str(run/(name+'.log')))
            time.sleep(.2)
    except StopRequested:print('正在停止本次启动的节点……',flush=True)
    finally:
        for sig in oldhandlers:signal.signal(sig,signal.SIG_IGN)
        stop_children(children)
        for f in logs:f.close()
        state['stopped_at']=datetime.datetime.now().isoformat();(run/'session.json').write_text(json.dumps(state,indent=2)+'\n')
        if STATE.exists() and json.loads(STATE.read_text()).get('supervisor_pid')==os.getpid():STATE.unlink()
        for sig,h in oldhandlers.items():signal.signal(sig,h)
    return 0

def control(action):
    if not STATE.exists():print('本脚本当前没有运行中的 Stage B 会话。');return 0
    state=json.loads(STATE.read_text())
    if not active_state(state):print('会话记录已过期；未向任何进程发送信号。日志：'+state['log_dir']);return 0
    if action=='status':print(json.dumps(state,indent=2,ensure_ascii=False));return 0
    os.kill(state['supervisor_pid'],signal.SIGTERM)
    end=time.monotonic()+20
    while time.monotonic()<end and active_state(state):time.sleep(.1)
    if active_state(state):raise ValueError('Supervisor has not stopped; inspect '+state['log_dir'])
    print('已停止本脚本启动的 Stage B 会话。');return 0

def main():
    p=argparse.ArgumentParser();sub=p.add_subparsers(dest='action',required=True)
    s=sub.add_parser('start');s.add_argument('--exposure-us',type=float,default=20000);s.add_argument('--lidar-config',default=str(ROOT/'config/mid360s_stage_b.json'));s.add_argument('--dry-run',action='store_true');s.add_argument('--execute',action='store_true')
    sub.add_parser('stop');sub.add_parser('status');a=p.parse_args()
    return start(a) if a.action=='start' else control(a.action)
if __name__=='__main__':
    try:sys.exit(main() or 0)
    except (ValueError,OSError,json.JSONDecodeError) as e:print('启动/控制失败：'+str(e),file=sys.stderr);sys.exit(2)
