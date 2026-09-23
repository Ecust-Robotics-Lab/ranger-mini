#!/usr/bin/env python3
"""Single Livox driver, no base control. Source Humble and ws_livox first."""
import argparse, fcntl, json, os, pathlib, signal, socket, subprocess, tempfile, time
ROOT = pathlib.Path(__file__).resolve().parents[1]
def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute',action='store_true');p.add_argument('--seconds',type=float,default=120)
    a=p.parse_args()
    if not 0<a.seconds<=3600:p.error('seconds must be in (0,3600]')
    cfg=ROOT/'config/mid360/MID360S_config.json'
    config=json.loads(cfg.read_text())
    assert 'Mid360s' in config and 'MID360' not in config, 'Expected Mid-360S device configuration'
    cmd=[str(ROOT/'ws_livox/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node'),'--ros-args','-r','__node:=mid360s_driver']
    for k,v in dict(xfer_format=1,multi_topic=0,data_src=0,publish_freq=10.0,output_data_type=0,frame_id='livox_frame',user_config_path=str(cfg),use_sim_time=False).items():
        cmd+=['-p',f'{k}:={str(v).lower() if isinstance(v,bool) else v}']
    if not a.execute:print('DRY-RUN',cmd);return 0
    lock=(ROOT/'ws_livox/.device.lock').open('a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    addresses=json.loads(subprocess.check_output(['ip','-j','address','show','enp2s0']))
    assert any(x.get('local')=='192.168.1.50' and x.get('prefixlen')==24 for x in addresses[0]['addr_info']), 'Host IP missing'
    route=json.loads(subprocess.check_output(['ip','-j','route','get','192.168.1.101']))[0]
    assert route['dev']=='enp2s0','Wrong LiDAR route'
    for line in subprocess.check_output(['ps','-eo','pid,comm,args'],text=True).splitlines():
        parts=line.split(None,2)
        if len(parts)==3 and parts[1] not in ('bash','python3','ps') and any(x in parts[2] for x in ('livox_ros_driver2_node','livox_lidar_quick_start','LivoxViewer')):
            raise RuntimeError('Existing Livox process: '+line)
    checks=[]
    try:
        for port in (56000,56101,56201,56301,56401,56501):
            s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);checks.append(s);s.bind(('0.0.0.0',port))
    finally:
        for s in checks:s.close()
    folder=pathlib.Path(tempfile.mkdtemp(prefix='driver-',dir=ROOT/'docs/03_evidence'))
    (folder/'config.json').write_bytes(cfg.read_bytes())
    (folder/'command.json').write_text(json.dumps(cmd,indent=2))
    print('LOG_DIR='+str(folder),flush=True)
    env=os.environ.copy();env['ROS_LOG_DIR']=str(folder/'ros_logs')
    env['LD_LIBRARY_PATH']=str(ROOT/'ws_livox/sdk_install/lib')+':'+env.get('LD_LIBRARY_PATH','')
    child=None
    def stop(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    started=time.time()
    try:
        with (folder/'driver.log').open('x') as log:
            child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
            (folder/'pid.json').write_text(json.dumps({'supervisor':os.getpid(),'driver':child.pid}))
            deadline=time.monotonic()+a.seconds
            while child.poll() is None and time.monotonic()<deadline:time.sleep(.2)
    except KeyboardInterrupt:pass
    finally:
        if child and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                child.terminate()
                try:child.wait(timeout=5)
                except subprocess.TimeoutExpired:child.kill();child.wait()
        result=dict(start_unix=started,end_unix=time.time(),driver_exit=child.returncode if child else None)
        (folder/'exit.json').write_text(json.dumps(result,indent=2))
    return 0 if child and child.returncode==0 else 2
if __name__=='__main__':raise SystemExit(main())
