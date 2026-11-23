"""Supervise the local task owner, independent watchdog and replaceable web facade."""
import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import sys
from workbench.core import ROOT
sys.path.insert(0,str(ROOT/'scripts'))
from session_manager import Component,read_session,session_live,run_guarded,stop_session

def main():
    p=argparse.ArgumentParser();p.add_argument('--map',dest='maps',type=Path,action='append',help='explicit immutable navigation-map folder; repeat to register maps');p.add_argument('--execute',action='store_true');p.add_argument('--demo',action='store_true')
    p.add_argument('--lan',action='store_true');p.add_argument('--port',type=int,default=8080);p.add_argument('--status',action='store_true')
    p.add_argument('--stop',action='store_true');p.add_argument('--attach',action='store_true',help='start only HTTP against an existing owner')
    p.add_argument('--data',type=Path,default=ROOT/'.runtime/navigation_workbench');a=p.parse_args()
    if not 1024<=a.port<=65535:p.error('port must be 1024..65535')
    directory=ROOT/'.runtime/sessions/navigation_workbench';socket=a.data/'owner.sock'
    if a.status:print(json.dumps(read_session(directory)[1],indent=2));return 0
    if a.stop:return 0 if stop_session(directory,timeout=90) else 2
    envpython=str(ROOT/'.runtime/navigation_workbench_venv/bin/python')
    daemon=[sys.executable,'-m','workbench.daemon','--socket',str(socket),'--data',str(a.data)]+(['--demo'] if a.demo else [])
    web=[envpython,'-m','workbench.server','--socket',str(socket),'--data',str(a.data),'--host','0.0.0.0' if a.lan else '127.0.0.1','--port',str(a.port)]
    for folder in a.maps or []:
        daemon+=['--map',str(folder.resolve())];web+=['--map',str(folder.resolve())]
    components=[Component('owner',daemon,resource='navigation_workbench'),
                Component('http',web,depends=('owner',),critical=False)]
    if not a.demo:
        components.insert(1,Component('watchdog',[sys.executable,'-m','workbench.watchdog','--heartbeat',str(a.data/'owner_heartbeat.json')],depends=('owner',)))
    if not a.execute:
        print(json.dumps({'action':'DRY-RUN','hardware_started':False,'components':[asdict(c) for c in components]},ensure_ascii=False,indent=2));return 0
    a.data.mkdir(parents=True,exist_ok=True)
    os.environ.update(ROS_LOCALHOST_ONLY='1',ROS_DOMAIN_ID=os.environ.get('ROS_DOMAIN_ID','215'))
    print(f'WORKBENCH_URL=http://127.0.0.1:{a.port}\nACCESS_KEY_FILE={a.data}/access.key',flush=True)
    if a.attach:
        if not session_live(read_session(directory)[1]) or not socket.exists():raise RuntimeError('没有存活的任务服务')
        os.execv(web[0],web)
    return run_guarded(directory,components,cleanup_grace=90)

if __name__=='__main__':raise SystemExit(main())
