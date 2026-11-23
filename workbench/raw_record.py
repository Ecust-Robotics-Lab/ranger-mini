"""Optional finite raw bag, independent of the localization lifetime."""
import argparse
import ctypes
import json
from pathlib import Path
import shutil
import signal
import sqlite3
import subprocess
import time
from workbench.core import atomic,ROOT

def run(folder,seconds):
    from scripts.fast_livo2.localizer.online_session import parser,record_topics
    args=parser().parse_args(['--public-tf','--record-navigation'])
    topics=record_topics(args)
    import hashlib
    context=json.loads((folder/'workbench_manifest.json').read_text())
    source=Path(context['localization_folder'])
    # Copy small identity/config/source evidence, never mutate the runtime source.
    for name in ('source_identity.json','provenance.json','manifest.json','config','sources'):
        p=source/name
        if p.is_dir():shutil.copytree(p,folder/name)
        elif p.is_file():shutil.copy2(p,folder/name)
    atomic(folder/'source_snapshot.json',{'files':{str(p.relative_to(folder)):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in folder.rglob('*') if p.is_file() and p.name!='worker.log'},'topics':topics})
    stopping=[False]
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:stopping.__setitem__(0,True))
    if shutil.disk_usage(folder).free<20e9:raise RuntimeError('可用空间不足 20GB')
    atomic(folder/'recording.json',{'state':'STARTING'})
    child=subprocess.Popen(['ros2','bag','record','-o',str(folder/'bag'),*topics],
        preexec_fn=lambda:ctypes.CDLL(None).prctl(1,signal.SIGINT))
    began=time.monotonic();reason='duration';recorded_at=None;counts={}
    while child.poll() is None:
        if stopping[0]:reason='requested';break
        if recorded_at is None:
            counts={}
            for database in (folder/'bag').glob('*.db3'):
                try:
                    with sqlite3.connect(f'file:{database}?mode=ro',uri=True,timeout=.1) as db:
                        for topic,count in db.execute('select topics.name,count(messages.id) from topics left join messages on messages.topic_id=topics.id group by topics.id'):
                            counts[topic]=counts.get(topic,0)+count
                except sqlite3.Error:pass
        required=('/livox/lidar','/livox/imu','/front/image_sync','/fast_livo/odometry','/fast_livo/deskewed_imu','/wheel/odom')
        if recorded_at is None and all(counts.get(t,0)>0 for t in required):recorded_at=time.monotonic()
        atomic(folder/'recording.json',{'state':'RECORDING' if recorded_at else 'STARTING','observed_messages':counts})
        if recorded_at and time.monotonic()-recorded_at>=seconds:break
        if recorded_at is None and time.monotonic()-began>30:reason='recording_input_timeout';break
        if shutil.disk_usage(folder).free<20e9:reason='disk_low';break
        time.sleep(.2)
    forced=False
    if child.poll() is None:
        child.send_signal(signal.SIGINT)
        try:child.wait(timeout=15)
        except subprocess.TimeoutExpired:
            forced=True;child.terminate()
            try:child.wait(timeout=3)
            except subprocess.TimeoutExpired:child.kill();child.wait()
    atomic(folder/'recording.json',{'state':'STOPPED','reason':reason,'observed_messages':counts})
    atomic(folder/'recorder_result.json',{'observed_messages':counts,'recorded_seconds':time.monotonic()-recorded_at if recorded_at else 0,'exit':child.returncode,'forced':forced,'reason':reason,'elapsed_seconds':time.monotonic()-began})
    return child.returncode if recorded_at is not None and not forced else 2

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--folder',type=Path,required=True);p.add_argument('--seconds',type=float,default=900)
    a=p.parse_args()
    try:raise SystemExit(run(a.folder,a.seconds))
    except Exception as exc:
        atomic(a.folder/'recorder_result.json',{'exit':2,'error':str(exc)});raise
