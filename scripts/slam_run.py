#!/usr/bin/env python3
"""Unique, auditable SLAM experiments. No driver or motion command is launched."""
import argparse
import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import sqlite3
import subprocess
import tempfile
import time
import yaml

ROOT = Path(__file__).resolve().parents[1]

def digest(p):
    h = hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda: f.read(1024*1024), b''):
            h.update(b)
    return h.hexdigest()

def dump(p, obj):
    p.write_text(json.dumps(obj, indent=2, ensure_ascii=False)+'\n')

def bag_inventory(bag):
    if not (bag/'metadata.yaml').is_file():
        raise ValueError('Require complete rosbag directory including metadata.yaml')
    meta = yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    if meta['storage_identifier'] != 'sqlite3':
        raise ValueError('Expected SQLite3 bag')
    counts = {}
    for rel in meta['relative_file_paths']:
        p = (bag/rel).resolve()
        if not p.is_relative_to(bag):
            raise ValueError('Bag metadata escapes directory')
        db = sqlite3.connect(p.as_uri()+'?mode=ro', uri=True)
        if db.execute('PRAGMA integrity_check').fetchone()[0] != 'ok':
            raise ValueError('SQLite integrity failure')
        for topic, typ, count in db.execute('SELECT t.name,t.type,count(m.id) FROM topics t LEFT JOIN messages m ON t.id=m.topic_id GROUP BY t.id'):
            key=(topic,typ); counts[key]=counts.get(key,0)+count
        db.close()
    expected={(t['topic_metadata']['name'], t['topic_metadata']['type']):t['message_count'] for t in meta['topics_with_message_count']}
    if counts != expected or sum(counts.values()) != meta['message_count']:
        raise ValueError('Metadata/SQLite count mismatch')
    for topic,typ in [('/livox/lidar','livox_ros_driver2/msg/CustomMsg'),('/livox/imu','sensor_msgs/msg/Imu')]:
        if not counts.get((topic,typ),0):
            raise ValueError('Missing raw input '+topic)
    return {'files':{str(p.relative_to(bag)):digest(p) for p in sorted(bag.rglob('*')) if p.is_file()},
            'counts':[{'topic':k[0],'type':k[1],'count':v} for k,v in counts.items()]}

def stop(p):
    if p.poll() is not None:return p.returncode
    os.killpg(p.pid, signal.SIGINT)
    try:return p.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGTERM)
        try:return p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
            return p.wait()

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('mode',choices=['offline','online'])
    ap.add_argument('--bag',type=Path)
    ap.add_argument('--config',type=Path,default=ROOT/'config/slam/ranger_mid360s.yaml')
    ap.add_argument('--motion-data-note',help='Describe manual loop, start/end stationary, and bag provenance')
    ap.add_argument('--accepted-offline',type=Path,help='Experiment with acceptance.json: three_dimensional=PASS')
    ap.add_argument('--seconds',type=float,default=120)
    ap.add_argument('--execute',action='store_true')
    args=ap.parse_args()
    cfg_path=args.config.resolve(); cfg=yaml.safe_load(cfg_path.read_text())
    if cfg['loop_closing']['with_height'] or cfg['lidar_loc']['force_2d']:
        ap.error('6DoF configuration must not impose height/2D constraints')
    if cfg['system']['with_ui'] or cfg['system']['with_2dui']:
        ap.error('These controlled entry points require runtime UI disabled')
    if args.seconds<=0 or args.seconds>3600:ap.error('seconds must be in (0,3600]')
    if args.mode=='offline' and args.execute and (not args.bag or not args.motion_data_note):
        ap.error('Mapping requires a motion bag and --motion-data-note; use the reader for static validation')
    if args.mode=='online' or cfg['system']['with_g2p5']:
        if args.execute and not args.accepted_offline:ap.error('First accept the offline 3D experiment')
        if args.accepted_offline:
            review=json.loads((args.accepted_offline/'acceptance.json').read_text())
            if review.get('three_dimensional')!='PASS':ap.error('Offline 3D acceptance is pending')
    if cfg['system']['with_g2p5']:
        g=cfg['g2p5']
        if any(g[k] is None for k in ['lidar_height','floor_height','min_th_floor','max_th_floor']):
            ap.error('g2p5 requires measured floor/installation and chosen obstacle heights')
        if not 0 <= g['min_th_floor'] < g['max_th_floor'] or g['grid_map_resolution']<=0:
            ap.error('Invalid g2p5 height/resolution')
    exe=ROOT/'ws_slam/build/lightning/bin'/('run_slam_'+args.mode)
    print(json.dumps({'mode':args.mode,'executable':str(exe),'config':str(cfg_path),
                      'bag':str(args.bag),'execute':args.execute,'outputs':'new directory under maps/'},ensure_ascii=False))
    if not args.execute:return 0
    if not exe.is_file():ap.error('Build Lightning first')
    bag=args.bag.resolve() if args.bag else None
    if bag == (ROOT/'bags/mid360-static-qg8qtyiv/raw').resolve():ap.error('Known static bag cannot be accepted as a motion dataset')
    inventory=bag_inventory(bag) if bag else None
    graph=None
    if args.mode=='online':
        import rclpy
        rclpy.init();node=rclpy.create_node('ranger_slam_preflight')
        try:
            deadline=time.monotonic()+3
            while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
            if 'lightning_slam' in node.get_node_names():raise RuntimeError('Existing Lightning node')
            graph={}
            for topic,typ in [('/livox/lidar','livox_ros_driver2/msg/CustomMsg'),('/livox/imu','sensor_msgs/msg/Imu')]:
                endpoints=node.get_publishers_info_by_topic(topic)
                graph[topic]=[{'node':e.node_name,'type':e.topic_type,'reliability':e.qos_profile.reliability.name,'durability':e.qos_profile.durability.name} for e in endpoints]
                if len(endpoints)!=1 or endpoints[0].topic_type!=typ or endpoints[0].qos_profile.reliability.name!='RELIABLE':
                    raise RuntimeError('Require one matching reliable publisher: '+str(graph))
        finally:node.destroy_node();rclpy.shutdown()
    lock=(ROOT/'ws_slam/.mapping.lock').open('a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    exp=Path(tempfile.mkdtemp(prefix='lightning-'+args.mode+'-',dir=ROOT/'maps'))
    shutil.copy2(cfg_path,exp/'config.yaml')
    shutil.copy2(ROOT/'config/mid360/extrinsics_pending.yaml',exp/'extrinsics.yaml')
    for name in ['source_versions.json','dependencies.json']:
        if (ROOT/'docs/04_evidence'/name).exists():shutil.copy2(ROOT/'docs/04_evidence'/name,exp/name)
    (exp/'patches').mkdir()
    for p in (ROOT/'patches').glob('stage4*'):shutil.copy2(p,exp/'patches'/p.name)
    cmd=[str(exe),'--config='+str(exp/'config.yaml')]
    if bag:cmd+=['--input_bag='+str(bag)]
    manifest={'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'command':cmd,
              'cwd':str(exp),'bag':str(bag) if bag else None,'bag_inventory':inventory,
              'input_graph':graph,'motion_data_note':args.motion_data_note,'mode':args.mode,'executable_sha256':digest(exe),
              'acceptance':'PENDING','online_or_2d_prerequisite':str(args.accepted_offline),
              'environment':{k:os.environ.get(k) for k in ['ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','OMP_NUM_THREADS','LD_LIBRARY_PATH']}}
    source=ROOT/'ws_slam/src/lightning'
    manifest['current_source_commit']=subprocess.check_output(['git','-C',str(source),'rev-parse','HEAD'],text=True).strip()
    manifest['current_source_status']=subprocess.check_output(['git','-C',str(source),'status','--short'],text=True)
    manifest['source_files_sha256']={str(q.relative_to(source)):digest(q) for q in sorted(source.rglob('*')) if q.is_file() and '.git' not in q.parts}
    manifest['built_libraries_sha256']={str(q.relative_to(ROOT)):digest(q) for q in sorted((ROOT/'ws_slam/build/lightning').rglob('*.so')) if q.is_file()}
    dump(exp/'manifest.json',manifest)
    dump(exp/'acceptance.json',{'three_dimensional':'PENDING','wall_ghosting':'PENDING','loop_alignment':'PENDING',
                              'ground':'PENDING','measured_scale':'PENDING','reproducibility':'PENDING','two_dimensional':'PENDING'})
    print('EXPERIMENT='+str(exp),flush=True)
    if args.mode=='offline':
        with (exp/'input_reader.json').open('w') as out, (exp/'input_reader.log').open('w') as err:
            check=subprocess.run([str(ROOT/'ws_slam/reader_build/check_lightning_bag'),str(bag)],stdout=out,stderr=err)
        if check.returncode:
            dump(exp/'result.json',{'input_reader_exit':check.returncode,'mapping_started':False})
            return 2
    result={};p=None
    try:
        with (exp/'run.log').open('w') as f:
            p=subprocess.Popen(cmd,cwd=exp,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
            if args.mode=='offline':result['exit']=p.wait(timeout=3600)
            else:
                # A separate client requests save before shutdown; it never starts a sensor/base driver.
                time.sleep(args.seconds)
                save=['ros2','service','call','/lightning/save_map','lightning/srv/SaveMap',"{map_id: 'new_map'}"]
                r=subprocess.run(save,capture_output=True,text=True,timeout=60)
                (exp/'save_service.log').write_text(r.stdout+r.stderr)
                result['save_service_exit']=r.returncode
                result['exit']=stop(p)
    except (Exception,KeyboardInterrupt) as e:
        result['error']=repr(e)
    finally:
        if p is not None and p.poll() is None:result['exit']=stop(p)
        result['files']={str(q.relative_to(exp)):digest(q) for q in sorted((exp/'data').rglob('*')) if q.is_file()}
        result['map_check_exit']=subprocess.run(['/usr/bin/python3',str(ROOT/'scripts/check_slam_map.py'),str(exp)]).returncode
        dump(exp/'result.json',result)
    print(json.dumps(result,ensure_ascii=False))
    return 0 if result.get('exit')==0 and result['map_check_exit']==0 and 'error' not in result else 2

if __name__=='__main__':raise SystemExit(main())
