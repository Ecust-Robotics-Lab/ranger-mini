#!/usr/bin/env python3
"""Actual C2/C1 -> adapter -> bridge with capture-bound diagnostic wheel/time. No devices."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'ws_nav/src/ranger_localization_bridge'),str(Path(__file__).parent)]
from analyze_wheel_alignment import diagnostic_feedback
from capture_clock import CaptureClock


def sha(p):
    d=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(4*1024*1024),b''):d.update(b)
    return d.hexdigest()


def query_compatibility(capture, identity):
    manifest=json.loads((capture/'manifest.json').read_text())
    def archived(relative):
        row=next(r for r in manifest['source_files'] if r['source']==str(ROOT/relative))
        path=capture/row['file']
        if sha(path)!=row['sha256']:raise ValueError('archived query configuration changed')
        return path
    frontend=yaml.safe_load(archived('config/fast_livo2/ranger_mid360s_hik_trial.yaml').read_text())['/**']['ros__parameters']
    expected=identity['definitions']
    checks={k:frontend[k]==expected['sensor_configuration_sha256'][k] for k in ['extrin_calib','camera','preprocess']}
    checks['time_offset']=frontend['time_offset']==expected['time_contract_sha256']['time_offset']
    checks['camera_driver']=yaml.safe_load(archived('config/fast_livo2/camera_live.yaml').read_text())==expected['sensor_configuration_sha256']['camera_driver']
    checks['lidar_driver']=json.loads(archived('config/fast_livo2/mid360s_live.json').read_text())==expected['sensor_configuration_sha256']['lidar_driver']
    if not all(checks.values()):raise ValueError('query/map sensor-time contract mismatch: '+str(checks))
    return {'checks':checks,'query_binary_sha256':next(r['sha256'] for r in manifest['source_files'] if r['source'].endswith('/fastlivo_mapping')),
        'map_frontend_binary_sha256':expected['frontend_sources_sha256']['binary_sha256'],
        'binary_note':'original recorded frontend versus online I/O-derived frontend; offline compatibility, not identical binaries'}


def accept_result(result, gap_expected):
    return bool(result['valid_samples'] and result['ready_samples'] and result['tf_checks']
        and result['tf_translation_error_max_m']<.02 and not result['end_stale_ready']
        and not any(result['gap_ready_samples']) and (not gap_expected or result['post_gap_ready_recovered'])
        and result['bridge_health'].get('innovation_unlocked')=='1'
        and result['dynamic_tf_publishers']==1 and not any(result['velocity_publishers'].values()))
def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture',type=Path,required=True);p.add_argument('--archive',type=Path,required=True)
    p.add_argument('--analysis',type=Path,required=True);p.add_argument('--identity-file',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--execute',action='store_true')
    p.add_argument('--domain',type=int,default=221);p.add_argument('--max-pairs',type=int)
    p.add_argument('--wheel-mode',choices=['diagnostic','raw'],default='diagnostic')
    p.add_argument('--wheel-gap-start',type=float,default=145.);p.add_argument('--wheel-gap-seconds',type=float,default=.8)
    args=p.parse_args()
    if not 1<=args.domain<=232:raise ValueError('isolated domain must be 1..232')
    if not args.execute:
        print(json.dumps({'action':'DRY-RUN','domain':args.domain,'no_devices':True,'diagnostic_time_and_wheel':True,
            'wheel_gap_start':args.wheel_gap_start,'wheel_gap_seconds':args.wheel_gap_seconds}));return 0
    if args.output.exists():raise ValueError('new output directory required')
    args.output.mkdir(parents=True)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain),ROS_LOCALHOST_ONLY='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
    os.environ['PYTHONPATH']=str(ROOT/'ws_nav/src/ranger_localization_bridge')+':'+os.environ.get('PYTHONPATH','')
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Bool
    from diagnostic_msgs.msg import DiagnosticArray
    from ranger_fast_livo_localizer.msg import LocalizerEstimate
    from ranger_localization_msgs.msg import LocalizationEstimate
    from ranger_localization_bridge.geometry import SE3,rpy_to_quat,se3_from_tf_msg
    from ranger_localization_bridge.alignment import PoseHistory
    import tf2_ros
    # Reuse existing exact-scan/cancel acknowledgment rules.
    spec=importlib.util.spec_from_file_location('offline_c1_evaluation',ROOT/'scripts/fast_livo2/localizer/validate_motion.py')
    evaluation=importlib.util.module_from_spec(spec);spec.loader.exec_module(evaluation)
    manifest=json.loads((args.archive/'manifest.json').read_text())
    analysis=json.loads((args.analysis/'analysis.json').read_text())
    if Path(analysis['source_bag']).resolve()!=(args.capture/'raw').resolve():raise ValueError('analysis belongs to another capture')
    if analysis['source_metadata_sha256']!=sha(args.capture/'raw/metadata.yaml'):raise ValueError('capture metadata changed')
    mapping=CaptureClock(args.analysis,args.capture)
    (args.output/'clock_mapping.json').write_text(json.dumps(mapping.to_dict(),indent=2)+'\n')
    (args.output/'executed_runner.py').write_bytes(Path(__file__).read_bytes())
    x=np.load(args.analysis/'streams.npz')
    wt=mapping.host_to_sensor(x['wheel_stamp']);wheel=x['wheel_values'];xy,yaw,rate=diagnostic_feedback(wheel)
    if args.wheel_mode=='raw':
        xy=wheel[:,:2];yaw=np.unwrap(Rotation.from_quat(wheel[:,3:7]).as_euler('xyz')[:,2])
    geometry=yaml.safe_load((ROOT/'config/fast_livo2/navigation_prep.yaml').read_text())['installation']
    xyz,rpy=geometry['t_base_imu_xyz'],geometry['t_base_imu_rpy']
    extrinsics=['-p','t_base_imu_xyz:='+json.dumps(xyz),'-p','t_base_imu_rpy:='+json.dumps(rpy)]
    identity=json.loads(args.identity_file.read_text())
    compatibility=query_compatibility(args.capture,identity)
    prefix=ROOT/'ws_fast_livo/online_localization_install/ranger_fast_livo_localizer'
    binary=prefix/'lib/ranger_fast_livo_localizer/localizer_node'
    commands={
      'localizer':[str(binary),'--ros-args','--params-file',str(ROOT/'config/fast_livo2/localizer.yaml'),
        '--params-file',str(ROOT/'config/fast_livo2/relocalizer.yaml'),'-p','archive_path:='+str(args.archive.resolve()),
        '-p','global_descriptor:=radial_pair_multiscale_v2','-p','registration:=small_gicp',*extrinsics],
      'adapter':[sys.executable,'-m','ranger_localization_bridge.fast_livo_localizer_adapter','--ros-args',
        '-p','use_sim_time:=true','-p','expected_map_uuid:='+manifest['map_uuid'],
        '-p','expected_manifest_sha256:='+sha(args.archive/'manifest.json'),*extrinsics],
      'bridge':[sys.executable,'-m','ranger_localization_bridge.bridge_node','--ros-args','-p','use_sim_time:=true',
        '-p','adapter_backend:=external','-p','latch_on_innovation_failure:=true']}
    for key in ['sensor_configuration_sha256','time_contract_sha256','frontend_sources_sha256']:
        commands['localizer']+=['-p',key+':='+identity[key]]
    dbpath=next((args.capture/'raw').glob('*.db3'))
    db=sqlite3.connect(dbpath.resolve().as_uri()+'?mode=ro',uri=True)
    ids={name:tid for tid,name in db.execute('select id,name from topics')}
    poses=list(db.execute('select data from messages where topic_id=? order by timestamp,id',(ids['/fast_livo/odometry'],)))
    clouds=[r[0] for r in db.execute('select id from messages where topic_id=? order by timestamp,id',(ids['/fast_livo/deskewed_imu'],))]
    if len(poses)!=len(clouds):raise ValueError('unpaired native output count')
    if args.max_pairs:poses=poses[:args.max_pairs];clouds=clouds[:args.max_pairs]
    start_ns=deserialize_message(poses[0][0],Odometry).header.stamp
    start_ns=start_ns.sec*10**9+start_ns.nanosec
    now_ns=start_ns+50_000_000;latest=[];standard=[];ready=[False];health={};processes=[];logs=[]
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def interrupted(signum,_frame):raise KeyboardInterrupt('offline replay interrupted by signal '+str(signum))
    for signum in [signal.SIGINT,signal.SIGTERM,signal.SIGHUP]:signal.signal(signum,interrupted)
    node=Node('offline_c2_wheel_chain',parameter_overrides=[Parameter('use_sim_time',value=True)])
    node.create_subscription(LocalizerEstimate,'/fast_livo/localizer/estimate',latest.append,100)
    node.create_subscription(LocalizationEstimate,'/localization/estimate',standard.append,100)
    node.create_subscription(Bool,'/navigation_ready',lambda m:ready.__setitem__(0,m.data),10)
    def on_health(m):
        if m.status:health.update({v.key:v.value for v in m.status[0].values})
    node.create_subscription(DiagnosticArray,'/localization_health',on_health,10)
    clock=node.create_publisher(Clock,'/clock',10)
    def pump(seconds=.02):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            c=Clock();c.clock.sec,c.clock.nanosec=divmod(now_ns,10**9);clock.publish(c)
            rclpy.spin_once(node,timeout_sec=.005)
    def wait(predicate,timeout=20):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            pump()
            if predicate():return
            if any(proc.poll() is not None for proc in processes):raise RuntimeError('child exited')
        raise TimeoutError('waiting for scan/state: '+(latest[-1].reason if latest else 'no estimate'))
    result={'passed':False,'hardware_started':False,'physical_sync_verified':False,'absolute_accuracy_validated':False,
        'wheel_source':args.wheel_mode,'wheel_note':'diagnostic reconstruction is not newly recorded corrected CAN output',
        'pacing':'scan acknowledgments; not real-time benchmark','clock_mapping':analysis['candidate_sensor_to_host_offset_ns'],
        'wheel_gap':{'start_seconds':args.wheel_gap_start,'duration_seconds':args.wheel_gap_seconds},
        'argv':commands,'source_capture':str(args.capture.resolve()),'map_uuid':manifest['map_uuid'],
        'query_frontend_note':'Recorded query frontend output; map identity file validates archive. Query binary provenance remains separate.',
        'query_compatibility':compatibility,
        'source_binary_sha256':sha(binary),'source_metadata_sha256':analysis['source_metadata_sha256']}
    index=0;history=PoseHistory(duration_s=10.,max_samples=1000);tf_checks=[];valid=ready_samples=0
    first_valid=None;gap_ready=[];post_gap_ready=[];states={};started=time.monotonic()
    def publish_wheel(until):
        nonlocal index
        while index<len(wt) and int(wt[index])<=until:
            ns=int(wt[index]);elapsed=(ns-start_ns)*1e-9
            drop=args.wheel_gap_start<=elapsed<args.wheel_gap_start+args.wheel_gap_seconds
            if not drop:
                m=Odometry();m.header.frame_id='odom';m.child_frame_id='base_link';m.header.stamp.sec,m.header.stamp.nanosec=divmod(ns,10**9)
                m.pose.pose.position.x,m.pose.pose.position.y=map(float,xy[index])
                q=rpy_to_quat(0.,0.,float(yaw[index]));m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w=q
                wheel_pub.publish(m);history.add(ns,SE3((float(xy[index,0]),float(xy[index,1]),0.),q))
            index+=1
    try:
        pump(2)
        for topic in ['/fast_livo/odometry','/fast_livo/deskewed_imu','/wheel/odom','/tf','/tf_static',
                      '/fast_livo/localizer/estimate','/localization/estimate','/cmd_vel','/base/safe_cmd_vel']:
            if node.get_publishers_info_by_topic(topic):raise RuntimeError('isolated domain occupied: '+topic)
        odom_pub=node.create_publisher(Odometry,'/fast_livo/odometry',16)
        cloud_pub=node.create_publisher(PointCloud2,'/fast_livo/deskewed_imu',16)
        wheel_pub=node.create_publisher(Odometry,'/wheel/odom',100)
        buffer=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buffer,node)
        for name,command in commands.items():
            log=(args.output/(name+'.log')).open('w');logs.append(log)
            processes.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        wait(lambda:latest and latest[-1].map_uuid==manifest['map_uuid'] and json.loads(latest[-1].quality_json).get('inputs_ready'))
        with (args.output/'samples.jsonl').open('w') as out:
            for i,(data,cid) in enumerate(zip(poses,clouds)):
                odom=deserialize_message(data[0],Odometry)
                ns=odom.header.stamp.sec*10**9+odom.header.stamp.nanosec;now_ns=ns+50_000_000
                cloud=deserialize_message(db.execute('select data from messages where id=?',(cid,)).fetchone()[0],PointCloud2)
                if cloud.header.stamp!=odom.header.stamp:raise ValueError('native timestamps differ')
                pump(.01);publish_wheel(ns+20_000_000);pump(.01)
                previous=latest[-1];odom_pub.publish(odom);cloud_pub.publish(cloud)
                wait(lambda:evaluation.pair_outcome(latest[-1],previous,ns,True))
                publish_wheel(ns+45_000_000);pump(.025)
                msg=latest[-1];elapsed=(ns-start_ns)*1e-9
                if msg.pose_valid:
                    valid+=1
                    if first_valid is None:first_valid=elapsed
                states[str(msg.state)]=states.get(str(msg.state),0)+1
                if ready[0]:ready_samples+=1
                if args.wheel_gap_start+.55<elapsed<args.wheel_gap_start+args.wheel_gap_seconds:gap_ready.append(ready[0])
                if args.wheel_gap_start+args.wheel_gap_seconds+.5<elapsed<args.wheel_gap_start+args.wheel_gap_seconds+3:post_gap_ready.append(ready[0])
                error=None
                if ready[0] and standard and standard[-1].state==standard[-1].VALID:
                    tf=buffer.lookup_transform('map','odom',rclpy.time.Time())
                    tf_ns=tf.header.stamp.sec*10**9+tf.header.stamp.nanosec
                    actual=se3_from_tf_msg(tf);candidates=[]
                    # TF is emitted on wheel callbacks. A newly delivered estimate
                    # can be newer than the last TF; compare eligible sample times,
                    # never join two independently "latest" arrivals.
                    for sample in standard:
                        stamp=sample.header.stamp.sec*10**9+sample.header.stamp.nanosec
                        if sample.state!=sample.VALID or not 0<=tf_ns-stamp<=500_000_000:continue
                        wp,_=history.lookup(stamp)
                        if wp is None:continue
                        p0=sample.pose.position;q0=sample.pose.orientation
                        expected=SE3((p0.x,p0.y,p0.z),(q0.x,q0.y,q0.z,q0.w))*wp.inverse()
                        delta=expected.inverse()*actual
                        angle=2*float(np.arccos(min(1.,abs(delta.q[3]))))
                        candidates.append((float(np.linalg.norm(delta.t)),angle,stamp))
                    if candidates:
                        error,angle,matched_stamp=min(candidates,key=lambda c:c[0]+c[1])
                        tf_checks.append(error)
                        out.write(json.dumps({'tf_contract':True,'tf_stamp_ns':tf_ns,'matched_sample_ns':matched_stamp,
                            'translation_error_m':error,'rotation_error_rad':angle})+'\n')
                out.write(json.dumps({'index':i,'elapsed':elapsed,'state':msg.state,'valid':msg.pose_valid,'reason':msg.reason,
                    'ready':ready[0],'bridge_health':dict(health),'tf_translation_check_m':error})+'\n')
                if i%100==0:print(json.dumps({'processed':i+1,'valid':valid,'ready_samples':ready_samples,'reason':msg.reason,'bridge':dict(health)}),flush=True)
                if len(latest)>100:del latest[:-30]
                if len(standard)>100:del standard[:-30]
        now_ns+=800_000_000;pump(.45)
        result.update(processed_pairs=len(poses),valid_samples=valid,first_valid_source_seconds=first_valid,ready_samples=ready_samples,
            states=states,tf_checks=len(tf_checks),tf_translation_error_max_m=max(tf_checks,default=None),
            gap_ready_samples=gap_ready,post_gap_ready_recovered=any(post_gap_ready),end_stale_ready=ready[0],
            bridge_health=dict(health),wall_seconds=time.monotonic()-started,
            velocity_publishers={t:len(node.get_publishers_info_by_topic(t)) for t in ['/cmd_vel','/base/safe_cmd_vel']},
            dynamic_tf_publishers=len(node.get_publishers_info_by_topic('/tf')))
        gap_expected=args.wheel_gap_start+args.wheel_gap_seconds+3<(ns-start_ns)*1e-9
        result['passed']=accept_result(result,gap_expected)
    except BaseException as exc:
        result['error']=repr(exc)
        if not isinstance(exc,KeyboardInterrupt):raise
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=8)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=3)
        for log in logs:log.close()
        result['child_exit_codes']=[p.returncode for p in processes]
        (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        db.close();node.destroy_node();rclpy.shutdown()
    return 0 if result['passed'] else 2


if __name__=='__main__':raise SystemExit(main())
