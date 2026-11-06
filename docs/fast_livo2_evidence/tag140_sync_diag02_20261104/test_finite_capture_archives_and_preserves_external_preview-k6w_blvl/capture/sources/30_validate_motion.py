#!/usr/bin/env python3
"""Real recorded frontend -> actual ROS backend, simulated clock, frame acknowledgments.

Default dry-run. No hardware, frontend feedback, navigation TF, or wheel publisher.
Uses variable wall-time pacing, so this is functional validation, not a real-time benchmark.
"""
import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import sys
import time

import numpy as np

ROOT=Path(__file__).resolve().parents[3]


def sha(path):
    digest=hashlib.sha256()
    with path.open('rb') as handle:
        for data in iter(lambda:handle.read(1024*1024),b''):digest.update(data)
    return digest.hexdigest()


def matrix(pose):
    if isinstance(pose,dict):
        t=np.array(pose['xyz']);x,y,z,w=pose['xyzw']
    else:
        t=np.array([pose.position.x,pose.position.y,pose.position.z])
        q=pose.orientation;x,y,z,w=q.x,q.y,q.z,q.w
    values=np.array([x,y,z,w],dtype=float)
    norm=np.linalg.norm(values)
    if not np.isfinite(t).all() or not np.isfinite(values).all() or abs(norm-1)>1e-3:
        raise ValueError('Invalid pose')
    x,y,z,w=values/norm
    result=np.eye(4);result[:3,3]=t
    result[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                  [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                  [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    return result


def load_pairs(folder):
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    files=list(folder.glob('*.db3'))
    if len(files)!=1:raise ValueError('Expected one native-frames database')
    connection=sqlite3.connect(files[0].resolve().as_uri()+'?mode=ro',uri=True)
    names=dict(connection.execute('SELECT id,name FROM topics'))
    rows={}
    for tid,data in connection.execute('SELECT topic_id,data FROM messages ORDER BY timestamp,id'):
        kind=Odometry if names[tid]=='/fast_livo/odometry' else PointCloud2
        if names[tid] not in ('/fast_livo/odometry','/fast_livo/deskewed_imu'):raise ValueError('Unexpected native topic')
        msg=deserialize_message(data,kind)
        ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        row=rows.setdefault(ns,{})
        if names[tid] in row:raise ValueError('Duplicate native message')
        row[names[tid]]=msg
    connection.close()
    if any(len(row)!=2 for row in rows.values()):raise ValueError('Unpaired native messages')
    return [(ns,row['/fast_livo/odometry'],row['/fast_livo/deskewed_imu']) for ns,row in sorted(rows.items())]


def read_pcd(path):
    header={}
    with path.open('rb') as handle:
        while True:
            raw=handle.readline()
            if not raw:raise ValueError('Truncated PCD header')
            line=raw.decode().strip()
            if not line:continue
            if line.startswith('#'):continue
            key,*values=line.split();header[key]=values
            if key=='DATA':break
        if header['DATA']!=['binary']:raise ValueError('Expected uncompressed binary PCD')
        dtype=[]
        for i,(name,size,kind,count) in enumerate(zip(header['FIELDS'],header['SIZE'],header['TYPE'],header['COUNT'])):
            if count!='1':raise ValueError('Unsupported PCD count')
            dtype.append((name if name!='_' else 'padding'+str(i),'<'+{'F':'f','I':'i','U':'u'}[kind]+size))
        data=np.frombuffer(handle.read(),dtype=np.dtype(dtype))
        if len(data)!=int(header['POINTS'][0]):raise ValueError('PCD point count mismatch')
        points=np.column_stack([data[n] for n in ('x','y','z','intensity')])
        if not np.isfinite(points).all():raise ValueError('Nonfinite PCD data')
        return points


def check_snapshot(folder):
    manifest=json.loads((folder/'manifest.json').read_text())
    committed=json.loads((folder/'COMMITTED').read_text())
    if committed['manifest_sha256']!=sha(folder/'manifest.json'):raise AssertionError('Commit marker hash mismatch')
    for name,digest in manifest['files'].items():
        if sha(folder/name)!=digest:raise AssertionError('Snapshot checksum mismatch: '+name)
    frames=manifest['keyframes'];factors=manifest['factors']
    if len({f['id'] for f in factors})!=len(factors):raise AssertionError('Duplicate factor ID')
    for factor in factors:
        if not factor['loop'] and not factor['prior']:
            expected=np.linalg.inv(matrix(frames[factor['i']]['U_local_imu']))@matrix(frames[factor['j']]['U_local_imu'])
            if np.max(np.abs(expected-matrix(factor['Z_i_j'])))>1e-8:raise AssertionError('Wrong odometry factor direction')
    first=next(f for f in factors if f['prior'])
    if not np.allclose(matrix(first['Z_i_j']),matrix(frames[0]['U_local_imu']),atol=1e-9):raise AssertionError('Wrong map origin')
    global_map=read_pcd(folder/'map.pcd');offset=0;worst=0.
    for keyframe in frames:
        local=read_pcd(folder/keyframe['directory']/'full.pcd')
        transform=matrix(keyframe['X_map_imu'])
        expected=local[:,:3].astype(float)@transform[:3,:3].T+transform[:3,3]
        actual=global_map[offset:offset+len(local),:3]
        worst=max(worst,float(np.max(np.abs(expected-actual))))
        offset+=len(local)
    if offset!=len(global_map) or worst>5e-5:raise AssertionError('Global map not rebuilt from individual optimized poses')
    records=[json.loads(line) for line in (folder/'candidates.jsonl').read_text().splitlines()]
    for row in records:
        source=frames[row['j']]
        if source['scan_id'] in row.get('target_scan_ids',[]):raise AssertionError('Source scan appears in historical submap')
        for scan_id in row.get('target_scan_ids',[]):
            target=next(f for f in frames if f['scan_id']==scan_id)
            if source['id']-target['id']<=manifest['config']['exclude_ids'] or (source['stamp_ns']-target['stamp_ns'])/1e9<=manifest['config']['exclude_time']:
                raise AssertionError('Submap violates exclusion windows')
    accepted=[r for r in records if r['decision']=='accepted_and_optimized']
    for row in accepted:
        if row['graph_error_after_same_factors']>row['graph_error_before_same_factors']+1e-6:raise AssertionError('Accepted graph objective increased')
    return manifest,records,{'checksummed_files':len(manifest['files']),'map_points':len(global_map),
        'maximum_map_rebuild_coordinate_error_m':worst,'scan_self_match_count':0,
        'accepted_loop_updates':len(accepted),'loop_factors':sum(f['loop'] for f in factors),
        'candidate_decisions':dict(Counter(r['decision'] for r in records)),
        'candidate_count':len(records),'keyframes':len(frames),'factors':len(factors)}


def check_archived_inputs(folder,pairs):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    manifest=json.loads((folder/'manifest.json').read_text())
    scans=set()
    for key in manifest['keyframes']:
        index=key['scan_id'];ns,odom,source=pairs[index]
        if index in scans or ns!=key['stamp_ns']:raise AssertionError('Archive scan identity mismatch')
        scans.add(index)
        archived=deserialize_message((folder/key['directory']/'scan.cdr').read_bytes(),PointCloud2)
        for name in ('header','height','width','fields','is_bigendian','point_step','row_step','is_dense'):
            if getattr(archived,name)!=getattr(source,name):raise AssertionError('Archived cloud field differs: '+name)
        if bytes(archived.data)!=bytes(source.data):raise AssertionError('Archived cloud payload differs')
        if not np.allclose(matrix(key['U_local_imu']),matrix(odom.pose.pose),atol=1e-9):
            raise AssertionError('Archived raw pose differs from input')
    return {'archived_keyframes_checked':len(scans),'unique_source_scan_ids':len(scans),
            'cloud_fields_header_and_payload_preserved':True,'raw_pose_and_integer_stamp_preserved':True}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--native',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--domain',type=int,default=192)
    parser.add_argument('--registration',choices=('pcl_icp','small_gicp','small_vgicp'),default='pcl_icp')
    args=parser.parse_args()
    if not 0<=args.domain<=232:parser.error('Invalid ROS domain')
    if not args.execute:
        print(json.dumps({'action':'DRY-RUN','native':str(args.native),'output':str(args.output),
            'domain':args.domain,'registration':args.registration,'default_thresholds':True,
            'acknowledgment_pacing':True,'hardware_started':False},indent=2));return 0
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain),ROS_LOCALHOST_ONLY='1')
    import rclpy
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from rosgraph_msgs.msg import Clock
    from diagnostic_msgs.msg import DiagnosticArray
    from ranger_fast_livo_backend.msg import BackendEstimate
    from ranger_fast_livo_backend.srv import ExportSnapshot
    pairs=load_pairs(args.native)
    raw_poses=[matrix(o.pose.pose) for _,o,_ in pairs]
    rclpy.init();node=rclpy.create_node('ranger_stage_b_motion_validation')
    backend=None;latest=None;metrics={};samples=[];errors=[];sessions=set();current_ns=[pairs[0][0]]
    odom=node.create_publisher(Odometry,'/fast_livo/odometry',32)
    cloud=node.create_publisher(PointCloud2,'/fast_livo/deskewed_imu',32)
    clock=node.create_publisher(Clock,'/clock',QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))
    def tick_clock():
        msg=Clock();msg.clock.sec,msg.clock.nanosec=divmod(current_ns[0],10**9);clock.publish(msg)
    node.create_timer(.02,tick_clock)
    def estimate(message):
        nonlocal latest
        latest=message
        if message.state==message.ERROR:errors.append(message.reason)
    def health(message):
        nonlocal metrics
        for status in message.status:
            for value in status.values:
                if value.key=='metrics':metrics=json.loads(value.value)
    node.create_subscription(BackendEstimate,'/fast_livo/backend/estimate',estimate,100)
    node.create_subscription(DiagnosticArray,'/fast_livo/backend/health',health,10)
    client=node.create_client(ExportSnapshot,'/fast_livo/backend/export_snapshot')
    def wait(predicate,timeout=15):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01)
            if backend is not None and backend.poll() is not None:raise RuntimeError('Backend exited '+str(backend.returncode))
            if errors:raise RuntimeError('Backend error: '+str(errors))
            if predicate():return
        raise TimeoutError('Backend wait expired; latest='+str(latest))
    outcome={'passed':False,'hardware_started':False,'domain':args.domain,'registration':args.registration,
             'pacing':'one input pair; wait for actual backend output stamp; not a real-time benchmark'}
    try:
        # Occupancy check precedes node startup. Own publishers are deliberately unique.
        end=time.monotonic()+1
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        for topic in ('/fast_livo/backend/estimate','/tf','/cmd_vel'):
            if node.get_publishers_info_by_topic(topic):raise RuntimeError('Occupied validation domain: '+topic)
        for topic in ('/fast_livo/odometry','/fast_livo/deskewed_imu'):
            if len(node.get_publishers_info_by_topic(topic))!=1:raise RuntimeError('Input topic is not exclusive')
        metadata={'source_native':str(args.native.resolve()),'frontend_gravity_aligned':False,
                  'physical_sync':'PARTIAL','hardware_started':False,
                  'native_database_sha256':sha(next(args.native.glob('*.db3'))),
                  'backend_config_sha256':sha(ROOT/'config/fast_livo2/backend.yaml')}
        (output/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
        command=[str(ROOT/'ws_fast_livo/backend_install/ranger_fast_livo_backend/lib/ranger_fast_livo_backend/backend_node'),
            '--ros-args','--params-file',str(ROOT/'config/fast_livo2/backend.yaml'),
            '-p','use_sim_time:=true','-p','data_root:='+str(output/'shards'),
            '-p','metadata_path:='+str(output/'metadata.json'),'-p','registration:='+args.registration]
        outcome['backend_argv']=command
        log=(output/'backend.log').open('wb')
        backend=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        wait(lambda:latest is not None and latest.reason=='frontend_publisher_identity_established_or_changed'
             and node.count_subscribers('/fast_livo/odometry')==1 and client.service_is_ready())
        latency=[];propagation_errors=[];started=time.monotonic()
        with (output/'estimates.jsonl').open('w') as handle:
            for index,(ns,o,c) in enumerate(pairs):
                current_ns[0]=ns;tick_clock();sent=time.monotonic()
                odom.publish(o);cloud.publish(c)
                wait(lambda:latest is not None and latest.pose_valid and
                     latest.header.stamp.sec*10**9+latest.header.stamp.nanosec==ns,timeout=20)
                m=latest;sessions.add(m.session_id)
                if len(sessions)!=1 or m.scan_id!=index:raise AssertionError('Session changed or scan was lost')
                latency.append((time.monotonic()-sent)*1000)
                pose=matrix(m.pose)
                row={'stamp_ns':ns,'scan_id':int(m.scan_id),'session_id':m.session_id,
                    'graph_version':int(m.graph_version),'reference_keyframe_id':int(m.reference_keyframe_id),
                    'loop_event_id':int(m.loop_event_id),'loop_accepted':m.loop_accepted,
                    'optimization_committed':m.optimization_committed,'pose_matrix':pose.tolist()}
                if samples and samples[-1]['graph_version']==row['graph_version']:
                    expected=np.array(samples[-1]['pose_matrix'])@np.linalg.inv(raw_poses[index-1])@raw_poses[index]
                    propagation_errors.append(float(np.max(np.abs(expected-pose))))
                samples.append(row);handle.write(json.dumps(row)+'\n')
                if index%100==0:print(f'{args.registration} ACK {index+1}/{len(pairs)} graph={m.graph_version} loop_event={m.loop_event_id}',flush=True)
        request=ExportSnapshot.Request();request.destination=str(output/'snapshot')
        future=client.call_async(request);wait(future.done,timeout=60)
        response=future.result()
        if not response.success:raise RuntimeError('Snapshot export: '+response.reason)
        manifest,records,checks=check_snapshot(output/'snapshot')
        checks.update(check_archived_inputs(output/'snapshot',pairs))
        if response.session_id!=samples[-1]['session_id'] or response.graph_version!=samples[-1]['graph_version']:
            raise AssertionError('Export graph does not match acknowledged output')
        last_key=manifest['keyframes'][samples[-1]['reference_keyframe_id']]
        expected=matrix(last_key['X_map_imu'])@np.linalg.inv(matrix(last_key['U_local_imu']))@raw_poses[-1]
        last_error=float(np.max(np.abs(expected-np.array(samples[-1]['pose_matrix']))))
        if max(propagation_errors or [0])>1e-7 or last_error>1e-7:raise AssertionError('Non-keyframe propagation mismatch')
        final_metrics=dict(metrics)
        if final_metrics.get('invalid_messages') or final_metrics.get('pair_cache_entries') or final_metrics.get('work_queue_entries'):
            raise AssertionError('Invalid input or undrained queues')
        if any(final_metrics.get('pair_counters',{}).values()):raise AssertionError('Pairing counter nonzero')
        public={t:len(node.get_publishers_info_by_topic(t)) for t in ('/tf','/tf_static','/cmd_vel','/base/safe_cmd_vel')}
        if any(public.values()):raise AssertionError('Unexpected public output')
        corrections=[np.linalg.norm(matrix(k['X_map_imu'])[:3,3]-matrix(k['U_local_imu'])[:3,3]) for k in manifest['keyframes']]
        outcome.update(passed=True,source_pairs=len(pairs),acknowledged_pairs=len(samples),source_duration_s=(pairs[-1][0]-pairs[0][0])/1e9,
            wall_seconds=time.monotonic()-started,session_id=response.session_id,graph_version=int(response.graph_version),
            loop_events=int(manifest['loop_event_id']),snapshot_checks=checks,diagnostics=final_metrics,
            nonkeyframe_propagation_checks=len(propagation_errors),max_nonkeyframe_propagation_error=max(propagation_errors or [0]),
            final_propagation_error=last_error,public_publishers=public,
            raw_endpoint_distance_m=float(np.linalg.norm(raw_poses[-1][:3,3]-raw_poses[0][:3,3])),
            optimized_endpoint_distance_m=float(np.linalg.norm(np.array(samples[-1]['pose_matrix'])[:3,3]-np.array(samples[0]['pose_matrix'])[:3,3])),
            keyframe_correction_m_p50_p95_max=np.percentile(corrections,[50,95,100]).tolist(),
            acknowledgment_latency_ms_p50_p95_max=np.percentile(latency,[50,95,100]).tolist(),
            accuracy_validated=False,physical_sync_proven=False)
    except Exception as exc:
        outcome['failure']=str(exc)
        raise
    finally:
        if backend:
            backend.send_signal(signal.SIGINT)
            try:backend.wait(timeout=15)
            except subprocess.TimeoutExpired:backend.terminate();backend.wait(timeout=5)
            outcome['backend_exit']=backend.returncode;log.close()
        (output/'result.json').write_text(json.dumps(outcome,indent=2)+'\n')
        node.destroy_node();rclpy.shutdown()
    print(json.dumps(outcome,indent=2),flush=True)
    return 0


if __name__=='__main__':raise SystemExit(main())
