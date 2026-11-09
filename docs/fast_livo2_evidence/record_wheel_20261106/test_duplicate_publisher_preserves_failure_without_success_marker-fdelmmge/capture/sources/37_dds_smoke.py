#!/usr/bin/env python3
"""Real ros2 bag play -> frontend -> DDS interface smoke test, isolated domain."""
import argparse
import hashlib
import json
import math
import os
import signal
import sqlite3
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock
from make_replay_view import create_view, header

ROOT=Path(__file__).resolve().parents[2]


def stop(p):
    if p and p.poll() is None:
        os.killpg(p.pid,signal.SIGINT)
        try: p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5)


def run(source,data,out,with_backend=False,existing_view=False):
    if out.exists() or data.exists(): raise ValueError('Outputs must be new')
    out.mkdir(parents=True);data.mkdir(parents=True)
    if existing_view:
        view=json.loads((source.parent/'view_manifest.json').read_text())
        digests={name:hashlib.sha256() for name in view['serialized_payload_sha256']}
        with sqlite3.connect(next(source.glob('*.db3')).resolve().as_uri()+'?mode=ro',uri=True) as connection:
            for name,payload,record_time in connection.execute('SELECT t.name,m.data,m.timestamp FROM messages m JOIN topics t ON m.topic_id=t.id ORDER BY m.timestamp,m.id'):
                if header(payload)[0]!=record_time:raise ValueError('Existing view record/header mismatch')
                digests[name].update(payload)
        if {k:v.hexdigest() for k,v in digests.items()}!=view['serialized_payload_sha256']:
            raise ValueError('Existing view payload digest mismatch')
        playback=source
        view=dict(view,reused_view=True,independent_payload_verification=True)
    else:
        view=create_view(source,data/'view',6);playback=data/'view'
    (out/'view_manifest.json').write_text(json.dumps(view,indent=2)+'\n')
    rclpy.init()
    node=rclpy.create_node('ranger_livo_dds_smoke')
    poses={};clouds={};clocks=[]
    key=lambda m:m.header.stamp.sec*10**9+m.header.stamp.nanosec
    qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(Odometry,'/fast_livo/odometry',lambda m:poses.update({key(m):m}),qos)
    node.create_subscription(PointCloud2,'/fast_livo/deskewed_imu',lambda m:clouds.update({key(m):m}),qos)
    clock_qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(Clock,'/clock',lambda m:clocks.append(m.clock.sec*10**9+m.clock.nanosec),clock_qos)
    backend_samples={};backend_errors=[]
    if with_backend:
        from ranger_fast_livo_backend.msg import BackendEstimate
        def on_backend(message):
            if message.pose_valid and message.state==message.VALID:backend_samples[key(message)]=message
            if message.state==message.ERROR:backend_errors.append(message.reason)
        node.create_subscription(BackendEstimate,'/fast_livo/backend/estimate',on_backend,qos)
    frontend=player=backend=None
    runtime_env=dict(os.environ)
    runtime_env['LD_LIBRARY_PATH']=':'.join([str(ROOT/'ws_fast_livo/deps_install/lib'),
        str(ROOT/'ws_fast_livo/backend_deps/lib'),str(ROOT/'ws_livox/install/livox_ros_driver2/lib'),
        runtime_env.get('LD_LIBRARY_PATH','')])
    try:
        with (out/'frontend.log').open('w') as log:
            frontend=subprocess.Popen([str(ROOT/'ws_fast_livo/install/fast_livo/lib/fast_livo/fastlivo_mapping'),
                '--ros-args','--params-file',str(ROOT/'config/fast_livo2/ranger_mid360s_hik_trial.yaml'),
                '-p','use_sim_time:=true','--log-level','warn'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env=runtime_env)
            deadline=time.monotonic()+10
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=0.05)
                if frontend.poll() is not None: raise RuntimeError('Frontend startup failed')
                if len(node.get_publishers_info_by_topic('/fast_livo/odometry'))==1: break
            else: raise RuntimeError('Frontend not discovered')
            # Wait for the frontend sensor subscriptions before playback.
            deadline=time.monotonic()+5
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=0.05)
                if all(node.count_subscribers(t)>=1 for t in ('/livox/lidar','/livox/imu','/front/image_sync')): break
            else: raise RuntimeError('Sensor subscriptions not discovered')
            if with_backend:
                backend_log=(out/'backend.log').open('w')
                backend=subprocess.Popen([str(ROOT/'ws_fast_livo/backend_install/ranger_fast_livo_backend/lib/ranger_fast_livo_backend/backend_node'),
                    '--ros-args','-p','use_sim_time:=true','-p','data_root:='+str((data/'backend_shards').resolve())],
                    stdout=backend_log,stderr=subprocess.STDOUT,start_new_session=True,env=runtime_env)
                backend_log.close()
                deadline=time.monotonic()+8
                while time.monotonic()<deadline:
                    rclpy.spin_once(node,timeout_sec=.05)
                    if backend.poll() is not None:raise RuntimeError('Backend startup failed')
                    if node.count_subscribers('/fast_livo/odometry')>=2 and node.get_publishers_info_by_topic('/fast_livo/backend/estimate'):break
                else:raise RuntimeError('Backend subscriptions not discovered')
            with (out/'player.log').open('w') as plog:
                player=subprocess.Popen(['ros2','bag','play',str(playback),'--clock','100',
                    '--rate','1','--read-ahead-queue-size','200'],stdout=plog,stderr=subprocess.STDOUT,start_new_session=True)
                deadline=time.monotonic()+30
                while time.monotonic()<deadline and player.poll() is None:
                    rclpy.spin_once(node,timeout_sec=0.02)
                    if frontend.poll() is not None: raise RuntimeError('Frontend exited during replay')
                if player.poll() is None: raise RuntimeError('Playback timeout')
                deadline=time.monotonic()+2
                while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=0.02)
            pairs=sorted(poses.keys() & clouds.keys())
            finite=all(all(math.isfinite(v) for v in [p.pose.pose.position.x,p.pose.pose.position.y,p.pose.pose.position.z,
                p.pose.pose.orientation.x,p.pose.pose.orientation.y,p.pose.pose.orientation.z,p.pose.pose.orientation.w]) for p in poses.values())
            source_frames=all(p.header.frame_id=='fast_livo_local' and p.child_frame_id=='fast_livo_imu' for p in poses.values())
            cloud_frames=all(c.header.frame_id=='fast_livo_imu' for c in clouds.values())
            no_public_tf=not node.get_publishers_info_by_topic('/tf')
            clock_aligned=bool(clocks) and all(abs(t-clocks[-1])<7e9 for t in pairs)
            result={'player_exit':player.returncode,'odom_count':len(poses),'cloud_count':len(clouds),'exact_pairs':len(pairs),
                'finite':finite,'pose_frames_correct':source_frames,'cloud_frame_correct':cloud_frames,
                'no_tf_publisher':no_public_tf,'clock_matches_sensor_epoch':clock_aligned,
                'serialized_payloads_preserved':view['payloads_identical'],
                'first_stamp_ns':pairs[0] if pairs else None,'last_stamp_ns':pairs[-1] if pairs else None,
                'max_points':max((c.width*c.height for c in clouds.values()),default=0),
                'success_criteria':'player exit 0, >=45 exact pairs in 6s, finite and correct frames, no TF publisher, sensor-clock epoch',
                'ROS_DOMAIN_ID':os.environ['ROS_DOMAIN_ID'],'ROS_LOCALHOST_ONLY':os.environ['ROS_LOCALHOST_ONLY']}
            result['passed']=player.returncode==0 and len(pairs)>=45 and all([finite,source_frames,cloud_frames,no_public_tf,clock_aligned])
            if with_backend:
                result.update(backend_valid_samples=len(backend_samples),backend_stamps_subset_of_frontend=set(backend_samples).issubset(pairs),backend_errors=backend_errors)
                result['passed'] &= len(backend_samples)>=40 and result['backend_stamps_subset_of_frontend'] and not backend_errors
        stop(backend)
        if backend:
            result['backend_exit']=backend.returncode;result['passed'] &= backend.returncode==0
        stop(frontend)
        result['frontend_exit']=frontend.returncode
        result['passed'] &= frontend.returncode==0
        (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2))
        return 0 if result['passed'] else 1
    finally:
        stop(player);stop(backend);stop(frontend);node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('source',type=Path);p.add_argument('--data',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--with-backend',action='store_true');p.add_argument('--existing-view',action='store_true')
    args=p.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or os.environ.get('ROS_DOMAIN_ID')!='174':
        p.error('Requires isolated ROS_DOMAIN_ID=174 ROS_LOCALHOST_ONLY=1')
    raise SystemExit(run(args.source,args.data,args.out,args.with_backend,args.existing_view))
