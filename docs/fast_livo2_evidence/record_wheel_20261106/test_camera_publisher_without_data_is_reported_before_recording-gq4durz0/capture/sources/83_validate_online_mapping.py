#!/usr/bin/env python3
"""Focused real-node checks: old sensor epoch, stale clock, seal identity and drain.

Default dry-run. Reuses eight recorded frontend pairs; no devices or wheel/TF.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'scripts'))
from session_manager import atomic_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--native', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--domain', type=int, default=211)
    args = parser.parse_args()
    if not 1 <= args.domain <= 232: parser.error('isolated domain required')
    if not args.execute:
        print(json.dumps({'action': 'DRY-RUN', 'hardware_started': False, 'domain': args.domain})); return 0
    args.output = args.output.resolve(); args.output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1',
        RANGER_SESSION_HEALTH_FILE=str(args.output/'health.json'),
        RANGER_LIVO_TIMESHARE=str(args.output/'unused_timeshare'))
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from rosgraph_msgs.msg import Clock
    from ranger_fast_livo_backend.msg import BackendEstimate
    from ranger_fast_livo_backend.srv import FinalizeMapping
    from mapping_monitor import Monitor
    pairs = {}
    with sqlite3.connect(next(args.native.glob('*.db3')).resolve().as_uri()+'?mode=ro', uri=True) as conn:
        names = dict(conn.execute('SELECT id,name FROM topics'))
        for tid, data in conn.execute('SELECT topic_id,data FROM messages ORDER BY timestamp,id LIMIT 16'):
            kind = Odometry if names[tid] == '/fast_livo/odometry' else PointCloud2
            message = deserialize_message(data, kind)
            pairs.setdefault(Monitor.stamp(message), {})[names[tid]] = message
    ordered = sorted(pairs.items())
    if len(ordered) != 8 or any(len(row) != 2 for _, row in ordered): raise ValueError('expected eight complete native pairs')
    rclpy.init()
    node = rclpy.create_node('ranger_online_mapping_contract_check')
    executor = SingleThreadedExecutor(); executor.add_node(node)
    latest, clock_ns, metrics, views = [None], [None], [{}], []
    node.create_subscription(BackendEstimate, '/fast_livo/backend/estimate', lambda m: latest.__setitem__(0, m), 10)
    node.create_subscription(Clock, '/clock', lambda m: clock_ns.__setitem__(0, m.clock.sec*10**9+m.clock.nanosec),
        QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(PointCloud2, '/fast_livo/backend/view/map', lambda m: views.append(m.width),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    odom = node.create_publisher(Odometry, '/fast_livo/odometry', 32)
    cloud = node.create_publisher(PointCloud2, '/fast_livo/deskewed_imu', 32)
    lidar = node.create_publisher(PointCloud2, '/livox/lidar', 10)
    backend = None; log = None; monitor = None; receipt_process = None; receipt_log = None
    result = {'passed': False, 'hardware_started': False, 'domain': args.domain}
    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end: executor.spin_once(timeout_sec=.01)
    def wait(predicate, timeout=10):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)
            if backend is not None and backend.poll() is not None: raise RuntimeError('backend exited')
            if predicate(): return
        raise TimeoutError('contract check timed out: '+str(latest[0]))
    try:
        spin(1)
        for topic in ('/fast_livo/backend/estimate', '/clock', '/tf', '/cmd_vel'):
            if node.get_publishers_info_by_topic(topic): raise RuntimeError('occupied domain: '+topic)
        monitor = Monitor(args.output, False); executor.add_node(monitor)
        receipt_topics={'/livox/lidar':'sensor_msgs/msg/PointCloud2',
            '/fast_livo/odometry':'nav_msgs/msg/Odometry','/fast_livo/deskewed_imu':'sensor_msgs/msg/PointCloud2'}
        receipt_command=[str(ROOT/'ws_fast_livo/online_install/ranger_fast_livo_backend/lib/ranger_fast_livo_backend/receipt_probe'),
            '--ros-args','-p','context:='+str(args.output),'-p','topics_json:='+json.dumps(json.dumps(receipt_topics))]
        receipt_log=(args.output/'receipt_probe.log').open('xb')
        receipt_process=subprocess.Popen(receipt_command,stdout=receipt_log,stderr=subprocess.STDOUT,start_new_session=True)
        command = [str(ROOT/'ws_fast_livo/online_install/ranger_fast_livo_backend/lib/ranger_fast_livo_backend/backend_node'),
            '--ros-args', '--params-file', str(ROOT/'config/fast_livo2/backend.yaml'),
            '-p', 'use_sim_time:=true', '-p', 'publish_map_view:=true',
            '-p', 'data_root:='+str(args.output/'shards')]
        atomic_json(args.output/'command.json', command)
        log = (args.output/'backend.log').open('xb')
        backend = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        wait(lambda: latest[0] is not None and latest[0].reason == 'frontend_publisher_identity_established_or_changed')
        wait(lambda: monitor.receipt_time is not None and node.count_subscribers('/livox/lidar')>=1)
        fake = PointCloud2(); fake.header.stamp.sec, fake.header.stamp.nanosec = divmod(ordered[0][0], 10**9)
        lidar.publish(fake); wait(lambda: clock_ns[0] is not None)
        started = time.monotonic()
        for ns, row in ordered:
            target = started+(ns-ordered[0][0])/1e9
            while time.monotonic() < target: executor.spin_once(timeout_sec=.005)
            odom.publish(row['/fast_livo/odometry']); cloud.publish(row['/fast_livo/deskewed_imu'])
        wait(lambda: latest[0] and Monitor.stamp(latest[0]) == ordered[-1][0] and latest[0].pose_valid)
        session = latest[0].session_id
        client = node.create_client(FinalizeMapping, '/fast_livo/backend/finalize_mapping')
        if not client.wait_for_service(timeout_sec=5): raise RuntimeError('missing finalization service')
        bad = FinalizeMapping.Request(); bad.expected_session_id = 'wrong-session'
        bad.destination = str(args.output/'wrong_map'); bad.timeout_seconds = 5.
        future = client.call_async(bad); wait(future.done)
        if future.result().success or (args.output/'wrong_map').exists(): raise AssertionError('wrong session accepted')
        before = clock_ns[0]; spin(.8)
        if clock_ns[0]-before < 600_000_000 or latest[0].pose_valid:
            raise AssertionError('clock froze on stalled sensor or stale pose remained valid')
        request = FinalizeMapping.Request(); request.expected_session_id = session
        request.destination = str(args.output/'map'); request.timeout_seconds = 10.
        future = client.call_async(request); wait(future.done, 15)
        response = future.result()
        if not response.success or response.processed_pairs != 8 or response.last_stamp_ns != ordered[-1][0]:
            raise AssertionError('seal/drain failed: '+str(response))
        odom.publish(ordered[-1][1]['/fast_livo/odometry'])
        cloud.publish(ordered[-1][1]['/fast_livo/deskewed_imu'])
        node.destroy_publisher(odom); node.destroy_publisher(cloud); spin(.5)
        if latest[0].session_id != session: raise AssertionError('sealed map reset after publishers disappeared')
        if not (args.output/'map/COMMITTED').is_file() or not any(width > 0 for width in views):
            raise AssertionError('archive or optimized map display missing')
        result.update(passed=True, old_epoch_accepted=True, stalled_clock_keeps_advancing=True,
            stale_pose_invalidated=True, wrong_session_rejected=True, sealed_publisher_loss_preserves_graph=True,
            processed_pairs=response.processed_pairs, last_stamp_ns=response.last_stamp_ns,
            map_display_points=max(views), session_id=session)
    except Exception as exc: result['error'] = str(exc)
    finally:
        if backend is not None and backend.poll() is None:
            backend.send_signal(signal.SIGINT)
            try: backend.wait(timeout=10)
            except subprocess.TimeoutExpired: backend.kill(); backend.wait(timeout=2)
        if log: log.close()
        if receipt_process is not None and receipt_process.poll() is None:
            receipt_process.send_signal(signal.SIGINT)
            try:receipt_process.wait(timeout=5)
            except subprocess.TimeoutExpired:receipt_process.kill();receipt_process.wait(timeout=2)
        if receipt_log:receipt_log.close()
        if monitor: executor.remove_node(monitor); monitor.destroy_node()
        executor.remove_node(node); node.destroy_node(); executor.shutdown(); rclpy.shutdown()
        atomic_json(args.output/'result.json', result)
    print(json.dumps(result, indent=2)); return 0 if result['passed'] else 2


if __name__ == '__main__': raise SystemExit(main())
