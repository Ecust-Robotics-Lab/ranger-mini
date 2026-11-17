"""Isolated mock-input transport through the installed B node; no devices."""
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'scripts/fast_livo2'), str(ROOT/'scripts')]


class StartupTransportTest(unittest.TestCase):
    def test_stable_clock_gates_first_frontend_pair(self):
        if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID', '0') == '0':
            self.skipTest('requires explicit isolated domain and ROS_LOCALHOST_ONLY=1')
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2, PointField
        from std_msgs.msg import String
        from mapping_monitor import Monitor
        from session_manager import identity, notify

        with tempfile.TemporaryDirectory(prefix='ranger-mapping-clock-') as tmp:
            folder = Path(tmp)
            os.environ['RANGER_SESSION_HEALTH_FILE'] = str(folder/'health.json')
            os.environ['RANGER_LIVO_TIMESHARE'] = str(folder/'unused_mock_timeshare')
            env = dict(os.environ)
            env['LD_LIBRARY_PATH'] = ':'.join([str(ROOT/'ws_fast_livo/online_install/ranger_fast_livo_backend/lib'),
                str(ROOT/'ws_fast_livo/backend_deps/lib'), env.get('LD_LIBRARY_PATH', '')])
            command = [str(ROOT/'ws_fast_livo/online_install/ranger_fast_livo_backend/lib/ranger_fast_livo_backend/backend_node'),
                '--ros-args', '--params-file', str(ROOT/'config/fast_livo2/backend.yaml'),
                '-p', 'use_sim_time:=true', '-p', 'data_root:='+str(folder/'backend')]
            with (folder/'backend.log').open('w') as log:
                proc = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT)
                record = identity(proc.pid)
                rclpy.init()
                monitor = Monitor(folder, False)
                driver = Node('mapping_clock_synthetic_inputs')
                receipts = driver.create_publisher(String, '/fast_livo/capture/input_receipts', 10)
                odometry = driver.create_publisher(Odometry, '/fast_livo/odometry', 10)
                clouds = driver.create_publisher(PointCloud2, '/fast_livo/deskewed_imu', 10)
                executor = SingleThreadedExecutor()
                executor.add_node(monitor)
                executor.add_node(driver)
                origin, sensor, counter = time.monotonic_ns(), 1_590_000_000_000_000_001, 0
                next_input = 0
                sent = 0
                try:
                    end = time.monotonic()+12
                    while time.monotonic() < end:
                        self.assertIsNone(proc.poll(), (folder/'backend.log').read_text()[-2000:])
                        executor.spin_once(timeout_sec=.005)
                        now = time.monotonic_ns()
                        if now >= next_input:
                            counter += 1
                            ns = sensor+now-origin+(2_900_000_000 if counter > 1 else 0)
                            row = {'count': counter, 'first_header_ns': sensor, 'first_steady_ns': origin,
                                   'last_header_ns': ns, 'last_steady_ns': now}
                            msg = String()
                            msg.data = json.dumps({'schema_version': 1, 'context': tmp,
                                'topics': {t: row for t in ('/livox/lidar', '/livox/imu', '/front/image_sync')}})
                            receipts.publish(msg)
                            if monitor.anchor and json.loads((folder/'health.json').read_text())['checks']['clock_ready']:
                                m = Odometry()
                                m.header.frame_id, m.child_frame_id = 'fast_livo_local', 'fast_livo_imu'
                                m.header.stamp.sec, m.header.stamp.nanosec = divmod(ns, 10**9)
                                m.pose.pose.orientation.w = 1.
                                cloud = PointCloud2()
                                cloud.header.stamp = m.header.stamp
                                cloud.header.frame_id = 'fast_livo_imu'
                                cloud.height, cloud.width, cloud.point_step, cloud.row_step = 1, 100, 16, 1600
                                cloud.fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                                                for i, n in enumerate(('x', 'y', 'z', 'intensity'))]
                                cloud.data = b''.join(struct.pack('<ffff', i*.1, j*.1, k*.1, 1.)
                                                      for i in range(5) for j in range(5) for k in range(4))
                                odometry.publish(m)
                                clouds.publish(cloud)
                                sent += 1
                            next_input = now+100_000_000
                        if sent >= 10 and monitor.metrics.get('processed_pairs', 0) >= 8:
                            break
                    self.assertGreaterEqual(sent, 10)
                    self.assertEqual(monitor.clock_error, '')
                    self.assertEqual(monitor.metrics.get('invalid_messages', -1), 0)
                    self.assertGreaterEqual(monitor.metrics.get('processed_pairs', 0), 8)
                    self.assertEqual(monitor.backend['state'], 1)
                    print(json.dumps({'synthetic_inputs_only': True, 'sent_pairs': sent,
                        'processed_pairs': monitor.metrics['processed_pairs'], 'invalid_messages': 0,
                        'backend_state': monitor.backend['state'], 'startup_resets': monitor.startup_anchor.resets}))
                finally:
                    executor.shutdown()
                    driver.destroy_node()
                    monitor.destroy_node()
                    rclpy.shutdown()
                    notify(record, signal.SIGINT)
                    try:
                        proc.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        notify(record, signal.SIGTERM)
                        proc.wait(timeout=5)


if __name__ == '__main__':
    unittest.main(verbosity=2)
