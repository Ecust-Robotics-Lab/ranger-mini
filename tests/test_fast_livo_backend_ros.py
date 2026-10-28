#!/usr/bin/env python3
"""Isolated synthetic/backend ROS tests. No device, CAN, navigation or velocity nodes."""
import json
import math
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
sys.path.insert(0, str(ROOT/'ws_nav/src/ranger_localization_bridge'))
os.environ.update(ROS_DOMAIN_ID='187', ROS_LOCALHOST_ONLY='1', PYTHONDONTWRITEBYTECODE='1')
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray
from ranger_fast_livo_backend.msg import BackendEstimate
from ranger_fast_livo_backend.srv import ExportSnapshot
from ranger_localization_bridge.geometry import SE3, rpy_to_quat, quat_rotate, se3_from_tf_msg
import tf2_ros

EVIDENCE = ROOT/'docs/fast_livo2_evidence/stage_b_20261026'
EXECUTABLE = ROOT/'ws_fast_livo/backend_install/ranger_fast_livo_backend/lib/ranger_fast_livo_backend/backend_node'


def stamp(message, ns):
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(ns, 10**9)


def fill_pose(message, pose):
    message.position.x, message.position.y, message.position.z = pose.t
    message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w = pose.q


class RosBackendTests(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node('stage_b_test_world', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.ns = 1_000*10**9; self.start_ns = self.ns; self.counter = 0
        self.sensors_on = self.wheel_on = True; self.jump = 0.
        self.base_imu = SE3((.2, -.1, .4), rpy_to_quat(.1, -.08, .2))
        self.correction = SE3((2., -1., .3), rpy_to_quat(.1, -.1, .4))
        rng = np.random.default_rng(42)
        self.points = rng.uniform(-4, 4, (600, 3))
        self.points[::3, 0] = -4; self.points[1::3, 1] = 3; self.points[2::3, 2] = -2
        self.clock = self.node.create_publisher(Clock, '/clock', 10)
        self.odom = self.node.create_publisher(Odometry, '/fast_livo/odometry', 32)
        self.cloud = self.node.create_publisher(PointCloud2, '/fast_livo/deskewed_imu', 32)
        self.wheel = self.node.create_publisher(Odometry, '/wheel/odom', 32)
        self.private = []; self.private_arrivals = []; self.ready = False; self.health = {}; self.standard = []
        self.node.create_subscription(BackendEstimate, '/fast_livo/backend/estimate', self.on_private, 100)
        self.node.create_subscription(Bool, '/navigation_ready', lambda m: setattr(self, 'ready', m.data), 10)
        self.node.create_subscription(DiagnosticArray, '/localization_health', self.on_health, 10)
        from ranger_localization_msgs.msg import LocalizationEstimate
        self.node.create_subscription(LocalizationEstimate, '/localization/estimate', lambda m: self.standard.append(m), 100)
        self.buffer = tf2_ros.Buffer(); self.listener = tf2_ros.TransformListener(self.buffer, self.node)
        self.processes = []
        self.directory = Path(tempfile.mkdtemp(prefix=self._testMethodName+'-', dir=EVIDENCE))

    def on_private(self, message):
        self.private.append(message); self.private_arrivals.append(time.monotonic())

    def on_health(self, message):
        if message.status:
            self.health = {v.key: v.value for v in message.status[0].values}

    def spawn(self, command, name):
        log = (self.directory/(name+'.log')).open('w')
        env = dict(os.environ)
        env['PYTHONPATH'] = str(ROOT/'ws_nav/src/ranger_localization_bridge')+':'+env.get('PYTHONPATH', '')
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=env)
        self.processes.append((proc, log))
        return proc

    def backend(self):
        return self.spawn([str(EXECUTABLE), '--ros-args', '-p', 'use_sim_time:=true',
                           '-p', 'data_root:='+str(self.directory/'shards')], 'backend-'+str(len(self.processes)))

    def wheel_pose(self, ns):
        t = (ns-self.start_ns)*1e-9
        return SE3((.15*t+.005*t*t, .025*t, .01*math.sin(t)), rpy_to_quat(.005*t, .003*t, .025*t))

    def send_sensors(self):
        ns = self.ns-30_000_000
        wheel = self.wheel_pose(ns)
        wheel = SE3((wheel.t[0]+self.jump, wheel.t[1], wheel.t[2]), wheel.q)
        imu = self.correction*wheel*self.base_imu
        odom = Odometry(); odom.header.frame_id = 'fast_livo_local'; odom.child_frame_id = 'fast_livo_imu'
        stamp(odom, ns); fill_pose(odom.pose.pose, imu)
        inverse = imu.inverse()
        xyz = np.asarray([quat_rotate(inverse.q, tuple(p)) for p in self.points])+np.asarray(inverse.t)
        data = np.column_stack((xyz, np.arange(len(xyz)))).astype('<f4')
        cloud = PointCloud2(); cloud.header.frame_id = 'fast_livo_imu'; stamp(cloud, ns)
        cloud.height = 1; cloud.width = len(data); cloud.point_step = 16; cloud.row_step = 16*len(data)
        for i, name in enumerate(('x', 'y', 'z', 'intensity')):
            cloud.fields.append(PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1))
        cloud.data = data.tobytes(); cloud.is_dense = True
        self.odom.publish(odom); self.cloud.publish(cloud)

    def pump(self, seconds, sensors=True):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            tick = time.monotonic(); self.ns += 20_000_000
            clock = Clock(); clock.clock.sec, clock.clock.nanosec = divmod(self.ns, 10**9); self.clock.publish(clock)
            if self.wheel_on:
                m = Odometry(); m.header.frame_id = 'odom'; m.child_frame_id = 'base_link'; stamp(m, self.ns)
                fill_pose(m.pose.pose, self.wheel_pose(self.ns)); self.wheel.publish(m)
            if sensors and self.sensors_on and self.counter % 5 == 0:
                self.send_sensors()
            self.counter += 1
            while time.monotonic()-tick < .018:
                rclpy.spin_once(self.node, timeout_sec=.002)
            time.sleep(max(0., .02-(time.monotonic()-tick)))

    def latest_valid(self):
        valid = [m for m in self.private if m.state == m.VALID and m.pose_valid]
        self.assertTrue(valid, 'No valid private estimate; see '+str(self.directory))
        return valid[-1]

    def tearDown(self):
        for proc, log in reversed(self.processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL); proc.wait(timeout=3)
            log.close()
        (self.directory/'observations.json').write_text(json.dumps({
            'private_messages': len(self.private), 'valid_samples': len({(m.session_id, m.header.stamp.sec, m.header.stamp.nanosec) for m in self.private if m.pose_valid}),
            'sessions': sorted({m.session_id for m in self.private}), 'bridge_health': self.health,
            'last_estimates': [{'state': m.state, 'reason': m.reason, 'session': m.session_id,
                                'stamp': m.header.stamp.sec+m.header.stamp.nanosec*1e-9} for m in self.private[-12:]],
            'process_exit_codes': [p.returncode for p, _ in self.processes], 'hardware': False}, indent=2)+'\n')
        self.node.destroy_node(); rclpy.shutdown()

    def test_private_export_and_multiple_publisher_rejection(self):
        proc = self.backend(); self.pump(2.)
        self.assertIsNone(proc.poll()); self.latest_valid()
        self.assertFalse(self.node.get_publishers_info_by_topic('/tf'))
        client = self.node.create_client(ExportSnapshot, '/fast_livo/backend/export_snapshot')
        self.assertTrue(client.wait_for_service(timeout_sec=3))
        request = ExportSnapshot.Request(); request.destination = str(self.directory/'snapshot')
        future = client.call_async(request)
        for _ in range(20):
            self.pump(.1)
            if future.done(): break
        self.assertTrue(future.done()); result = future.result(); self.assertTrue(result.success, result.reason)
        manifest = json.loads(Path(result.manifest_path).read_text())
        self.assertEqual(manifest['graph_version'], result.graph_version)
        self.assertTrue((self.directory/'snapshot/COMMITTED').exists())
        self.extra_odom = self.node.create_publisher(Odometry, '/fast_livo/odometry', 10)
        self.pump(1.)
        self.assertEqual(self.private[-1].state, BackendEstimate.ERROR)
        self.assertIn('multiple_publishers', self.private[-1].reason)

    def test_clock_reset_process_failure_and_restart(self):
        proc = self.backend(); self.pump(2.)
        old = self.latest_valid(); self.ns -= 100*10**9; self.pump(1.)
        new = self.latest_valid()
        self.assertNotEqual(old.session_id, new.session_id)
        self.assertLess(new.header.stamp.sec, old.header.stamp.sec)
        os.kill(proc.pid, signal.SIGKILL); proc.wait(timeout=3)
        self.pump(.5); arrivals = len(self.private); self.pump(.5)
        self.assertEqual(len(self.private), arrivals)
        restarted = self.backend(); self.pump(3.)
        self.assertIsNone(restarted.poll()); self.assertNotEqual(self.latest_valid().session_id, new.session_id)

    def test_adapter_interpolation_loss_and_large_jump_latch(self):
        self.backend()
        self.spawn([sys.executable, '-m', 'ranger_localization_bridge.fast_livo_adapter', '--ros-args',
                    '-p', 'use_sim_time:=true', '-p', 't_base_imu_xyz:=[0.2, -0.1, 0.4]',
                    '-p', 't_base_imu_rpy:=[0.1, -0.08, 0.2]'], 'adapter')
        self.spawn([sys.executable, '-m', 'ranger_localization_bridge.bridge_node', '--ros-args',
                    '-p', 'adapter_backend:=external', '-p', 'use_sim_time:=true',
                    '-p', 'latch_on_innovation_failure:=true'], 'bridge')
        self.pump(3.)
        self.assertTrue(self.ready, self.health)
        correction = se3_from_tf_msg(self.buffer.lookup_transform('map', 'odom', rclpy.time.Time()))
        delta = self.correction.inverse()*correction
        self.assertLess(math.dist(delta.t, (0., 0., 0.)), .003)
        self.assertLess(abs(delta.yaw()), .003)
        self.assertEqual(len(self.node.get_publishers_info_by_topic('/tf')), 1)
        self.wheel_on = False; self.pump(.9); self.assertFalse(self.ready)
        self.wheel_on = True; self.pump(1.); self.assertTrue(self.ready, self.health)
        self.sensors_on = False; self.pump(.3); last = self.private[-1]; self.pump(.8)
        self.assertFalse(self.ready); self.assertEqual(self.private[-1].header.stamp, last.header.stamp)
        self.sensors_on = True; self.pump(1.); self.assertTrue(self.ready, self.health)
        self.jump = 10.; self.pump(1.)
        private = self.latest_valid(); self.assertGreater(private.pose.position.x, 10.)
        self.assertFalse(self.ready); self.assertEqual(self.health.get('innovation_unlocked'), '0')
        self.jump = 0.; self.pump(1.); self.assertFalse(self.ready)
        self.assertEqual(self.health.get('innovation_unlocked'), '0')

    def test_existing_static_native_frames_preserve_stamps(self):
        import sqlite3
        self.sensors_on = self.wheel_on = False
        self.backend(); self.pump(1.)
        path = ROOT/'bags/fast-livo2-baseline-126a5fAo/result/native_frames/native_frames_0.db3'
        connection = sqlite3.connect(path.resolve().as_uri()+'?mode=ro', uri=True)
        topics = dict(connection.execute('select id,name from topics'))
        stamps = set(); count = 0
        for topic_id, _, data in connection.execute('select topic_id,timestamp,data from messages order by timestamp,id'):
            name = topics[topic_id]
            cls = Odometry if name == '/fast_livo/odometry' else PointCloud2
            message = deserialize_message(data, cls)
            ns = message.header.stamp.sec*10**9+message.header.stamp.nanosec
            if cls is Odometry:
                if count >= 20: break
                self.ns = ns; clock = Clock(); clock.clock = message.header.stamp; self.clock.publish(clock)
                self.odom.publish(message); stamps.add(ns)
            else:
                self.cloud.publish(message); count += 1; self.pump(.08, sensors=False)
        connection.close(); self.pump(.2, sensors=False)
        valid = {m.header.stamp.sec*10**9+m.header.stamp.nanosec for m in self.private if m.pose_valid}
        self.assertGreaterEqual(len(valid & stamps), 18)
        self.assertTrue(valid.issubset(stamps))
        self.assertFalse(self.node.get_publishers_info_by_topic('/tf'))

    def test_managed_entry_health_failure_cleans_owned_children(self):
        command = ['bash', str(ROOT/'scripts/run_fast_livo2_backend.sh'), '--execute', '--domain', '187']
        proc = self.spawn(command, 'managed-session')
        self.pump(5.)
        self.assertIsNone(proc.poll(), 'Managed startup failed; see '+str(self.directory))
        self.latest_valid()
        self.assertFalse(self.node.get_publishers_info_by_topic('/tf'))
        status = subprocess.run(['bash', str(ROOT/'scripts/run_fast_livo2_backend.sh'), '--status'],
                                text=True, capture_output=True, check=True)
        self.assertTrue(json.loads(status.stdout)['live'])
        self.extra_odom = self.node.create_publisher(Odometry, '/fast_livo/odometry', 10)
        self.pump(4.)
        self.assertIsNotNone(proc.poll())
        self.assertNotEqual(proc.returncode, 0)
        self.assertFalse(self.node.get_publishers_info_by_topic('/fast_livo/backend/estimate'))

    def test_invalid_messages_and_nonfinite_cloud_fail_closed(self):
        self.backend(); self.pump(1.5); self.latest_valid()
        self.sensors_on = False; self.pump(.3)
        before = self.private[-1].header.stamp
        bad = Odometry(); bad.header.frame_id = 'wrong'; bad.child_frame_id = 'fast_livo_imu'
        stamp(bad, self.ns); bad.pose.pose.orientation.w = 1.; self.odom.publish(bad)
        self.pump(.2)
        self.assertEqual(self.private[-1].header.stamp, before)
        self.assertFalse(self.private[-1].pose_valid)
        self.sensors_on = True; self.pump(.8); self.latest_valid()
        self.sensors_on = False; self.pump(.3)
        m = Odometry(); m.header.frame_id = 'fast_livo_local'; m.child_frame_id = 'fast_livo_imu'
        stamp(m, self.ns-30_000_000); m.pose.pose.orientation.w = 1.
        cloud = PointCloud2(); cloud.header.frame_id = 'fast_livo_imu'; stamp(cloud, self.ns-30_000_000)
        cloud.height = cloud.width = 1; cloud.point_step = cloud.row_step = 16
        cloud.fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1)
                        for i, n in enumerate(('x', 'y', 'z', 'intensity'))]
        cloud.data = struct.pack('<ffff', float('nan'), 0., 0., 0.)
        self.odom.publish(m); self.cloud.publish(cloud); self.pump(.5)
        self.assertEqual(self.private[-1].state, BackendEstimate.ERROR)
        self.assertFalse(self.private[-1].pose_valid)
        self.assertIn('non-finite', self.private[-1].reason)


if __name__ == '__main__':
    unittest.main(verbosity=2)
