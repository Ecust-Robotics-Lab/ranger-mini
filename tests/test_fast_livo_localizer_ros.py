#!/usr/bin/env python3
"""C1 isolated ROS acceptance with known 6DoF geometry; never connects to devices."""
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
import uuid

ROOT = Path(__file__).resolve().parents[1]
os.environ.update(ROS_DOMAIN_ID='196', ROS_LOCALHOST_ONLY='1', PYTHONDONTWRITEBYTECODE='1', OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1')
sys.path.insert(0, str(ROOT/'ws_nav/src/ranger_localization_bridge'))
sys.path.insert(0, str(ROOT/'scripts/fast_livo2/localizer'))
from compatibility import declaration
from ranger_localization_bridge.geometry import SE3, rpy_to_quat, quat_rotate, se3_from_tf_msg
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray
from ranger_localization_msgs.msg import LocalizationEstimate
from ranger_fast_livo_localizer.msg import LocalizerEstimate
from ranger_fast_livo_localizer.srv import LoadMap, Initialize
import tf2_ros

EVIDENCE = ROOT/'docs/fast_livo2_evidence/stage_c1_20261028'
EXECUTABLE = ROOT/'ws_fast_livo/backend_install/ranger_fast_livo_localizer/lib/ranger_fast_livo_localizer/localizer_node'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def pose_json(p):
    return {'xyz': list(p.t), 'xyzw': list(p.q)}


def fill_pose(msg, p):
    msg.position.x, msg.position.y, msg.position.z = p.t
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = p.q


def transform(points, p):
    return np.asarray([quat_rotate(p.q, tuple(v)) for v in points])+np.asarray(p.t)


def write_fixture(path, world, X):
    path.mkdir()
    shard = path/'keyframes/0'
    shard.mkdir(parents=True)
    points = np.column_stack((transform(world, X.inverse()), np.arange(len(world)))).astype('<f4')
    header = (f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
              f'WIDTH {len(world)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(world)}\nDATA binary\n').encode()
    for name in ['full.pcd', 'match.pcd']:
        (shard/name).write_bytes(header+points.tobytes())
    (shard/'scan.cdr').write_bytes(b'C1 synthetic fixture; checksummed opaque CDR not read by C1')
    (path/'map.pcd').write_bytes(header+np.column_stack((world, np.arange(len(world)))).astype('<f4').tobytes())
    (path/'original.tum').write_text('1.000000000 0 0 0 0 0 0 1\n')
    (path/'optimized.tum').write_text('1.000000000 '+' '.join(str(v) for v in [*X.t, *X.q])+'\n')
    files = {str(p.relative_to(path)): sha(p) for p in path.rglob('*') if p.is_file()}
    manifest = {'schema_version': 1, 'kind': 'stage_b_graph_archive', 'map_uuid': uuid.uuid4().hex,
                'session_id': 'synthetic-source', 'graph_version': 1,
                'transform_convention': 'T_A_B maps points in B to A', 'cloud_frame': 'fast_livo_imu',
                'pcd_fields': ['x', 'y', 'z', 'intensity'], 'provenance': {'kind': 'synthetic_known_truth'},
                'keyframes': [{'id': 0, 'scan_id': 0, 'stamp_ns': 10**9, 'directory': 'keyframes/0',
                               'U_local_imu': pose_json(SE3()), 'X_map_imu': pose_json(X), 'points': len(world)}],
                'files': files, 'factors': []}
    (path/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    (path/'COMMITTED').write_text(json.dumps({'manifest_sha256': sha(path/'manifest.json')})+'\n')
    compatibility = path.parent/(path.name+'-compatibility.json')
    compatibility.write_text(json.dumps(declaration(path, [path/'manifest.json']), indent=2)+'\n')
    return compatibility


class C1Ros(unittest.TestCase):
    def setUp(self):
        self.directory = Path(tempfile.mkdtemp(prefix=self._testMethodName+'-', dir=EVIDENCE))
        self.ns = 1000*10**9
        self.start_ns = self.ns
        self.base_imu = SE3((.2, -.1, .4), rpy_to_quat(.1, -.08, .2))
        self.correction = SE3((2., -1., .3), rpy_to_quat(.14, -.12, .4))
        self.frontend_origin = SE3((8., 2., -.7), rpy_to_quat(-.2, .1, -.6))
        rng = np.random.default_rng(42)
        self.points = rng.uniform(-4, 4, (800, 3))
        self.points[::3, 0] = -4
        self.points[1::3, 1] = 3
        self.points[2::3, 2] = -2
        self.archive = self.directory/'map'
        self.compatibility = write_fixture(self.archive, self.points, self.correction*self.base_imu)
        rclpy.init()
        self.node = Node('c1_test_world', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.clock = self.node.create_publisher(Clock, '/clock', 10)
        self.odom = self.node.create_publisher(Odometry, '/fast_livo/odometry', 16)
        self.cloud = self.node.create_publisher(PointCloud2, '/fast_livo/deskewed_imu', 16)
        self.wheel = self.node.create_publisher(Odometry, '/wheel/odom', 16)
        self.private = []
        self.standard = []
        self.ready = False
        self.health = {}
        self.node.create_subscription(LocalizerEstimate, '/fast_livo/localizer/estimate', self.private.append, 100)
        self.node.create_subscription(LocalizationEstimate, '/localization/estimate', self.standard.append, 100)
        self.node.create_subscription(Bool, '/navigation_ready', lambda m: setattr(self, 'ready', m.data), 10)
        self.node.create_subscription(DiagnosticArray, '/localization_health', self.on_health, 10)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self.node)
        self.processes = []

    def on_health(self, msg):
        if msg.status:
            self.health = {v.key: v.value for v in msg.status[0].values}

    def spawn(self, command, name):
        log = (self.directory/(name+'.log')).open('w')
        env = dict(os.environ)
        env['PYTHONPATH'] = str(ROOT/'ws_nav/src/ranger_localization_bridge')+':'+env.get('PYTHONPATH', '')
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        self.processes.append((proc, log))
        return proc

    def pump(self, seconds=.1):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            clock = Clock()
            clock.clock.sec, clock.clock.nanosec = divmod(self.ns, 10**9)
            self.clock.publish(clock)
            rclpy.spin_once(self.node, timeout_sec=.01)

    def wait(self, predicate, timeout=10):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self.pump(.025)
            if predicate():
                return
        self.fail('Timeout; '+str(self.directory)+'; last='+str([(m.state, m.reason) for m in self.private[-3:]]))

    def launch(self, registration='pcl_icp'):
        proc = self.spawn([str(EXECUTABLE), '--ros-args', '--params-file', str(ROOT/'config/fast_livo2/localizer.yaml'),
                           '-p', 'archive_path:='+str(self.archive), '-p', 'compatibility_path:='+str(self.compatibility),
                           '-p', 'registration:='+registration,
                           '-p', 't_base_imu_xyz:=[0.2, -0.1, 0.4]', '-p', 't_base_imu_rpy:=[0.1, -0.08, 0.2]'], 'localizer')
        self.wait(lambda: self.private and self.private[-1].map_uuid and json.loads(self.private[-1].quality_json).get('inputs_ready'))
        self.assertIsNone(proc.poll())
        return proc

    def wheel_pose(self, ns):
        t = (ns-self.start_ns)*1e-9
        return SE3((.4*t, .02*t, .01*t), rpy_to_quat(.01*t, -.005*t, .04*t))

    def sensors(self, ns, nan=False):
        local = self.frontend_origin*self.wheel_pose(ns)*self.base_imu
        global_imu = self.correction*self.wheel_pose(ns)*self.base_imu
        odom = Odometry()
        odom.header.frame_id, odom.child_frame_id = 'fast_livo_local', 'fast_livo_imu'
        odom.header.stamp.sec, odom.header.stamp.nanosec = divmod(ns, 10**9)
        fill_pose(odom.pose.pose, local)
        cloud = PointCloud2()
        cloud.header.frame_id = 'fast_livo_imu'
        cloud.header.stamp.sec, cloud.header.stamp.nanosec = divmod(ns, 10**9)
        cloud.height, cloud.width, cloud.point_step, cloud.row_step = 1, len(self.points), 16, 16*len(self.points)
        cloud.fields = [PointField(name=name, offset=4*k, datatype=PointField.FLOAT32, count=1)
                        for k, name in enumerate(['x', 'y', 'z', 'intensity'])]
        xyz = transform(self.points, global_imu.inverse())
        if nan:
            xyz[0, 0] = float('nan')
        cloud.data = np.column_stack((xyz, np.arange(len(xyz)))).astype('<f4').tobytes()
        cloud.is_dense = True
        return odom, cloud

    def send(self, wheel=True, wheel_jump=0., nan=False, cloud_offset=0):
        self.ns += 100_000_000
        sample_ns = self.ns-30_000_000
        self.pump(.04)
        if wheel:
            for t in [sample_ns-20_000_000, sample_ns+20_000_000]:
                m = Odometry()
                m.header.frame_id, m.child_frame_id = 'odom', 'base_link'
                m.header.stamp.sec, m.header.stamp.nanosec = divmod(t, 10**9)
                wp = self.wheel_pose(t)
                fill_pose(m.pose.pose, SE3((wp.t[0]+wheel_jump, wp.t[1], wp.t[2]), wp.q))
                self.wheel.publish(m)
        self.pump(.04)
        odom, cloud = self.sensors(sample_ns, nan)
        cloud.header.stamp.sec, cloud.header.stamp.nanosec = divmod(sample_ns+cloud_offset, 10**9)
        self.odom.publish(odom)
        self.cloud.publish(cloud)
        if nan or cloud_offset:
            self.pump(.25)
        else:
            self.wait(lambda: self.private and self.private[-1].latest_scan_stamp_ns == sample_ns)
        return self.private[-1]

    def initialize(self, request_id='initial', request=None, reference='base_link', bad=False):
        client = self.node.create_client(Initialize, '/fast_livo/localizer/initialize')
        self.assertTrue(client.wait_for_service(timeout_sec=2))
        if request is None:
            current = self.private[-1]
            request = Initialize.Request()
            request.request_id, request.map_uuid, request.session_id = request_id, current.map_uuid, current.session_id
            request.map_generation, request.reference_body = current.map_generation, reference
            request.initial_pose.header.frame_id = 'map'
            p = self.correction*self.wheel_pose(self.ns+100_000_000)
            if reference == 'fast_livo_imu':
                p = p*self.base_imu
            if bad:
                p = SE3((100., 100., 100.), p.q)
            fill_pose(request.initial_pose.pose.pose, p)
            for k in range(6):
                request.initial_pose.pose.covariance[7*k] = .01
        future = client.call_async(request)
        self.wait(future.done)
        return future.result(), request

    def track(self, request_id='initial'):
        response, request = self.initialize(request_id)
        self.assertTrue(response.accepted, response.reason)
        for _ in range(4):
            result = self.send()
        self.assertTrue(result.pose_valid, result.reason+' '+result.quality_json)
        return result, request

    def load(self, path, compatibility, request_id):
        client = self.node.create_client(LoadMap, '/fast_livo/localizer/load_map')
        self.assertTrue(client.wait_for_service(timeout_sec=2))
        req = LoadMap.Request()
        req.request_id, req.archive_path, req.compatibility_path = request_id, str(path), str(compatibility)
        future = client.call_async(req)
        self.wait(future.done)
        return future.result()

    def tearDown(self):
        for proc, log in reversed(self.processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait(timeout=3)
            log.close()
        observations = {'hardware_started': False, 'process_exit_codes': [p.returncode for p, _ in self.processes],
                        'private_messages': len(self.private), 'valid_unique_samples': len({m.last_confirmed_stamp_ns for m in self.private if m.pose_valid}),
                        'sessions': sorted({m.session_id for m in self.private}), 'bridge_health': self.health,
                        'estimates': [{'state': m.state, 'reason': m.reason, 'sample': m.last_confirmed_stamp_ns,
                                       'latest_scan': m.latest_scan_stamp_ns, 'map': m.map_uuid, 'session': m.session_id,
                                       'request_generation': m.request_generation, 'quality': json.loads(m.quality_json or '{}')}
                                      for m in self.private]}
        (self.directory/'observations.json').write_text(json.dumps(observations, indent=2)+'\n')
        self.node.destroy_node()
        rclpy.shutdown()

    def test_initialization_external_base_and_same_time_bridge_latch(self):
        self.launch()
        for module, name, extra in [('fast_livo_localizer_adapter', 'adapter',
                                     ['-p', 't_base_imu_xyz:=[0.2, -0.1, 0.4]', '-p', 't_base_imu_rpy:=[0.1, -0.08, 0.2]']),
                                    ('bridge_node', 'bridge', ['-p', 'adapter_backend:=external', '-p', 'latch_on_innovation_failure:=true'])]:
            self.spawn([sys.executable, '-m', 'ranger_localization_bridge.'+module, '--ros-args', '-p', 'use_sim_time:=true', *extra], name)
        self.pump(.5)
        result, _ = self.track()
        self.wait(lambda: self.ready)
        transform_result = se3_from_tf_msg(self.buffer.lookup_transform('map', 'odom', rclpy.time.Time()))
        error = self.correction.inverse()*transform_result
        self.assertLess(math.dist(error.t, (0., 0., 0.)), .02)
        self.assertLess(abs(error.yaw()), .01)
        self.assertEqual(len(self.node.get_publishers_info_by_topic('/tf')), 1)
        for _ in range(3):
            self.send(wheel=False)
        self.assertFalse(self.ready)
        self.assertIn('waiting', str(self.health).lower())
        for _ in range(7):
            self.send()
        self.wait(lambda: self.ready)
        result = self.send(wheel_jump=10.)
        self.assertTrue(result.pose_valid)
        self.wait(lambda: self.health.get('innovation_unlocked') == '0')
        self.send()
        self.assertFalse(self.ready)
        self.assertEqual(self.health.get('innovation_unlocked'), '0')
        self.assertTrue(any(m.state == m.VALID for m in self.standard))

    def test_wrong_initial_duplicate_requests_loss_and_reinitialize(self):
        self.launch()
        response, wrong = self.initialize('wrong', bad=True)
        self.assertTrue(response.accepted)
        for _ in range(3):
            self.assertFalse(self.send().pose_valid)
        result, request = self.track('correct')
        generation = result.request_generation
        duplicate, _ = self.initialize(request=request)
        self.assertTrue(duplicate.accepted)
        self.assertEqual(duplicate.request_generation, generation)
        request.initial_pose.pose.pose.position.x += .001
        mismatch, _ = self.initialize(request=request)
        self.assertFalse(mismatch.accepted)
        self.ns += 10**9
        self.pump(.3)
        self.assertEqual(self.private[-1].state, LocalizerEstimate.LOST)
        self.assertEqual(self.private[-1].last_confirmed_stamp_ns, result.last_confirmed_stamp_ns)
        for _ in range(2):
            self.assertFalse(self.send().pose_valid)
        new, _ = self.track('recover')
        self.assertGreater(new.request_generation, generation)

    def test_map_load_failure_retains_archive_then_switch_invalidates_initial(self):
        self.launch()
        result, stale_request = self.track()
        old_uuid = result.map_uuid
        response = self.load(self.directory/'missing', self.compatibility, 'bad-map')
        self.assertTrue(response.accepted)
        self.wait(lambda: self.private[-1].reason.startswith('map_load_failed'))
        self.assertEqual(self.private[-1].map_uuid, old_uuid)
        self.assertFalse(self.private[-1].pose_valid)
        old, _ = self.initialize(request=stale_request)
        self.assertFalse(old.accepted)
        self.track('old-map-recovery')
        new_map = self.directory/'other-map'
        compatibility = write_fixture(new_map, self.points, self.correction*self.base_imu)
        response = self.load(new_map, compatibility, 'new-map')
        self.assertTrue(response.accepted)
        self.wait(lambda: self.private[-1].map_uuid != old_uuid)
        for _ in range(2):
            self.assertFalse(self.send().pose_valid)
        result, _ = self.track('new-map-initial')
        self.assertNotEqual(result.map_uuid, old_uuid)

    def test_clock_reset_and_multiple_publishers_fail_closed(self):
        self.launch()
        result, request = self.track()
        self.ns -= 100*10**9
        self.start_ns -= 100*10**9
        self.pump(.4)
        self.assertNotEqual(self.private[-1].session_id, result.session_id)
        self.assertFalse(self.private[-1].pose_valid)
        old, _ = self.initialize(request=request)
        self.assertFalse(old.accepted)
        self.track('new-epoch')
        extra = self.node.create_publisher(Odometry, '/fast_livo/odometry', 10)
        self.wait(lambda: self.private[-1].reason.startswith('terminal_multiple_input'))
        self.assertEqual(self.private[-1].state, LocalizerEstimate.ERROR)
        self.node.destroy_publisher(extra)
        self.pump(.5)
        self.assertEqual(self.private[-1].state, LocalizerEstimate.ERROR)

    def test_invalid_point_and_one_ns_missing_pair_never_confirm(self):
        self.launch()
        response, _ = self.initialize()
        self.assertTrue(response.accepted)
        self.send(nan=True)
        self.assertFalse(self.private[-1].pose_valid)
        self.send(cloud_offset=1)
        self.pump(.4)
        self.assertFalse(self.private[-1].pose_valid)
        self.assertGreater(json.loads(self.private[-1].quality_json).get('pair_counters', {}).get('unpaired_timeout', 0), 0)
        for _ in range(4):
            result = self.send()
        self.assertTrue(result.pose_valid, result.reason)
        self.assertFalse(self.node.get_publishers_info_by_topic('/tf'))
        self.assertFalse(self.node.get_publishers_info_by_topic('/cmd_vel'))
        self.assertFalse(self.node.get_publishers_info_by_topic('/base/safe_cmd_vel'))

    def test_small_gicp_and_restart_require_fresh_initial(self):
        proc = self.launch('small_gicp')
        result, _ = self.track()
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=5)
        self.pump(.5)
        self.private.clear()
        self.launch('small_gicp')
        self.assertNotEqual(self.private[-1].session_id, result.session_id)
        for _ in range(2):
            self.assertFalse(self.send().pose_valid)
        self.track('restart-initial')

    def test_superseded_matching_and_repeated_map_request(self):
        self.launch()
        initial, old_request = self.initialize('superseded')
        self.assertTrue(initial.accepted)
        self.ns += 100_000_000
        self.pump(.03)
        odom, cloud = self.sensors(self.ns-30_000_000)
        self.odom.publish(odom)
        self.cloud.publish(cloud)
        self.pump(.025)
        response = self.load(self.archive, self.compatibility, 'reload-inflight')
        self.assertTrue(response.accepted)
        self.wait(lambda: self.private[-1].map_generation == response.map_generation and self.private[-1].reason == 'map_loaded_waiting_initial_pose')
        self.pump(.3)
        for message in self.private:
            if message.map_generation == response.map_generation:
                self.assertFalse(message.pose_valid)
        duplicate = self.load(self.archive, self.compatibility, 'reload-inflight')
        self.assertTrue(duplicate.accepted)
        self.assertEqual(duplicate.map_generation, response.map_generation)
        mismatch = self.load(self.directory/'different', self.compatibility, 'reload-inflight')
        self.assertFalse(mismatch.accepted)
        obsolete, _ = self.initialize(request=old_request)
        self.assertFalse(obsolete.accepted)
        self.track('after-cancellation')

    def test_adapter_rejects_retired_sessions_and_future_stamp(self):
        pub = self.node.create_publisher(LocalizerEstimate, '/fast_livo/localizer/estimate', 10)
        self.spawn([sys.executable, '-m', 'ranger_localization_bridge.fast_livo_localizer_adapter',
                    '--ros-args', '-p', 'use_sim_time:=true'], 'adapter')
        self.pump(.6)
        message = LocalizerEstimate()
        message.header.frame_id, message.imu_frame = 'map', 'fast_livo_imu'
        message.map_uuid, message.manifest_sha256 = 'synthetic-map', 'a'*64
        message.graph_version = message.map_generation = message.request_generation = 1
        message.initialization_request_id = 'initial'
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(self.ns-10_000_000, 10**9)
        message.last_confirmed_stamp_ns = self.ns-10_000_000
        message.state, message.pose_valid = message.VALID, True
        fill_pose(message.pose, self.correction*self.base_imu)

        def publish(session):
            message.session_id = session
            for _ in range(6):
                pub.publish(message)
                self.pump(.025)

        publish('first')
        self.wait(lambda: self.standard and self.standard[-1].state == LocalizationEstimate.VALID)
        publish('second')
        self.assertEqual(self.standard[-1].state, LocalizationEstimate.VALID)
        publish('first')
        self.assertEqual(self.standard[-1].state, LocalizationEstimate.ERROR)
        self.assertIn('retired', self.standard[-1].reason)
        message.header.stamp.sec += 1
        message.last_confirmed_stamp_ns += 10**9
        publish('second')
        self.assertEqual(self.standard[-1].state, LocalizationEstimate.ERROR)
        message.header.stamp.sec -= 1
        message.last_confirmed_stamp_ns -= 10**9
        publish('second')
        self.assertEqual(self.standard[-1].state, LocalizationEstimate.VALID)

    def test_managed_process_failure_cleans_only_owned_children(self):
        sys.path.insert(0, str(ROOT/'scripts'))
        from session_manager import read_session, session_live, same_process, notify
        directory = ROOT/'.runtime/sessions/fast_livo_localizer'
        self.assertFalse(session_live(read_session(directory)[1]), 'existing managed C1 session must be preserved')
        process = self.spawn(['bash', str(ROOT/'scripts/run_fast_livo2_localizer.sh'), '--execute', '--domain', '196',
                              '--archive', str(self.archive), '--compatibility', str(self.compatibility)], 'managed')
        self.wait(lambda: (read_session(directory)[1] or {}).get('components', {}).get('localizer', {}).get('state') == 'READY', timeout=20)
        state = read_session(directory)[1]
        child = state['components']['localizer']['process']
        self.assertTrue(same_process(child))
        notify(child, signal.SIGKILL)
        self.wait(lambda: process.poll() is not None, timeout=20)
        self.assertNotEqual(process.returncode, 0)
        final = read_session(directory)[1]
        self.assertTrue(final.get('guardian_cleanup_complete'))
        self.assertFalse(final.get('owned'))
        self.assertFalse(session_live(final))
        (self.directory/'managed_final.json').write_text(json.dumps(final, indent=2)+'\n')


if __name__ == '__main__':
    unittest.main(verbosity=2)
