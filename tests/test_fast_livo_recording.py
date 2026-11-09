#!/usr/bin/env python3
"""Capture lifecycle tests with synthetic ROS messages; never opens physical devices."""
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
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts/fast_livo2'))
sys.path.insert(0, str(ROOT/'scripts'))
from record_session import (TOPICS, check_lidar_host_address, input_problems,
                            main, quality_report, recorder_command, sync_summary)
from session_manager import atomic_json, identity


def publish(folder):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField, Image, Imu, CameraInfo
    from nav_msgs.msg import Odometry
    rclpy.init()
    node = rclpy.create_node('capture_synthetic_'+str(os.getpid()))
    classes = [PointCloud2, Imu, Image, Image, CameraInfo, Odometry, PointCloud2]
    pubs = {name: node.create_publisher(cls, name, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
            for name, cls in zip(TOPICS, classes)}
    def tick():
        now = node.get_clock().now().nanoseconds
        omit_file = folder/'omit_topics.json'
        omitted = json.loads(omit_file.read_text()) if omit_file.exists() else []
        for (name, pub), cls in zip(pubs.items(), classes):
            if name in omitted:
                continue
            msg = cls()
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(now, 10**9)
            msg.header.frame_id = 'livox_frame'
            if cls is PointCloud2:
                backend_cloud = name == '/fast_livo/deskewed_imu'
                msg.height, msg.width, msg.point_step, msg.row_step = 1, 1, 24, 24
                for i, name in enumerate(('x', 'y', 'z')):
                    msg.fields.append(PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1))
                msg.fields.append(PointField(name='timestamp', offset=16, datatype=PointField.FLOAT64, count=1))
                msg.data = struct.pack('<fff4xd', 1., 2., 3., float(now+1_000_000))
                if backend_cloud:
                    msg.header.frame_id = 'fast_livo_imu'
                    msg.point_step = msg.row_step = 16
                    msg.fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                                  for i,n in enumerate(('x','y','z','intensity'))]
                    msg.data = struct.pack('<ffff', 1., 2., 3., 1.)
            elif cls is Image:
                msg.height, msg.width, msg.encoding, msg.step = 1, 1, 'rgb8', 3
                msg.data = b'\x00\x80\xff'
            elif cls is Imu:
                msg.linear_acceleration.z = 9.81
            elif cls is Odometry:
                msg.header.frame_id = 'fast_livo_local'
                msg.child_frame_id = 'fast_livo_imu'
                msg.pose.pose.orientation.w = 1.
            pub.publish(msg)
    node.create_timer(.05, tick)
    (folder/'publisher_ready').touch()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()


def wait_until(predicate, timeout=30):
    deadline = time.monotonic()+timeout
    while time.monotonic()<deadline:
        try:
            value = predicate()
            if value:return value
        except (FileNotFoundError, KeyError, json.JSONDecodeError):pass
        time.sleep(.1)
    raise AssertionError('timed out waiting for synthetic capture')


def publish_wheel(folder):
    import rclpy
    from nav_msgs.msg import Odometry
    rclpy.init();node=rclpy.create_node('capture_synthetic_wheel_'+str(os.getpid()))
    pub=node.create_publisher(Odometry,'/wheel/odom',10)
    def tick():
        m=Odometry();m.header.frame_id='odom';m.child_frame_id='base_link'
        m.header.stamp=node.get_clock().now().to_msg();m.pose.pose.orientation.w=1.;pub.publish(m)
    node.create_timer(.02,tick);(folder/'wheel_publisher_ready').touch()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()


class CaptureUnit(unittest.TestCase):
    def test_defaults_execute_owned_capture_with_fast_profile_before_device_access(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)/'capture'
            with patch.dict(os.environ, RANGER_RUNTIME_CONFIG=str(Path(tmp)/'unused.yaml')), \
                 patch.object(sys, 'argv', ['record_session.py', '--output', str(output), '--visualization', 'none']), \
                 patch('record_session.read_session', return_value=(None, None)), \
                 patch('record_session.check_sensor_processes', side_effect=RuntimeError('mock device occupancy')) as probe, \
                 patch('record_session.run_guarded') as start:
                with self.assertRaisesRegex(RuntimeError, 'mock device occupancy'):
                    main()
                self.assertEqual(Path(os.environ['RANGER_RUNTIME_CONFIG']), ROOT/'config/profiles/fast_livo2_frontend.yaml')
                probe.assert_called_once_with()
                start.assert_not_called()
            self.assertFalse(output.exists())

    def test_default_refuses_existing_preview_without_implicit_attach(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)/'capture'
            with patch.dict(os.environ), \
                 patch.object(sys, 'argv', ['record_session.py', '--output', str(output), '--visualization', 'none']), \
                 patch('record_session.read_session', side_effect=[(Path(tmp)/'preview', {'live': True}), (None, None)]), \
                 patch('record_session.session_live', side_effect=[True, False]), \
                 patch('record_session.check_sensor_processes') as probe, \
                 patch('record_session.run_guarded') as start:
                with self.assertRaisesRegex(RuntimeError, 'preview already active; use --attach'):
                    main()
                probe.assert_not_called()
                start.assert_not_called()
            self.assertFalse(output.exists())

    def test_dry_run_accepts_explicit_profile_override_without_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            profile = Path(tmp)/'custom.yaml'
            profile.write_text('extends: '+str(ROOT/'config/runtime.yaml')+'\nmapping:\n  fast_livo2:\n    ros_domain_id: 191\n')
            output = Path(tmp)/'capture'
            env = dict(os.environ, RANGER_CAPTURE_SESSION_ROOT=str(Path(tmp)/'sessions'))
            p = subprocess.run(['bash', str(ROOT/'scripts/record_fast_livo2.sh'), '--dry-run',
                                '--runtime-config', str(profile), '--output', str(output)],
                               env=env, check=True, capture_output=True, text=True)
            plan = json.loads(p.stdout)
            self.assertEqual(plan['action'], 'DRY-RUN')
            self.assertEqual(plan['mode'], 'start-sensors')
            self.assertEqual(plan['runtime_config'], str(profile))
            self.assertEqual(plan['domain'], 191)
            self.assertFalse(output.exists())
            self.assertFalse((Path(tmp)/'sessions').exists())

    def test_lidar_bind_address_missing_is_rejected_without_device_access(self):
        interfaces = [{'addr_info': [{'family': 'inet', 'local': '192.168.3.18'}]}]
        with patch('record_session.subprocess.check_output', return_value=json.dumps(interfaces)) as read:
            with self.assertRaisesRegex(RuntimeError, '192.168.1.50.*192.168.3.18.*未启动驱动或录包'):
                check_lidar_host_address({'FAST_LIVO_LIDAR_CONFIG': str(ROOT/'config/fast_livo2/mid360s_live.json')})
            read.assert_called_once_with(['ip', '-j', '-4', 'address', 'show'], text=True)

    def test_present_bind_address_does_not_claim_sensor_is_ready(self):
        interfaces = [{'addr_info': [{'family': 'inet', 'local': '192.168.1.50'}]}]
        with patch('record_session.subprocess.check_output', return_value=json.dumps(interfaces)):
            check_lidar_host_address({'FAST_LIVO_LIDAR_CONFIG': str(ROOT/'config/fast_livo2/mid360s_live.json')})
        missing = input_problems(TOPICS, {}, {})
        self.assertEqual(len(missing), len(TOPICS))
        self.assertTrue(all('无发布者' in problem for problem in missing))

    def test_input_diagnostics_distinguish_discovery_from_live_data(self):
        topics = {'/missing': 'T', '/duplicate': 'T', '/type': 'T', '/qos': 'T', '/silent': 'T', '/healthy': 'T'}
        good = {'type': 'T', 'reliability': 'RELIABLE'}
        graph = {'/duplicate': [good, good], '/type': [dict(good, type='Wrong')],
                 '/qos': [dict(good, reliability='BEST_EFFORT')], '/silent': [good], '/healthy': [good]}
        self.assertEqual(input_problems(topics, graph, {'/healthy': 1}),
                         ['/missing 无发布者', '/duplicate 存在多个发布者', '/type 消息类型不匹配',
                          '/qos QoS 不是 RELIABLE', '/silent 有发布者但未收到数据'])

    def missing_stage(self, stage):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)/'capture'
            env = dict(os.environ, RANGER_CAPTURE_SESSION_ROOT=str(Path(tmp)/'sessions'), PYTHONDONTWRITEBYTECODE='1')
            result = subprocess.run([sys.executable, __file__, '--fake-missing', stage, '--execute',
                                     '--start-sensors', '--no-wheel', '--visualization', 'none', '--output', str(output)],
                                    env=env, text=True, capture_output=True, timeout=12)
            self.assertEqual(result.returncode, 2, result.stdout+result.stderr)
            self.assertIn('尚未录制', result.stdout)
            self.assertIn('CAPTURE WAITING', result.stdout)
            self.assertIn('SESSION ERROR: RuntimeError: startup readiness timeout', result.stderr)
            self.assertIn(stage+'(STARTING', result.stderr)
            self.assertIn('CAPTURE ERROR', result.stderr)
            self.assertNotIn('CAPTURE RECORDING', result.stdout)
            self.assertFalse((output/'raw').exists())
            self.assertFalse((output/'CAPTURE_COMPLETE').exists())
            state = json.loads((output/'result.json').read_text())['session_state']
            self.assertEqual(state['owned'], [])
            self.assertTrue(state['guardian_cleanup_complete'])

    def test_missing_lidar_reports_error_and_never_records(self):
        self.missing_stage('lidar')

    def test_missing_camera_reports_error_and_never_records(self):
        self.missing_stage('camera')

    def test_missing_frontend_reports_error_and_never_records(self):
        self.missing_stage('frontend')

    def test_auto_visualization_for_remote_and_desktop(self):
        with tempfile.TemporaryDirectory() as tmp:
            for display, expected in [('', 'foxglove'), (':0', 'rviz')]:
                env = dict(os.environ, DISPLAY=display, RANGER_CAPTURE_SESSION_ROOT=tmp,
                           PYTHONDONTWRITEBYTECODE='1')
                p = subprocess.run(['bash', str(ROOT/'scripts/record_fast_livo2.sh'), '--dry-run'],
                                   env=env, check=True, capture_output=True, text=True)
                plan = json.loads(p.stdout)
                self.assertEqual(plan['requested_visualization'], 'auto')
                self.assertEqual(plan['visualization'], expected)
                names = {c['name'] for c in plan['components']}
                self.assertIn(expected, names)
                self.assertNotIn('rviz' if expected == 'foxglove' else 'foxglove', names)
                self.assertFalse((Path(tmp)/'fast_livo_record').exists())

    def test_explicit_rviz_without_display_rejected_before_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)/'capture'
            env = dict(os.environ, DISPLAY='', RANGER_CAPTURE_SESSION_ROOT=tmp,
                       PYTHONDONTWRITEBYTECODE='1')
            p = subprocess.run([sys.executable, __file__, '--fake-owned',
                               '--no-wheel', '--visualization', 'rviz', '--output', str(output)],
                              env=env, capture_output=True, text=True)
            self.assertEqual(p.returncode, 2)
            self.assertIn('RViz requires DISPLAY', p.stderr)
            self.assertNotIn('Traceback', p.stderr)
            self.assertFalse(output.exists())

    def test_plan_no_motion_and_qos_and_no_sim_clock(self):
        command = recorder_command(Path('/tmp/new capture'), TOPICS)
        self.assertIn('/front/image_sync', command)
        self.assertIn('/front/image', command)
        self.assertNotIn('--use-sim-time', command)
        self.assertNotIn('/cmd_vel', command)
        p = subprocess.run(['bash', str(ROOT/'scripts/record_fast_livo2.sh'), '--dry-run', '--visualization', 'both'],
                           check=True, capture_output=True, text=True)
        plan = json.loads(p.stdout)
        self.assertEqual(plan['action'], 'DRY-RUN')
        self.assertEqual(plan['domain'], 175)
        self.assertEqual(Path(plan['runtime_config']), ROOT/'config/profiles/fast_livo2_frontend.yaml')
        self.assertEqual(Path(plan['frontend_config']), ROOT/'config/fast_livo2/ranger_mid360s_hik_trial.yaml')
        names = [c['name'] for c in plan['components']]
        self.assertIn('rviz', names)
        self.assertIn('foxglove', names)
        self.assertFalse({'base', 'navigation', 'bridge'} & set(names))
        self.assertIn('wheel',names)
        self.assertIn('/wheel/odom',plan['topics'])
        wheel=next(c['argv'] for c in plan['components'] if c['name']=='wheel')
        self.assertIn('receive_only:=true',wheel)
        writer = next(c['argv'] for c in plan['components'] if c['name']=='writer')
        self.assertIn('lidar_reliability:=reliable', writer)
        self.assertIn('lidar_queue_depth:=5', writer)

    def test_validation_and_sync_evidence(self):
        report = {'topics': {t: {'type': kind, 'count': 10, 'nonincreasing': 0} for t, kind in TOPICS.items()},
                  'point_times': {'total': 10}}
        self.assertEqual(quality_report(report, TOPICS)['data_checks'], 'PASS')
        report['topics']['/livox/lidar']['nonincreasing'] = 1
        self.assertEqual(quality_report(report, TOPICS)['data_checks'], 'FAIL')
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            (folder/'camera_sync.csv').write_text('frame_id,status\n1,OK\n2,BACKWARD\n4,OK\n')
            summary = sync_summary(folder)
            self.assertEqual(summary['status_counts']['BACKWARD'], 1)
            self.assertEqual(summary['hardware_frame_id_missing'], 1)
            self.assertFalse(summary['physical_sync_proven'])


class CaptureROS(unittest.TestCase):
    def setUp(self):
        evidence = os.environ.get('EVIDENCE_DIR')
        self.folder = Path(tempfile.mkdtemp(prefix=self._testMethodName+'-', dir=evidence))
        self.processes = []
        self.logs = []
        self.env = dict(os.environ, ROS_DOMAIN_ID='189', ROS_LOCALHOST_ONLY='1', PYTHONDONTWRITEBYTECODE='1',
                        RANGER_CAPTURE_SESSION_ROOT=str(self.folder/'sessions'))
        self.profile = self.folder/'runtime.yaml'
        self.profile.write_text('extends: '+str(ROOT/'config/runtime.yaml')+'\nmapping:\n  fast_livo2:\n    ros_domain_id: 189\n')
        assets = self.folder/'assets'
        assets.mkdir()
        atomic_json(assets/'manifest.json', {'domain': 189, 'synthetic': True})
        (assets/'camera_sync.csv').write_text('frame_id,status\n1,OK\n2,OK\n')
        self.source = self.start([sys.executable, __file__, '--publish', str(assets)], 'publisher')
        wait_until(lambda:(assets/'publisher_ready').exists())
        run = self.folder/'sessions/fast_livo_live/run-synthetic'
        run.mkdir(parents=True)
        def argv(name):
            if name == 'writer':return ['synthetic', 'timeshare_path:='+str(assets/'timeshare')]
            if name == 'lidar':return ['synthetic', 'user_config_path:='+str(ROOT/'config/fast_livo2/mid360s_live.json')]
            filename = 'camera_live.yaml' if name == 'camera' else 'ranger_mid360s_hik_trial.yaml'
            return ['synthetic', '--params-file', str(ROOT/'config/fast_livo2'/filename)]
        atomic_json(run/'state.json', {'guardian': identity(self.source.pid), 'owner': None, 'owned': [],
            'state': 'RUNNING', 'synthetic': True, 'components': {n:{'argv': argv(n)} for n in ('writer','lidar','camera','frontend')}})
        atomic_json(run.parent/'current.json', {'run_directory': str(run)})

    def start(self, argv, name):
        log = (self.folder/(name+'.log')).open('wb')
        self.logs.append(log)
        p = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, env=self.env, start_new_session=True)
        self.processes.append(p)
        return p

    def record(self, seconds, wheel=False):
        output = self.folder/'capture'
        p = self.start(['bash', str(ROOT/'scripts/record_fast_livo2.sh'), '--execute', '--attach',
                        *([] if wheel else ['--no-wheel']), '--seconds', str(seconds), '--output', str(output), '--runtime-config', str(self.profile)], 'entry')
        return p, output

    def ready(self, p, output):
        def check():
            if p.poll() is not None:raise AssertionError((self.folder/'entry.log').read_text())
            return json.loads((output/'recording.json').read_text())['state'] == 'RECORDING'
        wait_until(check)

    def tearDown(self):
        # Scoped session root; no physical or pre-existing session can be touched.
        subprocess.run(['bash', str(ROOT/'scripts/record_fast_livo2.sh'), '--stop'], env=self.env,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=70)
        for p in reversed(self.processes):
            if p.poll() is None:p.send_signal(signal.SIGINT)
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:p.kill();p.wait(timeout=3)
        for log in self.logs:log.close()

    def test_finite_capture_archives_and_preserves_external_preview(self):
        p, output = self.record(2)
        self.ready(p, output)
        self.assertEqual(p.wait(timeout=45), 0, (self.folder/'entry.log').read_text())
        result = json.loads((output/'result.json').read_text())
        self.assertTrue(result['capture_complete'])
        self.assertEqual(result['quality']['data_checks'], 'PASS')
        self.assertTrue((output/'CAPTURE_COMPLETE').exists())
        self.assertTrue((output/'session_logs/sync/camera_sync.csv').exists())
        subscribers = json.loads((output/'subscriptions_before.json').read_text())
        self.assertTrue(any(e['node']=='ranger_fast_livo_record_preflight' and
                            e['reliability']=='RELIABLE' for e in subscribers['/livox/lidar']))
        sources = json.loads((output/'manifest.json').read_text())['source_files']
        for suffix in ('timeshare_writer/src/timeshare_writer_node.cpp',
                       'timeshare_writer/lib/timeshare_writer/timeshare_writer_node',
                       'fast_livo/lib/fast_livo/fastlivo_mapping'):
            entry = next(e for e in sources if e['source'].endswith(suffix))
            self.assertEqual((output/entry['file']).read_bytes(), Path(entry['source']).read_bytes())
        self.assertFalse(result['session_state']['owned'])
        self.assertIsNone(self.source.poll())
        overwrite = subprocess.run(['bash', str(ROOT/'scripts/record_fast_livo2.sh'), '--execute', '--attach',
             '--output', str(output), '--runtime-config', str(self.profile)], env=self.env, capture_output=True)
        self.assertNotEqual(overwrite.returncode, 0)
        self.assertTrue((output/'CAPTURE_COMPLETE').exists())

    def test_wheel_auto_reuses_external_publisher_without_stopping_it(self):
        wheel=self.start([sys.executable,__file__,'--publish-wheel',str(self.folder)],'wheel_publisher')
        wait_until(lambda:(self.folder/'wheel_publisher_ready').exists())
        p,output=self.record(2,wheel=True)
        self.ready(p,output)
        self.assertEqual(p.wait(timeout=45),0,(self.folder/'entry.log').read_text())
        manifest=json.loads((output/'manifest.json').read_text())
        self.assertEqual(manifest['wheel']['source'],'external')
        self.assertIn('/wheel/odom',manifest['topics'])
        audit=json.loads((output/'audit.json').read_text())
        self.assertGreater(audit['topics']['/wheel/odom']['count'],20)
        self.assertIsNone(wheel.poll());self.assertIsNone(self.source.poll())
        self.assertNotIn('wheel',json.loads((output/'result.json').read_text())['session_state']['components'])

    def test_ctrl_c_finalizes_and_keeps_external_preview(self):
        p, output = self.record(60)
        self.ready(p, output)
        time.sleep(.6)
        p.send_signal(signal.SIGINT)
        self.assertEqual(p.wait(timeout=45), 0, (self.folder/'entry.log').read_text())
        self.assertTrue(json.loads((output/'result.json').read_text())['capture_complete'])
        self.assertIsNone(self.source.poll())

    def test_duplicate_publisher_preserves_failure_without_success_marker(self):
        p, output = self.record(60)
        self.ready(p, output)
        self.start([sys.executable, __file__, '--publish', str(self.folder/'assets')], 'duplicate')
        self.assertEqual(p.wait(timeout=45), 2)
        self.assertFalse((output/'CAPTURE_COMPLETE').exists())
        result = json.loads((output/'result.json').read_text())
        self.assertFalse(result['capture_complete'])
        self.assertIn('ambiguous', result['error']) if 'ambiguous' in result.get('error','') else self.assertIn('session or recorder failed', result['error'])
        self.assertTrue((output/'record.log').exists())
        self.assertIsNone(self.source.poll())

    def test_disk_reserve_rejects_before_recording(self):
        output = self.folder/'capture'
        p = self.start(['bash', str(ROOT/'scripts/record_fast_livo2.sh'), '--execute', '--attach',
             '--min-free-gb', '1000000', '--output', str(output), '--runtime-config', str(self.profile)], 'entry')
        self.assertNotEqual(p.wait(timeout=10), 0)
        self.assertFalse(output.exists())
        self.assertIsNone(self.source.poll())

    def owned_capture(self,wheel=False):
        self.source.send_signal(signal.SIGINT)
        self.source.wait(timeout=5)
        output = self.folder/'owned_capture'
        self.env['DISPLAY'] = ''
        p = self.start([sys.executable, __file__, '--fake-owned-wheel' if wheel else '--fake-owned',
            *([] if wheel else ['--no-wheel']),
            '--seconds', '2', '--output', str(output),
            '--runtime-config', str(self.profile)], 'owned_entry')
        self.ready(p, output)
        self.assertEqual(p.wait(timeout=45), 0, (self.folder/'owned_entry.log').read_text())
        result = json.loads((output/'result.json').read_text())
        self.assertTrue(result['capture_complete'])
        self.assertEqual(json.loads((output/'manifest.json').read_text())['visualization'], 'foxglove')
        self.assertTrue(result['session_state']['guardian_cleanup_complete'])
        self.assertEqual(result['session_state']['owned'], [])
        from session_manager import same_process
        self.assertFalse(same_process(result['session_state']['components']['frontend']['process']))
        if wheel:
            self.assertFalse(same_process(result['session_state']['components']['wheel']['process']))
            self.assertGreater(json.loads((output/'audit.json').read_text())['topics']['/wheel/odom']['count'],20)

    def test_owned_synthetic_session_is_cleaned_after_finite_capture(self):
        self.owned_capture()

    def test_wheel_auto_owned_feedback_is_stopped_after_recording(self):
        self.owned_capture(wheel=True)

    def test_recorder_crash_preserves_failure_and_external_session(self):
        p, output = self.record(60)
        self.ready(p, output)
        recorder = json.loads((output/'recorder_process.json').read_text())
        os.kill(recorder['pid'], signal.SIGKILL)
        self.assertEqual(p.wait(timeout=45), 2)
        self.assertFalse((output/'CAPTURE_COMPLETE').exists())
        self.assertFalse(json.loads((output/'result.json').read_text())['capture_complete'])
        self.assertIsNone(self.source.poll())

    def test_camera_publisher_without_data_is_reported_before_recording(self):
        atomic_json(self.folder/'assets/omit_topics.json', ['/front/image_sync'])
        p, output = self.record(2)
        self.assertEqual(p.wait(timeout=35), 2)
        result = json.loads((output/'result.json').read_text())
        self.assertFalse(result['capture_complete'])
        self.assertIn('/front/image_sync 有发布者但未收到数据', result['recorder_error'])
        self.assertIn('/front/image_sync 有发布者但未收到数据', (self.folder/'entry.log').read_text())
        self.assertFalse((output/'raw').exists())
        self.assertFalse((output/'CAPTURE_COMPLETE').exists())
        self.assertIsNone(self.source.poll())

    def test_frontend_stream_loss_reports_topic_and_stops(self):
        p, output = self.record(60)
        self.ready(p, output)
        atomic_json(self.folder/'assets/omit_topics.json', ['/fast_livo/odometry'])
        self.assertEqual(p.wait(timeout=20), 2)
        result = json.loads((output/'result.json').read_text())
        self.assertFalse(result['capture_complete'])
        self.assertIn('stalled for over 2 seconds: /fast_livo/odometry', result['recorder_error'])
        self.assertIn('stalled for over 2 seconds: /fast_livo/odometry', (self.folder/'entry.log').read_text())
        self.assertFalse((output/'CAPTURE_COMPLETE').exists())
        self.assertEqual(result['session_state']['owned'], [])
        self.assertIsNone(self.source.poll())


if __name__ == '__main__':
    if len(sys.argv)>1 and sys.argv[1] == '--publish':publish(Path(sys.argv[2]))
    elif len(sys.argv)>1 and sys.argv[1] == '--publish-wheel':publish_wheel(Path(sys.argv[2]))
    elif len(sys.argv)>1 and sys.argv[1] == '--fake-missing':
        stage = sys.argv[2]
        del sys.argv[1:3]
        import record_session
        from session_manager import Component, run_guarded
        sleep = [sys.executable, '-c', 'import time; time.sleep(30)']
        def missing_specs(cfg, visualization='none', port=8765):
            return [Component('probe', sleep),
                    Component('lidar', sleep, depends=('probe',), ready_key='lidar'),
                    Component('camera', sleep, depends=('lidar',), ready_key='camera'),
                    Component('frontend', sleep, depends=('camera',), ready_key='frontend')]
        original_health = record_session.CaptureHealth.__call__
        def missing_health(self):
            original_health(self)
            return {name: name != stage for name in ('probe_live', 'lidar', 'camera', 'frontend')}
        def short_guard(*args, **kwargs):
            kwargs.update(startup_timeout=.6, cleanup_grace=.2)
            return run_guarded(*args, **kwargs)
        record_session.specs = missing_specs
        record_session.CaptureHealth.__call__ = missing_health
        record_session.run_guarded = short_guard
        record_session.check_sensor_processes = lambda: None
        record_session.check_lidar_host_address = lambda cfg: None
        raise SystemExit(record_session.main())
    elif len(sys.argv)>1 and sys.argv[1] in ('--fake-owned','--fake-owned-wheel'):
        owned_wheel=sys.argv[1]=='--fake-owned-wheel'
        sys.argv.pop(1)
        import record_session
        from session_manager import Component
        def fake_specs(cfg, visualization='none', port=8765):
            return [Component('frontend', [sys.executable, __file__, '--publish',
                str(Path(os.environ['RANGER_LIVO_TIMESHARE']).parent)], resource='fast_livo_frontend')]
        record_session.specs = fake_specs
        record_session.check_sensor_processes = lambda: None
        record_session.check_lidar_host_address = lambda cfg: None
        if owned_wheel:
            record_session.wheel_capture.prepare=lambda:{'source':'owned_receive_only',
                'synthetic':True,'driver_command':[sys.executable,__file__,'--publish-wheel',
                                                 str(Path(os.environ['RANGER_LIVO_TIMESHARE']).parent)]}
        raise SystemExit(record_session.main())
    else:unittest.main(verbosity=2)
