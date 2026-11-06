#!/usr/bin/env python3
"""C2 real coarse/fine search, cancellation and recovery in localhost ROS only."""
import json
from pathlib import Path
import sys
import unittest
import test_fast_livo_localizer_ros as c1
from ranger_fast_livo_localizer.srv import Relocalize

c1.EVIDENCE = c1.ROOT/'docs/fast_livo2_evidence/stage_c2_20261028'


class C2Ros(unittest.TestCase):
    # Reuse the established simulator and owned-process cleanup, without
    # inheriting/rerunning the C1 test cases under a different class name.
    setUp = c1.C1Ros.setUp
    tearDown = c1.C1Ros.tearDown
    spawn = c1.C1Ros.spawn
    pump = c1.C1Ros.pump
    wait = c1.C1Ros.wait
    send = c1.C1Ros.send
    sensors = c1.C1Ros.sensors
    wheel_pose = c1.C1Ros.wheel_pose
    on_health = c1.C1Ros.on_health
    load = c1.C1Ros.load

    def launch(self):
        proc = self.spawn([str(c1.EXECUTABLE), '--ros-args', '--params-file', str(c1.ROOT/'config/fast_livo2/localizer.yaml'),
                           '--params-file', str(c1.ROOT/'config/fast_livo2/relocalizer.yaml'),
                           '-p', 'archive_path:='+str(self.archive), '-p', 'compatibility_path:='+str(self.compatibility),
                           '-p', 't_base_imu_xyz:=[0.2,-0.1,0.4]', '-p', 't_base_imu_rpy:=[0.1,-0.08,0.2]'], 'global_localizer')
        self.wait(lambda: self.private and self.private[-1].map_uuid and json.loads(self.private[-1].quality_json).get('inputs_ready'))
        self.assertIsNone(proc.poll())
        return proc

    def search_request(self, enable, request_id):
        client = self.node.create_client(Relocalize, '/fast_livo/localizer/relocalize')
        self.assertTrue(client.wait_for_service(timeout_sec=2))
        current = self.private[-1]
        request = Relocalize.Request(request_id=request_id, enable=enable, map_uuid=current.map_uuid,
                                     map_generation=current.map_generation, session_id=current.session_id)
        future = client.call_async(request)
        self.wait(future.done)
        self.assertTrue(future.result().accepted, future.result().reason)
        return future.result()

    def search_to_valid(self):
        for _ in range(20):
            message = self.send(wheel=False)
            if message.pose_valid:
                return message
        self.fail('No confirmed C2 -> C1 tracking: '+message.reason+' '+message.quality_json)

    def test_unknown_large_correction_and_full_pose_then_tracking(self):
        self.frontend_origin = c1.SE3((100., -50., 20.), c1.rpy_to_quat(.7, -.6, 2.3))
        self.launch()
        first = self.send(wheel=False)
        self.assertFalse(first.pose_valid)
        message = self.search_to_valid()
        self.assertTrue(message.initialization_request_id.startswith('global-'))
        self.assertEqual(message.header.stamp.sec*10**9+message.header.stamp.nanosec, message.latest_scan_stamp_ns)
        actual = message.pose.position
        expected = self.correction*self.wheel_pose(message.latest_scan_stamp_ns)*self.base_imu
        self.assertLess(sum((x-y)**2 for x, y in zip((actual.x, actual.y, actual.z), expected.t))**.5, .06)
        for _ in range(5):
            self.assertTrue(self.send(wheel=False).pose_valid)
        for topic in ['/tf', '/tf_static', '/cmd_vel', '/base/safe_cmd_vel']:
            self.assertFalse(self.node.get_publishers_info_by_topic(topic), topic)

    def test_cancel_revokes_old_work_restart_and_loss_recovery(self):
        self.launch()
        self.send(wheel=False)
        response = self.search_request(False, 'cancel-1')
        duplicate = self.search_request(False, 'cancel-1')
        self.assertEqual(response.request_generation, duplicate.request_generation)
        for _ in range(5):
            self.assertFalse(self.send(wheel=False).pose_valid)
        self.search_request(True, 'restart-1')
        good = self.search_to_valid()
        old_request = good.initialization_request_id
        original = self.points.copy()
        # Geometrically different, nearly planar query, not a rigidly shifted
        # copy that a legitimate global matcher should be able to localize.
        self.points[:, 0] *= 10
        self.points[:, 1] *= 10
        self.points[:, 2] = 0
        for _ in range(6):
            self.assertFalse(self.send(wheel=False).pose_valid)
        self.points = original
        recovered = self.search_to_valid()
        self.assertNotEqual(recovered.initialization_request_id, old_request)
        self.assertGreater(recovered.request_generation, good.request_generation)

    def test_map_switch_invalidates_search_support_and_old_request(self):
        self.launch()
        for _ in range(2):
            self.send(wheel=False)
        old = self.private[-1]
        old_session = old.session_id
        other = self.directory/'map-second'
        compatibility = c1.write_fixture(other, self.points, self.correction*self.base_imu)
        self.assertTrue(self.load(other, compatibility, 'map-switch').accepted)
        expected_uuid = json.loads((other/'manifest.json').read_text())['map_uuid']
        self.wait(lambda: self.private[-1].map_uuid == expected_uuid)
        self.assertNotEqual(old_session, self.private[-1].session_id)
        self.assertFalse(self.private[-1].pose_valid)
        client = self.node.create_client(Relocalize, '/fast_livo/localizer/relocalize')
        future = client.call_async(Relocalize.Request(request_id='stale-global-request', enable=True,
                                   map_uuid=old.map_uuid, map_generation=old.map_generation, session_id=old_session))
        self.wait(future.done)
        self.assertFalse(future.result().accepted)
        for _ in range(2):
            self.assertFalse(self.send(wheel=False).pose_valid)
        self.assertTrue(self.search_to_valid().pose_valid)

    def test_managed_c2_failure_preserves_unowned_processes(self):
        import signal
        sys.path.insert(0, str(c1.ROOT/'scripts'))
        from session_manager import read_session, session_live, same_process, notify
        directory = c1.ROOT/'.runtime/sessions/fast_livo_relocalizer'
        self.assertFalse(session_live(read_session(directory)[1]))
        process = self.spawn(['bash', str(c1.ROOT/'scripts/run_fast_livo2_relocalizer.sh'), '--execute', '--domain', '196',
                              '--archive', str(self.archive), '--compatibility', str(self.compatibility)], 'managed_c2')
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
        self.assertEqual(self.node.get_name(), 'c1_test_world')
        (self.directory/'managed_final.json').write_text(json.dumps(final, indent=2)+'\n')


if __name__ == '__main__':
    unittest.main()
