#!/usr/bin/env python3
"""Real-node online queue/freshness checks using the existing 6DoF fixture."""
import json
from pathlib import Path
import time
import unittest
import test_fast_livo_localizer_ros as c1

c1.EVIDENCE = c1.ROOT/'docs/fast_livo2_evidence/online_localization_20261106/ros_checks'
c1.EVIDENCE.mkdir(exist_ok=True)
c1.EXECUTABLE = c1.ROOT/'ws_fast_livo/online_localization_install/ranger_fast_livo_localizer/lib/ranger_fast_livo_localizer/localizer_node'


class OnlineRos(unittest.TestCase):
    setUp = c1.C1Ros.setUp
    tearDown = c1.C1Ros.tearDown
    spawn = c1.C1Ros.spawn
    pump = c1.C1Ros.pump
    wait = c1.C1Ros.wait
    sensors = c1.C1Ros.sensors
    wheel_pose = c1.C1Ros.wheel_pose
    send = c1.C1Ros.send
    initialize = c1.C1Ros.initialize
    on_health = c1.C1Ros.on_health

    def launch(self):
        process = self.spawn([str(c1.EXECUTABLE), '--ros-args', '--params-file',
            str(c1.ROOT/'config/fast_livo2/localizer.yaml'), '-p', 'archive_path:='+str(self.archive),
            '-p', 'compatibility_path:='+str(self.compatibility), '-p', 'online_scheduling:=true',
            '-p', 't_base_imu_xyz:=[0.2,-0.1,0.4]', '-p', 't_base_imu_rpy:=[0.1,-0.08,0.2]'], 'online')
        self.wait(lambda: self.private and self.private[-1].map_uuid and
                  json.loads(self.private[-1].quality_json).get('inputs_ready'))
        self.assertIsNone(process.poll())

    def valid(self):
        response, _ = self.initialize(request_id='initial-'+str(self.ns))
        self.assertTrue(response.accepted, response.reason)
        for _ in range(12):
            result = self.send(wheel=False)
            if result.pose_valid: return result
        self.fail('initialization failed: '+result.reason)

    def test_burst_coalesces_complete_pairs_and_keeps_fresh_output(self):
        self.launch(); self.valid()
        pairs = [self.sensors(self.ns+(i+1)*10_000_000) for i in range(20)]
        self.ns += 200_000_000
        self.pump(.02)
        for odom, cloud in pairs:
            self.odom.publish(odom); self.cloud.publish(cloud)
        self.pump(.5)
        quality = json.loads(self.private[-1].quality_json)
        self.assertGreater(quality['skipped_scans'], 0)
        self.assertLessEqual(quality['queued_jobs'], 2)
        self.assertTrue(any(m.pose_valid for m in self.private[-10:]))
        for message in self.private:
            if message.pose_valid:
                self.assertLessEqual(json.loads(message.quality_json)['sample_age_seconds'], .5)

    def test_clock_advance_revokes_pose_and_manual_requires_new_initial(self):
        self.launch(); self.valid()
        self.ns += 700_000_000; self.pump(.25)
        self.assertFalse(self.private[-1].pose_valid)
        self.assertEqual(self.private[-1].state, 3)
        for _ in range(3): self.assertFalse(self.send(wheel=False).pose_valid)
        self.assertTrue(self.valid().pose_valid)

    def test_slow_initial_evidence_cannot_publish_old_pose(self):
        self.launch(); response, _ = self.initialize(); self.assertTrue(response.accepted)
        # Freeze the clock while sending two supported views, then advance it
        # while full ambiguity work is pending. Only new samples may restore VALID.
        self.ns += 100_000_000; first = self.sensors(self.ns)
        self.pump(.02); self.odom.publish(first[0]); self.cloud.publish(first[1]); self.pump(.1)
        self.ns += 250_000_000; second = self.sensors(self.ns)
        self.pump(.02); self.odom.publish(second[0]); self.cloud.publish(second[1])
        self.ns += 700_000_000; self.pump(.5)
        self.assertFalse(self.private[-1].pose_valid)
        self.assertGreater(json.loads(self.private[-1].quality_json)['stale_results'], 0)
        for message in self.private:
            if message.pose_valid:
                self.assertLessEqual(json.loads(message.quality_json)['sample_age_seconds'], .5)


if __name__ == '__main__': unittest.main(verbosity=2)
