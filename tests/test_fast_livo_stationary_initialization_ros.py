#!/usr/bin/env python3
"""C2 static proof crosses the actual ROS handoff; manual C1 keeps motion policy."""
import json
import unittest
import test_fast_livo_relocalizer_ros as c2
c1=c2.c1
c1.EVIDENCE=c1.ROOT/'docs/fast_livo2_evidence/initialization_optimization_20261109/ros'
c1.EVIDENCE.mkdir(parents=True,exist_ok=True)
c1.EXECUTABLE=c1.ROOT/'ws_fast_livo/online_localization_install/ranger_fast_livo_localizer/lib/ranger_fast_livo_localizer/localizer_node'

class Stationary(unittest.TestCase):
    setUp=c1.C1Ros.setUp
    tearDown=c1.C1Ros.tearDown
    spawn=c1.C1Ros.spawn
    pump=c1.C1Ros.pump
    wait=c1.C1Ros.wait
    send=c1.C1Ros.send
    sensors=c1.C1Ros.sensors
    wheel_pose=c1.C1Ros.wheel_pose
    on_health=c1.C1Ros.on_health
    launch=c2.C2Ros.launch
    search_to_valid=c2.C2Ros.search_to_valid
    initialize=c1.C1Ros.initialize

    def freeze(self):
        frozen=self.wheel_pose(self.ns)
        self.wheel_pose=lambda ns:frozen

    def test_static_global_handoff_then_fresh_c1_confirmation(self):
        self.freeze();self.launch();message=self.search_to_valid()
        self.assertTrue(json.loads(message.quality_json).get('stationary_global_verification'))
        self.assertTrue(message.initialization_request_id.startswith('global-'))
        for _ in range(3):self.assertTrue(self.send(wheel=False).pose_valid)

    def test_manual_initialization_still_requires_motion(self):
        self.freeze();self.launch();self.send(wheel=False);self.initialize()
        for _ in range(8):self.assertFalse(self.send(wheel=False).pose_valid)

if __name__=='__main__':unittest.main()
