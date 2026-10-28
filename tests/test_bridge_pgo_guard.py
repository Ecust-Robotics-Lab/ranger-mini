#!/usr/bin/env python3
"""Regression for rejected corrections, repeated heartbeats and the offline latch."""
from collections import deque
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'ws_nav/src/ranger_localization_bridge'))
from ranger_localization_bridge.bridge_node import LocalizationBridge
from ranger_localization_bridge.alignment import PoseHistory
from ranger_localization_bridge.geometry import SE3
from ranger_localization_msgs.msg import LocalizationEstimate


def fixture(latch):
    sample = LocalizationEstimate(); sample.state = sample.VALID
    sample.header.stamp.sec = 2; sample.pose.position.x = 10.; sample.pose.orientation.w = 1.
    history = PoseHistory(); history.add(2_000_000_000, SE3())
    node = SimpleNamespace(sample=sample, tf_max_age=.5, _last_aligned_ns=1_000_000_000,
                           _last_checked_ns=None, odom_history=history, _innov_fails=0, _innov_last_err=None,
                           _history=deque([(1., SE3(), SE3())]), _map_to_odom=SE3(), innov_interval=.5,
                           innov_t_th=.5, innov_speed_th=.3, innov_yaw_th=.35,
                           _innovation_latched=False, latch_on_innovation_failure=latch)
    node._innovation = lambda *a: LocalizationBridge._innovation(node, *a)
    return node


class GuardTests(unittest.TestCase):
    def test_rejection_preserves_correction_and_accepted_history(self):
        node = fixture(False)
        LocalizationBridge._align_latest(node, 2.)
        self.assertEqual(node._map_to_odom.t, (0., 0., 0.))
        self.assertEqual(node._last_aligned_ns, 1_000_000_000)
        self.assertEqual(len(node._history), 1)
        self.assertEqual(node._innov_fails, 1)
        LocalizationBridge._align_latest(node, 2.)
        self.assertEqual(node._innov_fails, 1)

    def test_offline_latch_requires_new_bridge_process(self):
        node = fixture(True)
        LocalizationBridge._align_latest(node, 2.)
        self.assertTrue(node._innovation_latched)
        node.sample.pose.position.x = 0.
        node.sample.header.stamp.nanosec = 1
        LocalizationBridge._align_latest(node, 2.)
        self.assertEqual(node._last_aligned_ns, 1_000_000_000)
        LocalizationBridge._reset_alignment(node, 'clock reset')
        self.assertTrue(node._innovation_latched)


if __name__ == '__main__':
    unittest.main(verbosity=2)
