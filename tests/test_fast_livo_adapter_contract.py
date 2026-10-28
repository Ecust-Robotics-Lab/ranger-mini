#!/usr/bin/env python3
"""Private-message rejection must not poison accepted adapter watermarks."""
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'ws_nav/src/ranger_localization_bridge'))
from ranger_fast_livo_backend.msg import BackendEstimate
from ranger_localization_bridge.fast_livo_adapter import FastLivoAdapter, validate_backend_message


def estimate():
    m = BackendEstimate(); m.header.frame_id = 'map'; m.imu_frame = 'fast_livo_imu'
    m.header.stamp.sec = 100; m.state = m.VALID; m.pose_valid = True
    m.pose.orientation.w = 1.; m.session_id = 'session'; m.graph_version = 10
    return m


class AdapterContractTests(unittest.TestCase):
    def test_valid_and_invalid_contract(self):
        self.assertIsNone(validate_backend_message(estimate()))
        for field in ['frame', 'stamp', 'state', 'pose', 'finite', 'session', 'validity']:
            m = estimate()
            if field == 'frame': m.header.frame_id = 'wrong'
            elif field == 'stamp': m.header.stamp.nanosec = 10**9
            elif field == 'state': m.state = 255
            elif field == 'pose': m.pose.orientation.w = 0.
            elif field == 'finite': m.pose.position.x = float('inf')
            elif field == 'session': m.session_id = ''
            else: m.pose_valid = False
            with self.subTest(field=field): self.assertIsNotNone(validate_backend_message(m))

    def test_rejected_and_future_messages_preserve_watermark(self):
        node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100*10**9)),
                               last_clock=100*10**9, message=None, received=None, session='session',
                               last_ns=100*10**9, last_version=10, max_age=.5, tick=lambda: None)
        invalid = estimate(); invalid.header.frame_id = 'bad'; invalid.header.stamp.sec = 101
        FastLivoAdapter.on_estimate(node, invalid)
        self.assertEqual(node.last_ns, 100*10**9)
        future = estimate(); future.header.stamp.sec = 101
        FastLivoAdapter.on_estimate(node, future)
        self.assertEqual(node.last_ns, 100*10**9)
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_200_000_000))
        valid = estimate(); valid.header.stamp.nanosec = 100
        FastLivoAdapter.on_estimate(node, valid)
        self.assertIsNone(node.input_error)
        self.assertEqual(node.last_ns, 100*10**9+100)

    def test_graph_regression_rejected_without_lowering_watermark(self):
        node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100*10**9)),
                               last_clock=100*10**9, message=None, received=None, session='session',
                               last_ns=100*10**9, last_version=10, max_age=.5, tick=lambda: None)
        msg = estimate(); msg.graph_version = 9
        FastLivoAdapter.on_estimate(node, msg)
        self.assertIsNotNone(node.input_error)
        self.assertEqual(node.last_version, 10)


if __name__ == '__main__':
    unittest.main(verbosity=2)
