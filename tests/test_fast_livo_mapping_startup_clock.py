"""Replay real failed-start receipts through the shared monitor without ROS nodes."""
import json
from pathlib import Path
from types import SimpleNamespace
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'scripts/fast_livo2'), str(ROOT/'scripts')]
from mapping_monitor import Monitor
from fast_livo2.navigation.live_clock import StartupAnchor


class MappingStartupTests(unittest.TestCase):
    def test_real_startup_receipts_do_not_lock_first_burst(self):
        path = ROOT/'docs/fast_livo2_evidence/p0_closure_clock_fix_20261111/failed_startup_receipts.json'
        rows = json.loads(path.read_text())
        with tempfile.TemporaryDirectory() as tmp:
            obj = SimpleNamespace(folder=Path(tmp), replay=False, localization=False,
                counts={}, last={}, startup_anchor=StartupAnchor(), anchor=None, clock_error='')
            for entry in rows:
                lidar = entry['lidar']
                message = SimpleNamespace(data=json.dumps({'schema_version': 1, 'context': tmp,
                    'topics': {'/livox/lidar': lidar}}))
                with patch('mapping_monitor.time.monotonic_ns', return_value=lidar['last_steady_ns']):
                    Monitor.receipts(obj, message)
            self.assertIsNotNone(obj.anchor)
            self.assertEqual(obj.clock_error, '')
            first = rows[0]['lidar']
            raw_offset = (obj.anchor[0]-first['first_header_ns']-obj.anchor[1]+first['first_steady_ns'])
            self.assertGreater(raw_offset, 2_000_000_000)
            saved = tuple(obj.anchor)
            changed = dict(rows[-1]['lidar'])
            changed['last_header_ns'] += 3_000_000_000
            message.data = json.dumps({'schema_version': 1, 'context': tmp,
                'topics': {'/livox/lidar': changed}})
            with patch('mapping_monitor.time.monotonic_ns', return_value=changed['last_steady_ns']):
                Monitor.receipts(obj, message)
            self.assertEqual(tuple(obj.anchor), saved)
            self.assertIn('diverged', obj.clock_error)


if __name__ == '__main__':
    unittest.main(verbosity=2)
