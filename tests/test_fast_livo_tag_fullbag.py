#!/usr/bin/env python3
"""Geometry/time and actual detector stream checks, without a ROS graph."""
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
path = ROOT/'scripts/fast_livo2/tags/extract_bag.py'
spec = importlib.util.spec_from_file_location('tag_fullbag', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
EVIDENCE = ROOT/'docs/fast_livo2_evidence/apriltag_fullbag_20261029'
TOOL = ROOT/'ws_fast_livo/backend_install/ranger_fast_livo_backend/lib/ranger_fast_livo_backend/tag_image_tool'
CONFIG = ROOT/'config/fast_livo2/apriltag_camera.json'


def record(ns, xyz=(0, 0, 0), yaw=0., identity=0):
    return {'message_id': identity, 'stamp_ns': ns, 'pose': {'xyz': list(xyz),
            'xyzw': Rotation.from_euler('z', yaw).as_quat().tolist()}}


class PoseAssociation(unittest.TestCase):
    def test_epoch_precision_slerp_and_nonzero_translation(self):
        # 2020-scale ns cannot be interpolated by converting absolute times to float.
        ns = 1_590_195_801_300_123_320
        history = module.PoseHistory([record(ns, (1, 2, 3), .2, 10),
                                      record(ns+100_000_000, (1.1, 2.02, 3.01), .5, 20)])
        middle = history.lookup(ns+50_000_000)
        self.assertTrue(middle['available'])
        np.testing.assert_allclose(middle['pose']['xyz'], [1.05, 2.01, 3.005], atol=1e-12)
        self.assertAlmostEqual(Rotation.from_quat(middle['pose']['xyzw']).as_euler('xyz')[2], .35, places=12)
        self.assertEqual(middle['support_stamps_ns'], [ns, ns+100_000_000])
        self.assertEqual(history.lookup(ns)['kind'], 'exact')
        self.assertEqual(history.lookup(ns-1)['reason'], 'no_bracket_no_extrapolation')
        self.assertEqual(history.lookup(ns+100_000_001)['reason'], 'no_bracket_no_extrapolation')

    def test_missing_bracket_gap_duplicate_epoch_and_motion_rejected(self):
        ns = 10_000_000_000
        for rows, query, reason in [([], ns, 'no_bracket_no_extrapolation'),
                                   ([record(ns), record(ns+200_000_000)], ns+10, 'pose_gap_excessive'),
                                   ([record(ns), record(ns)], ns, 'nonmonotonic_or_duplicate_pose_epoch'),
                                   ([record(ns+1), record(ns)], ns, 'nonmonotonic_or_duplicate_pose_epoch'),
                                   ([record(ns), record(ns+100_000_000, (10, 0, 0))], ns+10, 'pose_interval_motion_excessive')]:
            self.assertEqual(module.PoseHistory(rows).lookup(query)['reason'], reason)


class DetectorStream(unittest.TestCase):
    def setUp(self):
        self.directory = Path(tempfile.mkdtemp(prefix=self._testMethodName+'-', dir=EVIDENCE))
        self.command = [str(TOOL), '--stream', str(CONFIG), 'tag36h11', '{"1":0.155}']

    def request(self, index, image, step=None):
        data = image.tobytes()
        return {'schema_version': 1, 'request_id': index, 'payload_bytes': len(data),
                'width': image.shape[1], 'height': image.shape[0], 'step': image.shape[1] if step is None else step,
                'encoding': 'mono8', 'stamp_ns': 1_590_195_801_300_123_320+index,
                'context': 'test-offline-stream', 'image_id': 'image-'+str(index),
                'image_reference': 'synthetic or existing read-only fixture'}, data

    def test_multiple_frames_bad_stride_recovers_and_legacy_cli_matches(self):
        known = ROOT/'outputs/fast-livo-apriltag-20261029-sized-audit/image_745.png'
        if not known.exists():
            self.skipTest('existing real image audit fixture missing')
        image = cv2.imread(str(known), cv2.IMREAD_GRAYSCALE)
        blank = np.full_like(image, 255)
        with (self.directory/'stream.log').open('wb') as log:
            process = subprocess.Popen(self.command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=log)
            try:
                request, data = self.request(0, blank)
                first = module.request_frame(process, request, data)
                self.assertTrue(first['success'])
                self.assertFalse(first['observations'])
                request, data = self.request(1, image, step=10)
                self.assertFalse(module.request_frame(process, request, data)['success'])
                request, data = self.request(2, image)
                actual = module.request_frame(process, request, data)
                self.assertTrue(actual['success'])
                self.assertEqual([(o['family'], o['id']) for o in actual['observations']], [('tag36h11', 1)])
                legacy = subprocess.run([str(TOOL), str(CONFIG), str(known), str(request['stamp_ns']), 'tag36h11', '{"1":0.155}'],
                                        capture_output=True, text=True, timeout=15)
                self.assertEqual(legacy.returncode, 0, legacy.stderr)
                expected = json.loads(legacy.stdout)
                np.testing.assert_allclose(actual['observations'][0]['corners'], expected['observations'][0]['corners'], atol=1e-9)
                self.assertEqual(actual['observations'][0]['hypotheses'], expected['observations'][0]['hypotheses'])
                (self.directory/'response.json').write_text(json.dumps(actual, indent=2)+'\n')
                process.stdin.close()
                self.assertEqual(process.wait(timeout=5), 0)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait(timeout=5)
                process.stdout.close()

    def test_truncated_payload_and_oversize_header_fail_closed(self):
        image = np.full((1024, 1280), 255, dtype=np.uint8)
        request, data = self.request(0, image)
        for payload in [(json.dumps(request)+'\n').encode()+data[:20], b'x'*16385+b'\n']:
            process = subprocess.run(self.command, input=payload, capture_output=True, timeout=10)
            self.assertEqual(process.returncode, 2)
            self.assertFalse(process.stdout)


if __name__ == '__main__':
    unittest.main(verbosity=2)
