#!/usr/bin/env python3
"""Independent projection fixtures for diagnostic attribution, not field acceptance."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('motion_geometry', ROOT/'scripts/fast_livo2/tags/diagnose_motion_geometry.py')
diagnostic = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diagnostic)
fixture_spec = importlib.util.spec_from_file_location('independent_projection', ROOT/'tests/test_fast_livo_tag_consistency.py')
fixture = importlib.util.module_from_spec(fixture_spec)
fixture_spec.loader.exec_module(fixture)


class MotionGeometry(unittest.TestCase):
    def test_local_split_has_separated_support_and_integer_epoch(self):
        origin = 1_590_195_535_300_312_660
        stamps = [origin+i*100_000_000 for i in range(150)]
        groups = diagnostic.local_split(stamps, origin)
        self.assertEqual(set(groups), {0, 1, 2})
        for group in groups.values():
            self.assertFalse(set(group['train']) & set(group['heldout']))
            self.assertGreaterEqual(min(stamps[i] for i in group['heldout'])-
                                    max(stamps[i] for i in group['train']), 500_000_000)

    def fixture(self, drift):
        model = {'K': [700., 0., 640., 0., 705., 512., 0., 0., 1.],
                 'D': [-.08, .05, .001, -.002, .01]}
        tag = fixture.pose(.15, -.1, 2., .12, -.2, .05)
        origin = 1_590_195_535_300_312_660
        cameras, observations = [], []
        for i in range(150):
            true_camera = fixture.pose(.03*np.sin(i*.06), -.01*np.cos(i*.04), 0., .04, -.03, .001*i)
            error = fixture.pose(drift*(i//50), 0., 0.)
            cameras.append(error@true_camera)
            observations.append({'index': i, 'stamp_ns': origin+i*100_000_000,
                                 'observation': {'observation_id': str(i), 'size_m': .14,
                                                 'corners': fixture.project(true_camera, tag, model),
                                                 'hypotheses': [{'T_camera_tag': diagnostic.common.pose_json(np.linalg.inv(true_camera)@tag)}]}})
        return cameras, observations, model, origin

    def test_fixed_trajectory_recovers_one_landmark(self):
        result = diagnostic.diagnose(*self.fixture(0.))
        self.assertLess(result['global']['holdout_rmse_px']['maximum'], 1e-6)
        self.assertTrue(all(b['local_heldout_px']['maximum'] < 1e-6 for b in result['local_blocks']))

    def test_time_varying_trajectory_error_is_not_a_valid_fixed_landmark(self):
        result = diagnostic.diagnose(*self.fixture(.04))
        self.assertGreater(result['global']['holdout_rmse_px']['p50'], 5.)
        self.assertTrue(all(b['local_heldout_px']['maximum'] < 1e-5 for b in result['local_blocks']))
        self.assertGreater(np.linalg.norm(np.asarray(result['local_blocks'][-1]['local_pose']['xyz'])-
                                          result['local_blocks'][0]['local_pose']['xyz']), .07)


if __name__ == '__main__':
    unittest.main()
