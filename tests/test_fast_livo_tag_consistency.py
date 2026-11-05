#!/usr/bin/env python3
"""Independent geometry and software-clock failure fixtures; no ROS graph/hardware."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('consistency', ROOT/'scripts/fast_livo2/tags/analyze_capture_consistency.py')
analysis = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analysis)


def pose(x=0, y=0, z=0, r=0, p=0, yaw=0):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('xyz', [r, p, yaw]).as_matrix()
    T[:3, 3] = [x, y, z]
    return T


def project(camera, tag, model, size=.14):
    result = []
    for x, y in [(-1, 1), (1, 1), (1, -1), (-1, -1)]:
        world = tag@np.array([x*size/2, y*size/2, 0, 1.])
        local = camera[:3, :3].T@(world[:3]-camera[:3, 3])
        x, y = local[:2]/local[2]
        k1, k2, p1, p2, k3 = model['D']
        rr = x*x+y*y
        radial = 1+k1*rr+k2*rr**2+k3*rr**3
        xd = x*radial+2*p1*x*y+p2*(rr+2*x*x)
        yd = y*radial+p1*(rr+2*y*y)+2*p2*x*y
        result.append([model['K'][0]*xd+model['K'][2], model['K'][4]*yd+model['K'][5]])
    return result


class Timing(unittest.TestCase):
    def test_missing_writer_scan_explains_repeated_camera_stamp(self):
        epoch = 1_590_195_569_700_082_520
        stamps = [epoch+i*100_000_000 for i in range(4)]
        writer = [{'seq': i+1, 'scan_start_ns': stamps[j], 'arrival_ns': j*100_000_000}
                  for i, j in enumerate([0, 1, 3])]
        camera = [{'frame_id': i, 'dev_ts_ticks': 10_000_000*i, 'arrival_ns': i*100_000_000+20_000_000,
                   'publish_ns': i*100_000_000+22_000_000 if i != 2 else 0,
                   'ts_seq': [2, 4, 4, 6][i], 'ts_low_ns': stamps[[0, 1, 1, 3][i]],
                   'status': 'BACKWARD' if i == 2 else 'OK'} for i in range(4)]
        result, _ = analysis.analyze_sync(camera, writer, stamps)
        self.assertEqual(result['recorded_lidar_scans_missing_from_writer'], [stamps[2]])
        self.assertTrue(result['camera_frame_ids_contiguous'])
        bad = result['anomalies'][0]
        self.assertTrue(bad['same_stamp_as_previous'])
        self.assertEqual(bad['following_writer_stamp_gap_ms'], 200.)
        self.assertEqual(bad['bag_scans_absent_from_writer_in_following_gap'], [stamps[2]])
        self.assertFalse(result['physical_exposure_offset_identified'])

    def test_block_split_is_disjoint_purged_and_thinned_at_large_epoch(self):
        epoch = 1_590_195_569_700_082_520
        stamps = [epoch+i*100_000_000 for i in range(600)]
        train, held, purged = analysis.split_indices(stamps, epoch)
        self.assertFalse(set(train)&set(held))
        self.assertFalse((set(train)|set(held))&set(purged))
        self.assertGreater(len(held), 10)
        self.assertTrue(all(stamps[b]-stamps[a] >= 500_000_000 for a, b in zip(train, train[1:])))
        self.assertTrue(all(min(abs(stamps[i]-stamps[j]) for j in held) >= 1_000_000_000 for i in train))


class FixedTrajectory(unittest.TestCase):
    def setUp(self):
        self.model = {'K': [700., 0., 640., 0., 705., 512., 0., 0., 1.], 'D': [-.08, .05, .001, -.002, .01]}
        self.cameras = [pose(.04*i, -.01*i, .005*i, .1, -.04+.003*i, .02*i) for i in range(12)]
        self.tag = self.cameras[0]@pose(.2, -.1, 2.2, .2, -.25, .3)
        self.observations = [{'index': i, 'stamp_ns': i, 'observation': {'size_m': .14, 'corners': project(C, self.tag, self.model)}}
                             for i, C in enumerate(self.cameras)]

    def test_vectorized_raw_brown_projection_matches_independent_truth(self):
        error, depth = analysis.projected_errors(self.tag, np.linalg.inv(self.cameras), self.observations, self.model)
        self.assertTrue(depth.all())
        np.testing.assert_allclose(error, 0., atol=1e-10)
        invalid = self.tag.copy()
        invalid[2, 3] = -3.
        _, depth = analysis.projected_errors(invalid, np.linalg.inv(self.cameras), self.observations, self.model)
        self.assertFalse(depth.any())

    def test_fit_recovers_fixed_tag_and_rejects_contradictory_holdout(self):
        seed = self.tag@pose(.01, -.01, .01, .01, -.01, .01)
        train, held = list(range(8)), list(range(8, 12))
        fit = analysis.fit_bundle(seed, self.cameras, self.observations, self.model, train, held)
        self.assertLess(fit['holdout_rmse_px']['maximum'], 1e-5)
        np.testing.assert_allclose(analysis.common.matrix(fit['pose']), self.tag, atol=1e-6)
        self.observations[-1]['observation']['corners'] = project(self.cameras[-1], self.tag@pose(.2), self.model)
        fit = analysis.fit_bundle(seed, self.cameras, self.observations, self.model, train, held)
        self.assertGreater(fit['holdout_rmse_px']['maximum'], 20.)
        self.assertLess(fit['train_rmse_px']['maximum'], 1e-5)

    def test_known_time_offset_has_correct_sign_with_curved_motion_and_lever_arm(self):
        epoch = 1_590_195_569_700_082_520
        E = pose(.1, -.02, .04, .02, -.03, .04)
        def trajectory(t):
            return pose(.1*np.sin(3*t), .05*np.cos(4*t), .02*t, .1*np.sin(3*t), .08*np.cos(4*t), .2*t*t)
        native = [{'message_id': i, 'stamp_ns': epoch+i*10_000_000,
                   'pose': analysis.common.pose_json(trajectory(i*.01))} for i in range(151)]
        history = analysis.common.PoseHistory(native)
        observations = [{'index': i, 'stamp_ns': epoch+(i+1)*100_000_000,
                         'observation': {'size_m': .14, 'corners': project(trajectory((i+1)*.1+.05)@E, self.tag, self.model)}}
                        for i in range(12)]
        train, held = list(range(8)), list(range(8, 12))
        correct, support = analysis.associate(history, observations, E, 50_000_000)
        self.assertTrue(all(r['available'] for r in support))
        fit = analysis.fit_bundle(self.tag, correct, observations, self.model, train, held)
        self.assertLess(fit['holdout_rmse_px']['maximum'], 1e-5)
        zero, _ = analysis.associate(history, observations, E, 0)
        fit_zero = analysis.fit_bundle(self.tag, zero, observations, self.model, train, held)
        self.assertGreater(fit_zero['holdout_rmse_px']['p50'], 1.)
        self.assertFalse(fit['clock_calibrated'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
