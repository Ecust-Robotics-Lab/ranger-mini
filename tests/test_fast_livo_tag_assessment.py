#!/usr/bin/env python3
"""Independent fixed-Tag geometry and healthy C1 segment checks; no ROS graph."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('tag_assessment', ROOT/'scripts/fast_livo2/tags/assess_landmark.py')
assessment = importlib.util.module_from_spec(spec)
spec.loader.exec_module(assessment)
common = assessment.common


def pose(x=0, y=0, z=0, r=0, p=0, yaw=0):
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler('xyz', [r, p, yaw]).as_matrix()
    transform[:3, 3] = [x, y, z]
    return transform


def local(fraction):
    return pose(.05*fraction, -.01*fraction, .005*fraction, .1, -.05, .03*fraction)


def project_independent(camera, tag, size, model):
    points = []
    for x, y in [(-1, 1), (1, 1), (1, -1), (-1, -1)]:
        world = tag@np.asarray([x*size/2, y*size/2, 0, 1.])
        point = camera[:3, :3].T@(world[:3]-camera[:3, 3])
        xn, yn = point[0]/point[2], point[1]/point[2]
        k1, k2, p1, p2, k3 = model['D']
        rr = xn*xn+yn*yn
        radial = 1+k1*rr+k2*rr*rr+k3*rr*rr*rr
        xd = xn*radial+2*p1*xn*yn+p2*(rr+2*xn*xn)
        yd = yn*radial+p1*(rr+2*yn*yn)+2*p2*xn*yn
        points.append([model['K'][0]*xd+model['K'][2], model['K'][4]*yd+model['K'][5]])
    return points


class HealthyC1(unittest.TestCase):
    def setUp(self):
        self.epoch = 1_590_195_801_300_123_320
        self.correction = pose(2, -3, 1, .2, -.1, .4)
        self.E = pose(.15, -.06, .08, .1, -.2, .3)
        self.native, self.samples = [], []
        for i in range(10):
            ns = self.epoch+i*100_000_000
            self.native.append({'message_id': i, 'stamp_ns': ns, 'pose': common.pose_json(local(i))})
            self.samples.append({'index': i, 'stamp_ns': ns, 'last_confirmed_stamp_ns': ns, 'state': 1,
                                 'pose_valid': True, 'reason': 'map_match_confirmed',
                                 'quality': {'map_uuid': 'map-test', 'graph_version': 7, 'inputs_ready': True,
                                             'map_loading': False, 'converged': True, 'geometrically_accepted': True,
                                             'rmse_m': .03, 'source_overlap': .9, 'correspondences': 300,
                                             'normalized_plane_hessian_eigen_ratio': .1,
                                             'T_target_source': common.pose_json(self.correction@local(i))}})

    def sequence(self):
        return assessment.C1Sequence(self.samples, self.native, 'map-test', 7)

    def test_noncommuting_pose_lever_arm_and_different_image_time(self):
        camera, support = self.sequence().camera(self.epoch+350_000_000, self.E)
        np.testing.assert_allclose(camera, self.correction@local(3.5)@self.E, atol=1e-12)
        self.assertEqual(support['scan_stamp_ns'], self.epoch+400_000_000)
        self.assertEqual(support['segment_scan_stamps_ns'], [self.epoch+i*100_000_000 for i in [2, 3, 4]])
        self.assertEqual(support['native_interpolation']['alpha'], .5)

    def test_degraded_map_mismatch_and_unbound_native_rejected(self):
        for change in ['state', 'map', 'native']:
            self.setUp()
            if change == 'state':
                self.samples[3]['state'] = 2
            elif change == 'map':
                self.samples[3]['quality']['map_uuid'] = 'wrong-map'
            else:
                self.native = [row for row in self.native if row['stamp_ns'] != self.samples[3]['stamp_ns']]
            with self.assertRaises(ValueError):
                self.sequence().camera(self.epoch+350_000_000, self.E)

    def test_global_correction_jump_is_not_interpolated_or_hidden(self):
        self.samples[4]['quality']['T_target_source'] = common.pose_json(pose(100)@self.correction@local(4))
        with self.assertRaisesRegex(ValueError, 'correction discontinuity'):
            self.sequence().camera(self.epoch+350_000_000, self.E)
        with self.assertRaisesRegex(ValueError, 'cannot bracket'):
            self.sequence().camera(self.epoch+1_000_000_000, self.E)


class FixedTag(unittest.TestCase):
    def setUp(self):
        self.model = {'K': [500., 0., 320., 0., 505., 240., 0., 0., 1.],
                      'D': [-.08, .05, .001, -.002, .01]}
        self.cameras = [pose(.06*i, -.01*i, .005*i, .1, -.05, .01*i) for i in range(9)]
        self.tag = self.cameras[0]@pose(.2, -.1, 2.2, .2, -.25, .3)
        self.observations = [{'image_index': i, 'observation': {'size_m': .155,
                              'corners': project_independent(camera, self.tag, .155, self.model)}}
                             for i, camera in enumerate(self.cameras)]

    def test_fixed_trajectory_fit_recovers_truth_on_heldout_frames(self):
        seed = self.tag@pose(.01, -.01, .02, .01, -.01, .01)
        fitted = assessment.fit_fixed_pose(seed, self.cameras, self.observations, self.model, 6)
        self.assertTrue(fitted['optimizer_success'])
        np.testing.assert_allclose(common.matrix(fitted['pose']), self.tag, atol=1e-6)
        self.assertLess(fitted['holdout_rmse_px']['maximum'], 1e-5)
        self.assertFalse(fitted['trajectory_optimized'])
        self.assertFalse(fitted['time_optimized'])

    def test_inconsistent_heldout_image_cannot_be_accepted_as_static_landmark(self):
        self.observations[-1]['observation']['corners'] = project_independent(
            self.cameras[-1], self.tag@pose(.2), .155, self.model)
        fitted = assessment.fit_fixed_pose(self.tag, self.cameras, self.observations, self.model, 6)
        self.assertLess(fitted['train_rmse_px']['maximum'], 1e-5)
        self.assertGreater(fitted['frame_errors'][-1]['rmse_px'], 20.)
        self.assertGreater(fitted['holdout_rmse_px']['maximum'], 20.)

    def test_mirror_modes_keep_identity_and_do_not_average_normals(self):
        candidates = []
        for i in range(4):
            for h in [0, 1]:
                transform = self.tag@pose(.001*i, 0, 0, 0, h*np.deg2rad(61), .001*i)
                candidates.append({'T': transform, 'observation_id': 'image-'+str(i), 'image_index': i, 'hypothesis_id': h})
        candidates.append(candidates[0])
        modes = assessment.cluster(candidates)
        self.assertEqual(len(modes), 2)
        self.assertEqual([len(mode) for mode in modes], [4, 4])
        delta = assessment.rotation_distance(assessment.center([row['T'] for row in modes[0]]),
                                              assessment.center([row['T'] for row in modes[1]]))
        self.assertAlmostEqual(np.rad2deg(delta), 61., places=3)


if __name__ == '__main__':
    unittest.main(verbosity=2)
