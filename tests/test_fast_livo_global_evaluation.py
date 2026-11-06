#!/usr/bin/env python3
"""Provenance failures and independent image-curve fixtures; no ROS graph."""
import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import cv2
import numpy as np
from scipy.spatial import cKDTree

ROOT=Path(__file__).resolve().parents[1]
def module(name,path):
    spec=importlib.util.spec_from_file_location(name,ROOT/path);out=importlib.util.module_from_spec(spec);spec.loader.exec_module(out);return out
evaluation=module('global_evaluation','scripts/fast_livo2/localizer/validate_motion.py')
calibration=module('hole_check','docs/fast_livo2_evidence/c2_cross_session_20261104/check_calibration.py')


class Provenance(unittest.TestCase):
    def test_wrong_map_or_query_cannot_be_declared_independent(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'source.json'
            path.write_text(json.dumps({'source_manifest_sha256':'map','source_database_sha256':'query','different_capture':True}))
            for map_sha,query_sha in [('other-map','query'),('map','other-query')]:
                with self.assertRaisesRegex(ValueError,'does not match'):
                    evaluation.query_declaration(path,map_sha,query_sha,True)
            self.assertTrue(evaluation.query_declaration(path,'map','query',True)['different_capture'])
            with self.assertRaisesRegex(ValueError,'same-graph initial'):
                evaluation.query_declaration(path,'map','query',False)

    def test_legacy_no_declaration_remains_supported(self):
        self.assertIsNone(evaluation.query_declaration(None,'map','query',False))


class CancelledInput(unittest.TestCase):
    @staticmethod
    def message(**changes):
        fields = dict(INITIALIZING=0, state=0, pose_valid=False,
                      reason='verifying_multiframe_motion_support',
                      latest_scan_stamp_ns=100, map_uuid='map', session_id='session',
                      request_generation=3, initialization_request_id='global-3')
        fields.update(changes)
        return SimpleNamespace(**fields)

    def test_timeout_releases_only_cancelled_input_without_forging_an_ack(self):
        previous = self.message()
        timeout = self.message(reason='initial_verification_timeout')
        self.assertEqual(evaluation.pair_outcome(timeout, previous, 200, True),
                         'cancelled_initial_verification_timeout')
        self.assertEqual(timeout.latest_scan_stamp_ns, 100)
        self.assertFalse(timeout.pose_valid)
        self.assertIsNone(evaluation.pair_outcome(timeout, previous, 200, False))
        exact = self.message(latest_scan_stamp_ns=200)
        self.assertEqual(evaluation.pair_outcome(exact, previous, 200, True), 'acknowledged')

    def test_stale_or_valid_heartbeat_cannot_release_an_unprocessed_input(self):
        previous = self.message()
        self.assertIsNone(evaluation.pair_outcome(previous, previous, 200, True))
        for changes in [dict(pose_valid=True), dict(state=1),
                        dict(reason='searching'), dict(latest_scan_stamp_ns=300)]:
            message = self.message(reason='initial_verification_timeout')
            for field, value in changes.items():
                setattr(message, field, value)
            self.assertIsNone(evaluation.pair_outcome(message, previous, 200, True))
        timeout = self.message(reason='initial_verification_timeout')
        self.assertIsNone(evaluation.pair_outcome(self.message(reason=timeout.reason), timeout, 200, True))

    def test_map_session_or_attempt_switch_is_not_a_timeout_ack(self):
        previous = self.message()
        for field in ['map_uuid', 'session_id', 'request_generation', 'initialization_request_id']:
            timeout = self.message(reason='initial_verification_timeout')
            setattr(timeout, field, 'other')
            self.assertIsNone(evaluation.pair_outcome(timeout, previous, 200, True))


class ImageGeometry(unittest.TestCase):
    def test_rotated_ellipse_recovered_despite_unrelated_edge_pixels(self):
        truth=((640.,512.),(180.,155.),27.)
        boundary=calibration.ellipse_points(truth,720)
        rng=np.random.default_rng(3)
        clutter=rng.uniform([530,415],[750,610],size=(400,2))
        fitted,quality=calibration.robust_ellipse(np.concatenate([boundary,clutter]),truth)
        distance=cKDTree(calibration.ellipse_points(fitted)).query(boundary)[0]
        self.assertLess(np.percentile(distance,95),.15)
        self.assertGreater(quality['angular_bins_of_36'],30)
        shifted=boundary+[8,0]
        self.assertGreater(np.sqrt(np.mean(cKDTree(calibration.ellipse_points(fitted)).query(shifted)[0]**2)),4.)

    def test_short_partial_arc_is_not_accepted_as_full_hole_support(self):
        truth=((640.,512.),(180.,155.),27.)
        arc=calibration.ellipse_points(truth,720)[:150]
        with self.assertRaisesRegex(ValueError,'angular coverage'):
            calibration.robust_ellipse(arc,truth)

    def test_projected_circle_center_is_not_used_as_image_ellipse_center(self):
        theta=np.arange(720)*2*np.pi/720
        R=cv2.Rodrigues(np.array([.5,.3,.1]))[0]
        center=np.array([.2,.1,1.5])
        circle=np.column_stack([.12*np.cos(theta),.12*np.sin(theta),np.zeros(len(theta))])@R.T+center
        K=np.array([[800.,0,640],[0,810.,512],[0,0,1.]])
        projected=cv2.projectPoints(circle,np.zeros(3),np.zeros(3),K,np.zeros(5))[0].reshape(-1,2)
        fitted=cv2.fitEllipse(projected.astype(np.float32))
        center_pixel=cv2.projectPoints(center.reshape(1,3),np.zeros(3),np.zeros(3),K,np.zeros(5))[0].ravel()
        self.assertGreater(np.linalg.norm(center_pixel-fitted[0]),.5)
        self.assertLess(np.percentile(cKDTree(calibration.ellipse_points(fitted)).query(projected)[0],95),.1)


if __name__=='__main__':unittest.main(verbosity=2)
