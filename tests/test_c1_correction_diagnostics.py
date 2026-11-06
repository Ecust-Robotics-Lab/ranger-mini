#!/usr/bin/env python3
"""Independent pose fixtures for map/local correction attribution; no ROS graph."""
from copy import deepcopy
import importlib.util
from pathlib import Path
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('c1_correction',ROOT/'scripts/fast_livo2/tags/diagnose_c1_corrections.py')
d=importlib.util.module_from_spec(spec);spec.loader.exec_module(d)

def T(t,r):
    p=np.eye(4);p[:3,3]=t;p[:3,:3]=Rotation.from_euler('xyz',r).as_matrix();return p

class Corrections(unittest.TestCase):
    def fixture(self, offsets=(0.,0.,0.)):
        alignment=T([4,-3,2],[.21,-.17,.8]);samples=[];native=[]
        for i,dx in enumerate(offsets):
            stamp=1590195535300312660+i*100000000
            u=T([.2*i,-.1*i,.04*i],[.04*i,-.06*i,.1*i]);x=alignment@u;x[0,3]+=dx
            pj=d.common.pose_json(x)
            q={'T_target_source':pj,'prediction':pj,'converged':True,'geometrically_accepted':True,
               'inputs_ready':True,'map_loading':False,'map_uuid':'map','graph_version':4,
               'rmse_m':.04,'source_overlap':.9,'correspondences':500,'normalized_plane_hessian_eigen_ratio':.1,
               'native_diagnostics':{'target_keyframe_ids':[2,4,7]}}
            samples.append({'stamp_ns':stamp,'index':i,'state':1,'pose_valid':True,'last_confirmed_stamp_ns':stamp,
                            'reason':'map_match_confirmed','pose_map_imu':pj,'map_uuid':'map','session_id':'session',
                            'request_generation':3,'quality':q})
            native.append({'stamp_ns':stamp,'pose':d.common.pose_json(u)})
        return samples,native

    def test_constant_six_dof_map_alignment_does_not_create_motion_error(self):
        rows=d.correction_rows(*self.fixture())
        self.assertTrue(all(r['body_residual_from_first_alignment_m']<1e-12 for r in rows))
        self.assertTrue(all(r.get('geometric_step_translation_m',0)<1e-12 for r in rows))
        self.assertTrue(all(r.get('geometric_step_rotation_deg',0)<1e-10 for r in rows))

    def test_known_two_centimeter_alternation_is_measured_at_robot_origin(self):
        rows=d.correction_rows(*self.fixture((0.,.02,-.02)))
        self.assertAlmostEqual(rows[1]['geometric_step_translation_m'],.02)
        self.assertAlmostEqual(rows[2]['geometric_step_translation_m'],.04)
        self.assertAlmostEqual(rows[2]['body_residual_from_first_alignment_m'],.02)
        self.assertLess(rows[2]['geometric_step_rotation_deg'],1e-10)

    def test_map_coordinate_rotation_and_translation_preserve_body_step_norm(self):
        samples,native=self.fixture((0.,.02,-.02));expected=d.correction_rows(samples,native)
        world=T([100,-200,30],[.3,-.4,.7])
        for r in samples:
            for key in ['pose_map_imu']:
                r[key]=d.common.pose_json(world@d.pose(r[key]))
            for key in ['T_target_source','prediction']:
                r['quality'][key]=d.common.pose_json(world@d.pose(r['quality'][key]))
        actual=d.correction_rows(samples,native)
        for a,b in zip(expected[1:],actual[1:]):self.assertAlmostEqual(a['geometric_step_translation_m'],b['geometric_step_translation_m'],places=10)

    def test_target_id_order_is_not_a_submap_change(self):
        samples,native=self.fixture();samples[1]['quality']['native_diagnostics']['target_keyframe_ids']=[7,2,4]
        samples[2]['quality']['native_diagnostics']['target_keyframe_ids']=[7,2,5]
        rows=d.correction_rows(samples,native)
        self.assertFalse(rows[1]['target_changed']);self.assertTrue(rows[2]['target_changed'])
        self.assertAlmostEqual(rows[2]['target_jaccard_distance'],.5)

    def test_unconfirmed_pose_never_enters_analysis(self):
        samples,native=self.fixture();samples[1]['pose_valid']=False
        rows=d.correction_rows(samples,native)
        self.assertEqual(len(rows),2);self.assertFalse(rows[1]['consecutive_native_interval'])
        samples[1]['pose_valid']=True;samples[1]['last_confirmed_stamp_ns']-=1
        with self.assertRaisesRegex(ValueError,'exact geometric'):d.correction_rows(samples,native)

    def test_session_attempt_missing_native_and_clock_errors_are_rejected(self):
        samples,native=self.fixture()
        for field in ['session_id','request_generation','map_uuid']:
            changed=deepcopy(samples);changed[1][field]='different'
            with self.assertRaises(ValueError):d.correction_rows(changed,native)
        with self.assertRaisesRegex(ValueError,'missing exact'):d.correction_rows(samples,native[1:])
        with self.assertRaisesRegex(ValueError,'nonmonotonic'):d.correction_rows(samples[::-1],native)

if __name__=='__main__':unittest.main(verbosity=2)
