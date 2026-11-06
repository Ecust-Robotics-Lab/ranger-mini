#!/usr/bin/env python3
"""Independent analytic motion fixtures for offline raw-gyro timing diagnostics."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('gyro_diag',ROOT/'scripts/fast_livo2/tags/analyze_imu_timing.py')
tool=importlib.util.module_from_spec(spec);spec.loader.exec_module(tool)
EPOCH=1590199535300312660


def hypothesis(R, identity):
    return {'hypothesis_id':identity,'T_camera_tag':{'xyz':[.2,-.1,1.2],'xyzw':Rotation.from_matrix(R).as_quat().tolist()}}


class Timing(unittest.TestCase):
    def test_three_axis_relative_rotation_direction_with_nonidentity_extrinsic(self):
        t=np.arange(0,2.001,.001)
        roll=.2*np.sin(2*t);pitch=.15*np.cos(3*t);yaw=.4*t*t
        rd=.4*np.cos(2*t);pd=-.45*np.sin(3*t);yd=.8*t
        rates=np.column_stack([rd-yd*np.sin(pitch),pd*np.cos(roll)+yd*np.sin(roll)*np.cos(pitch),
                               -pd*np.sin(roll)+yd*np.cos(roll)*np.cos(pitch)])
        history=tool.GyroHistory(EPOCH+np.rint(t*1e9).astype(np.int64),rates)
        E=Rotation.from_euler('xyz',[.2,-.3,.5]).as_matrix()
        a,b=301,1745
        actual=history.relative([EPOCH+a*1000000],[EPOCH+b*1000000],E)[0]
        world=Rotation.from_euler('xyz',np.column_stack([roll,pitch,yaw])).as_matrix()
        expected=E.T@world[a].T@world[b]@E
        np.testing.assert_allclose(actual,expected,atol=2e-6)
        fixed=Rotation.from_euler('xyz',[-.5,.1,.7]).as_matrix()
        ca=(world[a]@E).T@fixed;cb=(world[b]@E).T@fixed
        candidates,identities=tool.relative_candidates([hypothesis(ca,7)],[hypothesis(cb,9)])
        np.testing.assert_allclose(candidates[0],expected,atol=1e-12)
        self.assertEqual(identities,[[7,9]])

    def test_known_positive_shift_recovered_with_changing_angular_rate(self):
        axis=np.array([.2,-.4,.6]);axis/=np.linalg.norm(axis)
        t=np.arange(0,8.001,.002)
        angle=lambda t:-.3/2*np.cos(2*t)-.2/5*np.cos(5*t)
        rates=(.3*np.sin(2*t)+.2*np.sin(5*t))[:,None]*axis
        history=tool.GyroHistory(EPOCH+np.rint(t*1e9).astype(np.int64),rates)
        E=Rotation.from_euler('xyz',[.1,-.4,.7]).as_matrix()
        fixed=Rotation.from_euler('xyz',[-.1,.2,.3]).as_matrix()
        pairs=[]
        for i,a in enumerate(np.arange(.6,6.6,.3)):
            b=a+.3
            ca=(Rotation.from_rotvec(angle(a+.1)*axis).as_matrix()@E).T@fixed
            cb=(Rotation.from_rotvec(angle(b+.1)*axis).as_matrix()@E).T@fixed
            candidates,_=tool.relative_candidates([hypothesis(ca,0)],[hypothesis(cb,0)])
            pairs.append({'pair_id':str(i),'start_ns':EPOCH+round(a*1e9),'end_ns':EPOCH+round(b*1e9),
                'candidates_padded':np.repeat(candidates,4,axis=0).tolist(),'candidate_valid':[True,False,False,False],
                'primary_combination':0,'rotation_rich':True,'split':'holdout' if i%3==2 else 'train'})
        _,profile=tool.profile(history,pairs,E,[-100000000,0,50000000,100000000,150000000,200000000])
        best=min(profile,key=lambda r:r['subsets']['train']['primary_clipped_mean_deg'])
        self.assertEqual(best['offset_ns'],100000000)
        self.assertLess(best['subsets']['holdout']['primary_error_deg']['maximum'],.001)
        self.assertGreater(profile[1]['subsets']['holdout']['primary_error_deg']['p50'],.1)
        self.assertFalse(best['offset_is_calibrated'])

    def test_two_branches_each_preserved_as_four_combinations(self):
        R=Rotation.from_euler('xyz',[.2,.3,.4]).as_matrix()
        S=Rotation.from_euler('xyz',[-.2,.3,.1]).as_matrix()
        candidates,ids=tool.relative_candidates([hypothesis(R,0),hypothesis(S,1)],
                                               [hypothesis(S,2),hypothesis(R,3)])
        self.assertEqual(ids,[[0,2],[0,3],[1,2],[1,3]])
        self.assertEqual(candidates.shape,(4,3,3))
        np.testing.assert_allclose(candidates[1],np.eye(3),atol=1e-12)

    def test_gaps_no_extrapolation_duplicate_and_nonfinite_rejected(self):
        stamps=np.array([EPOCH,EPOCH+5000000,EPOCH+50000000,EPOCH+55000000])
        history=tool.GyroHistory(stamps,np.zeros((4,3)))
        self.assertEqual(history.supported([EPOCH,EPOCH+5000000,EPOCH-1],
                                          [EPOCH+5000000,EPOCH+50000000,EPOCH+1]).tolist(),[True,False,False])
        with self.assertRaisesRegex(ValueError,'no bracket'):
            history.relative([EPOCH],[EPOCH+55000000],np.eye(3))
        with self.assertRaises(ValueError):tool.GyroHistory([EPOCH,EPOCH],np.zeros((2,3)))
        with self.assertRaises(ValueError):tool.GyroHistory([EPOCH,EPOCH+1],[[0,0,0],[np.nan,0,0]])

    def test_constant_angular_rate_does_not_identify_time_offset(self):
        stamps=EPOCH+np.arange(1001,dtype=np.int64)*2000000
        history=tool.GyroHistory(stamps,np.tile([.2,-.1,.3],(1001,1)))
        predicted=Rotation.from_rotvec(np.array([.2,-.1,.3])*.3).as_matrix()
        pairs=[{'pair_id':'constant','start_ns':EPOCH+500000000,'end_ns':EPOCH+800000000,
                'candidates_padded':np.tile(predicted,(4,1,1)).tolist(),'candidate_valid':[True,False,False,False],
                'primary_combination':0,'rotation_rich':True,'split':'train'}]
        _,scores=tool.profile(history,pairs,np.eye(3),[-100000000,0,100000000])
        self.assertLess(np.ptp([r['subsets']['train']['primary_clipped_mean_deg'] for r in scores]),1e-10)

    def test_static_bias_uses_image_stability_and_training_only(self):
        stamps=EPOCH+np.arange(10001,dtype=np.int64)*5000000
        bias=np.array([.03,-.01,.02]);rates=np.tile(bias,(len(stamps),1))
        rates[stamps>=EPOCH+40000000000]+=[.01,.02,.03]
        history=tool.GyroHistory(stamps,rates)
        corners=np.array([[10.,20.],[30.,20.],[30.,40.],[10.,40.]])
        rows=[]
        for i in range(500):
            # Stable first and final 5 s; intermediate pixels move despite flat gyro.
            c=corners if i<50 or i>=450 else corners+i
            rows.append({'stamp_ns':EPOCH+i*100000000,'rejection_reasons':[],
                         'observation':{'corners':c.tolist()}})
        result=tool.image_static_bias(rows,history,EPOCH,50)
        np.testing.assert_allclose(result['bias_radps'],bias,atol=1e-12)
        self.assertEqual([(b['start_s'],b['split']) for b in result['blocks']],[(0,'train'),(45,'holdout')])
        self.assertFalse(result['runtime_bias_changed'])


if __name__=='__main__':unittest.main(verbosity=2)
