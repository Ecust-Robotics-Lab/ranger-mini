"""Known-motion checks for offline clock, frame and feedback diagnostics."""
from pathlib import Path
import sys
import unittest
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/fast_livo2/navigation'))
from analyze_wheel_alignment import bracket_count,diagnostic_feedback,fit_lag,median_ns,rotation_candidate,spin_axis


class AlignmentTests(unittest.TestCase):
    def test_integer_epoch_offset_and_no_extrapolation(self):
        epoch=203_782_752_741_349_211
        self.assertEqual(median_ns(np.array([epoch+2,epoch,epoch-2],dtype=np.int64)),epoch)
        source=np.array([100,200,300],dtype=np.int64)
        result=bracket_count(source,np.array([50,150,350],dtype=np.int64))
        self.assertEqual(result['bracketed_without_extrapolation'],1)
        self.assertEqual(bracket_count(source,np.array([100,300],dtype=np.int64))['bracketed_without_extrapolation'],2)
        self.assertEqual(bracket_count(np.array([0,200_000_000]),np.array([100_000_000]))['bracketed_without_extrapolation'],0)

    def test_known_delay_with_gain_and_bias(self):
        t=np.arange(0.,30.,.01)
        def f(t):return np.sin(.7*t)+.3*np.cos(2.3*t)
        result=fit_lag(t,1.2*f(t+.042)+.03,t,f(t),2.,28.)
        self.assertAlmostEqual(result['lag_seconds'],.042,places=3)
        self.assertAlmostEqual(result['gain'],1.2,places=4)
        self.assertAlmostEqual(result['bias_rad_s'],.03,places=4)

    def test_moving_reflection_preserves_spin_direction(self):
        values=np.zeros((4,13));values[:,:2]=[[0,0],[1,1],[2,2],[2,2]]
        values[:,3:7]=Rotation.from_euler('z',[0,.1,.2,.4]).as_quat()
        values[:3,7]=1;values[:,-1]=[.1,.1,.1,.2]
        xy,yaw,rate=diagnostic_feedback(values)
        np.testing.assert_allclose(xy,[[0,0],[1,-1],[2,-2],[2,-2]],atol=1e-12)
        np.testing.assert_allclose(yaw,[0,-.1,-.2,0],atol=1e-12)
        np.testing.assert_allclose(rate,[-.1,-.1,-.1,.2],atol=1e-12)

    def test_known_spin_axis_is_not_assumed_to_be_sensor_z(self):
        t=np.arange(0,20,.02);wheel=np.zeros((len(t),13));wheel[:,-1]=.8*np.sin(t)
        rbi=Rotation.from_euler('xyz',[.1,.4,-2.])
        axis=rbi.inv().apply([0.,0.,1.])
        gyro=1.02*wheel[:,-1,None]*axis+np.array([.01,-.02,.03])
        estimated,quality=spin_axis(t,gyro,t,wheel)
        np.testing.assert_allclose(estimated,axis,atol=1e-12)
        self.assertAlmostEqual(quality['gyro_per_wheel_spin_scale'],1.02)

    def test_signed_straight_motion_resolves_forward_axis(self):
        t=np.arange(0,20,.1);wheel=np.zeros((len(t),13));wheel[:,7]=-.4
        rbi=Rotation.from_euler('xyz',[.12,.3,-2.4])
        rwb=Rotation.from_euler('z',.2);rwi=rwb*rbi
        fast=np.zeros((len(t),13));fast[:,:3]=t[:,None]*rwb.apply([-.4,0,0])
        fast[:,3:7]=rwi.as_quat()
        axis=rbi.inv().apply([0.,0.,1.])
        result,_=rotation_candidate(t,fast,t,wheel,axis,1.,19.)
        np.testing.assert_allclose(result,rbi.as_matrix(),atol=1e-12)


if __name__=='__main__':unittest.main(verbosity=2)
