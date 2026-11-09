"""Projection checks cover occlusion, unknown space and source coordinate convention."""
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/fast_livo2/navigation'))
from grid import floor_plane,make_grid,raytrace_free,transform


class GridTests(unittest.TestCase):
    def test_obstacle_stops_free_ray_and_unknown_stays_unknown(self):
        occupied=np.zeros((9,12),bool);occupied[4,5]=True
        free=raytrace_free(occupied,[1,4],[[10,4]])
        self.assertTrue(free[4,4]);self.assertFalse(free[4,5]);self.assertFalse(free[4,9])
        self.assertFalse(free[0,0])

    def test_floor_fit_excludes_ceiling_and_uses_map_coordinates(self):
        rng=np.random.default_rng(3);xy=rng.uniform(-2,2,(2000,2))
        z=.01*xy[:,0]-.005*xy[:,1]-.6+rng.normal(0,.003,len(xy))
        points=np.concatenate([np.column_stack((xy,z)),np.column_stack((xy,z+2.8))])
        plane,quality=floor_plane(points,np.array([[0.,0.,0.]]))
        np.testing.assert_allclose(plane,[.01,-.005,-.6],atol=.002)
        self.assertFalse(quality['field_floor_verified'])

    def test_body_points_transform_once_and_obstacles_win(self):
        pose={'xyz':[10.,20.,.5],'xyzw':[0.,0.,np.sqrt(.5),np.sqrt(.5)]}
        body=np.array([[1.,0.,0.],[2.,0.,-.5]])
        np.testing.assert_allclose(transform(body,pose),[[10,21,.5],[10,22,0]],atol=1e-12)
        values,origin,counts=make_grid([(body,pose)],np.array([0.,0.,0.]),resolution=.1)
        wall=np.floor((np.array([10.,21.])-origin)/.1).astype(int)
        beyond=np.floor((np.array([10.,22.])-origin)/.1).astype(int)
        self.assertEqual(values[wall[1],wall[0]],100)
        self.assertEqual(values[beyond[1],beyond[0]],-1)
        self.assertGreater(counts['unknown'],0)
        self.assertGreater(counts['free'],0)

    def test_repeated_clear_views_remove_transient_but_retain_observed_wall(self):
        pose={'xyz':[0.,0.,.5],'xyzw':[0.,0.,0.,1.]}
        transient=np.array([[1.,0.,0.],[3.,0.,0.]])
        clear=np.array([[3.,0.,0.]])
        values,origin,_=make_grid([(transient,pose)]+[(clear,pose)]*4,np.zeros(3),resolution=.1)
        def cell(x):
            xy=np.floor((np.array([x,0.])-origin)/.1).astype(int)
            return values[xy[1],xy[0]]
        self.assertEqual(cell(1.),0)
        self.assertEqual(cell(3.),100)


if __name__=='__main__':unittest.main(verbosity=2)
