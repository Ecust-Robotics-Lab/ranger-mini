"""Independent optimizer checks: swept SDF, model constraints and signed modes."""
import sys
from pathlib import Path
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ws_nav/src/ranger_navigation'))
from ranger_navigation.geometry import Body
from ranger_navigation.models import Mode,Limits,feasible
from ranger_navigation.exact_controller import ExactController,_project
from exact_mppi.mppi_jax.tools.signed_distance import minimum_signed_distance_from_pose_to_obs_points
import jax.numpy as jnp


def test_narrow_visibility_keeps_full_body_and_hard_coverage_checks():
    from ranger_navigation.planner import Grid
    from ranger_navigation.geometry import validate_sweep
    from ranger_navigation.coverage_policy import CoveragePolicy
    policy=CoveragePolicy()
    body=Body(clearance=.03)
    controller=ExactController(body,samples=128,visibility_reserve=.04)
    cells=np.full((80,80),-1,dtype=np.int16);cells[33:46,:]=0
    pose=np.array([0.,-.025,0.]);path=np.linspace(pose,[.4,-.025,0.],60)
    result=controller.compute(pose,np.zeros(3),path,Mode.PARALLEL,np.empty((0,2)),(cells,np.zeros(3)))
    grid=Grid(cells,.05,(-2.,-2.,0.))
    assert validate_sweep(result['trajectory'],[],body,coverage=lambda p,b,m:grid.covered(p,b,policy),
                          coverage_margin=policy.total_margin)[0]
    assert result['trajectory'][-1,0]>.01
    critic=controller.optimizers[Mode.PARALLEL].critic_manager_
    assert np.allclose(np.array(critic.half),body.half+.025)
    assert np.allclose(np.array(critic.protected_half),body.half+.015)
    assert np.allclose(np.array(critic.reserve_half),body.half+.04)


def test_upstream_signed_distance_agrees_with_independent_geometry():
 rng=np.random.default_rng(44);body=Body();points=rng.uniform(-1,1,(512,2))
 for yaw in [.0,.4,1.4,2.8]:
  pose=np.array([.2,-.1,yaw]);independent=body.distances(pose,points).min()
  value=minimum_signed_distance_from_pose_to_obs_points(jnp.array(pose),jnp.array(points),jnp.ones(512),jnp.zeros((1,2)),jnp.array(body.half[None,:]))
  assert abs(float(value)-independent)<1e-6


@pytest.mark.parametrize('mode,initial,direction',[
 (Mode.DRIVE,[.1,0,.1/.55],[1.,0.]),
 (Mode.DRIVE,[-.1,0,-.1/.55],[1.,0.]),
 (Mode.PARALLEL,[0.,.014,0.],[1.,0.]),
 (Mode.PARALLEL,[.1,.1,0.],[2**-.5,2**-.5]),
 (Mode.SPIN,[0.,0.,.4],[1.,0.])])
def test_candidate_projection_keeps_velocity_acceleration_and_mode_together(mode,initial,direction):
 rng=np.random.default_rng(21);requested=rng.normal(size=(60,3))
 values=np.array(_project(jnp.array(requested),jnp.array(initial),mode,Limits(),.05,jnp.array(direction)))
 assert all(feasible(v,mode,tolerance=1e-5) for v in values)
 differences=np.diff(np.vstack((initial,values)),axis=0)
 assert np.max(np.linalg.norm(differences[:,:2],axis=1))<=.01501
 assert np.max(np.abs(differences[:,2]))<=.02501
 if mode==Mode.PARALLEL:
  assert np.max(np.abs(values[:,0]*direction[1]-values[:,1]*direction[0]))<1e-6


def test_optimized_parallel_axis_and_no_silent_point_truncation():
 c=ExactController(samples=128)
 goal=np.array([-.5,.5,0.]);path=np.linspace(np.zeros(3),goal,60)
 result=c.compute(np.zeros(3),np.zeros(3),path,Mode.PARALLEL,np.empty((0,2)))
 assert np.max(np.abs(result['controls'][:,0]+result['controls'][:,1]))<1e-5
 with pytest.raises(ValueError,match='capacity'):
  c.compute(np.zeros(3),np.zeros(3),path,Mode.PARALLEL,np.zeros((513,2)))


def test_visibility_and_braking_are_part_of_candidate_prediction():
    c=ExactController(samples=128)
    cells=np.zeros((80,80),dtype=np.int16);cells[:,55:]=-1
    path=np.column_stack((np.linspace(0,.6,60),np.zeros((60,2))))
    result=c.compute(np.zeros(3),np.zeros(3),path,Mode.PARALLEL,np.empty((0,2)),(cells,np.zeros(3)))
    from ranger_navigation.planner import Grid
    grid=Grid(cells,.05,(-2.,-2.,0.))
    assert all(grid.free(p,c.body,c.body.clearance+.015) for p in result['trajectory'])
    assert np.max(np.abs(result['controls'][-1]))<1e-5


def test_sparse_direct_guidance_does_not_pull_back_at_a_clear_door():
    from ranger_navigation.geometry import densify,validate_sweep
    from ranger_navigation.planner import Grid
    body=Body(clearance=.03)
    cells=np.zeros((140,160),np.int16);cells[:,74]=100;cells[50:70,74]=0
    grid=Grid(cells,.05,(-3.,-3.,0.))
    points=(np.column_stack(np.nonzero(cells)[::-1])+.5)*.05+[-3.,-3.]
    c=ExactController(body,samples=128,visibility_reserve=.04)
    pose=np.array([.2597,0.,0.]);path=np.array([[0.,0.,0.],[1.5,0.,0.]])
    sparse=c.compute(pose,np.zeros(3),path,Mode.PARALLEL,points)
    c.reset()
    dense=c.compute(pose,np.zeros(3),densify(path,body,.025),Mode.PARALLEL,points)
    assert sparse['command'][0]>0 and dense['command'][0]>0
    assert sparse['trajectory'][-1,0]>pose[0]+.1
    assert validate_sweep(sparse['trajectory'],points,body,grid=grid,
                          margin=.07,map_margin=.04)[0]
