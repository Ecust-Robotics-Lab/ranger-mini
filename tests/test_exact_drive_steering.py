"""Drive predictions must match a fixed physical steering position."""
import sys
from pathlib import Path

import numpy as np
import pytest
import jax.numpy as jnp

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ws_nav/src/ranger_navigation'))
from ranger_navigation.exact_controller import ExactController,_project
from ranger_navigation.geometry import Body,integrate,validate_sweep
from ranger_navigation.models import Limits,Mode,feasible


@pytest.mark.parametrize('sign,curvature',[(1.,1/.55),(1.,-1/.55),(-1.,1/.55),(-1.,-1/.55)])
def test_noisy_drive_candidates_keep_steering_and_signed_arc(sign,curvature):
    limits=Limits(speed=.1,yaw_rate=.2)
    requested=np.random.default_rng(61).normal(size=(60,3))
    commands=np.array(_project(jnp.array(requested),jnp.zeros(3),Mode.DRIVE,limits,.05,
                              jnp.array([1.,0.]),jnp.array([curvature,sign])))
    assert np.all(sign*commands[:,0]>=-1e-7)
    assert np.max(abs(commands[:,2]-curvature*commands[:,0]))<1e-7
    assert all(feasible(v,Mode.DRIVE,limits) for v in commands)
    differences=np.diff(np.vstack((np.zeros(3),commands)),axis=0)
    assert np.max(abs(differences[:,0]))<=.01501
    assert np.max(abs(differences[:,2]))<=.02501
    assert np.max(abs(commands[-1]))<1e-7


@pytest.mark.parametrize('sign,curvature',[(1.,1/.55),(-1.,-1/.55)])
def test_optimized_drive_moves_on_arc_and_finishes_stopped(sign,curvature):
    limits=Limits(speed=.1,yaw_rate=.2)
    path=np.array([integrate(np.zeros(3),[sign,0.,sign*curvature],s) for s in np.linspace(0,.45,31)])
    controller=ExactController(Body(clearance=.03),limits,samples=128,visibility_reserve=.04)
    result=controller.compute(np.zeros(3),np.zeros(3),path,Mode.DRIVE,np.empty((0,2)))
    commands=result['controls']
    assert sign*result['command'][0]>.001
    assert np.max(abs(commands[:,2]-curvature*commands[:,0]))<1e-5
    assert sign*result['trajectory'][-1,0]>.04
    assert np.max(abs(commands[-1]))<1e-6
    assert validate_sweep(result['trajectory'],[],controller.body)[0]


def test_changing_drive_curvature_requires_stopped_segments():
    controller=ExactController(samples=128)
    a=np.array([integrate(np.zeros(3),[1.,0.,1/.55],s) for s in np.linspace(0,.15,7)])
    b=np.array([integrate(a[-1],[1.,0.,-1/.55],s) for s in np.linspace(0,.15,7)])
    with pytest.raises(ValueError,match='drive_guidance_changes_curvature'):
        controller.compute(np.zeros(3),np.zeros(3),np.vstack((a,b[1:])),Mode.DRIVE,[])


def test_drive_closed_loop_retains_one_steering_angle_across_updates():
    limits=Limits(speed=.1,yaw_rate=.2)
    curvature=1/.55
    path=np.array([integrate(np.zeros(3),[1.,0.,curvature],s) for s in np.linspace(0,.45,31)])
    controller=ExactController(Body(clearance=.03),limits,samples=128,visibility_reserve=.04)
    pose=np.zeros(3);speed=np.zeros(3)
    for _ in range(100):
        result=controller.compute(pose,speed,path,Mode.DRIVE,[])
        speed=result['command']
        assert speed[0]>=-1e-7 and abs(speed[2]-curvature*speed[0])<1e-5
        pose=integrate(pose,speed,.05)
    assert pose[0]>.15 and pose[2]>.2


@pytest.mark.parametrize('mode,sign',[(Mode.PARALLEL,1.),(Mode.SPIN,1.),(Mode.SPIN,-1.)])
def test_fixed_axis_segments_cannot_average_opposite_motion(mode,sign):
    requested=np.random.default_rng(9).normal(size=(60,3))
    commands=np.array(_project(jnp.array(requested),jnp.zeros(3),mode,Limits(),.05,
                              jnp.array([1.,0.]),jnp.array([0.,sign])))
    axis=commands[:,2] if mode==Mode.SPIN else commands[:,0]
    assert np.all(sign*axis>=-1e-7)
    assert np.max(abs(commands[-1]))<1e-7
