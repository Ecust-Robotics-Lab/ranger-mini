"""Short real requests must not acquire avoidable steering/mode changes."""
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ws_nav/src/ranger_navigation'))
from ranger_navigation.geometry import Body,validate_sweep
from ranger_navigation.models import Mode,Limits
from ranger_navigation.planner import Grid,Planner,Segment,reanchor_parallel


def test_thirty_centimetre_request_at_arbitrary_yaw_has_one_fixed_axis():
    grid=Grid(np.zeros((120,120),np.int16),.05,(-3,-3,0))
    start=np.array([.1,.2,.134])
    goal=start+np.array([.3*np.cos(start[2]),.3*np.sin(start[2]),0])
    body=Body(clearance=.03)
    segments=Planner(body,static_reserve=.01).plan(grid,start,goal,Mode.DRIVE)
    assert len(segments)==1 and segments[0].mode==Mode.PARALLEL
    assert np.allclose(segments[0].poses,[start,goal])
    assert validate_sweep(segments[0].poses,[],body,grid=grid,map_margin=.04)[0]


def test_stopped_parallel_axis_can_be_reanchored_after_pose_noise():
    grid=Grid(np.zeros((120,120),np.int16),.05,(-3,-3,0))
    body=Body(clearance=.03)
    segment=Segment(Mode.PARALLEL,np.array([[0.,0.,0.],[.3,0.,0.]]))
    actual=np.array([.008,-.009,.006])
    anchored=reanchor_parallel(segment,actual,body,grid,[],.01)
    assert anchored is not None
    assert np.allclose(anchored.poses[0],actual)
    assert np.allclose(anchored.poses[-1],[.3,0.,actual[2]])
    assert np.allclose(segment.poses[0],[0.,0.,0.])


def test_reanchor_cannot_move_verified_axis_through_a_new_obstacle():
    grid=Grid(np.zeros((120,120),np.int16),.05,(-3,-3,0))
    body=Body(clearance=.03)
    segment=Segment(Mode.PARALLEL,np.array([[0.,0.,0.],[1.,0.,0.]]))
    assert reanchor_parallel(segment,np.array([.008,-.009,.006]),body,grid,[[.7,0.]],.01) is None


def test_large_pose_change_requires_a_new_route():
    grid=Grid(np.zeros((120,120),np.int16),.05,(-3,-3,0))
    segment=Segment(Mode.PARALLEL,np.array([[0.,0.,0.],[.3,0.,0.]]))
    for actual in ([0,.04,0],[0,0,.04],[.34,0,0]):
        assert reanchor_parallel(segment,np.array(actual),Body(),grid,[],.01) is None


def test_stopped_progress_along_verified_line_can_resume():
    grid=Grid(np.zeros((120,120),np.int16),.05,(-3,-3,0))
    segment=Segment(Mode.PARALLEL,np.array([[0.,0.,0.],[.3,0.,0.]]))
    anchored=reanchor_parallel(segment,np.array([.18,.005,.003]),Body(),grid,[],.01)
    assert anchored is not None
    assert np.allclose(anchored.poses[0],[.18,.005,.003])
    assert reanchor_parallel(segment,np.array([.18,.005,.003]),Body(),grid,[[.6,0]],.01) is None
