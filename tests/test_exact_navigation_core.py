"""No ROS/CAN: geometry, obstacle visibility, task generations and safety faults."""
from pathlib import Path
import sys
import math
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ws_nav/src/ranger_navigation'))
from ranger_navigation.geometry import Body,load_geometry,integrate,validate_sweep,wrap
from ranger_navigation.models import Mode,Limits,feasible,rollout
from ranger_navigation.planner import Grid,Planner,NoPath,Segment
from ranger_navigation.obstacles import ObstacleMemory
from ranger_navigation.task import Task
from ranger_navigation.safety_core import Safety


def room():
    return Grid(np.zeros((100,120)),.05,(-2.,-2.,0.))


def ring(radius=3.):
    a=np.linspace(-np.pi,np.pi,1440,endpoint=False)
    return np.column_stack((radius*np.cos(a),radius*np.sin(a),np.full(len(a),.4)))


def ready_safety():
    core=Safety()
    core.enabled=core.ready=core.vehicle_ok=core.can_control=True
    core.actual_mode=1
    core.coverage=lambda *args:True
    for key in ('wheel','ready','cloud','mode','system'):core.feed(key,10.,100.)
    core.command('navigation',[.01,.01,0],1,10.,100.)
    return core


def test_shared_geometry_has_actual_rotated_installation():
    body,r,t=load_geometry(ROOT/'config/fast_livo2/navigation_prep.yaml')
    assert np.allclose(body.half,[.37,.26])
    # Camera-facing +X supersedes the former pi-yaw axis. The measured working
    # tilt must still be present; a zero-rotation installation is invalid.
    assert np.linalg.norm(np.cross(r[:,2],[0,0,1]))>.3
    assert np.allclose(r@r.T,np.eye(3))
    p=np.array([[1.,2.,3.]])
    assert np.allclose(((p@r.T+t)-t)@r,p)


@pytest.mark.parametrize('command,mode',[
    ([.2,0,.2],Mode.DRIVE),([-.2,0,-.2],Mode.DRIVE),
    ([0,.2,0],Mode.PARALLEL),([-.1,.1,0],Mode.PARALLEL),
    ([0,0,.4],Mode.SPIN),([0,0,-.4],Mode.SPIN)])
def test_all_supported_motion_commands(command,mode):
    assert feasible(command,mode)
    assert np.isfinite(integrate([0,0,0],command,.1)).all()


@pytest.mark.parametrize('command,mode',[
    ([.1,.1,.1],Mode.PARALLEL),([0,0,.1],Mode.DRIVE),
    ([.1,0,.1],Mode.SPIN),([.3,.3,0],Mode.PARALLEL),
    ([float('nan'),0,0],Mode.DRIVE)])
def test_unexecutable_commands_are_rejected(command,mode):
    assert not feasible(command,mode)


def test_rectangle_detects_rotation_sweep_missed_by_endpoints():
    body=Body(clearance=0.)
    points=np.array([[.43,0.]])
    assert body.distances([0,0,0],points)[0]>0
    assert body.distances([0,0,np.pi/2],points)[0]>0
    okay,_,_=validate_sweep([[0,0,0],[0,0,np.pi/2]],points,body)
    assert not okay


def test_grid_sat_checks_cell_area_not_just_center():
    cells=np.zeros((80,80));cells[40,40]=100
    grid=Grid(cells,.05,(-2.,-2.,0.))
    assert not grid.free([-.36,0.,0.],Body(clearance=0.))
    assert grid.free([-.5,0.,0.],Body(clearance=0.))
    assert not grid.free([-1.9,0,0])


def test_unknown_and_impossible_goal_are_never_straight_line_fallback():
    cells=np.zeros((80,80));cells[:,40:43]=-1
    g=Grid(cells,.05,(-2.,-2.,0.))
    with pytest.raises(NoPath):Planner(timeout=.4).plan(g,[-1,0,0],[1,0,0])
    with pytest.raises(NoPath):Planner().plan(g,[-1,0,0],[0,0,0])


@pytest.mark.parametrize('goal',[[1.,0.,0.],[-.7,0.,0.],[0.,.7,0.],[.6,.6,0.],[0.,0.,np.pi/2]])
def test_lattice_connects_exact_goal_with_contiguous_safe_segments(goal):
    planner=Planner();grid=room();segments=planner.plan(grid,[0,0,0],goal)
    p=np.zeros(3)
    for s in segments:
        assert np.allclose(s.poses[0],p)
        assert planner.edge_free(grid,s.poses)
        if s.mode==Mode.PARALLEL:assert np.max(np.abs(wrap(s.poses[:,2]-s.poses[0,2])))<1e-6
        if s.mode==Mode.SPIN:assert np.allclose(s.poses[:,:2],s.poses[0,:2])
        p=s.poses[-1]
    assert np.allclose(p,goal)


def test_obstacle_memory_empty_cloud_and_sparse_rays_do_not_clear():
    m=ObstacleMemory();m.update(ring(),np.eye(3),np.zeros(3),np.zeros(3),10.)
    before=m.points().copy()
    assert not m.update(np.empty((0,3)),np.eye(3),np.zeros(3),np.zeros(3),10.1)
    assert np.array_equal(m.points(),before)
    assert not m.covers(np.zeros(3),m.body,.04)
    assert not m.update(ring(),np.eye(3),np.zeros(3),np.zeros(3),9.)


def test_occluded_obstacle_retained_and_observed_farther_ray_clears():
    m=ObstacleMemory()
    obstacle=np.array([[1.,0.,.4]])
    m.update(np.vstack((ring(),obstacle)),np.eye(3),np.zeros(3),np.zeros(3),10.)
    assert np.min(np.linalg.norm(m.points()-[1.,0.],axis=1))<.05
    # A nearer occluder hides the old return; it must stay in memory.
    m.update(np.vstack((ring(),[[.6,0,.4]])),np.eye(3),np.zeros(3),np.zeros(3),10.1)
    assert np.min(np.linalg.norm(m.points()-[1.,0.],axis=1))<.05
    m.update(ring(),np.eye(3),np.zeros(3),np.zeros(3),10.2)
    assert np.min(np.linalg.norm(m.points()-[1.,0.],axis=1))>.2


def test_floor_rays_observe_only_until_the_height_band_crossing():
    floor=ring(3.);floor[:,2]=0.
    m=ObstacleMemory()
    assert m.update(floor,np.eye(3),np.zeros(3),np.zeros(3),10.)
    assert len(m.points())==0
    assert np.allclose(m.ranges,2.5)
    assert m.covers(np.zeros(3),m.body,.055)
    assert not m.covers([2.4,0.,0.],m.body,.055)
    # An actual height-band obstacle bounds the farther floor-ray evidence.
    m.update(np.vstack((floor,[[1.,0.,.4]])),np.eye(3),np.zeros(3),np.zeros(3),10.1)
    assert m.ranges[m.bins(np.array([[1.,0.]]))[0]]==1.
    # Empty angular sectors remain unknown even if all other sectors see floor.
    m=ObstacleMemory();assert m.update(floor[floor[:,0]<0],np.eye(3),np.zeros(3),np.zeros(3),10.)
    assert not m.covers([.4,0.,0.],m.body,.055)


def test_spatial_sampling_keeps_circumference_not_only_nearest_points():
    m=ObstacleMemory()
    cloud=np.vstack((ring(1.),ring(3.)))
    assert m.update(cloud,np.eye(3),np.zeros(3),np.zeros(3),1.)
    sampled=m.sampled(360)
    assert len(set(m.bins(sampled)))==180
    assert len(m.points())>len(sampled)
    # Safety independently checks a point omitted by optimization sampling.
    omitted=next(p for p in m.points() if np.min(np.linalg.norm(sampled-p,axis=1))>.001)
    assert not validate_sweep([[*omitted,0]],m.points(),m.body)[0]


def test_obstacle_memory_capacity_never_silently_truncates():
    m=ObstacleMemory(max_cells=3)
    assert not m.update(ring(),np.eye(3),np.zeros(3),np.zeros(3),1.)
    assert m.error=='obstacle_capacity_exceeded'


def test_task_cancel_replacement_and_stale_plan_generation():
    t=Task();t.begin([1,0,0],1.);old=t.generation
    t.cancel();assert t.state=='CANCELED'
    assert not t.plan([],old,2.)
    t.begin([0,1,0],3.);new=t.generation
    assert new>old
    assert not t.plan([],old,3.)
    assert t.plan([Segment(Mode.PARALLEL,np.array([[0,0,0],[0,1,0]]))],new,3.)
    t.tick([0,0,0],[0,0,0],1,False,'navigation',3.1)
    assert t.state=='PAUSED' and t.generation>new and t.segments
    assert not t.plan([],new,3.2)


def test_task_success_requires_continuous_feedback_stop():
    t=Task();t.begin([0,0,0],1.);t.plan([],t.generation,1.)
    t.tick([0,0,0],[0,0,.1],0,True,'navigation',1.1)
    assert t.state!='SUCCEEDED'
    t.tick([0,0,0],[0,0,0],0,True,'navigation',1.2)
    t.tick([0,0,0],[0,0,0],0,True,'navigation',1.69)
    assert t.state!='SUCCEEDED'
    t.tick([0,0,0],[0,0,0],0,True,'navigation',1.71)
    assert t.state=='SUCCEEDED'


def test_mode_switch_never_uses_a_fixed_wait_as_confirmation():
    t=Task();t.begin([0,1,0],1.)
    t.plan([Segment(Mode.PARALLEL,np.array([[0,0,0],[0,1,0]]))],t.generation,1.)
    assert t.tick([0,0,0],[0,0,0],0,True,'navigation',2.) is None
    assert t.state=='SWITCHING'
    assert t.tick([0,0,0],[0,0,0],0,True,'navigation',4.1) is None
    assert t.state=='FAILED' and t.reason=='mode_switch_timeout'


def test_blocked_timeout_and_teleop_takeover_invalidate():
    t=Task();t.begin([1,0,0],0.)
    t.block('no_route',.1);t.block('no_route',15.2);assert t.state=='FAILED'
    t.begin([1,0,0],16.)
    t.tick([0,0,0],[0,0,0],0,True,'teleop',16.1)
    assert t.state=='WAITING'


@pytest.mark.parametrize('stream',['wheel','cloud','ready','mode','system'])
def test_each_stale_or_future_stream_stops_independently(stream):
    for stamp in (9.,11.):
        c=ready_safety();c.feed(stream,stamp,100.)
        command,mode=c.evaluate(10.,100.)
        assert np.max(np.abs(command))==0 and mode is None


def test_gate_default_off_estop_and_command_lease():
    c=ready_safety();c.enabled=False
    assert c.evaluate(10,100)[1] is None
    c.enabled=True;c.estop=True
    assert c.evaluate(10,100)[1] is None
    c.estop=False
    assert np.linalg.norm(c.evaluate(10,100)[0])>0
    assert np.linalg.norm(c.evaluate(10.11,100.11)[0])==0


def test_teleop_controls_velocity_and_mode_together_and_rc_takes_priority():
    c=ready_safety();c.command('teleop',[0,0,.01],2,10.,100.)
    assert c.evaluate(10,100)[0].sum()==0 and c.owner=='teleop'
    c.stop_since=99.;command,request=c.evaluate(10,100)
    assert request==2 and np.max(np.abs(command))==0
    c.can_control=False
    assert c.evaluate(10,100)[1] is None and c.owner=='rc'


def test_final_collision_monitor_can_interrupt_a_valid_command():
    c=ready_safety();command,_=c.evaluate(10,100)
    c.obstacles=np.array([[.38,.1]])
    output,minimum=c.collision(command)
    assert not output.any() and minimum<.1
    assert c.reason.startswith('collision_monitor:')
    c.obstacles=np.empty((0,2));c.coverage=lambda *args:False
    assert not c.collision(command)[0].any()


def test_observed_free_memory_expires_without_derived_self_refresh():
    m=ObstacleMemory();m.update(ring(),np.eye(3),np.zeros(3),np.zeros(3),10.)
    half=ring();half=half[half[:,0]<-.2]
    m.update(half,np.eye(3),np.zeros(3),np.zeros(3),10.1)
    assert m.covers(np.array([.1,0.,0.]),m.body,.095)
    # Historical free evidence may support an emergency stop, but must not
    # attract new motion into a shadow that will expire during the horizon.
    assert not m.control_grid.free([.1,0.,0.],m.body,.055)
    m.update(half,np.eye(3),np.zeros(3),np.zeros(3),11.9)
    m.update(half,np.eye(3),np.zeros(3),np.zeros(3),12.2)
    assert not m.covers(np.array([.1,0.,0.]),m.body,.095)


def test_reprojected_free_cells_agree_with_independent_rectangle_checks():
    cells=np.zeros((80,80));cells[30:55,50:]=-1
    g=Grid(cells,.05,(-2.,-2.,0.));rng=np.random.default_rng(5)
    centers=rng.uniform(-1.,1.,(100,2));yaw=.63
    actual=g.squares_free(centers,yaw)
    expected=[g.free([*p,yaw],Body(length=.05,width=.05,clearance=0.)) for p in centers]
    assert np.array_equal(actual,expected)


def test_dynamic_voxels_do_not_gain_a_second_rasterization_margin():
    body=Body()
    base=Grid(np.zeros((100,100)),.05,(-2.,-2.,0.))
    # The actual body has 12cm clearance to this voxel center; the required
    # point uncertainty + physical clearance + sweep bound is 9.5cm.
    pose=np.zeros(3);point=np.array([[.49,0.]])
    assert validate_sweep([pose],point,body,margin=body.clearance+.04)[0]
    assert base.with_obstacles(point).free(pose,body,body.clearance+.015)
    assert not base.with_obstacles([[.46,0.]]).free(pose,body,body.clearance+.015)
    unknown=base.cells.copy();unknown[40:60,40:60]=-1
    assert not Grid(unknown,.05,(-2.,-2.,0.)).with_obstacles(point).free(pose,body)


def test_fixed_parallel_axis_replans_after_transverse_stopping_error():
    task=Task();task.begin([1.,.5,0.],0.)
    task.plan([Segment(Mode.PARALLEL,np.array([[0.,.5,0.],[1.,.5,0.]]))],task.generation,0.)
    generation=task.generation
    assert task.tick([0.,.46,0.],np.zeros(3),Mode.PARALLEL,True,'navigation',1.) is None
    assert task.state=='PLANNING' and task.reason=='segment_cross_track_replan'
    assert task.generation>generation


def test_legacy_side_slip_start_requires_supported_mode_confirmation():
    segments=Planner().plan(room(),[0.,0.,0.],[.3,0.,0.],start_mode=3)
    assert segments and all(s.mode in (Mode.DRIVE,Mode.PARALLEL,Mode.SPIN) for s in segments)
    task=Task();task.begin([.3,0.,0.],0.)
    task.plan(segments,task.generation,0.)
    assert task.tick(np.zeros(3),np.zeros(3),3,True,'navigation',.1) is None
    assert task.state=='SWITCHING'
