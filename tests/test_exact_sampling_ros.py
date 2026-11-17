"""Early DDS arrival must wait for sample time, never poison valid history."""
from pathlib import Path
import sys,time
from types import SimpleNamespace
import numpy as np
import pytest
pytest.importorskip('rclpy')
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'ws_nav/src/ranger_navigation'),str(ROOT/'ws_nav/src/ranger_localization_bridge')]
from ranger_navigation.coordinator import Coordinator
from ranger_navigation.geometry import Body
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from ranger_localization_msgs.msg import LocalizationEstimate


def wheel(stamp):
    m=Odometry();m.header.stamp.sec=int(stamp);m.header.stamp.nanosec=round((stamp-int(stamp))*1e9)
    m.pose.pose.orientation.w=1.
    return m


def test_future_cloud_waits_before_transform_lookup():
    looked=[]
    class Harness:
        process_cloud=Coordinator.process_cloud
        front_locked=False
        ns=10_000_000_000
        def get_clock(self):return SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=self.ns))
        def now(self):return self.ns*1e-9
    h=Harness();future=10_004_000_000;cloud=object()
    h.clouds={future:cloud};h.fast_stamps={future:time.monotonic()}
    h.history=SimpleNamespace(lookup=lambda ns:(looked.append(ns) or (None,'waiting for odometry bracket')))
    h.process_cloud();assert not looked and h.clouds[future] is cloud
    h.ns=future;h.process_cloud();assert looked==[future] and h.clouds[future] is cloud


def test_future_wheel_does_not_hide_recent_exact_tf_sample():
    now=time.monotonic();queries=[];old,new=wheel(9.98),wheel(10.02)
    transform=TransformStamped();transform.transform.rotation.w=1.
    class Harness:
        sample=Coordinator.sample
        def now(self):return self.clock
        def invalidate(self,*args):raise AssertionError('unexpected invalidation')
    h=Harness();h.clock=10.;h.ready=h.safety_ready=True;h.front_locked=False
    h.wheel=new;h.wheel_samples=[old,new];h.grid=object();h.expected_map=''
    h.estimate=LocalizationEstimate();h.estimate.state=h.estimate.VALID;h.estimate.header.frame_id='map';h.estimate.base_frame='base_link';h.estimate.header.stamp.sec=9;h.estimate.header.stamp.nanosec=900_000_000
    for key in ('ready','wheel','cloud','estimate','owner','actual_mode','safety'):
        setattr(h,key+'_received',now)
    h.memory=SimpleNamespace(error=None,last_stamp=9.9)
    h.buffer=SimpleNamespace(lookup_transform=lambda a,b,t:(queries.append(t.nanoseconds) or transform))
    assert h.sample(now) is not None
    assert h.reference_wheel is old and queries==[9_980_000_000]
    # Keeping history must not extend its validity after the original bound.
    h.clock=10.4
    assert h.sample(now) is None


def test_fault_without_session_does_not_retire_active_localizer():
    from types import SimpleNamespace
    from ranger_navigation.coordinator import Coordinator
    invalidations=[]
    node=SimpleNamespace(loc_session='active',retired_loc=set(),
                         invalidate=lambda reason,**kw:invalidations.append(reason))
    Coordinator.on_estimate(node,SimpleNamespace(reason='localization lost',state=3,VALID=1,DEGRADED=2))
    assert node.loc_session=='active' and not node.retired_loc
    assert invalidations==['localization_invalid']
    recovered=SimpleNamespace(reason='map_match_confirmed; session=active',state=1,VALID=1)
    Coordinator.on_estimate(node,recovered)
    assert node.estimate is recovered
    replacement=SimpleNamespace(reason='map_match_confirmed; session=new',state=1,VALID=1)
    Coordinator.on_estimate(node,replacement)
    Coordinator.on_estimate(node,recovered)
    assert node.estimate is replacement and 'active' in node.retired_loc


def scheduler_harness(monkeypatch):
    import ranger_navigation.coordinator as module
    from ranger_navigation.models import Mode,Limits
    from ranger_navigation.planner import Segment,Grid
    from ranger_navigation.task import Task
    clock=[100.]
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    segment=Segment(Mode.PARALLEL,np.array([[0.,0.,0.],[.5,0.,0.]]))
    sent=[];submitted=[];sampled=[];polls=[]
    task=SimpleNamespace(state='EXECUTING',reason='tracking',index=0,segments=[segment],generation=1,
                         goal=np.array([.5,0.,0.]),TERMINAL=Task.TERMINAL,tick=lambda *args:segment)
    def hold(reason='guidance_unobserved'):task.state,task.reason='OBSERVING',reason
    task.hold_observation=hold
    memory=SimpleNamespace(last_stamp=9.9,visibility_pose=np.zeros(3),
                           points=lambda:np.empty((0,2)),sampled=lambda n:(sampled.append(n) or np.empty((0,2))),
                           control_grid=Grid(np.zeros((80,80)),.05,(-2.,-2.,0.)))
    class Harness:
        tick=Coordinator.tick
        now=lambda self:10.
        process_cloud=lambda self:None
        sample=lambda self,now:(np.zeros(3),np.zeros(3),np.zeros(3))
    h=Harness();h.worker=SimpleNamespace(pending={'kind':'control'},status=lambda:{'ready':True},
                        poll=lambda:(polls.append(clock[0]) or None),submit=lambda *args,**kw:(submitted.append((args,kw)) or True))
    h.task=task;h.memory=memory;h.body=Body();h.limits=Limits();h.grid=memory.control_grid
    h.last_clock=None;h.last_path_check=(9.9,1,0);h.command=None
    h.last_plan=h.last_control=h.last_state_publish=-1e9
    h.actual_mode=Mode.PARALLEL;h.owner='navigation';h.reference_wheel=wheel(9.98)
    h.session='offline';h.metrics={};h.goal_handle=None;h.input_reason='valid';h.static_reserve=.01
    pub=SimpleNamespace(publish=lambda m:sent.append(m))
    for name in ['command_pub','velocity_pub','state_pub','local_pub','minimum_pub','path_pub']:setattr(h,name,pub)
    return h,clock,sent,submitted,sampled,polls


def test_busy_worker_does_not_prepare_inputs_but_still_polls_and_withdraws(monkeypatch):
    h,clock,sent,submitted,sampled,polls=scheduler_harness(monkeypatch)
    h.tick()
    assert polls and sent and not submitted and not sampled
    h.sample=lambda now:None
    h.command=object()
    clock[0]+=.005;h.tick()
    assert h.command is None and not sent[-2].valid and len(polls)==2


def test_available_worker_prepares_one_control_at_twenty_hertz(monkeypatch):
    h,clock,sent,submitted,sampled,polls=scheduler_harness(monkeypatch)
    h.worker.pending=None
    h.tick()
    assert len(submitted)==len(sampled)==1 and submitted[0][0][0]=='control'
    clock[0]+=.005;h.tick()
    assert len(submitted)==len(sampled)==1 and len(polls)==2
    clock[0]+=.05;h.tick()
    assert len(submitted)==len(sampled)==2


def test_missing_coverage_rechecks_at_control_rate_without_sampling_obstacles(monkeypatch):
    h,clock,sent,submitted,sampled,polls=scheduler_harness(monkeypatch)
    h.worker.pending=None;checks=[]
    h.memory.control_grid.free=lambda *args:(checks.append(clock[0]) and False)
    h.tick()
    count=len(checks)
    assert count and h.task.state=='OBSERVING' and not sampled and not submitted
    clock[0]+=.005;h.tick()
    assert len(checks)==count and len(polls)==2
    clock[0]+=.05;h.tick()
    assert len(checks)>count and not sampled


def test_feedback_rate_limit_does_not_delay_cancel_result(monkeypatch):
    h,clock,sent,submitted,sampled,polls=scheduler_harness(monkeypatch)
    feedback=[];finished=[]
    h.goal_handle=SimpleNamespace(publish_feedback=lambda m:feedback.append(m))
    h.finish=lambda state:finished.append(state)
    h.tick();clock[0]+=.005;h.tick()
    assert len(feedback)==1
    h.task.state='CANCELED'
    clock[0]+=.005;h.tick()
    assert finished==['CANCELED'] and len(feedback)==1
