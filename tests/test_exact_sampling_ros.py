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
