"""Safety revocation, candidate reuse and execution-aware recovery contracts."""
from pathlib import Path
import sys,json
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ws_nav/src/ranger_navigation'))
from ranger_navigation.control_history import ControlHistory
from ranger_navigation.execution import ExecutionFeedback
from ranger_navigation.models import Mode,Limits,rollout
from ranger_navigation.geometry import Body
from ranger_navigation.planner import Segment
from ranger_navigation.recovery import resume_reason,command_live
from ranger_navigation.task import Task
from ranger_navigation.safety_core import Safety


def history():
    h=ControlHistory();u=np.column_stack((np.linspace(.03,.1,60),np.zeros((60,2))))
    trajectory=rollout([0,0,0],u,.05);h.remember(1,10.,u,trajectory)
    return h,u,trajectory


def test_unacknowledged_or_unexecuted_results_do_not_advance_history():
    h,u,p=history()
    assert h.select(None,10.2,p[0],True) is None
    # A candidate can be accepted by geometry, yet stopped by steering control.
    a=h.select(1,10.2,p[0],True)
    np.testing.assert_array_equal(a,u)
    assert h.diagnostic['shift_steps']==0
    a=h.select(1,10.2,p[3],True)
    assert h.diagnostic['shift_steps']==3
    np.testing.assert_array_equal(a[:-3],u[3:])
    assert np.all(a[-3:]==0)
    a=h.select(1,10.2,p[3],True,steering_wait=True)
    assert h.diagnostic['shift_steps']==0


def test_only_validated_ack_is_promoted_and_survives_unaccepted_solves():
    h,u,p=history();h.select(1,10.1,p[0],True)
    for i in range(2,15):h.remember(i,10.1,np.zeros_like(u),np.zeros_like(p))
    np.testing.assert_array_equal(h.select(1,10.2,p[0],True),u)
    assert h.select(999,10.2,p[0],True) is None


@pytest.mark.parametrize('stamp,pose,ready,reason',[
    (9.99,[0,0,0],True,'history_expired_or_clock_regressed'),
    (11.01,[0,0,0],True,'history_expired_or_clock_regressed'),
    (10.1,[0,0,0],False,'execution_feedback_unavailable'),
    (10.1,[0,.11,0],True,'measured_pose_disagrees_with_history'),
    (10.1,[0,0,.11],True,'measured_pose_disagrees_with_history')])
def test_invalid_history_is_never_a_warm_start(stamp,pose,ready,reason):
    h,_,_=history();assert h.select(1,stamp,pose,ready) is None
    assert h.diagnostic['reason']==reason


def test_new_route_discards_all_prior_acknowledgements():
    h,u,p=history();h.select(1,10.1,p[0],True);h.reset()
    assert h.select(1,10.2,p[0],True) is None


def task():
    t=Task();t.begin([1,0,0],0.)
    t.plan([Segment(Mode.PARALLEL,np.array([[0.,0.,0.],[1.,0.,0.]]))],t.generation,0.)
    t.tick([0,0,0],[0,0,0],Mode.PARALLEL,True,'navigation',0.)
    return t


def test_soft_pause_revokes_output_but_does_not_destroy_route_history():
    t=task();generation,history=t.generation,t.history_generation
    t.pause_inputs('inputs_invalid',.1)
    assert t.generation==generation+1 and t.history_generation==history
    assert not t.plan([],generation,.15)
    t.invalidate('localization_session_changed')
    assert t.history_generation==history+1 and not t.segments


def test_steering_wait_does_not_trigger_global_replan_and_has_its_own_timeout():
    t=task();generation=t.generation
    for i in range(1,30):
        t.tick([0,0,0],[0,0,0],Mode.PARALLEL,True,'navigation',i*.1,True)
        assert t.state=='EXECUTING' and t.generation==generation
    t.tick([0,0,0],[0,0,0],Mode.PARALLEL,True,'navigation',3.2,True)
    assert t.state=='FAILED' and t.reason=='steering_execution_timeout'


def test_motionless_tracking_still_has_goal_wide_progress_timeout():
    t=task()
    t.tick([0,0,0],[0,0,0],Mode.PARALLEL,True,'navigation',15.1)
    assert t.state=='FAILED' and t.reason=='no_progress_timeout'


def test_same_mode_on_route_motion_can_be_revalidated_without_forcing_stop():
    s=Segment(Mode.PARALLEL,np.array([[0.,0.,0.],[1.,0.,0.]]))
    assert resume_reason(s,np.array([.2,0,0]),[.08,0,0],Mode.PARALLEL,Body(),Limits())=='resume_from_measured_motion'
    assert resume_reason(s,np.array([.2,.05,0]),[.08,0,0],Mode.PARALLEL,Body(),Limits()).startswith('awaiting_stop')
    assert resume_reason(s,np.array([.2,0,0]),[.08,.02,0],Mode.PARALLEL,Body(),Limits())=='awaiting_stop_cross_axis_velocity'
    assert resume_reason(s,np.array([.2,0,0]),[.08,0,0],Mode.DRIVE,Body(),Limits())=='awaiting_actual_mode'


def report(session='driver',reason='tracking'):
    return dict(schema_version=1,session=session,host_stamp_ns=100_000_000_000,requested_body=[.1,0,0],
                submitted_sdk=[-.1,0,0],feedback_ready=True,receive_only=False,mode=1,reason=reason,
                command_age_s=.02,submission_age_s=.02,feedback_age_s=.02,mode_age_s=.02,system_age_s=.02)


def test_execution_feedback_preserves_host_sample_age_and_rejects_retired_sessions():
    e=ExecutionFeedback();r=report()
    assert e.update(json.dumps(r),100.02,5.)=='accepted'
    assert e.ready(5.04,Mode.PARALLEL) and not e.ready(5.04,Mode.DRIVE)
    assert not e.ready(5.26,Mode.PARALLEL)
    r=report('new');assert e.update(json.dumps(r),100.03,5.1)=='session_changed'
    assert e.update(json.dumps(report()),100.04,5.11)=='retired'
    assert e.session=='new'
    assert e.update(json.dumps(report('new')),100.5,5.2)=='invalid'
    assert not e.ready(5.2,Mode.PARALLEL)


def test_steering_wait_comes_from_fresh_actual_driver_feedback():
    e=ExecutionFeedback();assert e.update(json.dumps(report(reason='steering_alignment')),100.,5.)=='accepted'
    assert e.waiting(5.04,1)
    assert not e.waiting(5.3,1)
    r=report(reason='command_watchdog');e.update(json.dumps(r),100.,5.)
    assert not e.ready(5.,1)


def test_command_heartbeat_cannot_extend_original_lease():
    s=Safety();s.enabled=s.ready=s.vehicle_ok=s.can_control=True;s.actual_mode=1
    s.coverage=lambda *args:True
    for name in ['ready','wheel','mode','system','cloud']:s.feed(name,10.,10.)
    s.command('navigation',[.01,0,0],1,10.,10.,deadline=10.05)
    assert s.evaluate(10.01,10.01)[0][0]>.0
    # Same request repeated with a fresh receipt, original deadline unchanged.
    s.command('navigation',[.01,0,0],1,10.,10.049,deadline=10.05)
    assert s.evaluate(10.051,10.051)[0][0]==0
    assert s.reason=='command_expired_or_invalid'
    command=({'generation':3,'submitted':10.},np.array([.1,0,0]),10_000_000_000)
    assert command_live(command,3,10.09,10.09)
    assert not command_live(command,3,10.101,10.101)
    assert not command_live(command,4,10.05,10.05)


def test_no_progress_rechecks_execution_without_replanning_or_repeated_revocation():
    t=task();generation,history=t.generation,t.history_generation;segment=t.segments[0]
    assert t.tick([0,0,0],[0,0,0],Mode.PARALLEL,True,'navigation',3.1) is segment
    assert t.state=='OBSERVING' and t.reason=='execution_progress_recheck'
    assert t.generation==generation+1 and t.history_generation==history
    t.state='EXECUTING'
    t.tick([0,0,0],[0,0,0],Mode.PARALLEL,True,'navigation',3.2)
    assert t.state=='EXECUTING' and t.generation==generation+1
    t.tick([0,0,0],[0,0,0],Mode.PARALLEL,True,'navigation',15.1)
    assert t.state=='FAILED' and t.reason=='no_progress_timeout'
