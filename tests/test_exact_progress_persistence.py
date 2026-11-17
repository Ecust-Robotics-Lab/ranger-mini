"""Repeated planning/confirmation must not renew an immobile goal forever."""
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ws_nav/src/ranger_navigation'))
from ranger_navigation.task import Task
from ranger_navigation.planner import Segment
from ranger_navigation.models import Mode


def route():
    return [Segment(Mode.PARALLEL,np.array([[0.,0.,0.],[.3,0.,0.]]))]


def test_repeated_replans_and_confirmations_cannot_hide_a_stall():
    t=Task();t.begin([.3,0,0],0.)
    for i in range(181):
        now=i*.1
        if i%10==0:
            t.plan(route(),t.generation,now)
        t.tick([0,0,0],[0,0,0],Mode.PARALLEL,True,'navigation',now)
        if t.state=='FAILED':break
    assert t.state=='FAILED' and t.reason=='no_progress_timeout'
    assert 15.<=now<=15.2


def test_invalid_inputs_pause_budget_but_do_not_erase_prior_stall():
    t=Task();t.begin([.3,0,0],0.)
    for i in range(251):
        now=i*.1;healthy=not 5.<=now<15.
        t.tick([0,0,0],[0,0,0],Mode.PARALLEL,healthy,'navigation',now)
    assert t.state!='FAILED'
    t.tick([0,0,0],[0,0,0],Mode.PARALLEL,True,'navigation',25.3)
    assert t.reason=='no_progress_timeout'


def test_real_translation_renews_progress_but_small_jitter_does_not():
    t=Task();t.begin([2.,0,0],0.)
    for i in range(301):
        now=i*.1
        t.tick([.04*(i//40),.003*np.sin(i),.02*np.sin(i)],
               [0,0,0],Mode.PARALLEL,True,'navigation',now)
    assert t.state!='FAILED'
    assert t.stall_elapsed<4.1


def test_spin_progress_counts_rotation_without_translation():
    t=Task();t.begin([0,0,2.],0.)
    t.plan([Segment(Mode.SPIN,np.array([[0.,0.,0.],[0.,0.,2.]]))],t.generation,0.)
    for i in range(201):
        t.tick([0,0,max(0,i-6)*.005],[0,0,.05 if i>6 else 0.],
               Mode.SPIN,True,'navigation',i*.1)
    assert t.state!='FAILED'
    assert t.stall_elapsed<2.2


def test_missing_observation_preserves_route_mode_and_stall_budget():
    t=Task();t.begin([.3,0,0],0.)
    t.plan(route(),t.generation,0.)
    generation=t.generation
    t.hold_observation()
    for i in range(150):
        segment=t.tick([0,0,0],[0,0,0],Mode.PARALLEL,True,'navigation',i*.1)
        assert segment is t.segments[0]
        assert t.state=='OBSERVING' and t.generation==generation
    t.tick([0,0,0],[0,0,0],Mode.PARALLEL,True,'navigation',15.1)
    assert t.state=='FAILED' and t.reason=='no_progress_timeout'


def test_final_goal_uses_agreed_tolerance_and_one_stopped_dwell():
    t=Task();t.begin([.3,0,0],0.)
    t.plan(route(),t.generation,0.)
    t.tick([.2,0,.08],[0,0,0],Mode.PARALLEL,True,'navigation',0.)
    assert t.state=='STOPPING'
    t.tick([.2,0,.08],[0,0,0],Mode.PARALLEL,True,'navigation',.49)
    assert t.state!='SUCCEEDED'
    t.tick([.2,0,.08],[0,0,0],Mode.PARALLEL,True,'navigation',.51)
    assert t.state=='SUCCEEDED'


def test_intermediate_transition_keeps_its_tighter_tolerance():
    t=Task();t.begin([.3,0,1.],0.)
    t.plan(route()+[Segment(Mode.SPIN,np.array([[.3,0,0],[.3,0,1.]]))],t.generation,0.)
    t.tick([.2,0,0],[0,0,0],Mode.PARALLEL,True,'navigation',0.)
    assert t.index==0 and t.state!='STOPPING'


def test_tiny_terminal_spin_does_not_force_a_slow_precision_translation():
    t=Task();t.begin([.6,0,.005],0.)
    t.plan([Segment(Mode.PARALLEL,np.array([[0.,0.,0.],[.6,0.,0.]])),
            Segment(Mode.SPIN,np.array([[.6,0.,0.],[.6,0.,.005]]))],t.generation,0.)
    t.tick([.48,0,0],[0,0,0],Mode.PARALLEL,True,'navigation',0.)
    assert t.state=='STOPPING' and t.index==0
    t.tick([.48,0,0],[0,0,0],Mode.PARALLEL,True,'navigation',.51)
    assert t.state=='SUCCEEDED' and t.index==0
