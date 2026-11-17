"""Brief input loss revokes controls while preserving a route for revalidation."""
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ws_nav/src/ranger_navigation'))
from ranger_navigation.task import Task
from ranger_navigation.planner import Segment
from ranger_navigation.models import Mode


def executing():
 t=Task();t.begin([1.,0,0],0.)
 t.plan([Segment(Mode.PARALLEL,np.array([[0.,0.,0.],[1.,0.,0.]]))],t.generation,0.)
 t.tick([0,0,0],[0,0,0],Mode.PARALLEL,True,'navigation',0.)
 return t


def test_short_gap_revokes_old_results_but_preserves_route_for_recheck():
 t=executing();generation=t.generation;segment=t.segments[0]
 t.tick([.1,0,0],[.1,0,0],Mode.PARALLEL,True,'navigation',.2)
 assert t.tick([0,0,0],[0,0,0],None,False,'navigation',.25) is None
 assert t.state=='PAUSED' and t.generation==generation+1 and t.segments[0] is segment
 t.tick([0,0,0],[0,0,0],None,False,'navigation',.35)
 assert t.generation==generation+1
 assert not t.plan([],generation,.36)
 assert t.tick([.11,0,0],[0,0,0],Mode.PARALLEL,True,'navigation',.45) is segment
 assert t.state=='OBSERVING'  # fresh independent route/coverage check is mandatory


def test_manual_takeover_always_discards_paused_route():
 t=executing();t.pause_inputs('inputs_invalid',.1)
 t.tick([0,0,0],[0,0,0],Mode.PARALLEL,True,'teleop',.2)
 assert t.state=='WAITING' and not t.segments


def test_hard_invalidation_discards_route_after_transient_pause():
 t=executing();t.pause_inputs('inputs_invalid',.1)
 t.invalidate('localization_session_changed')
 assert not t.segments and t.pause_since is None


def test_fixed_curvature_drive_pause_keeps_arc_but_revokes_old_generation():
 t=Task();t.begin([1.,1.,.5],0.)
 segment=Segment(Mode.DRIVE,np.array([[0.,0.,0.],[.1,.01,.2]]))
 t.plan([segment],t.generation,0.)
 generation=t.generation
 t.pause_inputs('inputs_invalid',.1)
 assert t.state=='PAUSED' and t.segments[0] is segment
 assert t.generation==generation+1 and not t.plan([],generation,.15)
 assert t.tick([0,0,0],[0,0,0],Mode.DRIVE,True,'navigation',.2) is segment
 assert t.state=='OBSERVING'


def test_long_input_pause_does_not_create_a_false_progress_timeout_on_recovery():
 t=executing();t.tick([.1,0,0],[.1,0,0],Mode.PARALLEL,True,'navigation',.2)
 t.tick([0,0,0],[0,0,0],None,False,'navigation',.25)
 t.tick([0,0,0],[0,0,0],None,False,'navigation',9.)
 t.tick([.1,0,0],[0,0,0],Mode.PARALLEL,True,'navigation',9.1)
 assert t.state=='OBSERVING' and t.last_progress>9.


def test_pause_at_reached_tiny_segment_finishes_stop_before_next_mode():
 t=Task();t.begin([.004,0,1.],0.)
 segments=[Segment(Mode.PARALLEL,np.array([[0.,0.,0.],[.004,0.,0.]])),
           Segment(Mode.SPIN,np.array([[.004,0.,0.],[.004,0.,1.]]))]
 t.plan(segments,t.generation,0.)
 assert t.tick([0,0,0],[0,0,.04],Mode.SPIN,True,'navigation',.05) is None
 assert t.state=='STOPPING'
 old=t.generation;t.pause_inputs('inputs_invalid',.1)
 assert t.generation==old+1
 assert t.tick([0,0,0],[0,0,0],Mode.SPIN,True,'navigation',.2) is None
 assert t.state=='STOPPING' and t.index==0
 t.tick([0,0,0],[0,0,0],Mode.SPIN,True,'navigation',.71)
 assert t.index==1 and t.state=='SWITCHING'
 t.tick([0,0,0],[0,0,0],Mode.SPIN,True,'navigation',.72)
 assert t.state=='EXECUTING'


def test_slow_real_spin_progress_does_not_replan_every_three_seconds():
 t=Task();t.begin([0,0,1.5],0.)
 t.plan([Segment(Mode.SPIN,np.array([[0.,0.,0.],[0.,0.,1.5]]))],t.generation,0.)
 t.tick([0,0,0],[0,0,0],Mode.SPIN,True,'navigation',0.)
 generation=t.generation
 for second in range(1,11):
  t.tick([0,0,.04*second],[0,0,.04],Mode.SPIN,True,'navigation',float(second))
  assert t.state=='EXECUTING' and t.generation==generation


def test_motionless_spin_still_fails_goal_wide_timeout():
 t=Task();t.begin([0,0,1.5],0.)
 t.plan([Segment(Mode.SPIN,np.array([[0.,0.,0.],[0.,0.,1.5]]))],t.generation,0.)
 t.tick([0,0,0],[0,0,0],Mode.SPIN,True,'navigation',0.)
 t.tick([0,0,0],[0,0,0],Mode.SPIN,True,'navigation',3.2)
 assert t.state=='EXECUTING'
 t.tick([0,0,0],[0,0,0],Mode.SPIN,True,'navigation',15.1)
 assert t.state=='FAILED' and t.reason=='no_progress_timeout'


def test_precise_door_goal_does_not_accept_fifteen_centimetres_early():
 t=Task(goal_position_tolerance=.04,goal_yaw_tolerance=.04);t.begin([.3,0,0],0.)
 t.plan([Segment(Mode.PARALLEL,np.array([[0.,0.,0.],[.3,0.,0.]]))],t.generation,0.)
 t.tick([0,0,0],[0,0,0],Mode.PARALLEL,True,'navigation',0.)
 assert t.tick([.2,0,0],[0,0,0],Mode.PARALLEL,True,'navigation',.1) is not None
 assert t.state=='EXECUTING'
 t.tick([.265,0,0],[0,0,0],Mode.PARALLEL,True,'navigation',.2)
 t.tick([.265,0,0],[0,0,0],Mode.PARALLEL,True,'navigation',.71)
 assert t.state=='SUCCEEDED'


def test_precise_door_heading_does_not_skip_required_spin():
 t=Task(goal_position_tolerance=.04,goal_yaw_tolerance=.04);t.begin([0,0,.3],0.)
 t.plan([Segment(Mode.SPIN,np.array([[0.,0.,0.],[0.,0.,.3]]))],t.generation,0.)
 t.tick([0,0,0],[0,0,0],Mode.SPIN,True,'navigation',0.)
 assert t.tick([0,0,.2],[0,0,0],Mode.SPIN,True,'navigation',.1) is not None
 assert t.state=='EXECUTING'


def test_precise_goal_translation_finishes_inside_terminal_spin_position_tolerance():
 t=Task(goal_position_tolerance=.04,goal_yaw_tolerance=.04);t.begin([.3,0,.3],0.)
 t.plan([Segment(Mode.PARALLEL,np.array([[0.,0.,0.],[.3,0.,0.]])),
         Segment(Mode.SPIN,np.array([[.3,0.,0.],[.3,0.,.3]]))],t.generation,0.)
 t.tick([0,0,0],[0,0,0],Mode.PARALLEL,True,'navigation',0.)
 assert t.tick([.25,0,0],[0,0,0],Mode.PARALLEL,True,'navigation',.1) is not None
 assert t.state=='EXECUTING' and t.index==0
