"""Single-goal lifecycle; timeouts use steady time, never simulated ROS time."""
import numpy as np
from .geometry import wrap
from .models import Limits,Mode,stopped


class Task:
    TERMINAL = ('SUCCEEDED','CANCELED','FAILED','REPLACED')

    def __init__(self,limits=Limits(),segment_yaw_tolerance=.06,goal_position_tolerance=.15,goal_yaw_tolerance=.15):
        self.limits = limits
        if not np.isfinite(segment_yaw_tolerance) or not 0 < segment_yaw_tolerance <= .06:
            raise ValueError('segment yaw tolerance must be finite and in (0, .06]rad')
        self.segment_yaw_tolerance = float(segment_yaw_tolerance)
        if (not np.isfinite([goal_position_tolerance,goal_yaw_tolerance]).all()
                or not .02<=goal_position_tolerance<=.15 or not .03<=goal_yaw_tolerance<=.15):
            raise ValueError('goal tolerances must be finite, position .02..0.15m and yaw .03..0.15rad')
        self.goal_position_tolerance=float(goal_position_tolerance)
        self.goal_yaw_tolerance=float(goal_yaw_tolerance)
        self.generation = 0
        self.state,self.reason = 'IDLE','startup'
        self.goal,self.segments = None,[]
        self.index = 0
        self.stopped_since = self.blocked_since = self.switch_since = None
        self.last_progress = self.progress_pose = None
        self.stall_elapsed = 0.
        self.progress_tick = None
        self.progress_active = False
        self.pause_since = None

    def invalidate(self,reason,state='WAITING'):
        self.generation += 1
        self.state,self.reason = state,reason
        self.segments,self.index = [],0
        self.stopped_since = self.switch_since = None
        self.blocked_since = None
        self.pause_since = None

    def pause_inputs(self,reason,now):
        if self.state in self.TERMINAL:
            return
        reusable=(self.index<len(self.segments) and
                  self.segments[self.index].mode in (Mode.DRIVE,Mode.PARALLEL,Mode.SPIN))
        if not reusable:
            if self.state!='WAITING':
                self.invalidate(reason)
            return
        if self.state!='PAUSED':
            self.generation += 1  # revoke every pending result and control history
            self.pause_since = now
            self.stopped_since = None
        self.state,self.reason = 'PAUSED',reason

    def begin(self,goal,now):
        self.goal = np.asarray(goal,dtype=float)
        self.invalidate('new_goal','PLANNING')
        self.blocked_since,self.last_progress = None,now
        self.progress_pose = None
        self.stall_elapsed = 0.
        self.progress_tick = now
        self.progress_active = False

    def cancel(self):
        self.invalidate('canceled','CANCELED')

    def plan(self,segments,generation,now):
        if generation!=self.generation or self.state in self.TERMINAL:
            return False
        self.generation += 1  # accepted route is a new optimizer-history generation
        self.segments,self.index = segments,0
        self.state,self.reason = 'SWITCHING','plan_accepted'
        self.switch_since = now
        return True

    def block(self,reason,now):
        if self.state in self.TERMINAL:
            return
        if self.blocked_since is None:
            self.blocked_since = now
        self.state,self.reason = 'BLOCKED',reason
        if now-self.blocked_since>=15.:
            self.invalidate('blocked_timeout','FAILED')

    def hold_observation(self,reason='guidance_unobserved'):
        if self.state not in self.TERMINAL:
            self.state,self.reason = 'OBSERVING',reason

    def tick(self,pose,speed,actual_mode,healthy,owner,now):
        if self.goal is None or self.state in self.TERMINAL:
            return None
        # This budget belongs to the goal, not an individual planning attempt.
        # Replanning/mode confirmation must not indefinitely renew a stalled
        # task. Exclude intervals without valid autonomous control ownership.
        active = healthy and owner=='navigation'
        if active and self.progress_active and self.progress_tick is not None:
            self.stall_elapsed += max(0.,now-self.progress_tick)
        self.progress_tick,self.progress_active = now,active
        if not healthy or owner!='navigation':
            if owner!='navigation':
                if self.state!='WAITING':
                    self.invalidate('manual_takeover')
            else:
                self.pause_inputs('inputs_invalid',now)
            return None
        if self.state=='PAUSED':
            if self.pause_since is not None and self.last_progress is not None:
                self.last_progress += max(0.,now-self.pause_since)
            self.pause_since = None
            self.state,self.reason = 'OBSERVING','revalidate_after_pause'
        if self.state=='WAITING':
            self.state,self.reason = 'PLANNING','recheck_after_pause'
        # During a translation, steering/yaw jitter is not positional progress.
        spin = ((self.index<len(self.segments) and self.segments[self.index].mode==Mode.SPIN)
                or (np.linalg.norm(np.asarray(pose)[:2]-self.goal[:2])<=self.goal_position_tolerance
                    and abs(wrap(pose[2]-self.goal[2]))>self.goal_yaw_tolerance))
        moved = self.progress_pose is not None and (
            np.linalg.norm(np.asarray(pose)[:2]-self.progress_pose[:2])>.03 or
            (spin and abs(wrap(pose[2]-self.progress_pose[2]))>.03))
        if self.progress_pose is None or moved:
            self.progress_pose,self.last_progress = np.array(pose),now
            self.blocked_since = None
            self.stall_elapsed = 0.
        elif self.stall_elapsed>=15.:
            self.invalidate('no_progress_timeout','FAILED')
            return None
        if self.state in ('PLANNING','BLOCKED'):
            return None
        if now-self.last_progress>3. and self.state=='EXECUTING' and not spin:
            self.block('no_progress',now)
            return None
        if self.index>=len(self.segments):
            reached = (np.linalg.norm(np.asarray(pose)[:2]-self.goal[:2])<=self.goal_position_tolerance
                       and abs(wrap(pose[2]-self.goal[2]))<=self.goal_yaw_tolerance)
            if not reached:
                self.invalidate('goal_drifted','PLANNING')
            elif stopped(speed,self.limits):
                self.stopped_since = now if self.stopped_since is None else self.stopped_since
                if now-self.stopped_since>=.5:
                    self.state,self.reason = 'SUCCEEDED','goal_reached_and_stopped'
            else:
                self.stopped_since = None
            return None
        seg = self.segments[self.index]
        target = seg.poses[-1]
        # Tighter internal segment tolerance keeps mode transitions on-route.
        final_segment = self.index==len(self.segments)-1
        # An exact planner may append a tiny goal spin because the newest
        # localization yaw differs from the requested yaw. Once both goal
        # tolerances are met, that optional spin must not force the preceding
        # translation to use the tighter intermediate-waypoint tolerance.
        tail=self.segments[self.index+1:]
        optional_goal_spin=(np.linalg.norm(target[:2]-self.goal[:2])<1e-6
            and abs(wrap(pose[2]-self.goal[2]))<self.goal_yaw_tolerance
            and all(s.mode==Mode.SPIN and np.linalg.norm(s.poses[-1,:2]-self.goal[:2])<1e-6 for s in tail))
        final_segment = final_segment or optional_goal_spin
        reached = (np.linalg.norm(np.asarray(pose)[:2]-target[:2])<(self.goal_position_tolerance if final_segment else min(.06,self.goal_position_tolerance))
                   and abs(wrap(pose[2]-(self.goal[2] if final_segment else target[2])))<(self.goal_yaw_tolerance if final_segment else self.segment_yaw_tolerance))
        if reached:
            self.state,self.reason = 'STOPPING','segment_end'
            if stopped(speed,self.limits):
                self.stopped_since = now if self.stopped_since is None else self.stopped_since
                if now-self.stopped_since>=self.limits.stop_dwell:
                    if final_segment:
                        self.state,self.reason = 'SUCCEEDED','goal_reached_and_stopped'
                        return None
                    self.index += 1
                    self.stopped_since = None
                    self.switch_since = now
                    self.state = 'SWITCHING'
            else:
                self.stopped_since = None
            return None
        if self.state=='OBSERVING':
            # A reached segment above only needs a new stopped dwell, even
            # after a pause. Otherwise keep waiting for a freshly validated
            # trajectory; no old command is resumed by this state.
            return seg
        if actual_mode!=seg.mode or self.state=='SWITCHING':
            if self.state=='SWITCHING' and seg.mode!=Mode.SPIN and stopped(speed,self.limits):
                # A new mode must start from the stopped body pose used by its
                # sweep/model. Drive cannot remove yaw error while stationary.
                # Spin predicts about the actual center and its actual sweep
                # is independently validated; small terminal position error
                # must not cause endless short-translation/spin replans.
                # Replan translations from
                # feedback rather than carrying a tolerance-sized offset into
                # a route with only a few centimetres of clearance.
                error = np.asarray(pose)-seg.poses[0]
                if (abs(wrap(error[2]))>.005 or
                        (seg.mode!=Mode.PARALLEL and np.linalg.norm(error[:2])>.005)):
                    self.invalidate('segment_start_pose_replan','PLANNING')
                    return None
            if self.state=='SWITCHING' and seg.mode==Mode.PARALLEL:
                axis = target[:2]-seg.poses[0,:2]
                length = np.linalg.norm(axis)
                if length>1e-6:
                    axis = axis/length
                    error = np.asarray(pose)[:2]-seg.poses[0,:2]
                    transverse = error-np.dot(error,axis)*axis
                    # A fixed-wheel-axis segment cannot remove the previous
                    # segment's stopping error perpendicular to its path.
                    # Replan from the actual stopped pose before choosing the
                    # next steering axis; never slide a verified line sideways.
                    if np.linalg.norm(transverse)>.005:
                        self.invalidate('segment_cross_track_replan','PLANNING')
                        return None
            self.switch_since = now if self.switch_since is None else self.switch_since
            if now-self.switch_since>self.limits.switch_timeout:
                self.invalidate('mode_switch_timeout','FAILED')
                return None
            if actual_mode==seg.mode and stopped(speed,self.limits):
                # The independent gate already requires the stop dwell before
                # issuing a mode change. Once actual feedback confirms it,
                # another fixed wait only repeats the same transition delay.
                self.state,self.reason = 'EXECUTING','mode_confirmed'
                self.last_progress = now
                self.stopped_since = None
            return None
        self.state,self.reason = 'EXECUTING','tracking'
        return seg
