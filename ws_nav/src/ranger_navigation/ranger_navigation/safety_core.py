"""Arbitration, permission gate and independent command/braking sweep check."""
import numpy as np
from .geometry import Body,validate_sweep
from .models import Limits,feasible,stopped,rollout,braking_commands
from .coverage_policy import CoveragePolicy


class Safety:
    def __init__(self,body=Body(),limits=Limits(),coverage_policy=CoveragePolicy()):
        self.coverage_policy=coverage_policy
        self.body,self.limits = body,limits
        self.enabled,self.estop,self.ready = False,False,False
        self.inputs = {}
        self.commands = {}
        self.actual_mode,self.vehicle_ok,self.can_control = None,False,False
        self.velocity,self.pose = np.zeros(3),np.zeros(3)
        self.stop_since,self.switch_since,self.requested = None,None,None
        self.owner,self.reason = 'none','startup'
        self.obstacles = np.empty((0,2))
        self.coverage = None

    def feed(self,name,stamp,now):
        self.inputs[name] = stamp,now

    def command(self,owner,velocity,mode,sample,now,valid=True,deadline=None):
        self.commands[owner] = (np.asarray(velocity,dtype=float),mode,sample,now,valid,
                                now+.1 if deadline is None else deadline)

    def fresh(self,name,sample_now,steady_now,age):
        if name not in self.inputs:
            return False
        stamp,received = self.inputs[name]
        return 0<=sample_now-stamp<=age and 0<=steady_now-received<=age

    def evaluate(self,sample_now,now):
        zero = np.zeros(3)
        self.owner = 'navigation'
        teleop = self.commands.get('teleop')
        if teleop and 0<=now-teleop[3]<=.2:
            self.owner = 'teleop'
        if not self.can_control:
            self.owner = 'rc'
        if stopped(self.velocity,self.limits) and self.fresh('wheel',sample_now,now,.25):
            self.stop_since = now if self.stop_since is None else self.stop_since
        else:
            self.stop_since = None
        tests = [
            (self.enabled,'permission_off'),(not self.estop,'soft_estop'),
            (self.ready and self.fresh('ready',sample_now,now,.25),'localization_not_ready'),
            (self.fresh('wheel',sample_now,now,.25),'wheel_stale'),
            (self.fresh('mode',sample_now,now,.25),'mode_feedback_stale'),
            (self.fresh('system',sample_now,now,.25),'system_feedback_stale'),
            (self.fresh('cloud',sample_now,now,.25),'observation_stale'),
            (self.vehicle_ok and self.can_control,'chassis_fault_or_rc'),
        ]
        for okay,reason in tests:
            if not okay:
                self.reason = reason
                self.switch_since = None
                return zero,None
        cmd = self.commands.get(self.owner)
        if cmd is None or not cmd[4] or not (0<=now-cmd[3]<=.10 and 0<=sample_now-cmd[2]<=.25
                                           and np.isfinite(cmd[5]) and now<=cmd[5]<=now+.1):
            self.reason = 'command_expired_or_invalid'
            return zero,None
        velocity,mode,*_ = cmd
        if mode not in (0,1,2) or not feasible(velocity,mode,self.limits):
            self.reason = 'command_mode_or_limits_invalid'
            return zero,None
        if self.requested!=mode:
            self.requested,self.switch_since = mode,now
        if self.actual_mode!=mode:
            if self.switch_since is None:
                self.switch_since = now
            if now-self.switch_since>self.limits.switch_timeout:
                self.reason = 'mode_switch_timeout'
                return zero,None
            self.reason = 'stopping_for_mode_switch'
            request = mode if self.stop_since is not None and now-self.stop_since>=self.limits.stop_dwell else None
            return zero,request
        self.switch_since = None
        if self.coverage is None:
            self.reason = 'visibility_absent'
            return zero,None
        if np.linalg.norm(velocity[:2]-self.velocity[:2])>self.limits.acceleration*.1+.025 or abs(velocity[2]-self.velocity[2])>self.limits.yaw_acceleration*.1+.04:
            self.reason = 'acceleration_disagrees_with_feedback'
            return zero,mode
        self.reason = 'passing'
        return velocity,mode

    def inputs_ready(self,sample_now,now):
        return (not self.estop and self.ready and self.vehicle_ok and self.can_control
                and all(self.fresh(name,sample_now,now,.25) for name in ('ready','wheel','mode','system','cloud')))

    def collision(self,command):
        """Last output stage: check actual footprint, command lease and braking.

        Zero always interrupts immediately; no smoothing or arbitration follows.
        The braking model is an offline engineering bound, not a measured brake.
        """
        if np.max(np.abs(command))<1e-9:
            return np.zeros(3),float('inf')
        controls = np.vstack((np.tile(command,(2,1)),braking_commands(command,self.limits)))
        poses = rollout(self.pose,controls,.05)
        okay,minimum,reason = validate_sweep(poses,self.obstacles,self.body,
                                            coverage=self.coverage,
                                            margin=self.body.clearance+.04,coverage_margin=self.coverage_policy.total_margin)
        if not okay:
            self.reason = 'collision_monitor:'+reason
            return np.zeros(3),minimum
        return command,minimum
