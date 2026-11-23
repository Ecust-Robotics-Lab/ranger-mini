"""Accepted optimizer candidates, aligned to measured progress, never commands.

Control generations revoke output independently of this cache. Only an explicit
coordinator validation acknowledgement promotes a candidate to reusable history.
"""
from dataclasses import dataclass
import numpy as np
from .geometry import wrap


@dataclass
class Candidate:
    request_id: int
    stamp: float
    controls: np.ndarray
    trajectory: np.ndarray


class ControlHistory:
    def __init__(self,maximum_age=1.,dt=.05):
        self.maximum_age,self.dt=maximum_age,dt
        self.reset()

    def reset(self):
        self.candidates={}
        self.accepted=None
        self.diagnostic={'reused':False,'reason':'new_route'}

    def remember(self,request_id,stamp,controls,trajectory):
        controls=np.asarray(controls,dtype=float);trajectory=np.asarray(trajectory,dtype=float)
        if (controls.ndim!=2 or controls.shape[1]!=3 or trajectory.shape!=(len(controls)+1,3)
                or not np.isfinite(controls).all() or not np.isfinite(trajectory).all() or not np.isfinite(stamp)):
            raise ValueError('invalid_control_history')
        self.candidates[request_id]=Candidate(request_id,stamp,controls.copy(),trajectory.copy())
        while len(self.candidates)>8:del self.candidates[min(self.candidates)]

    def select(self,accepted_request_id,stamp,pose,execution_ready,steering_wait=False):
        self.diagnostic={'reused':False,'reason':'no_validated_candidate'}
        if accepted_request_id in self.candidates:
            self.accepted=self.candidates[accepted_request_id]
        previous=self.accepted
        if previous is None or previous.request_id!=accepted_request_id:return None
        age=stamp-previous.stamp
        if not 0<=age<=self.maximum_age:
            self.diagnostic['reason']='history_expired_or_clock_regressed';return None
        if not execution_ready:
            self.diagnostic['reason']='execution_feedback_unavailable';return None
        pose=np.asarray(pose,dtype=float)
        if pose.shape!=(3,) or not np.isfinite(pose).all():raise ValueError('invalid_history_pose')
        elapsed=min(int(np.floor(age/self.dt+1e-6)),len(previous.controls)-1)
        candidates=previous.trajectory[:elapsed+2]
        error=np.linalg.norm(candidates[:,:2]-pose[:2],axis=1)+.15*np.abs(wrap(candidates[:,2]-pose[2]))
        nearest=int(np.argmin(error))
        if (np.linalg.norm(candidates[nearest,:2]-pose[:2])>.10
                or abs(wrap(candidates[nearest,2]-pose[2]))>.10):
            self.diagnostic['reason']='measured_pose_disagrees_with_history';return None
        # A queued/blocked SDK command cannot advance the warm start. Even in
        # normal execution, time alone never consumes more than measured motion.
        shift=0 if steering_wait else min(elapsed,nearest)
        sequence=np.zeros_like(previous.controls)
        sequence[:len(sequence)-shift]=previous.controls[shift:]
        self.diagnostic=dict(reused=True,reason='measured_progress',source_request=previous.request_id,
                             shift_steps=shift,sample_age_s=age)
        return sequence
