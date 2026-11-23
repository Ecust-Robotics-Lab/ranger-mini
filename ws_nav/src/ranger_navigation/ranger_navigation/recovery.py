"""Same-mode recovery checks; no grace period extends stale input validity."""
import math
import numpy as np
from .geometry import wrap,densify
from .models import Mode,feasible,stopped


def resume_reason(segment,pose,speed,actual_mode,body,limits):
    if actual_mode!=segment.mode:return 'awaiting_actual_mode'
    if not feasible(speed,segment.mode,limits,tolerance=.03):return 'awaiting_mode_velocity'
    if stopped(speed,limits):return 'resume_from_rest'
    # Keep the original wheel axis/curvature while still moving. Re-anchoring
    # and changing steering axes remain restricted to a confirmed stop.
    path=densify(segment.poses,body,.015)
    error=np.linalg.norm(path[:,:2]-np.asarray(pose)[:2],axis=1)+.15*np.abs(wrap(path[:,2]-pose[2]))
    nearest=int(np.argmin(error))
    if (np.linalg.norm(path[nearest,:2]-pose[:2])>.03
            or (segment.mode!=Mode.SPIN and abs(wrap(path[nearest,2]-pose[2]))>.03)):
        return 'awaiting_stop_off_route'
    if segment.mode==Mode.PARALLEL:
        delta=path[-1,:2]-path[0,:2];axis=delta/max(np.linalg.norm(delta),1e-9)
        c,s=math.cos(pose[2]),math.sin(pose[2]);world=np.asarray(speed[:2])@np.array([[c,s],[-s,c]])
        if np.linalg.norm(world-axis*np.dot(world,axis))>limits.acceleration*.05:
            return 'awaiting_stop_cross_axis_velocity'
    return 'resume_from_measured_motion'


def command_live(command,generation,steady_now,sample_now):
    if command is None:return False
    meta,_,stamp=command
    return (meta['generation']==generation and 0<=steady_now-meta['submitted']<=.1
            and 0<=sample_now-stamp*1e-9<=.25)
