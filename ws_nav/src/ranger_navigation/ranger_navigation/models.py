"""V2 modes in the SDK wire numbering. Mode changes always require a stop."""
from enum import IntEnum
from dataclasses import dataclass
import math
import numpy as np
from .geometry import integrate


class Mode(IntEnum):
    DRIVE = 0
    PARALLEL = 1
    SPIN = 2


@dataclass(frozen=True)
class Limits:
    speed: float = .30
    yaw_rate: float = .50
    acceleration: float = .30
    yaw_acceleration: float = .50
    min_radius: float = .55
    stop_linear: float = .05
    stop_angular: float = .025
    stop_dwell: float = .5
    switch_timeout: float = 3.


def feasible(command, mode, limits=Limits(), tolerance=1e-6):
    v = np.asarray(command, dtype=float)
    if v.shape != (3,) or not np.isfinite(v).all():
        return False
    x,y,w = v
    if np.linalg.norm(v[:2]) > limits.speed+tolerance or abs(w)>limits.yaw_rate+tolerance:
        return False
    if mode == Mode.DRIVE:
        return abs(y)<=tolerance and abs(w)<=abs(x)/limits.min_radius+tolerance
    if mode == Mode.PARALLEL:
        return abs(w)<=tolerance
    if mode == Mode.SPIN:
        return abs(x)+abs(y)<=tolerance
    return False


def stopped(velocity, limits=Limits()):
    return (np.isfinite(velocity).all() and np.linalg.norm(velocity[:2])<=limits.stop_linear
            and abs(velocity[2])<=limits.stop_angular)


def rollout(pose, commands, dt):
    rows = [np.asarray(pose, dtype=float)]
    for v in commands:
        rows.append(integrate(rows[-1], v, dt))
    return np.asarray(rows)


def braking_commands(v, limits=Limits(), dt=.05):
    """Engineering braking model; actual chassis stopping distance remains unverified."""
    v = np.asarray(v, dtype=float).copy()
    out = []
    for _ in range(math.ceil(max(limits.speed/limits.acceleration,
                                 limits.yaw_rate/limits.yaw_acceleration)/dt)+2):
        norm = np.linalg.norm(v[:2])
        v[:2] *= max(0., 1.-limits.acceleration*dt/max(norm,1e-12))
        v[2] = math.copysign(max(0., abs(v[2])-limits.yaw_acceleration*dt),v[2])
        out.append(v.copy())
        if np.max(np.abs(v))<1e-9:
            break
    return np.asarray(out)
