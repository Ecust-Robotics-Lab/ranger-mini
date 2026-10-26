"""Bounded, monotonic odometry history. Integer timestamps are nanoseconds."""
from bisect import bisect_left
import math

from .geometry import SE3, quat_normalize


def interpolate(a, b, fraction):
    """Linear translation and shortest-arc quaternion SLERP (full 6DoF)."""
    dot = sum(x*y for x, y in zip(a.q, b.q))
    q = b.q
    if dot < 0:
        q, dot = tuple(-v for v in q), -dot
    dot = min(1., max(-1., dot))
    if dot > .9995:
        rotation = quat_normalize(tuple(x + fraction*(y-x) for x, y in zip(a.q, q)))
    else:
        angle = math.acos(dot)
        left = math.sin((1-fraction)*angle)/math.sin(angle)
        right = math.sin(fraction*angle)/math.sin(angle)
        rotation = tuple(left*x + right*y for x, y in zip(a.q, q))
    return SE3(tuple(x + fraction*(y-x) for x, y in zip(a.t, b.t)), rotation)


class PoseHistory:
    def __init__(self, duration_s=2., max_gap_s=.1, max_samples=1000):
        if not all(math.isfinite(v) and v > 0 for v in [duration_s, max_gap_s]):
            raise ValueError('history duration and gap must be positive and finite')
        if max_samples < 2 or duration_s < max_gap_s:
            raise ValueError('history needs >=2 samples and duration >= maximum gap')
        self.duration_ns = round(duration_s*1e9)
        self.max_gap_ns = round(max_gap_s*1e9)
        self.max_samples = max_samples
        self.samples = []

    def clear(self):
        self.samples.clear()

    def add(self, stamp_ns, pose):
        if self.samples and stamp_ns == self.samples[-1][0]:
            return 'duplicate'
        result = 'added'
        if self.samples and stamp_ns < self.samples[-1][0]:
            self.clear()
            result = 'regressed'
        self.samples.append((stamp_ns, pose))
        cutoff = stamp_ns - self.duration_ns
        while self.samples and (self.samples[0][0] < cutoff or len(self.samples) > self.max_samples):
            self.samples.pop(0)
        return result

    def lookup(self, stamp_ns):
        if not self.samples:
            return None, 'no odometry history'
        stamps = [s[0] for s in self.samples]
        i = bisect_left(stamps, stamp_ns)
        if i < len(stamps) and stamps[i] == stamp_ns:
            return self.samples[i][1], 'exact'
        if i == 0:
            return None, 'before odometry history'
        if i == len(stamps):
            return None, 'waiting for odometry bracket'
        left, right = self.samples[i-1], self.samples[i]
        if right[0]-left[0] > self.max_gap_ns:
            return None, 'odometry bracket gap too large'
        fraction = (stamp_ns-left[0])/(right[0]-left[0])
        return interpolate(left[1], right[1], fraction), 'interpolated'


def stamp_ns(stamp):
    return stamp.sec*1_000_000_000 + stamp.nanosec
