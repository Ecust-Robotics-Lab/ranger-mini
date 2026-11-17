"""Shared body geometry and independent (NumPy) swept collision validation."""
from dataclasses import dataclass
from pathlib import Path
import math
import numpy as np
import yaml


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


@dataclass(frozen=True)
class Body:
    length: float = .74
    width: float = .52
    clearance: float = .04
    sensor_origin_xy: tuple = (0.,0.)
    sensor_origin_z: float = .60

    @property
    def half(self):
        return np.array([self.length/2, self.width/2])

    @property
    def radius(self):
        return float(np.linalg.norm(self.half))

    @property
    def footprint(self):
        x, y = self.half
        return np.array([[x,y],[x,-y],[-x,-y],[-x,y]])

    def distances(self, pose, points):
        """Analytic rectangle SDF; independent of the JAX implementation."""
        p = np.asarray(points, dtype=float).reshape(-1, 2) - np.asarray(pose)[:2]
        c, s = math.cos(pose[2]), math.sin(pose[2])
        local = p @ np.array([[c,-s],[s,c]])
        q = np.abs(local) - self.half
        return np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(np.max(q, axis=1), 0)


def load_geometry(path, clearance=.04):
    data = yaml.safe_load(Path(path).read_text())
    length, width = data['body']['reference_envelope_length_width_m']
    body = Body(float(length), float(width), float(clearance),
                tuple(data['installation']['t_base_lidar_xyz'][:2]),
                float(data['installation']['t_base_lidar_xyz'][2]))
    if not np.allclose(data['body']['footprint_xy'], body.footprint):
        raise ValueError('body envelope and footprint disagree')
    ins = data['installation']
    return body, rotation(ins['t_base_imu_rpy']), np.array(ins['t_base_imu_xyz'], dtype=float)


def integrate(pose, velocity, dt):
    """Exact constant body-twist SE(2) integration, including reverse/parallel."""
    x, y, yaw = pose
    vx, vy, w = velocity
    a = w * dt
    ss = np.sinc(a / np.pi) * dt
    cc = (1-math.cos(a))/w if abs(w) > 1e-9 else 0.
    dx, dy = ss*vx-cc*vy, cc*vx+ss*vy
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([x+c*dx-s*dy, y+s*dx+c*dy, float(wrap(yaw+a))])


def densify(poses, body, step=.015):
    poses = np.asarray(poses, dtype=float).reshape(-1,3)
    if not len(poses):
        return poses
    rows = [poses[0]]
    for a,b in zip(poses[:-1], poses[1:]):
        delta = b-a
        delta[2] = wrap(delta[2])
        n = max(1, math.ceil((np.linalg.norm(delta[:2])+body.radius*abs(delta[2]))/step))
        rows.extend(a+delta*t/n for t in range(1,n+1))
    return np.asarray(rows)


def validate_sweep(poses, points, body, grid=None, coverage=None, margin=None, map_margin=None):
    """Conservative inter-sample bound: footprint motion <= step, then add step.

    Used after optimization on the actual command rollout and braking tail.
    Full retained points are checked, independently of optimization sampling.
    """
    step = .015
    margin = body.clearance if margin is None else margin
    map_margin = margin if map_margin is None else map_margin
    minimum = float('inf')
    for p in densify(poses, body, step):
        if grid is not None and not grid.free(p, body, map_margin+step):
            return False, minimum, 'map_or_unknown_collision'
        # Coverage cells already represent their complete area. Point-voxel
        # uncertainty belongs to the point SDF, not a second coverage inflation.
        if coverage is not None and not coverage(p, body, body.clearance+step):
            return False, minimum, 'unobserved_sweep'
        ds = body.distances(p, points)
        if len(ds):
            minimum = min(minimum, float(ds.min()))
            if minimum < margin+step:
                return False, minimum, 'obstacle_collision'
    return True, minimum, 'valid'
