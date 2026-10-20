"""Rigid transform helpers; no algorithm-specific semantics."""
import math
from geometry_msgs.msg import TransformStamped

def quat_normalize(q):
    x, y, z, w = q
    n = math.hypot(x, y, z, w)
    if n < 1e-12:
        # zero/unset quaternion (nav_msgs default when a driver fills only
        # position): substitute identity instead of raising ZeroDivisionError
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return quat_normalize((sr * cp * cy - cr * sp * sy,
                           cr * sp * cy + sr * cp * sy,
                           cr * cp * sy - sr * sp * cy,
                           cr * cp * cy + sr * sp * sy))


class SE3:
    """Minimal rigid transform: translation + unit quaternion."""

    def __init__(self, xyz=(0.0, 0.0, 0.0), q=(0.0, 0.0, 0.0, 1.0)):
        self.t = tuple(xyz)
        self.q = quat_normalize(q)

    def __mul__(self, other):
        q = quat_multiply(self.q, other.q)
        t = quat_rotate(self.q, other.t)
        return SE3((t[0] + self.t[0], t[1] + self.t[1], t[2] + self.t[2]), q)

    def inverse(self):
        qi = quat_conjugate(self.q)
        t = quat_rotate(qi, (-self.t[0], -self.t[1], -self.t[2]))
        return SE3(t, qi)

    def yaw(self):
        x, y, z, w = self.q
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def xyz(self):
        return self.t


def quat_rotate(q, v):
    """Rotate vector v by unit quaternion q: q * (v, 0) * q^-1."""
    qv = (v[0], v[1], v[2], 0.0)
    r = quat_multiply(quat_multiply(q, qv), quat_conjugate(q))
    return (r[0], r[1], r[2])


def se3_from_tf_msg(msg):
    tr = msg.transform
    return SE3((tr.translation.x, tr.translation.y, tr.translation.z),
               (tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w))


def tf_msg_from_se3(se3, parent, child, stamp):
    m = TransformStamped()
    m.header.stamp = stamp
    m.header.frame_id = parent
    m.child_frame_id = child
    m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = se3.t
    m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = se3.q
    return m


def se3_from_odom(msg):
    p = msg.pose.pose.position
    o = msg.pose.pose.orientation
    return SE3((p.x, p.y, p.z), (o.x, o.y, o.z, o.w))
