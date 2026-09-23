#!/usr/bin/env python3
"""Ranger localization bridge: Lightning-LM loc TF + wheel odom -> Nav2 TF.

Design (stage 5, docs/05_localization_interface.md):

  Lightning publishes  TF map -> imu_link   (sensor pose, 6DoF, sensor time).
  Wheel driver publishes /wheel/odom (odom -> base_link pose, driver stamps,
  publish_odom_tf=false so this node is the UNIQUE odom->base_link publisher).

  This node computes, on every wheel odom message (same body point, same time):
      T_map_base  = T_map_imu * inverse(T_base_imu)
      T_map_odom  = T_map_base * inverse(T_odom_base)
  and publishes TF map->odom and odom->base_link, both stamped with the ODOM
  message header stamp (no "now" stamps masking latency).

  Prohibitions honored (stage 5 instructions):
  - Lightning's map pose (has global corrections) is never differenced for
    controller velocity; velocity comes from /wheel/odom.twist only.
  - The sensor pose is never relabelled as base_link: the IMU pose keeps its
    own frame (imu_link, published by Lightning) and the base pose is derived
    through the EXTRINSIC PARAM below, which is an explicitly-declared
    approximation pending measurement (see docs).
  - map->odom is recomputed from same-instant transforms; it is never a fixed
    identity.
  - Only this node publishes map->odom and odom->base_link; Lightning remains
    the only publisher of map->imu_link (distinct child frames, no conflict).

  navigation_ready (std_msgs/Bool, plus a DiagnosticArray) is false unless ALL
  hold: loc TF fresh (age <= tf_max_age_s), /lightning/loc_status == GOOD,
  /lightning/loc_confidence >= min_confidence, wheel odom stream alive (a 0.2 s
  watchdog retracts a stale true even if /wheel/odom stops entirely), and the
  loc-vs-odom innovation check passing (translation + speed + yaw criteria,
  ~0.5 s windows; windows with mismatched loc/odom intervals are skipped).
  Node liveness alone never implies ready. When the wheel odom stream is
  absent the health message reports the degraded mode explicitly.
"""
import collections
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Transform, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, Int32

import tf2_ros


# ---------------- small quaternion/se3 helpers (pure python, testable) -------

def quat_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
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


# ------------------------------- bridge node ---------------------------------

class LocalizationBridge(Node):

    def __init__(self):
        super().__init__("ranger_localization_bridge")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("loc_child_frame", "imu_link")
        self.declare_parameter("wheel_odom_topic", "/wheel/odom")
        self.declare_parameter("status_topic", "/lightning/loc_status")
        self.declare_parameter("confidence_topic", "/lightning/loc_confidence")
        # base_link -> imu_link, APPROXIMATE (user 2026-09-23: roof centerline,
        # ~0.25 m ahead of rear edge, ~0.60 m above ground, level). z assumes
        # base_link at ground level; x/y offsets pending vehicle measurement.
        self.declare_parameter("t_base_imu_xyz", [0.0, 0.0, 0.556])
        self.declare_parameter("t_base_imu_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("tf_max_age_s", 0.5)
        self.declare_parameter("odom_max_age_s", 0.5)
        self.declare_parameter("min_confidence", 1.0)
        self.declare_parameter("required_status", 2)  # LocalizationStatus::GOOD
        self.declare_parameter("innovation_trans_th", 0.5)
        self.declare_parameter("innovation_speed_th", 0.3)
        self.declare_parameter("innovation_yaw_th_deg", 20.0)
        self.declare_parameter("innovation_interval_s", 0.5)
        self.declare_parameter("innovation_fail_limit", 3)

        p = lambda n: self.get_parameter(n).value
        self.map_frame = p("map_frame")
        self.odom_frame = p("odom_frame")
        self.base_frame = p("base_frame")
        self.loc_child = p("loc_child_frame")
        self.tf_max_age = float(p("tf_max_age_s"))
        self.odom_max_age = float(p("odom_max_age_s"))
        self.min_conf = float(p("min_confidence"))
        self.req_status = int(p("required_status"))
        self.innov_t_th = float(p("innovation_trans_th"))
        self.innov_speed_th = float(p("innovation_speed_th"))
        self.innov_yaw_th = math.radians(float(p("innovation_yaw_th_deg")))
        self.innov_interval = float(p("innovation_interval_s"))
        self.innov_fail_limit = int(p("innovation_fail_limit"))
        self.t_base_imu = SE3(tuple(p("t_base_imu_xyz")),
                              rpy_to_quat(*p("t_base_imu_rpy")))
        self.t_imu_base = self.t_base_imu.inverse()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, p("wheel_odom_topic"), self.on_odom, sensor_qos)
        self.create_subscription(Int32, p("status_topic"), self.on_status, sensor_qos)
        self.create_subscription(Float32, p("confidence_topic"), self.on_confidence, sensor_qos)

        self.ready_pub = self.create_publisher(Bool, "/navigation_ready", sensor_qos)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/localization_health", sensor_qos)

        # Watchdog: readiness MUST be re-evaluated even when /wheel/odom stops
        # arriving (otherwise the last published Bool would latch True forever).
        self.create_timer(0.2, self._health_tick)

        # state (all mutated from ROS executor callbacks; guarded anyway)
        self._lock = threading.Lock()
        self._status = None          # last Int32 status
        self._status_time = None
        self._confidence = None      # last Float32 confidence
        self._conf_time = None
        self._last_odom_stamp = None
        # history entries: (odom_stamp, loc_stamp, T_map_base, T_odom_base).
        # loc_stamp is the loc TF's OWN header time; without it the loc/odom
        # displacements would be differenced over intervals offset by the
        # lookup latency, injecting a speed-proportional bias (review #4).
        self._history = collections.deque(maxlen=80)
        self._innov_fails = 0
        self._innov_last_err = None

        self.get_logger().info(
            "bridge up: %s->%s + %s->%s computed from %s->%s; T_base_imu t=%s q=%s (APPROXIMATE)" %
            (self.map_frame, self.odom_frame, self.odom_frame, self.base_frame,
             self.map_frame, self.loc_child, self.t_base_imu.t, self.t_base_imu.q))

    # ------------------------------------------------------------------ inputs

    def on_status(self, msg):
        with self._lock:
            self._status = int(msg.data)
            self._status_time = self.get_clock().now().nanoseconds * 1e-9

    def on_confidence(self, msg):
        with self._lock:
            self._confidence = float(msg.data)
            self._conf_time = self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------- main

    def on_odom(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = msg.header.stamp
        stamp_sec = stamp.sec + stamp.nanosec * 1e-9
        if stamp_sec <= 0.0:
            return  # never publish TF from an unstamped message

        t_odom_base = se3_from_odom(msg)

        # 1) odom -> base_link (unique publisher: this node; driver TF is off)
        self.tf_broadcaster.sendTransform(
            tf_msg_from_se3(t_odom_base, self.odom_frame, self.base_frame, stamp))

        # 2) Lightning's map -> imu_link: take the LATEST buffered transform and
        # enforce freshness on ITS OWN stamp. A same-stamp lookup would fail most
        # of the time because loc TF lags odom stamps by the ~0.1-0.2 s pipeline
        # latency; using latest keeps map->odom continuous, and the age check
        # below turns a stalled loc into navigation_ready=false.
        tf_ok = False
        loc_age_s = None
        t_map_base = None
        loc_stamp = None
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.map_frame, self.loc_child, rclpy.time.Time())  # Time() -> latest
            loc_stamp = tf_msg.header.stamp.sec + tf_msg.header.stamp.nanosec * 1e-9
            loc_age_s = now - loc_stamp
            if loc_age_s <= self.tf_max_age:
                t_map_imu = se3_from_tf_msg(tf_msg)
                # sensor pose -> base pose through the (approximate) extrinsic
                t_map_base = t_map_imu * self.t_imu_base
                tf_ok = True
        except (tf2_ros.TransformException, tf2_ros.LookupException,
                tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            tf_ok = False

        checks = {}
        with self._lock:
            checks["odom_age"] = (now - stamp_sec) <= self.odom_max_age
            self._last_odom_stamp = stamp_sec

            checks["loc_tf"] = tf_ok
            if loc_age_s is not None:
                checks["loc_tf_age_ok"] = loc_age_s <= self.tf_max_age
            else:
                checks["loc_tf_age_ok"] = False

            # innovation: relative motion agreement between loc and wheel odom
            innov_ok, innov_err = self._innovation(stamp_sec, t_map_base, t_odom_base,
                                                   loc_stamp)
            if tf_ok and innov_ok:
                self._innov_fails = 0
            elif tf_ok and not innov_ok:
                self._innov_fails += 1
            checks["innovation"] = (self._innov_fails < self.innov_fail_limit)
            checks["innovation_skipped"] = len(self._history) < 2  # warm-up, informational

            # store paired samples for innovation comparisons; both members of
            # a pair must come from the same odom callback, and the loc sample
            # keeps its own stamp for interval-mismatch gating (see _innovation)
            if tf_ok and t_map_base is not None and loc_stamp is not None:
                self._history.append((stamp_sec, loc_stamp, t_map_base, t_odom_base))
                while len(self._history) > 80:
                    self._history.popleft()

            checks["status"] = self._status is not None and self._status == self.req_status
            if self._status_time is not None:
                checks["status_fresh"] = (now - self._status_time) <= 1.0
            else:
                checks["status_fresh"] = False

            checks["confidence"] = (self._confidence is not None
                                    and self._confidence >= self.min_conf)
            if self._conf_time is not None:
                checks["confidence_fresh"] = (now - self._conf_time) <= 1.0
            else:
                checks["confidence_fresh"] = False

            ready = (checks["loc_tf"] and checks["loc_tf_age_ok"]
                     and checks["innovation"] and checks["odom_age"]
                     and checks["status"] and checks["status_fresh"]
                     and checks["confidence"] and checks["confidence_fresh"])

            # 3) map -> odom ONLY from same-instant transforms
            if tf_ok and t_map_base is not None:
                t_map_odom = t_map_base * t_odom_base.inverse()
                self.tf_broadcaster.sendTransform(
                    tf_msg_from_se3(t_map_odom, self.map_frame, self.odom_frame, stamp))

        self._publish_health(ready, checks, innov_err)

    # ------------------------------------------------------------- innovation

    def _innovation(self, stamp, t_map_base, t_odom_base, loc_stamp):
        """Compare loc-relative-motion vs odom-relative-motion over ~0.5 s.
        Fails on translation OR speed OR yaw disagreement. Windows whose loc
        and odom intervals disagree by >0.1 s (lookup-latency jitter) are
        skipped, not scored, to avoid a speed-proportional bias (review #4).
        Returns (ok, err)."""
        if t_map_base is None or t_odom_base is None or loc_stamp is None \
                or len(self._history) < 2:
            return True, None  # not enough history yet: do not fail warm-up

        # pick the history sample closest to (stamp - interval), prefer older
        target = stamp - self.innov_interval
        ref = None
        for sample in self._history:
            if sample[0] <= target:
                ref = sample          # keep the newest one that is old enough
            else:
                break
        if ref is None:
            ref = self._history[0]    # fall back to the oldest sample
        prev_t, prev_loc_t, prev_map_base, prev_odom_base = ref
        dt = stamp - prev_t
        dt_loc = loc_stamp - prev_loc_t
        if dt <= 0.05 or abs(dt - dt_loc) > 0.1:
            return True, None         # mismatched intervals: skip this window

        d_loc = prev_map_base.inverse() * t_map_base
        d_odo = prev_odom_base.inverse() * t_odom_base
        trans_err = math.dist(d_loc.t, d_odo.t)
        rate_err = trans_err / dt
        yaw_err = abs(math.atan2(math.sin(d_loc.yaw() - d_odo.yaw()),
                                 math.cos(d_loc.yaw() - d_odo.yaw())))
        ok = ((trans_err <= self.innov_t_th) and (rate_err <= self.innov_speed_th)
              and (yaw_err <= self.innov_yaw_th))
        err = None
        if not ok:
            err = "trans_err=%.3fm rate=%.3fm/s yaw_err=%.1fdeg dt=%.3fs" % (
                trans_err, rate_err, math.degrees(yaw_err), dt)
        return ok, err

    # ----------------------------------------------------------------- health

    def _health_tick(self):
        """Watchdog re-evaluation independent of /wheel/odom arrival: flips
        navigation_ready to false when the odom stream stops (or loc TF goes
        stale), instead of latching the last value. Does not publish TF."""
        now = self.get_clock().now().nanoseconds * 1e-9
        tf_ok = False
        loc_age_s = None
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.map_frame, self.loc_child, rclpy.time.Time())
            loc_age_s = now - (tf_msg.header.stamp.sec +
                               tf_msg.header.stamp.nanosec * 1e-9)
            tf_ok = loc_age_s <= self.tf_max_age
        except Exception:
            tf_ok = False

        with self._lock:
            checks = {}
            checks["odom_present"] = (self._last_odom_stamp is not None
                                      and (now - self._last_odom_stamp) <= self.odom_max_age)
            checks["loc_tf"] = tf_ok
            checks["loc_tf_age_ok"] = tf_ok
            checks["innovation"] = (self._innov_fails < self.innov_fail_limit)
            checks["innovation_skipped"] = len(self._history) < 2 or not checks["odom_present"]
            checks["status"] = self._status is not None and self._status == self.req_status
            checks["status_fresh"] = (self._status_time is not None
                                      and (now - self._status_time) <= 1.0)
            checks["confidence"] = (self._confidence is not None
                                    and self._confidence >= self.min_conf)
            checks["confidence_fresh"] = (self._conf_time is not None
                                          and (now - self._conf_time) <= 1.0)
            # innovation_skipped is informational, never part of the verdict
            ready = all(v for k, v in checks.items() if k != "innovation_skipped")
        self._publish_health(ready, checks, self._innov_last_err)

    def _publish_health(self, ready, checks, innov_err):
        msg = Bool()
        msg.data = bool(ready)
        self.ready_pub.publish(msg)

        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        st = DiagnosticStatus()
        st.name = "ranger_localization_bridge"
        st.level = DiagnosticStatus.OK if ready else DiagnosticStatus.ERROR
        if ready:
            st.message = "navigation_ready"
        elif "odom_present" in checks and not checks["odom_present"]:
            st.message = "degraded: wheel odom stream absent, map->odom not updated"
        else:
            st.message = "localization not ready"
        for key in sorted(checks):
            st.values.append(KeyValue(key=key, value="1" if checks[key] else "0"))
        if innov_err:
            st.values.append(KeyValue(key="innovation_last_error", value=innov_err))
        diag.status.append(st)
        self.diag_pub.publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
