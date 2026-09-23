#!/usr/bin/env python3
"""Stage 5 synthetic tests for ranger_localization_bridge (no hardware).

Part A: pure-math checks (SE3 helpers, map->odom identity, zero-quaternion guard).
Part B: cross-process ROS test in an isolated ROS_DOMAIN_ID (bridge runs in a
subprocess like real deployment; single-process multi-node DDS discovery proved
unreliable in this env). Each phase verifies ITS named gate via
/localization_health, TF checks compare VALUES against the fake's known ground
truth (loc == odom trajectory, T_base_imu = identity => map->odom ~ identity),
and phases are separated by settling to a stable ready=True baseline so no
leftover transient can satisfy a check for the wrong reason (v2, rewritten
after adversarial review; the v1 checks were shown vacuous).
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "ws_nav/src/ranger_localization_bridge"))

# force (not setdefault): a pre-exported domain must not silently break isolation
os.environ["ROS_DOMAIN_ID"] = "179"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, Int32

import tf2_ros

from ranger_localization_bridge.bridge_node import (SE3, quat_normalize,
                                                    rpy_to_quat, se3_from_tf_msg,
                                                    tf_msg_from_se3)

RESULTS = []


def check(name, cond, detail=""):
    RESULTS.append((name, bool(cond)))
    print(("PASS " if cond else "FAIL ") + name + (" | " + detail if detail else ""))


# ---------------------------------------------------------------- Part A ----

def test_math():
    a = SE3((1.0, -2.0, 0.5), rpy_to_quat(0.1, -0.2, 0.7))
    b = SE3((0.3, 0.4, -0.1), rpy_to_quat(-0.05, 0.3, -1.2))
    r = (a * b) * b.inverse()
    check("A.se3_roundtrip", math.dist(r.t, a.t) < 1e-9 and math.dist(r.q, a.q) < 1e-9)

    back = (a * b.inverse()) * b
    check("A.map_odom_identity", math.dist(back.t, a.t) < 1e-9)

    # a default-constructed (all-zero) quaternion must not raise
    q = quat_normalize((0.0, 0.0, 0.0, 0.0))
    check("A.zero_quat_guard", q == (0.0, 0.0, 0.0, 1.0))


# ---------------------------------------------------------------- Part B ----

class FakeWorld(Node):
    """Lightning-like TF/status/confidence + wheel odom on one shared trajectory.

    Phases:
      run      - everything consistent (ground truth: x=2t, y=0.3t^2, yaw=0.15t)
      stall_tf - loc TF stops; status/confidence KEEP publishing (pure TF stall)
      jump     - loc keeps continuity but claims +1 m/s extra speed from t_switch
      no_odom  - odom stops; loc keeps publishing (watchdog must drop ready)
    """

    def __init__(self):
        super().__init__("fake_world")
        self._t0 = time.monotonic()          # per instance, not import-time
        self.phase = "run"
        self.phase_t0 = self._t0
        self.br = tf2_ros.TransformBroadcaster(self)
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.status_pub = self.create_publisher(Int32, "/lightning/loc_status", qos)
        self.conf_pub = self.create_publisher(Float32, "/lightning/loc_confidence", qos)
        self.odom_pub = self.create_publisher(Odometry, "/wheel/odom", qos)
        self.last_ready = None
        self.consec_true = 0
        self.ready_msgs = 0                  # messages seen in current phase
        self.last_health = {}                # KeyValue name -> value ("1"/"0")
        self.health_msg = None
        self.create_subscription(Bool, "/navigation_ready", self._on_ready, qos)
        from diagnostic_msgs.msg import DiagnosticArray
        self.create_subscription(DiagnosticArray, "/localization_health",
                                 self._on_health, qos)
        self.create_timer(0.01, self.tick_loc)       # 100 Hz loc TF
        self.create_timer(0.01, self.tick_status)    # 100 Hz status/conf (always on)
        self.create_timer(0.02, self.tick_odom)      # 50 Hz odom

    def _on_ready(self, msg):
        self.last_ready = msg.data
        self.ready_msgs += 1
        self.consec_true = self.consec_true + 1 if msg.data else 0

    def _on_health(self, msg):
        self.health_msg = msg.status[0].message if msg.status else None
        self.last_health = {kv.key: kv.value for kv in msg.status[0].values} \
            if msg.status else {}

    def _stamp(self, back=0.0):
        return (self.get_clock().now() +
                rclpy.duration.Duration(seconds=-back)).to_msg()

    def _t(self):
        return time.monotonic() - self._t0

    def tick_status(self):
        self.status_pub.publish(Int32(data=2))
        self.conf_pub.publish(Float32(data=3.0))

    def tick_loc(self):
        if self.phase == "stall_tf":
            return
        t = self._t()
        x, y, yaw = 2.0 * t, 0.3 * t * t, 0.15 * t
        if self.phase == "jump":
            # continuous from phase entry: pure sustained +1 m/s divergence
            x += 1.0 * (t - (self.phase_t0 - self._t0))
        pose = SE3((x, y, 0.0), rpy_to_quat(0.0, 0.0, yaw))
        self.br.sendTransform(tf_msg_from_se3(pose, "map", "imu_link", self._stamp(0.05)))

    def tick_odom(self):
        if self.phase == "no_odom":
            return
        t = self._t()
        msg = Odometry()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = 2.0 * t
        msg.pose.pose.position.y = 0.3 * t * t
        yaw = 0.15 * t
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.odom_pub.publish(msg)

    def set_phase(self, phase):
        self.phase = phase
        self.phase_t0 = time.monotonic()
        self.ready_msgs = 0
        self.consec_true = 0


BRIDGE_PROC = None  # set in test_ros; spin() fail-fasts if the bridge dies


def spin(fake, step=0.05, batch=10):
    """Drain a small batch of callbacks per cycle: the fake receives ~300
    msg/s (TF+status+conf+ready+health) and a single spin_once per cycle
    starves the 5 Hz health/ready messages (observed as missed watchdog
    frames in B.odom_stop_watchdog)."""
    global BRIDGE_PROC
    if BRIDGE_PROC is not None and BRIDGE_PROC.poll() is not None:
        raise RuntimeError("bridge subprocess died rc=%s" % BRIDGE_PROC.poll())
    for _ in range(batch):
        rclpy.spin_once(fake, timeout_sec=0.0)
    rclpy.spin_once(fake, timeout_sec=step)


def settle_true(fake, n=5, timeout=20.0):
    """Drain transient Falses: require n consecutive ready=True messages."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        spin(fake)
        if fake.consec_true >= n:
            return True
    return False


def hold_and_check(fake, seconds, condition, min_msgs=10, grace_s=0.0):
    """Spin a fixed window; return (condition_held_everywhere, msgs_seen,
    violation_detail, collected_health_snapshots). condition is evaluated on
    per-message state only (never pair last_ready with last_health: they are
    separate topics and can skew); gate attribution is post-hoc from the
    collected health snapshots. Samples within grace_s of entry are ignored
    (debounce transients are by design)."""
    t0 = time.monotonic()
    msgs0 = fake.ready_msgs
    violated = None
    healths = []
    while time.monotonic() - t0 < seconds:
        spin(fake)
        healths.append(dict(fake.last_health))
        if time.monotonic() - t0 >= grace_s and not condition():
            violated = (fake.last_ready, dict(fake.last_health))
    msgs = fake.ready_msgs - msgs0
    return (violated is None and msgs >= min_msgs), msgs, violated, healths


def tf_se3(buf, parent, child):
    try:
        return se3_from_tf_msg(buf.lookup_transform(parent, child, rclpy.time.Time()))
    except Exception:
        return None


def test_ros():
    import subprocess
    global BRIDGE_PROC
    rclpy.init()
    pkg = os.path.join(os.path.dirname(__file__), "..",
                       "ws_nav/src/ranger_localization_bridge")
    env = dict(os.environ)
    env["PYTHONPATH"] = pkg + os.pathsep + env.get("PYTHONPATH", "")
    bridge_proc = subprocess.Popen(
        [sys.executable, "-m", "ranger_localization_bridge.bridge_node",
         "--ros-args", "-p", "t_base_imu_xyz:=[0.0,0.0,0.0]",
         "-p", "tf_max_age_s:=0.4", "-p", "odom_max_age_s:=0.4"],
        env=env)
    BRIDGE_PROC = bridge_proc
    try:
        fake = FakeWorld()
        buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(buf, fake)

        # baseline: stable ready=True (5 consecutive) before any phase test
        check("B.ready_true", settle_true(fake, n=5, timeout=25.0))

        # VALUE checks against ground truth (loc==odom, identity extrinsic):
        # map->odom must stay ~identity (loc TF lags 0.05 s => ~0.1 m at 2 m/s)
        ok_v = False
        t0 = time.monotonic()
        while time.monotonic() - t0 < 5.0 and not ok_v:
            spin(fake)
            mo = tf_se3(buf, "map", "odom")
            mb = tf_se3(buf, "map", "base_link")
            if mo is not None and mb is not None:
                t = fake._t()
                ok_v = (math.dist(mo.t, (0, 0, 0)) < 0.3 and abs(mo.yaw()) < 0.1
                        and abs(mb.t[0] - 2.0 * t) < 0.4
                        and abs(mb.t[1] - 0.3 * t * t) < 0.4)
        check("B.tf_values_vs_ground_truth", ok_v)

        # pure loc-TF stall (status/conf keep flowing): the TF-staleness gate
        # itself must eventually fire. Innovation may trip FIRST (frozen TF +
        # moving odom is a legitimate divergence) - so wait specifically for a
        # health message with loc_tf/loc_tf_age_ok == 0 while status/conf == 1
        check("B.stall_settle_precondition", settle_true(fake, n=3, timeout=10.0))
        fake.set_phase("stall_tf")
        stalled_ok = False
        t0 = time.monotonic()
        while time.monotonic() - t0 < 8.0:
            spin(fake)
            h = fake.last_health
            if fake.last_ready is False and h \
                    and (h.get("loc_tf") == "0" or h.get("loc_tf_age_ok") == "0") \
                    and h.get("status") == "1" and h.get("confidence") == "1":
                stalled_ok = True
                break
        check("B.stall_tf_gate_fires", stalled_ok,
              detail="health=%s" % fake.last_health)

        # sustained +1 m/s loc speed divergence: after the by-design debounce
        # (~3 failing callbacks) ready must go false and STAY false while >=50
        # messages arrive (with the gate deleted the bridge would flip true).
        # Invariant is judged on /navigation_ready alone; attribution (which
        # gate fired) is post-hoc from collected health snapshots.
        # resume loc TF first: the precondition must be reached in a healthy
        # state, otherwise the jump invariant would pass for the wrong reason
        fake.set_phase("run")
        check("B.jump_settle_precondition", settle_true(fake, n=3, timeout=25.0))
        fake.set_phase("jump")
        ok_hold, msgs, viol, window_healths = hold_and_check(
            fake, seconds=3.0, grace_s=1.0,
            condition=lambda: fake.last_ready is not True,
            min_msgs=50)
        innov_failed = any(h.get("innovation") == "0" or "innovation_last_error" in h
                           for h in window_healths if h)
        check("B.jump_sustained_never_ready", ok_hold and innov_failed,
              detail="msgs=%d violated=%s innov_failed=%s" % (msgs, viol, innov_failed))

        # recovery
        fake.set_phase("run")
        check("B.recover_ready_true", settle_true(fake, n=5, timeout=15.0))

        # odom stream stops while loc keeps publishing: watchdog (not the odom
        # callback) must drop ready within ~1.5 s and mark odom_present=0
        fake.set_phase("no_odom")
        wd_ok = False
        t0 = time.monotonic()
        while time.monotonic() - t0 < 4.0:
            spin(fake)
            if fake.last_ready is False and fake.last_health.get("odom_present") == "0":
                wd_ok = True
                break
        check("B.odom_stop_watchdog", wd_ok, detail="health=%s" % fake.last_health)

        fake.destroy_node()
    finally:
        bridge_proc.terminate()
        try:
            bridge_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            bridge_proc.kill()
        rclpy.shutdown()


if __name__ == "__main__":
    test_math()
    test_ros()
    failed = [n for n, ok in RESULTS if not ok]
    print(f"\n{len(RESULTS) - len(failed)}/{len(RESULTS)} checks passed")
    sys.exit(1 if failed else 0)
