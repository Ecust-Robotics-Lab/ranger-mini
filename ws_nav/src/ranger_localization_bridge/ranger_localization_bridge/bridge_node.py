#!/usr/bin/env python3
"""Backend-independent localization/odometry bridge.

Consumes LocalizationEstimate (map-frame base pose, sample time, health).
Publishes map->odom, odom->base_link and the legacy readiness/diagnostics.
Batch 2 retains latest-pose/current-odom composition. Sampling-time alignment
is explicitly deferred to batch 3; do not describe these as same-time poses.
"""
import collections
import math

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from ranger_localization_msgs.msg import LocalizationEstimate

from .geometry import (SE3, quat_normalize, rpy_to_quat, se3_from_tf_msg,
                       tf_msg_from_se3, se3_from_odom)
from .health import fresh, readiness
import tf2_ros


class LocalizationBridge(Node):
    def __init__(self):
        super().__init__('ranger_localization_bridge')
        d = self.declare_parameter
        # Legacy direct invocation composes the adapter in main(). The launch
        # wrapper uses separate processes and explicitly selects external.
        self.adapter_backend = d('adapter_backend', 'lightning').value
        if self.adapter_backend not in ['lightning', 'external']:
            raise ValueError('adapter_backend must be lightning or external')
        self.map_frame = d('map_frame', 'map').value
        self.odom_frame = d('odom_frame', 'odom').value
        self.base_frame = d('base_frame', 'base_link').value
        self.tf_max_age = float(d('tf_max_age_s', 0.5).value)
        self.odom_max_age = float(d('odom_max_age_s', 0.5).value)
        self.heartbeat_max_age = float(d('heartbeat_max_age_s', 0.5).value)
        self.innov_t_th = float(d('innovation_trans_th', 0.5).value)
        self.innov_speed_th = float(d('innovation_speed_th', 0.3).value)
        self.innov_yaw_th = math.radians(float(d('innovation_yaw_th_deg', 20.0).value))
        self.innov_interval = float(d('innovation_interval_s', 0.5).value)
        self.innov_fail_limit = int(d('innovation_fail_limit', 3).value)
        for value in [self.tf_max_age, self.odom_max_age, self.heartbeat_max_age,
                      self.innov_t_th, self.innov_speed_th, self.innov_yaw_th,
                      self.innov_interval, self.innov_fail_limit]:
            if not math.isfinite(value) or value <= 0:
                raise ValueError('bridge thresholds must be finite and positive')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.ready_pub = self.create_publisher(Bool, '/navigation_ready', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/localization_health', 10)
        self.create_subscription(Odometry, d('wheel_odom_topic', '/wheel/odom').value, self.on_odom, 10)
        self.create_subscription(LocalizationEstimate, d('estimate_topic', '/localization/estimate').value, self.on_estimate, 10)
        self.sample = self.received = self._last_odom_stamp = self._last_map_update = None
        self.input_error = None
        self._history = collections.deque(maxlen=80)
        self._innov_fails = 0
        self._innov_last_err = None
        self.create_timer(0.2, self._health_tick)
        self.get_logger().info('generic bridge: %s -> %s -> %s; max pose age %.3fs' %
                               (self.map_frame, self.odom_frame, self.base_frame, self.tf_max_age))

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_estimate(self, msg):
        now = self.now_s()
        q, p = msg.pose.orientation, msg.pose.position
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        valid_state = msg.state in [msg.INITIALIZING, msg.VALID, msg.DEGRADED, msg.LOST, msg.ERROR]
        valid_pose = all(math.isfinite(v) for v in values) and math.isclose(math.hypot(*values[3:]), 1.0, abs_tol=1e-3)
        valid_frames = msg.header.frame_id == self.map_frame and msg.base_frame == self.base_frame
        # INITIALIZING without a pose is a legitimate heartbeat, but never ready.
        if not valid_state or not valid_frames or (msg.state == msg.VALID and
                (not valid_pose or not fresh(now, stamp, self.tf_max_age))):
            self.sample = None
            self.input_error = 'invalid estimate: frame/state/pose/sample time'
        else:
            self.sample = msg
            self.input_error = None
        self.received = now
        self._health_tick()

    def on_odom(self, msg):
        now = self.now_s()
        stamp = msg.header.stamp
        stamp_sec = stamp.sec + stamp.nanosec * 1e-9
        values = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                  msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                  msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        if (not fresh(now, stamp_sec, self.odom_max_age) or
                msg.header.frame_id != self.odom_frame or msg.child_frame_id != self.base_frame or
                not all(math.isfinite(v) for v in values)):
            self._last_odom_stamp = None
            self._health_tick()
            return
        self._last_odom_stamp = stamp_sec
        t_odom_base = se3_from_odom(msg)
        self.tf_broadcaster.sendTransform(tf_msg_from_se3(t_odom_base, self.odom_frame, self.base_frame, stamp))
        sample = self.sample
        loc_stamp = None if sample is None else sample.header.stamp.sec + sample.header.stamp.nanosec * 1e-9
        t_map_base = None
        if sample is not None and sample.state == sample.VALID and fresh(now, loc_stamp, self.tf_max_age):
            p, q = sample.pose.position, sample.pose.orientation
            t_map_base = SE3((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
            ok, err = self._innovation(stamp_sec, t_map_base, t_odom_base, loc_stamp)
            self._innov_fails = 0 if ok else self._innov_fails + 1
            self._innov_last_err = err
            self._history.append((stamp_sec, loc_stamp, t_map_base, t_odom_base))
            # Retained composition policy; batch 3 will interpolate odom at loc_stamp.
            self.tf_broadcaster.sendTransform(tf_msg_from_se3(
                t_map_base * t_odom_base.inverse(), self.map_frame, self.odom_frame, stamp))
            self._last_map_update = stamp_sec
        self._health_tick()

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

    def _health_tick(self):
        ready, checks = readiness(self.now_s(), self.sample, self.received,
            self._last_odom_stamp, self.tf_max_age, self.odom_max_age,
            self.heartbeat_max_age, self._innov_fails < self.innov_fail_limit)
        checks['map_transform'] = fresh(self.now_s(), self._last_map_update, self.odom_max_age)
        ready = ready and checks['map_transform']
        checks['innovation_skipped'] = len(self._history) < 2
        self.ready_pub.publish(Bool(data=ready))
        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        st = DiagnosticStatus()
        st.name = 'ranger_localization_bridge'
        st.level = DiagnosticStatus.OK if ready else DiagnosticStatus.ERROR
        st.message = 'navigation_ready' if ready else (
            'degraded: wheel odom stream absent, map->odom not updated'
            if not checks['odom_present'] else 'localization not ready')
        st.values = [KeyValue(key=k, value='1' if v else '0') for k, v in sorted(checks.items())]
        st.values.append(KeyValue(key='backend_state', value=str(self.sample.state) if self.sample else 'unknown'))
        st.values.append(KeyValue(key='backend_reason', value=self.input_error or
                                  (self.sample.reason if self.sample else 'no estimate')))
        if self.sample:
            st.values.append(KeyValue(key='sample_time', value=str(
                self.sample.header.stamp.sec + self.sample.header.stamp.nanosec * 1e-9)))
        if self._innov_last_err:
            st.values.append(KeyValue(key='innovation_last_error', value=self._innov_last_err))
        diag.status = [st]
        self.diag_pub.publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationBridge()
    nodes = [node]
    if node.adapter_backend == 'lightning':
        from .lightning_adapter import LightningAdapter
        nodes.append(LightningAdapter())
    executor = rclpy.executors.SingleThreadedExecutor()
    for n in nodes:
        executor.add_node(n)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        for n in nodes:
            n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
