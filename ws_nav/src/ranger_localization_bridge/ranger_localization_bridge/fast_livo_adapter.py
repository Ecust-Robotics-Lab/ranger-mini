#!/usr/bin/env python3
"""Explicit offline PGO adapter. No TF broadcaster and no latest-TF lookup."""
import math

import rclpy
from rclpy.node import Node
from ranger_localization_msgs.msg import LocalizationEstimate
from ranger_fast_livo_backend.msg import BackendEstimate
from .geometry import SE3, rpy_to_quat
from .health import fresh


def validate_backend_message(msg):
    if (msg.header.frame_id != 'map' or msg.imu_frame != 'fast_livo_imu' or not msg.session_id
            or msg.header.stamp.sec < 0 or msg.header.stamp.nanosec >= 10**9
            or msg.state not in [0, 1, 2, 3, 4]):
        return 'invalid private backend contract'
    if msg.state == msg.VALID and not msg.pose_valid:
        return 'VALID backend without valid pose'
    if msg.pose_valid:
        p, q = msg.pose.position, msg.pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if not all(math.isfinite(v) for v in values) or not math.isclose(math.hypot(*values[3:]), 1., abs_tol=1e-3):
            return 'invalid backend pose'
    return None


class FastLivoAdapter(Node):
    def __init__(self):
        super().__init__('ranger_fast_livo_adapter')
        d = self.declare_parameter
        self.max_age = float(d('sample_max_age', .5).value)
        self.heartbeat_age = float(d('heartbeat_max_age', .5).value)
        xyz = d('t_base_imu_xyz', [0., 0., .556]).value
        rpy = d('t_base_imu_rpy', [0., 0., 0.]).value
        if (len(xyz) != 3 or len(rpy) != 3 or not all(math.isfinite(x) for x in [*xyz, *rpy])
                or not all(math.isfinite(x) and x > 0 for x in [self.max_age, self.heartbeat_age])):
            raise ValueError('invalid adapter extrinsics/timeouts')
        self.imu_to_base = SE3(xyz, rpy_to_quat(*rpy)).inverse()
        self.message = self.received = self.last_clock = None
        self.session = None
        self.last_ns = self.last_version = None
        self.input_error = None
        self.pub = self.create_publisher(LocalizationEstimate, '/localization/estimate', 10)
        self.create_subscription(BackendEstimate, '/fast_livo/backend/estimate', self.on_estimate, 10)
        self.create_timer(.05, self.tick)

    def on_estimate(self, msg):
        now = self.get_clock().now().nanoseconds
        ns = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
        self.input_error = validate_backend_message(msg)
        if self.last_clock is not None and now < self.last_clock:
            self.message = self.received = self.session = self.last_ns = self.last_version = None
        self.last_clock = now
        eligible = not self.input_error and (not msg.pose_valid or fresh(now*1e-9, ns*1e-9, self.max_age))
        if eligible and msg.session_id != self.session:
            self.session = msg.session_id
            self.last_ns = self.last_version = None
        if eligible and msg.pose_valid and ((self.last_ns is not None and ns < self.last_ns)
                               or (self.last_version is not None and msg.graph_version < self.last_version)):
            self.input_error = 'backend sampling time/graph version regressed'
        if eligible and msg.pose_valid and not self.input_error:
            self.last_ns, self.last_version = ns, msg.graph_version
        self.message, self.received = msg, now*1e-9
        self.tick()

    def tick(self):
        now_ns = self.get_clock().now().nanoseconds
        if self.last_clock is not None and now_ns < self.last_clock:
            self.message = self.received = self.session = self.last_ns = self.last_version = None
        self.last_clock = now_ns
        now = now_ns*1e-9
        out = LocalizationEstimate()
        out.header.frame_id, out.base_frame = 'map', 'base_link'
        out.state, out.reason = out.INITIALIZING, 'waiting for private PGO estimate'
        msg = self.message
        if msg is not None:
            out.header.stamp = msg.header.stamp
            out.state, out.reason = msg.state, msg.reason
            out.reason += f'; session={msg.session_id}; graph={msg.graph_version}; loop_event={msg.loop_event_id}'
            p, q = msg.pose.position, msg.pose.orientation
            values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            if (msg.header.frame_id != 'map' or msg.imu_frame != 'fast_livo_imu' or not msg.session_id
                    or msg.header.stamp.sec < 0 or msg.header.stamp.nanosec >= 10**9
                    or msg.state not in [0, 1, 2, 3, 4] or self.input_error):
                out.state, out.reason = out.ERROR, self.input_error or 'invalid private backend contract'
            elif msg.pose_valid and (not all(math.isfinite(v) for v in values)
                                     or not math.isclose(math.hypot(*values[3:]), 1., abs_tol=1e-3)):
                out.state, out.reason = out.ERROR, 'invalid backend pose'
            elif msg.pose_valid:
                body = SE3(values[:3], values[3:]) * self.imu_to_base
                out.pose.position.x, out.pose.position.y, out.pose.position.z = body.t
                out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w = body.q
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
                if not fresh(now, stamp, self.max_age) or not fresh(now, self.received, self.heartbeat_age):
                    out.state, out.reason = out.LOST, 'backend pose/heartbeat expired'
            elif msg.state == msg.VALID:
                out.state, out.reason = out.ERROR, 'VALID backend without valid pose'
            if not fresh(now, self.received, self.heartbeat_age):
                out.state, out.reason = out.LOST, 'backend heartbeat expired'
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FastLivoAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
