#!/usr/bin/env python3
"""Explicit offline C1 conversion to the existing public localization contract."""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from ranger_localization_msgs.msg import LocalizationEstimate
from ranger_fast_livo_localizer.msg import LocalizerEstimate
from .geometry import SE3, rpy_to_quat
from .health import fresh


def validate_message(msg):
    if (msg.header.frame_id != 'map' or msg.imu_frame != 'fast_livo_imu' or not msg.session_id
            or msg.header.stamp.sec < 0 or not 0 <= msg.header.stamp.nanosec < 10**9
            or msg.state not in range(5)):
        return 'invalid private C1 contract'
    if msg.state == msg.VALID and not msg.pose_valid:
        return 'VALID without confirmed C1 pose'
    if msg.pose_valid:
        p, q = msg.pose.position, msg.pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if (not msg.map_uuid or len(msg.manifest_sha256) != 64 or not msg.graph_version
                or not msg.map_generation or not msg.request_generation or not msg.initialization_request_id
                or msg.last_confirmed_stamp_ns != msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
                or not all(math.isfinite(v) for v in values)
                or not math.isclose(math.hypot(*values[3:]), 1., abs_tol=1e-3)):
            return 'invalid confirmed C1 identity/pose/time'
    return None


class LocalizerAdapter(Node):
    def __init__(self):
        super().__init__('ranger_fast_livo_localizer_adapter')
        d = self.declare_parameter
        if not self.get_parameter('use_sim_time').value:
            raise ValueError('C1 adapter is currently offline only: use_sim_time=true required')
        self.max_age = float(d('sample_max_age', .5).value)
        self.heartbeat_age = float(d('heartbeat_max_age', .5).value)
        xyz = d('t_base_imu_xyz', [0., 0., .556]).value
        rpy = d('t_base_imu_rpy', [0., 0., 0.]).value
        if (len(xyz) != 3 or len(rpy) != 3 or not all(math.isfinite(v) for v in [*xyz, *rpy])
                or not all(math.isfinite(v) and v > 0 for v in [self.max_age, self.heartbeat_age])):
            raise ValueError('invalid C1 adapter extrinsics/timeouts')
        self.imu_to_base = SE3(xyz, rpy_to_quat(*rpy)).inverse()
        self.message = self.received = self.last_clock = None
        self.session = None
        self.retired_sessions = set()
        self.last_identity = self.last_ns = None
        self.generation_floor = None
        self.error = None
        self.pub = self.create_publisher(LocalizationEstimate, '/localization/estimate', 10)
        self.create_subscription(LocalizerEstimate, '/fast_livo/localizer/estimate', self.on_estimate, 10)
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_estimate(self, msg):
        now = self.get_clock().now().nanoseconds
        if self.last_clock is not None and now < self.last_clock:
            if self.session:
                self.retired_sessions.add(self.session)
            self.message = self.received = self.last_identity = self.last_ns = self.session = None
            self.generation_floor = None
        self.last_clock = now
        self.error = validate_message(msg)
        identity = (msg.map_generation, msg.request_generation, msg.map_uuid, msg.manifest_sha256, msg.graph_version)
        ns = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if msg.session_id in self.retired_sessions:
            self.error = 'retired C1 session rejected'
        elif not self.error:
            if self.session != msg.session_id:
                if len(self.retired_sessions) >= 128:
                    self.error = 'C1 session history capacity; reopen adapter session'
                else:
                    if self.session:
                        self.retired_sessions.add(self.session)
                    self.session = msg.session_id
                    self.last_identity = self.last_ns = None
                    self.generation_floor = None
            if self.generation_floor is not None and identity[:2] < self.generation_floor:
                self.error = 'C1 announced generation regressed'
            if self.last_identity is not None:
                previous = self.last_identity
                if (identity[:2] < previous[:2] or (msg.pose_valid and identity[:2] == previous[:2] and identity[2:] != previous[2:])
                        or (msg.pose_valid and self.last_ns is not None and ns < self.last_ns)):
                    self.error = 'C1 request/map/time regression rejected'
            if msg.pose_valid and not fresh(now*1e-9, ns*1e-9, self.max_age):
                self.error = 'C1 sample is stale/future'
            if not self.error:
                self.generation_floor = identity[:2]
                if msg.pose_valid:
                    self.last_identity = identity
                    self.last_ns = ns
        # Rejected candidates never replace the accepted identity/time cache.
        self.message, self.received = msg, time.monotonic()
        self.tick()

    def tick(self):
        now = self.get_clock().now().nanoseconds
        out = LocalizationEstimate()
        out.header.frame_id, out.base_frame = 'map', 'base_link'
        out.state, out.reason = out.INITIALIZING, 'waiting for C1 estimate'
        msg = self.message
        if msg is not None:
            out.header.stamp = msg.header.stamp
            out.state, out.reason = msg.state, msg.reason
            out.reason += (f'; map={msg.map_uuid}; graph={msg.graph_version}; session={msg.session_id}; '
                           f'initial={msg.initialization_request_id}; request_generation={msg.request_generation}')
            if self.error:
                out.state, out.reason = out.ERROR, self.error
            elif msg.pose_valid:
                p, q = msg.pose.position, msg.pose.orientation
                pose = SE3((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))*self.imu_to_base
                out.pose.position.x, out.pose.position.y, out.pose.position.z = pose.t
                out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w = pose.q
                if not fresh(now*1e-9, msg.last_confirmed_stamp_ns*1e-9, self.max_age):
                    out.state, out.reason = out.LOST, 'confirmed C1 sample expired'
            if self.received is None or time.monotonic()-self.received > self.heartbeat_age:
                out.state, out.reason = out.LOST, 'C1 heartbeat expired'
            if len(self.get_publishers_info_by_topic('/fast_livo/localizer/estimate')) != 1:
                out.state, out.reason = out.ERROR, 'C1 publisher identity unavailable/nonunique'
            if self.last_clock is not None and now < self.last_clock:
                out.state, out.reason = out.LOST, 'clock regressed; waiting for new C1 session'
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizerAdapter()
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
