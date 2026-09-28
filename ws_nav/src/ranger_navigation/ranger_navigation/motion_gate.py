#!/usr/bin/env python3
"""ranger_motion_gate: last kinematic/permission authority BEFORE collision_monitor.

Position in the velocity chain (stage 6 design):

  Nav2 controller / teleop
    -> twist_mux (arbitration)
    -> nav2_velocity_smoother (smoothing)
    -> THIS NODE (motion permission, speed caps, Ackermann kinematics)
    -> nav2_collision_monitor (LAST; nothing may re-smooth after it)
    -> /base/safe_cmd_vel (ranger driver's only velocity input)

Publishes a continuous zero/valid stream on its output topic so the chain
never goes silent while this node is alive. Gate fails CLOSED: any failed
condition outputs exactly zero.

Permission model (AGENTS.md: per-run authorization, not surviving restarts):
  - /ranger/motion_enable (std_msgs/Bool, VOLATILE): sticky state, starts False
    on every process start; True enables, False disables. Deliberately NOT
    transient_local so a latched/alive enabler cannot re-arm a restarted gate
    by itself: re-enabling after any restart is an explicit human action
    (scripts/motion_permission.sh).
  - /ranger/soft_estop (std_msgs/Bool): True LATCHES an emergency stop until an
    explicit False is received.
  - /navigation_ready (std_msgs/Bool, from ranger_localization_bridge) must be
    True and fresh; localization not ready -> no motion.
  - /livox/points_rviz header must be fresh; obstacle sensor stale -> no motion
    (compensates collision_monitor's fail-open behaviour on TF/staleness).

Kinematic enforcement (Ranger Mini V2, ugv_sdk params: min_turn_radius=0.4764,
the driver switches to SPINNING below it and to lateral PARALLEL when vy!=0):
  - vy must be ~0, otherwise the whole command is rejected to zero (no lateral).
  - vx is clamped to [0, max_vx]: no reverse in v1.
  - |vx|/|wz| must stay >= min_turn_radius_gate (> chassis 0.4764); violating
    wz is clipped down; wz at near-zero vx is zeroed (no in-place rotation).
"""

import json
import math

import rclpy
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool

EPS = 1e-9


def enforce_kinematics(vx, vy, wz, max_vx, max_wz, min_turn_radius, vx_turn_min,
                       wz_eps=1e-6, vy_eps=1e-6, vx_eps=1e-9):
    """Pure gate function. Returns (vx, vy, wz, flags).

    Flags list which enforcement actions were applied; empty means the command
    passed unchanged. A 'lateral_rejected' or 'invalid' flag means the whole
    command was zeroed.
    """
    flags = []
    vals = (vx, vy, wz)
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in vals):
        return 0.0, 0.0, 0.0, ['invalid']
    if abs(vy) > vy_eps:
        # Lateral command: the chassis would switch to PARALLEL/SIDE_SLIP mode.
        # Reject the entire command (fail closed), do not try to "fix" it.
        return 0.0, 0.0, 0.0, ['lateral_rejected']
    if vx < -vx_eps:
        vx = 0.0
        flags.append('reverse_clamped')
    # 顺序要求：先做 vx/wz 限幅、后做半径裁剪（对抗审查确认的原 bug：先用未限幅
    # vx 裁半径再截 vx，输出半径会跌破底盘 0.4764 触发驱动 SPINNING 切换，
    # 例：vx=0.25,wz=0.5 -> 旧序输出 (0.20,0.4545)=半径 0.44）。
    if vx > max_vx:
        vx = max_vx
        flags.append('vx_capped')
    if abs(wz) > max_wz:
        wz = math.copysign(max_wz, wz)
        flags.append('wz_capped')
    if abs(wz) > wz_eps:
        if abs(vx) < vx_turn_min:
            # Pure/near-pure rotation: chassis would switch to SPINNING mode.
            wz = 0.0
            flags.append('spin_prevented')
        else:
            radius = abs(vx) / abs(wz)
            if radius < min_turn_radius:
                wz = math.copysign(abs(vx) / min_turn_radius, wz)
                flags.append('radius_clipped')
                if abs(wz) > max_wz:  # 防御：min_turn_radius*max_wz < max_vx 时兜底
                    wz = math.copysign(max_wz, wz)
    return float(vx), 0.0, float(wz), flags


class MotionGateNode(Node):

    def __init__(self):
        super().__init__('ranger_motion_gate')
        d = self.declare_parameter
        self.publish_rate = float(d('publish_rate', 20.0).value)
        self.max_vx = float(d('max_vx', 0.20).value)
        self.max_wz = float(d('max_wz', 0.60).value)
        self.min_turn_radius = float(d('min_turn_radius', 0.55).value)
        self.vx_turn_min = float(d('vx_turn_min', 0.05).value)
        self.cmd_timeout = float(d('cmd_timeout', 0.5).value)
        self.ready_timeout = float(d('ready_timeout', 1.0).value)
        self.cloud_timeout = float(d('cloud_timeout', 1.0).value)

        self.enabled = False           # sticky motion permission, False on start
        self.estop = False             # latched soft e-stop
        self.ready_time = None
        self.ready_value = False
        self.cloud_time = None
        self.cmd_time = None
        self.cmd = Twist()
        self.last_flags = ['startup']
        self._was_passing = False
        self._ramp_t0 = None

        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel_in', self.cmd_cb, 1)
        self.enable_sub = self.create_subscription(Bool, 'motion_enable', self.enable_cb, 10)
        self.estop_sub = self.create_subscription(Bool, 'soft_estop', self.estop_cb, 10)
        self.ready_sub = self.create_subscription(Bool, '/navigation_ready', self.ready_cb, 10)
        self.cloud_sub = self.create_subscription(PointCloud2, 'obstacle_cloud', self.cloud_cb, 10)

        self.out_pub = self.create_publisher(Twist, 'cmd_vel_out', 1)
        self.permitted_pub = self.create_publisher(Bool, 'motion_permitted', 1)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 1)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.tick)
        self.get_logger().info(
            'ranger_motion_gate up: permission=False (issue motion_enable:=true to arm); '
            f'max_vx={self.max_vx} max_wz={self.max_wz} min_turn_radius={self.min_turn_radius}')

    # -- callbacks ---------------------------------------------------------
    def cmd_cb(self, msg):
        if not all(math.isfinite(v) for v in (msg.linear.x, msg.linear.y, msg.angular.z)):
            self.get_logger().error('non-finite input twist ignored')
            return
        self.cmd = msg
        self.cmd_time = self.get_clock().now()

    def enable_cb(self, msg):
        if self.enabled != bool(msg.data):
            self.get_logger().warn(f'motion permission -> {bool(msg.data)}')
        self.enabled = bool(msg.data)

    def estop_cb(self, msg):
        if msg.data and not self.estop:
            self.get_logger().error('SOFT E-STOP latched (publish soft_estop:=false to clear)')
        if not msg.data and self.estop:
            self.get_logger().warn('soft e-stop cleared')
        self.estop = bool(msg.data)

    def ready_cb(self, msg):
        self.ready_value = bool(msg.data)
        self.ready_time = self.get_clock().now()

    def cloud_cb(self, msg):
        # time_type=0 host-receive stamps share this node's clock. Header stamp
        # zero (unset) falls back to arrival time.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.cloud_time = self.get_clock().now() if stamp <= 0.0 else \
            rclpy.time.Time.from_msg(msg.header.stamp)

    # -- main loop ---------------------------------------------------------
    def tick(self):
        now = self.get_clock().now()

        ready_fresh = (self.ready_time is not None and
                       (now - self.ready_time).nanoseconds * 1e-9 <= self.ready_timeout)
        ready_ok = ready_fresh and self.ready_value
        cloud_fresh = (self.cloud_time is not None and
                       (now - self.cloud_time).nanoseconds * 1e-9 <= self.cloud_timeout)
        cmd_fresh = (self.cmd_time is not None and
                     (now - self.cmd_time).nanoseconds * 1e-9 <= self.cmd_timeout)

        passing = self.enabled and not self.estop and ready_ok and cloud_fresh and cmd_fresh

        out = Twist()
        flags = []
        if passing:
            out.linear.x, out.linear.y, out.angular.z, flags = enforce_kinematics(
                self.cmd.linear.x, self.cmd.linear.y, self.cmd.angular.z,
                self.max_vx, self.max_wz, self.min_turn_radius, self.vx_turn_min)
            # 再武装斜坡：阻断→放行瞬间不透传 smoother 的驻留指令（0→0.15 阶跃
            # 交给底盘未验收的默认加速度），从 0 以 0.30 m/s² 爬升。
            if not self._was_passing:
                self._ramp_t0 = now
            ramp_allow = 0.30 * (now - self._ramp_t0).nanoseconds * 1e-9
            if out.linear.x > ramp_allow:
                out.linear.x = ramp_allow
                flags.append('rearm_ramp')
        else:
            self._ramp_t0 = None
            flags = []
            if not self.enabled:
                flags.append('permission_off')
            if self.estop:
                flags.append('soft_estop')
            if not ready_ok:
                flags.append('navigation_not_ready' if self.ready_time is not None
                             else 'navigation_no_data')
            if not cloud_fresh:
                flags.append('obstacle_cloud_stale' if self.cloud_time is not None
                             else 'obstacle_cloud_no_data')
            if not cmd_fresh:
                flags.append('cmd_stale_or_absent')

        self._was_passing = passing
        self.out_pub.publish(out)
        self.permitted_pub.publish(Bool(data=passing))
        self.last_flags = flags

        status = DiagnosticStatus()
        status.name = 'ranger_motion_gate'
        status.level = DiagnosticStatus.OK if (passing and not flags) else DiagnosticStatus.WARN
        if self.estop or (self.enabled and not ready_ok and self.ready_time is not None):
            status.level = DiagnosticStatus.ERROR
        status.message = 'passing' if passing else 'blocked: ' + ','.join(flags)
        status.values = [
            KeyValue(key='permission', value=str(self.enabled)),
            KeyValue(key='soft_estop', value=str(self.estop)),
            KeyValue(key='navigation_ready', value=str(ready_ok)),
            KeyValue(key='obstacle_cloud_fresh', value=str(cloud_fresh)),
            KeyValue(key='cmd_fresh', value=str(cmd_fresh)),
            KeyValue(key='flags', value=json.dumps(flags)),
        ]
        diag = DiagnosticArray()
        diag.header.stamp = now.to_msg()
        diag.status.append(status)
        self.diag_pub.publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = MotionGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # rclpy's global SIGINT handler may already have shut the context down
        for fini in (node.destroy_node, rclpy.shutdown):
            try:
                fini()
            except Exception:
                pass


if __name__ == '__main__':
    main()
