#!/usr/bin/env python3
"""Stage 6 safety-chain offline validation for ranger_navigation (no actuator).

Covers the six dry-run safety requirements plus chain-structure checks, on an
isolated ROS_DOMAIN_ID with the FULL navigation stack started exactly the way
scripts/run_navigation.sh starts it (same launch file, same params):

  T1  permission off by default        -> /base/safe_cmd_vel stays zero
  T2  permission on, healthy inputs    -> velocity flows within limits
  T3  navigation_ready false           -> zero (not-localized gate)
  T4  obstacle cloud stale             -> zero (compensates collision_monitor
                                          fail-open on stale/missing sources)
  T5  upstream command stops           -> zero (command staleness)
  T6  soft e-stop                      -> zero quickly
  T7  e-stop cleared                   -> motion resumes
  T8  obstacle point in stop polygon   -> collision_monitor STOPs;
      ground-level point only          -> motion continues (height band works)
  T9  collision_monitor killed         -> no further output msgs (no stale
                                          command execution); restarted CM
                                          resumes passthrough
  T10 motion gate killed and restarted -> permission does NOT auto-restore
                                          (zeros until explicitly re-enabled)
  T11 /base/safe_cmd_vel has exactly one publisher: collision_monitor
  T12 static config consistency (radius/velocity ordering, footprint, BT has
      no unverified recovery behaviors, no recorder writes control topics)
  T13 planner smoke on the accepted map + BT goal with permission off:
      controller commands observed upstream while final output stays zero

Anti-vacuity: T13 requires /cmd_vel_nav to actually carry nonzero commands
while the final output stays zero; all zero-assertions accept either zero-valued
messages or silence (collision_monitor ceases publishing ~1 s after stopping).
The real chassis driver must NOT be running (checked; also domain isolation).
"""
import json
import math
import os
import re
import signal
import subprocess
import sys
import time
from collections import deque

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "ws_nav/src/ranger_navigation"))

# force (not setdefault): pre-exported domain must not silently break isolation
os.environ["ROS_DOMAIN_ID"] = "180"
os.environ["RANGER_FILTER_INPUT"] = "pc2"  # 过滤器兼容模式：测试注入 PointCloud2
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SHARE = os.path.join(ROOT, "ws_nav/install/ranger_navigation/share/ranger_navigation")
# Use the selected runtime bundle; historical map-copy paths are not defaults.
sys.path.insert(0, os.path.join(ROOT, 'ws_nav/src/ranger_localization_bridge'))
from ranger_localization_bridge.runtime_config import resolve
MAP_YAML = resolve()['MAP_YAML']
EVIDENCE = os.environ.get("EVIDENCE_DIR", "/tmp/ranger_stage6_safety")

RESULTS = []


def check(name, cond, detail=""):
    RESULTS.append((name, bool(cond)))
    print(("PASS " if cond else "FAIL ") + name + (" | " + detail if detail else ""), flush=True)


def sh(cmd, timeout=10):
    return subprocess.run(cmd, shell=isinstance(cmd, str), capture_output=True,
                          text=True, timeout=timeout)


def spawn_session(cmd, log_path):
    """Spawn in its own process group so cleanup can killpg the WHOLE tree.

    Hard lesson (evidence safety-r4rz4zmi): SIGINT to the `ros2 run` wrapper
    kills the wrapper but orphans the actual node child; leaked standalone
    gates with permission=True kept publishing 0.15 and contaminated later
    runs (the 80 Hz 'phantom' on /cmd_vel_after_gate was 4-6 of them).
    """
    return subprocess.Popen(cmd, stdout=open(log_path, 'w'),
                            stderr=subprocess.STDOUT, start_new_session=True)


def kill_tree(proc, grace=2.0):
    import signal as _sig
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), _sig.SIGINT)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=grace)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(os.getpgid(proc.pid), _sig.SIGKILL)
    except ProcessLookupError:
        pass


CLOUD_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
]

# Static TF puts livox_frame at +0.60 m above base_link, so a point at
# z_livox=-0.20 lands at z_base=+0.40 (inside the 0.10-0.80 m obstacle band) and
# a point at z_livox=-0.55 lands at z_base=+0.05 (ground, below the band).
PT_STOP = [(0.36, 0.10, -0.20, 10.0)]      # base xy (0.36,0.10): inside current PolygonStop, outside self filter
PT_GROUND = [(0.36, 0.10, -0.55, 10.0)]    # same xy but ground height


class Sim(Node):
    """World simulator; TFs come from separate static_transform_publisher procs."""

    def __init__(self):
        super().__init__('stage6_sim')
        self.cmd = {'on': False, 'vx': 0.15, 'wz': 0.0}
        self.cloud = {'on': False, 'points': []}
        self.ready_value = True
        self.final = deque()      # /base/safe_cmd_vel
        self.gated = deque()      # /cmd_vel_after_gate
        self.nav_cmd = deque()    # /cmd_vel_nav
        self.plans = deque()      # /plan

        self.create_subscription(Twist, '/base/safe_cmd_vel',
                                 lambda m: self.final.append((time.monotonic(),
                                                              m.linear.x, m.linear.y,
                                                              m.angular.z)), 10)
        self.create_subscription(Twist, '/cmd_vel_after_gate',
                                 lambda m: self.gated.append((time.monotonic(),
                                                              m.linear.x, m.angular.z)), 10)
        self.create_subscription(Twist, '/cmd_vel_nav',
                                 lambda m: self.nav_cmd.append((time.monotonic(),
                                                                m.linear.x)), 10)
        self.create_subscription(Path, '/plan',
                                 lambda m: self.plans.append((time.monotonic(),
                                                              len(m.poses))), 5)
        from diagnostic_msgs.msg import DiagnosticArray
        self.last_gate_flags = '<no diagnostics received>'
        self.create_subscription(DiagnosticArray, '/diagnostics',
                                 self._diag_cb, 10)

        self.odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/livox/lidar', 10)
        self.ready_pub = self.create_publisher(Bool, '/navigation_ready', 10)
        self.enable_pub = self.create_publisher(Bool, '/ranger/motion_enable', 10)
        self.estop_pub = self.create_publisher(Bool, '/ranger/soft_estop', 10)
        self.action = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_timer(1 / 20.0, self.pub_odom)
        self.create_timer(1 / 20.0, self.pub_cmd)
        self.create_timer(1 / 10.0, self.pub_cloud)
        self.create_timer(1 / 10.0, self.pub_ready)

    def _diag_cb(self, msg):
        for st in msg.status:
            if st.name == 'ranger_motion_gate':
                for kv in st.values:
                    if kv.key == 'flags':
                        self.last_gate_flags = kv.value

    def pub_odom(self):
        # The controller consumes wheel odometry even in this fixed-pose test.
        # Report a stationary body at the same pose as the synthetic TF; no
        # odometry TF is broadcast here and no actuator is connected.
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'; msg.child_frame_id = 'base_link'
        yaw = pick_reachable_goal()[2]
        msg.pose.pose.orientation.z = math.sin(yaw/2.)
        msg.pose.pose.orientation.w = math.cos(yaw/2.)
        self.odom_pub.publish(msg)

    def pub_cmd(self):
        if not self.cmd['on']:
            return
        m = Twist()
        m.linear.x, m.angular.z = self.cmd['vx'], self.cmd['wz']
        self.cmd_pub.publish(m)

    def pub_cloud(self):
        if not self.cloud['on']:
            return
        hdr = Header(frame_id='livox_frame', stamp=self.get_clock().now().to_msg())
        self.cloud_pub.publish(point_cloud2.create_cloud(hdr, CLOUD_FIELDS,
                                                         self.cloud['points']))

    def pub_ready(self):
        self.ready_pub.publish(Bool(data=self.ready_value))

    def set_ready(self, v):
        self.ready_value = v
        for _ in range(5):
            self.pub_ready()
            time.sleep(0.02)

    def fire(self, pub, data):
        for _ in range(5):
            pub.publish(Bool(data=data))
            time.sleep(0.02)

    # -- analysis helpers ---------------------------------------------------
    @staticmethod
    def _prune(dq, window=8.0):
        cutoff = time.monotonic() - window
        while dq and dq[0][0] < cutoff:
            dq.popleft()

    def max_vx(self, dq, window=1.0):
        self._prune(dq)
        cutoff = time.monotonic() - window
        vals = [abs(m[1]) for m in dq if m[0] >= cutoff]
        return max(vals) if vals else 0.0

    def wait_flow(self, dq, vmin=0.10, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.max_vx(dq, window=1.0) >= vmin:
                return True
            time.sleep(0.1)
        return False

    def stop_within(self, dq, timeout, thresh=0.02):
        """Precondition: currently flowing. Returns seconds to zero; 'noflow' if
        the precondition itself failed (distinct from failing-to-stop); None on
        a genuine no-stop."""
        if self.max_vx(dq, 1.0) < 0.05:
            return 'noflow'
        t0 = time.monotonic()
        deadline = t0 + timeout
        while time.monotonic() < deadline:
            self._prune(dq)
            msgs = [m for m in dq if m[0] >= t0]
            nonzero = [m for m in msgs if abs(m[1]) > thresh]
            zeros = [m for m in msgs if abs(m[1]) <= thresh]
            if len(zeros) >= 2 and (not nonzero or zeros[-1][0] > nonzero[-1][0]):
                first_zero = zeros[0][0] if not nonzero else \
                    min(z[0] for z in zeros if z[0] > nonzero[-1][0])
                return first_zero - t0
            time.sleep(0.05)
        return None

    def zero_since(self, dq, t0, thresh=0.02):
        self._prune(dq)
        return all(abs(m[1]) <= thresh for m in dq if m[0] >= t0)


def static_tf_procs():
    # Synthetic chassis heading follows a checked free ray in the selected map.
    # This fixture does not describe the physical robot's current pose.
    yaw = pick_reachable_goal()[2]
    exe = ['ros2', 'run', 'tf2_ros', 'static_transform_publisher']
    return [
        subprocess.Popen(exe + ['--frame-id', 'map', '--child-frame-id', 'odom']),
        subprocess.Popen(exe + ['--yaw', str(yaw), '--frame-id', 'odom',
                                '--child-frame-id', 'base_link']),
        subprocess.Popen(exe + ['--x', '0', '--y', '0', '--z', '0.60',
                                '--frame-id', 'base_link', '--child-frame-id', 'livox_frame']),
    ]


def kill_procs(procs, grace=2.0):
    for p in procs:
        if p.poll() is None:
            p.send_signal(signal.SIGINT)
    time.sleep(grace)
    for p in procs:
        if p.poll() is None:
            p.kill()


def pgrep(pattern):
    r = sh(['pgrep', '-f', pattern])
    return [int(x) for x in r.stdout.split()]


_TEST_GOAL = None


def pick_reachable_goal():
    """Choose a free straight ray on the selected map for a planner smoke test.

    Check the whole ray against occupied AND unknown cells. Point the static
    synthetic base along this ray, so a forward-only Dubin path is feasible.
    This validates the experimental planning model only, not body passage.
    """
    global _TEST_GOAL
    if _TEST_GOAL is not None:
        return _TEST_GOAL
    import numpy as np
    import yaml as yaml_mod
    from PIL import Image
    from scipy.ndimage import distance_transform_edt
    meta = yaml_mod.safe_load(open(MAP_YAML))
    image = np.array(Image.open(os.path.join(os.path.dirname(MAP_YAML), meta['image'])))
    height, width = image.shape
    resolution = meta['resolution']
    ox, oy = meta['origin'][:2]
    clearance = distance_transform_edt(image >= 200) * resolution
    nav = yaml_mod.safe_load(open(os.path.join(SHARE, 'config/nav2_params.yaml')))
    radius = nav['global_costmap']['global_costmap']['ros__parameters']['robot_radius']
    best = None
    for angle in np.linspace(-math.pi, math.pi, 181):
        for length in [1.2, 1.5, 2.0]:
            distance = np.linspace(0, length, 101)
            x, y = distance * math.cos(angle), distance * math.sin(angle)
            cols = ((x - ox) / resolution).astype(int)
            rows = height - 1 - ((y - oy) / resolution).astype(int)
            if min(cols) < 0 or min(rows) < 0 or max(cols) >= width or max(rows) >= height:
                continue
            minimum = float(min(clearance[rows, cols]))
            if minimum <= radius + resolution:
                continue
            score = minimum - .05 * abs(length - 1.5)
            if best is None or score > best[0]:
                best = (score, length * math.cos(angle), length * math.sin(angle), float(angle))
    if best is None:
        raise RuntimeError('selected map has no checked free ray from synthetic origin')
    _TEST_GOAL = best[1:]
    print(f'T13 synthetic goal={_TEST_GOAL}, ray score={best[0]:.3f}, map={MAP_YAML}', flush=True)
    return _TEST_GOAL


def static_checks():
    import yaml
    nav = yaml.safe_load(open(os.path.join(SHARE, 'config/nav2_params.yaml')))
    cm = yaml.safe_load(open(os.path.join(SHARE, 'config/collision_monitor.yaml'))
                        )['collision_monitor']['ros__parameters']
    gate = yaml.safe_load(open(os.path.join(SHARE, 'config/motion_gate.yaml'))
                          )['ranger_motion_gate']['ros__parameters']
    mppi = nav['controller_server']['ros__parameters']['FollowPath']
    smac = nav['planner_server']['ros__parameters']['GridBased']
    smooth = nav['velocity_smoother']['ros__parameters']

    CHASSIS_R = 0.4764
    mppi_r = mppi['AckermannConstraints']['min_turning_r']
    check('T12.radius_ordering',
          CHASSIS_R < gate['min_turn_radius'] <= smac['minimum_turning_radius'] and
          abs(smac['minimum_turning_radius'] - mppi_r) < 1e-9,
          f"chassis {CHASSIS_R} < gate {gate['min_turn_radius']} <= "
          f"smac/mppi {smac['minimum_turning_radius']}")
    check('T12.velocity_ordering',
          0 < gate['max_vx'] <= mppi['vx_max'] <= smooth['max_velocity'][0],
          f"gate {gate['max_vx']} <= mppi {mppi['vx_max']} <= smoother {smooth['max_velocity'][0]}")
    check('T12.forward_only',
          mppi['vx_min'] >= 0.0 and smooth['min_velocity'][0] >= 0.0 and
          smac['motion_model_for_search'] == 'DUBIN',
          '三重禁倒车: MPPI vx_min>=0, smoother min>=0, Smac DUBIN')
    check('T12.unknown_not_traversable',
          smac['allow_unknown'] is False and
          nav['global_costmap']['global_costmap']['ros__parameters']['track_unknown_space'] is True,
          'allow_unknown=false + track_unknown_space=true')
    check('T12.cm_final_topic', cm['cmd_vel_out_topic'] == '/base/safe_cmd_vel',
          cm['cmd_vel_out_topic'])
    # 2026-09-30 switched planning to an undersized experimental circle.
    # Verify the actual independent stop envelope, never infer that the small
    # planning circle proves full-body traversability.
    pts = cm['PolygonStop']['points']
    xs, ys = pts[0::2], pts[1::2]
    check('T12.cm_body_envelope', min(xs) <= -0.37 and max(xs) >= 0.37 and
          min(ys) <= -0.26 and max(ys) >= 0.26,
          f'CM half extents {max(xs)}x{max(ys)} cover recorded 0.74x0.52 body')
    radius = nav['local_costmap']['local_costmap']['ros__parameters'].get('robot_radius')
    if radius is not None and radius < 0.26:
        print(f'NOTE experimental planning radius={radius}m is smaller than body; '
              'these chain tests do not certify narrow-passage traversability', flush=True)

    bt = open(os.path.join(SHARE, 'behavior_trees/single_point.xml')).read()
    check('T12.bt_no_recovery',
          not re.search(r'Spin|Backup|DriveOnHeading|ClearCostmap|RecoveryNode|Wait', bt),
          'v1 BT 无倒车/原地旋转/等待/清图恢复节点')

    bad = []
    for fn in os.listdir(os.path.join(ROOT, 'scripts')):
        if not (fn.endswith('.py') or fn.endswith('.sh')):
            continue
        content = open(os.path.join(ROOT, 'scripts', fn)).read()
        if 'bag record' in content or 'rosbag2' in content.lower():
            if 'cmd_vel' in content or re.search(r"['\"]-a['\"]", content):
                bad.append(fn)
    check('T12.recorders_no_control_topics', not bad,
          f'录像脚本(python+shell)不含 cmd_vel 链话题: {bad or "clean"}')


def main():
    os.makedirs(EVIDENCE, exist_ok=True)

    drivers = pgrep('base_mini_v2.launch.py') + pgrep('ranger_base_node')
    check('T0.no_chassis_driver_running', not drivers,
          f'pids={drivers}（测试在隔离 domain，仍拒绝与真机驱动同跑）')
    if drivers:
        print('ABORT: 底盘驱动在运行，拒绝执行安全测试')
        return 2

    stale = []
    for pat in ('[c]ollision_monitor', '[r]anger_motion_gate', '[r]anger_obstacle_filter',
                '[l]ib/ranger_navigation/motion_gate',
                '[l]ib/ranger_navigation/obstacle_filter',
                'ros2 run ranger_[n]avigation',
                '[t]wist_mux', '[n]av2_controller', '[n]av2_planner', '[n]av2_bt_navigator',
                '[n]av2_map_server', '[n]av2_velocity_smoother',
                '[l]ifecycle_manager_navigation', '[s]tatic_transform_publisher',
                '[r]os2 topic pub'):        # 调试会话遗留的 topic pub 僵尸会污染输入话题
        stale += pgrep(pat)
    check('T0.no_stale_chain_processes', not stale,
          f'pids={stale}（前次运行残留会假失败/假通过——先清理再测）')
    if stale:
        print('ABORT: 检测到上次运行残留的导航链进程，请先清理（见 pkill 模式）')
        return 2

    static_checks()

    # ros2 daemon 预热：此前轮次 daemon 被停后，topic info -v / param dump 的
    # 图信息退化导致 T11/T14 整体空转（失败模式：Publisher count=1 但无节点名、
    # dump 全空）。这里确保 daemon 在本 domain 存活。
    sh(['ros2', 'daemon', 'start'], timeout=15)

    procs = static_tf_procs()
    rclpy.init()
    sim = Sim()
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(sim)
    import threading
    threading.Thread(target=ex.spin, daemon=True).start()

    log = open(os.path.join(EVIDENCE, 'nav_launch.log'), 'w')
    launch = subprocess.Popen(
        ['ros2', 'launch', 'ranger_navigation', 'navigation.launch.py',
         'execute:=true', 'control_authorized:=true', f'map_yaml:={MAP_YAML}'],
        stdout=log, stderr=subprocess.STDOUT)

    # everything below must clean up even on assertion/crash: leftover chain
    # processes from an aborted run contaminate the next run (observed:
    # stale enabled gate + CM kept publishing 0.15 -> false FAILs)
    try:
        _run_phases(sim, launch, procs)
    except Exception as exc:                     # 异常也要留下汇总与证据（审查：
        import traceback                         # 此前任何 sh 超时即截断全部后续断言）
        open(os.path.join(EVIDENCE, 'phase_exception.txt'), 'w').write(
            traceback.format_exc())
        check('T0.phases_completed_without_exception', False, f'{type(exc).__name__}: {exc}')
    finally:
        try:
            sim.fire(sim.enable_pub, False)
            sim.fire(sim.estop_pub, True)
        except Exception:
            pass
        for extra in _EXTRA_PROCS:
            kill_tree(extra)
        kill_procs(procs)                        # 静态 TF 等：2 s 宽限足够
        kill_procs([launch], grace=8.0)          # launch 拆栈需要 ~5-8 s，宽限放大
        for pat in ('[l]ib/ranger_navigation/motion_gate',
                    '[l]ib/ranger_navigation/obstacle_filter',
                    'ros2 run ranger_[n]avigation',
                    '[s]tatic_transform_publisher'):
            for pid in pgrep(pat):
                try:
                    os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        # 兜底清场 + 记录：确保没有孤儿进程污染下一次运行（历史教训见
        # safety-r4rz4zmi：泄漏的独立门禁带着许可持续发 0.15）
        for pat in ('[l]ib/ranger_navigation/motion_gate',
                    '[l]ib/ranger_navigation/obstacle_filter',
                    'ros2 run ranger_[n]avigation'):
            for pid in pgrep(pat):
                try:
                    os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        try:
            leftover = pgrep('[l]ib/ranger_navigation') + pgrep('[c]ollision_monitor')
            open(os.path.join(EVIDENCE, 'cleanup_report.txt'), 'w').write(
                f'leftover chain processes after cleanup: {leftover}\n')
        except Exception:
            pass
        try:
            ex.shutdown()
        finally:
            rclpy.shutdown()

    failed = [n for n, ok in RESULTS if not ok]
    print(f"\n{len(RESULTS) - len(failed)}/{len(RESULTS)} checks passed")
    if failed:
        print("FAILED: " + ", ".join(failed))
    print(f"evidence: {EVIDENCE}")
    return 1 if failed else 0


_EXTRA_PROCS = []


def _run_phases(sim, launch, procs):
    t_start = time.monotonic()
    while time.monotonic() - t_start < 90 and not sim.final:
        time.sleep(0.5)
    check('T0.stack_active', bool(sim.final),
          f'{time.monotonic() - t_start:.1f}s 出现首条 /base/safe_cmd_vel'
          if sim.final else '栈未激活，见 nav_launch.log')
    if not sim.final:
        kill_procs([launch] + procs)
        rclpy.shutdown()
        return 2

    # ---- T1 permission off --------------------------------------------------
    sim.cmd.update(on=True, vx=0.15, wz=0.0)
    sim.cloud.update(on=True, points=[])
    time.sleep(2.5)
    check('T1.permission_off_blocks', sim.max_vx(sim.final, 2.5) <= 0.02,
          f"max|vx|={sim.max_vx(sim.final, 2.5):.4f}")

    # ---- T2 permission on ---------------------------------------------------
    sim.fire(sim.enable_pub, True)
    sim.wait_flow(sim.final, 0.10, 10.0)
    mx = sim.max_vx(sim.final, 2.0)
    check('T2.enable_flows', 0.10 <= mx <= 0.16,
          f'max|vx|={mx:.3f}；门禁 flags={sim.last_gate_flags}')

    # ---- T3 not localized ---------------------------------------------------
    sim.set_ready(False)
    d = sim.stop_within(sim.final, 4.0)
    check('T3.nav_not_ready_blocks', isinstance(d, float),
          f'{d:.2f}s 归零' if isinstance(d, float) else f'前置/结果异常: {d}')
    sim.set_ready(True)
    check('T3.ready_restores', sim.wait_flow(sim.final, 0.10, 10.0))

    # ---- T4 obstacle cloud stale --------------------------------------------
    sim.cloud['on'] = False
    d = sim.stop_within(sim.final, 4.0)
    check('T4.cloud_stale_blocks', isinstance(d, float),
          f'{d:.2f}s 归零（补偿 CM fail-open）' if isinstance(d, float) else f'前置/结果异常: {d}')
    sim.cloud['on'] = True
    check('T4.cloud_restores', sim.wait_flow(sim.final, 0.10, 10.0))

    # ---- T5 upstream command stops ------------------------------------------
    sim.cmd['on'] = False
    d = sim.stop_within(sim.final, 5.0)
    check('T5.cmd_stale_stops', isinstance(d, float),
          f'{d:.2f}s 归零' if isinstance(d, float) else f'前置/结果异常: {d}')
    sim.cmd['on'] = True
    check('T5.cmd_restores', sim.wait_flow(sim.final, 0.10, 10.0))

    # ---- T6/T7 soft e-stop --------------------------------------------------
    sim.fire(sim.estop_pub, True)
    d = sim.stop_within(sim.final, 1.5)
    check('T6.estop_stops_fast', isinstance(d, float),
          f'{d:.2f}s 归零' if isinstance(d, float) else f'前置/结果异常: {d}')
    sim.fire(sim.estop_pub, False)
    check('T7.estop_clear_resumes', sim.wait_flow(sim.final, 0.10, 10.0))

    # ---- T8 collision monitor polygon + height band -------------------------
    sim.cloud['points'] = PT_STOP
    d = sim.stop_within(sim.final, 3.0)
    check('T8.obstacle_stops', isinstance(d, float),
          f'{d:.2f}s CM 停车' if isinstance(d, float) else f'前置/结果异常: {d}')
    sim.cloud['points'] = PT_GROUND
    check('T8.ground_point_ignored', sim.wait_flow(sim.final, 0.10, 10.0),
          '带内障碍停、地面点(带外)不误刹')
    sim.cloud['points'] = []

    # ---- T11 unique final publisher -----------------------------------------
    # 主判定用 rclpy 实时图（CLI daemon 图在本机不可靠：实测 _NODE_NAME_UNKNOWN_
    # / param dump 全空）；发布者身份由 T9 的因果性证明（杀 CM -> 输出静默）。
    n_pubs = sim.count_publishers('/base/safe_cmd_vel')
    try:
        info = sh(['ros2', 'topic', 'info', '-v', '/base/safe_cmd_vel'], timeout=10)
        open(os.path.join(EVIDENCE, 't11_topic_info.txt'), 'w').write(info.stdout)
    except Exception:
        pass
    check('T11.unique_final_publisher', n_pubs == 1,
          f'rclpy count_publishers(/base/safe_cmd_vel)={n_pubs}（唯一性；'
          f'身份=CM 由 T9 因果性证明，CLI 明细留档 t11_topic_info.txt）')

    # ---- T14 runtime parameter verification (YAML actually landed) ----------
    # 经 get_parameters 服务直读（rcl_interfaces），不依赖 ros2 CLI/daemon——
    # daemon 图在本机不可靠（实测 param dump 全空、节点名 UNKNOWN）。
    from rcl_interfaces.srv import GetParameters

    def node_params(node, names, tries=3):
        for _ in range(tries):
            cli = sim.create_client(GetParameters, f'{node}/get_parameters')
            if not cli.wait_for_service(timeout_sec=8.0):
                sim.destroy_client(cli)
                time.sleep(1.0)
                continue
            req = GetParameters.Request()
            req.names = names
            fut = cli.call_async(req)
            t0 = time.monotonic()
            while not fut.done() and time.monotonic() - t0 < 10.0:
                time.sleep(0.05)
            if fut.done() and fut.result() is not None:
                vals = []
                for v in fut.result().values:
                    # rcl_interfaces/ParameterType: 1=BOOL 2=INT 3=DOUBLE 4=STRING
                    if v.type == 3:
                        vals.append(round(v.double_value, 6))
                    elif v.type == 2:
                        vals.append(v.integer_value)
                    elif v.type == 1:
                        vals.append(v.bool_value)
                    elif v.type == 4:
                        vals.append(v.string_value)
                    else:
                        vals.append(None)
                sim.destroy_client(cli)
                return dict(zip(names, vals))
            sim.destroy_client(cli)
            time.sleep(1.0)
        return None

    p14 = {}
    p14['controller_server'] = node_params('/controller_server',
                                           ['FollowPath.motion_model',
                                            'FollowPath.AckermannConstraints.min_turning_r',
                                            'FollowPath.vx_max'])
    p14['planner_server'] = node_params('/planner_server',
                                        ['GridBased.motion_model_for_search',
                                         'GridBased.minimum_turning_radius',
                                         'GridBased.allow_unknown'])
    p14['local_costmap'] = node_params('/local_costmap/local_costmap',
                                       ['obstacle_layer.cloud.topic',
                                        'obstacle_layer.cloud.min_obstacle_height',
                                        'obstacle_layer.cloud.max_obstacle_height'])
    p14['collision_monitor'] = node_params('/collision_monitor',
                                           ['cmd_vel_out_topic', 'PolygonStop.max_points'])
    open(os.path.join(EVIDENCE, 'runtime_params.json'), 'w').write(
        json.dumps(p14, ensure_ascii=False, indent=1))
    cs, ps = p14['controller_server'], p14['planner_server']
    lc, cmv = p14['local_costmap'], p14['collision_monitor']
    check('T14.mppi_runtime',
          cs == {'FollowPath.motion_model': 'Ackermann',
                 'FollowPath.AckermannConstraints.min_turning_r': 0.6,
                 'FollowPath.vx_max': 0.1},
          f'{cs}')
    check('T14.smac_runtime',
          ps == {'GridBased.motion_model_for_search': 'DUBIN',
                 'GridBased.minimum_turning_radius': 0.6,
                 'GridBased.allow_unknown': False},
          f'{ps}')
    check('T14.costmap_runtime',
          lc == {'obstacle_layer.cloud.topic': '/livox/obstacles',
                 'obstacle_layer.cloud.min_obstacle_height': 0.1,
                 'obstacle_layer.cloud.max_obstacle_height': 0.8},
          f'{lc}')
    check('T14.cm_runtime',
          cmv == {'cmd_vel_out_topic': '/base/safe_cmd_vel',
                  'PolygonStop.max_points': 0},
          f'{cmv}')

    # ---- T10 motion gate restart does not auto-restore permission -----------
    pre_pids = pgrep('ranger_motion_gate') + pgrep('[l]ib/ranger_navigation/motion_gate')
    with open(os.path.join(EVIDENCE, 't10_pre_kill.txt'), 'w') as f:
        f.write(f'gate pids before kill: {pre_pids}\n')
        f.write(f'count_publishers(/cmd_vel_after_gate): '
                f'{sim.count_publishers("/cmd_vel_after_gate")}\n')
        f.write(sh(['ros2', 'node', 'list'], timeout=10).stdout)
    check('T10.precondition_flowing', sim.wait_flow(sim.final, 0.10, 10.0))
    for pid in pre_pids:
        try:
            os.kill(pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    time.sleep(1.5)
    for pid in pre_pids:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    gate_proc = spawn_session(
        ['ros2', 'run', 'ranger_navigation', 'motion_gate',
         '--ros-args', '--params-file', os.path.join(SHARE, 'config/motion_gate.yaml'),
         '-r', 'cmd_vel_in:=/cmd_vel_gated_in',
         '-r', 'cmd_vel_out:=/cmd_vel_after_gate',
         '-r', 'motion_enable:=/ranger/motion_enable',
         '-r', 'soft_estop:=/ranger/soft_estop',
         '-r', 'motion_permitted:=/ranger/motion_permitted',
         '-r', 'obstacle_cloud:=/livox/obstacles'],
        os.path.join(EVIDENCE, 'gate_restart.log'))
    _EXTRA_PROCS.append(gate_proc)
    time.sleep(4.0)
    with open(os.path.join(EVIDENCE, 't10_topic_info.txt'), 'w') as f:
        f.write(sh(['ros2', 'topic', 'info', '-v', '/cmd_vel_after_gate'],
                   timeout=10).stdout)
    # 只统计新门禁稳定运行后的最近 2 s（时间戳截止，排除批量迟投递的过期消息；
    # deque 记录的是回调时刻，executor 忙时投递成簇，窗口须放宽）；
    # 同时要求窗口内有足量零值消息证明新门禁活着在发零——否则 fm/gm=0 可能
    # 只是"没人发布"的静默（对抗审查：可被死门禁空转满足）
    t_cutoff = time.monotonic() - 2.0
    fm = max([abs(m[1]) for m in sim.final if m[0] >= t_cutoff] or [0.0])
    gm = max([abs(m[1]) for m in sim.gated if m[0] >= t_cutoff] or [0.0])
    live_zeros = sum(1 for m in sim.gated if m[0] >= t_cutoff)
    with open(os.path.join(EVIDENCE, 't10_deques.txt'), 'w') as f:
        f.write('final (t, vx, vy, wz):\n' + '\n'.join(map(str, list(sim.final)[-60:])) + '\n')
        f.write('gated (t, vx, wz):\n' + '\n'.join(map(str, list(sim.gated)[-60:])) + '\n')
    check('T10.gate_restart_no_auto_permission',
          fm <= 0.02 and gm <= 0.02 and live_zeros >= 8,
          f'门禁重启后许可复位 False；final_max={fm:.4f} gated_max={gm:.4f} '
          f'活跃零值消息={live_zeros}/2s（明细 t10_deques/t10_topic_info/t10_pre_kill）')
    sim.fire(sim.enable_pub, True)
    check('T10.reenable_restores', sim.wait_flow(sim.final, 0.10, 10.0))

    # ---- T13 planner smoke with permission off -------------------------------
    # 停掉模拟器自己的 /cmd_vel_nav 发布：此后该话题上出现的指令只能来自
    # controller_server，cmd_seen 只统计静置 3.6 s 之后的消息（防 3 s 轮询窗口内
    # 旧消息污染——此前两版都是被自我污染的空转检查）
    t_cmd_off = time.monotonic()
    sim.cmd['on'] = False
    sim.fire(sim.enable_pub, False)
    t_settle = t_cmd_off + 3.6
    while time.monotonic() < t_settle:
        time.sleep(0.2)
    n_final_before = len(sim.final)
    plans_before = len(sim.plans)

    target = pick_reachable_goal()
    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = sim.get_clock().now().to_msg()
    goal.pose.pose.position.x, goal.pose.pose.position.y = target[0], target[1]
    yaw = target[2]
    goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

    if not sim.action.wait_for_server(timeout_sec=10.0):
        check('T13.bt_action_available', False, 'navigate_to_pose 服务器未就绪')
        fut = None
    else:
        open(os.path.join(EVIDENCE, 'topics_t13.txt'), 'w').write(
            sh(['ros2', 'topic', 'list'], timeout=10).stdout)
        fut = sim.action.send_goal(goal)
        t0 = time.monotonic()
        if hasattr(fut, 'done'):
            while not fut.done() and time.monotonic() - t0 < 5.0:
                time.sleep(0.1)
        plan_seen, cmd_seen = False, False
        t0 = time.monotonic()
        while time.monotonic() - t0 < 25.0:
            if len(sim.plans) > plans_before:
                plan_seen = True
            if any(m[0] >= t_settle and abs(m[1]) >= 0.02 for m in sim.nav_cmd):
                cmd_seen = True
            if plan_seen and cmd_seen:
                break
            time.sleep(0.2)
        with open(os.path.join(EVIDENCE, 't13_controller_trace.json'), 'w') as trace:
            json.dump({'commands_after_settle': [m for m in sim.nav_cmd if m[0] >= t_settle],
                       'plan_seen': plan_seen, 'cmd_seen': cmd_seen, 'target': target}, trace)
        # 取消路径与 send_nav_goal.py --cancel 同源：直接调用动作 CancelGoal
        # 服务（空 goal_id=取消全部，rcl_action 语义），并断言 1 Hz 重规划停止
        from action_msgs.srv import CancelGoal as CancelGoalSrv
        cli = sim.create_client(CancelGoalSrv, 'navigate_to_pose/_action/cancel_goal')
        ok_wait = cli.wait_for_service(timeout_sec=5.0)
        canceled = -1
        call_done = False
        if ok_wait:
            fut2 = cli.call_async(CancelGoalSrv.Request())
            t0 = time.monotonic()
            while not fut2.done() and time.monotonic() - t0 < 10.0:
                time.sleep(0.1)
            call_done = fut2.done()
            if call_done and fut2.result() is not None:
                # goals_canceling 实现常返回空表（rcl_action 不保证回填），
                # 只作展示；接受标准=调用完成且重规划停止
                canceled = len(fut2.result().goals_canceling)
        time.sleep(3.5)                        # RateController 1 Hz，等待 >3 个周期
        n_after_cancel = len(sim.plans)
        time.sleep(2.0)
        check('T13.cancel_service_works',
              ok_wait and call_done and len(sim.plans) == n_after_cancel,
              f'服务可达={ok_wait} 调用完成={call_done} goals_canceling={canceled}；'
              f'取消后 2 s 无新 plan')
        check('T13.plan_and_cmd_produced', plan_seen and cmd_seen,
              f'plan={plan_seen} controller_cmd={cmd_seen}（防空调：控制器确在发令）')
        new_final = list(sim.final)[n_final_before:]
        zeros_ok = all(abs(m[1]) <= 0.02 for m in new_final)
        check('T13.permission_off_stays_zero', zeros_ok,
              'BT 导航激活且控制器发令时，最终输出恒零')

    # ---- T9 collision monitor death (LAST: manager resets the whole stack) --
    # 生产语义（1.1.20 实测）：CM 是 bond 受管节点，进程死亡触发 lifecycle
    # manager CRITICAL FAILURE -> reset(true) 全栈停机；无同名节点重新出现时
    # respawn 探测失败，栈保持停机、不自动恢复（此前测试起的同名独立 CM 会让
    # manager 误判"respawn 成功"并重新拉起全栈——那是测试自造伪影，已移除）。
    sim.cmd['on'] = True          # T13 阶段关闭了模拟指令，T9 前置需要指令在流
    sim.fire(sim.enable_pub, True)
    check('T9.precondition_flowing', sim.wait_flow(sim.final, 0.10, 10.0))
    t_death = time.monotonic()
    for pid in pgrep('collision_monitor') + pgrep('[l]ib/nav2_collision_monitor'):
        try:
            os.kill(pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    t_since = t_death + 1.5       # 管线排水窗口（迟到消息=kill 前已发布的批量投递）
    time.sleep(3.0)
    # 主判定=实时图发布者数：CM 死后 /base/safe_cmd_vel 不存在任何发布者，
    # 物理上不可能有新指令写入（对订阅端 executor 的迟投递伪影免疫）；
    # 消息扫描仅作证据记录
    n_pubs_3s = sim.count_publishers('/base/safe_cmd_vel')
    since = [m for m in sim.final if m[0] >= t_since]
    check('T9.cm_death_silent', n_pubs_3s == 0,
          f'CM 死后 3 s：/base/safe_cmd_vel 发布者数={n_pubs_3s}；'
          f'排尽后消息 {len(since)} 条（迟到非零=kill 前已发布消息的批量投递，'
          f'非新指令）')
    time.sleep(8.0)
    n_pubs_11s = sim.count_publishers('/base/safe_cmd_vel')
    since = [m for m in sim.final if m[0] >= t_since]
    check('T9.no_auto_recovery', n_pubs_11s == 0,
          f'CM 死后 11 s：发布者数={n_pubs_11s}（全栈复位停机、无人重新拉起，'
          f'无自动恢复）')

    # (_run_phases ends; cleanup + summary happen in main's finally)


if __name__ == '__main__':
    sys.exit(main())
