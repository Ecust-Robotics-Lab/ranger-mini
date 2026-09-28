#!/usr/bin/env python3
"""阶段 6 单点导航目标发送（map 系）。

用法:
  python3 scripts/send_nav_goal.py --x -0.6 --y 1.9            # 朝向缺省=当前车头朝向(TF)
  python3 scripts/send_nav_goal.py --x -0.6 --y 1.9 --yaw-deg 108
  python3 scripts/send_nav_goal.py --cancel                      # 取消当前所有导航目标

对抗审查后修正：
  - 缺省 --yaw-deg 不再发送全零四元数（会被 Nav2 解析为 map 系 0°朝向），
    改为用 TF 查询 map->base_link 的当前 yaw；TF 不可得则报错要求显式 --yaw-deg。
  - --cancel 原先调 BasicNavigator.cancelTask（只对本进程发过的目标有效，跨进程
    恒空操作）。改为直接调用 navigate_to_pose 动作的 CancelGoal 服务并以空
    goal_id 请求取消全部目标（rcl_action 语义）。
  - 发送前预检 action server 可用性（BasicNavigator.goToPose 对不可用服务器是
    无限等待），不再打印无法兑现的超时承诺。

发送目标本身不等于放行运动：速度是否流动由 twist_mux -> smoother ->
ranger_motion_gate（许可/定位/传感器/运动学门禁）-> collision_monitor 决定。
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.srv import CancelGoal as CancelGoalSrv
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def get_current_yaw(nav):
    """TF 查询 map->base_link 的当前 yaw；失败返回 None。"""
    import tf2_ros
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, nav)
    deadline = time.time() + 4.0
    while time.time() < deadline and rclpy.ok():
        try:
            tr = buf.lookup_transform('map', 'base_link', rclpy.time.Time())
            q = tr.transform.rotation
            return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        except Exception:
            rclpy.spin_once(nav, timeout_sec=0.2)
    return None


def cancel_all(node):
    cli = node.create_client(CancelGoalSrv, 'navigate_to_pose/_action/cancel_goal')
    if not cli.wait_for_service(timeout_sec=5.0):
        print('错误：navigate_to_pose 动作服务不可用（导航栈未在运行？）')
        return 3
    req = CancelGoalSrv.Request()          # 空 goal_id => 取消全部目标（rcl_action）
    fut = cli.call_async(req)
    deadline = time.time() + 5.0
    while not fut.done() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not fut.done():
        print('错误：取消请求超时')
        return 3
    resp = fut.result()
    n = len(resp.goals_canceling)
    print(f'已请求取消：{n} 个目标进入取消流程（return_code={resp.return_code}）')
    return 0


def main():
    a = argparse.ArgumentParser(description=__doc__)
    a.add_argument('--x', type=float, help='目标 x（map 系，米）')
    a.add_argument('--y', type=float, default=0.0)
    a.add_argument('--yaw-deg', type=float, default=None,
                   help='目标朝向（度）；缺省=TF 查询当前车头朝向')
    a.add_argument('--timeout', type=float, default=120.0)
    a.add_argument('--cancel', action='store_true')
    args = a.parse_args()

    rclpy.init()
    nav = BasicNavigator()
    rc = 0
    try:
        if args.cancel:
            rc = cancel_all(nav)
            return rc

        if args.x is None:
            print('错误：需要 --x（或 --cancel）')
            return 2

        if args.yaw_deg is not None:
            yaw = math.radians(args.yaw_deg)
        else:
            yaw = get_current_yaw(nav)
            if yaw is None:
                print('错误：--yaw-deg 缺省时需要可用 TF(map->base_link)；'
                      '定位未就绪或 TF 异常。请显式给 --yaw-deg 或等定位 ready。')
                return 2
            print(f'朝向缺省 -> 当前车头朝向 {math.degrees(yaw):.1f}°（TF）')

        probe = ActionClient(nav, NavigateToPose, 'navigate_to_pose')
        if not probe.wait_for_server(timeout_sec=8.0):
            print('错误：navigate_to_pose 服务器不可用（导航栈未运行/未激活），未发送目标。')
            return 3

        # BasicNavigator.goToPose 接收 PoseStamped（内部自行包装 action goal，
        # 此前误传 NavigateToPose.Goal 整体触发断言）
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = nav.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = args.x, args.y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        print(f'发送目标 map({args.x:.2f}, {args.y:.2f}, yaw={math.degrees(yaw):.1f}°)，'
              f'超时 {args.timeout}s')
        nav.goToPose(pose)
        t0 = time.time()
        while not nav.isTaskComplete():
            if time.time() - t0 > args.timeout:
                cancel_all(nav)
                print('超时，已请求取消。注：若门禁未放行，目标不会驱动车动'
                      '（nav_safety.sh status 查看门禁判定）。')
                return 2
            rclpy.spin_once(nav, timeout_sec=0.1)

        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print('结果: SUCCEEDED（到达目标）')
        elif result == TaskResult.CANCELED:
            print('结果: CANCELED')
            rc = 1
        elif result == TaskResult.FAILED:
            print('结果: FAILED（规划/跟随失败；查 /plan、costmap、TF 与门禁状态）')
            rc = 1
        else:
            print(f'结果: {result}')
            rc = 1
        return rc
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
