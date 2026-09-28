"""Stage 6 single-point navigation bringup. Default is DRY-RUN.

Actual start requires execute:=true AND control_authorized:=true (fresh,
per AGENTS.md the authorization does not survive process restarts). Even when
started, the chain outputs zeros until /ranger/motion_enable is set true on
ranger_motion_gate.

Expected environment: field_session (lidar, base driver, localization, bridge)
already running. This launch never starts the chassis driver itself.

Velocity chain (nothing after collision_monitor):
  controller_server -> /cmd_vel_nav -> twist_mux -> /cmd_vel_mux
  -> velocity_smoother -> /cmd_vel_gated_in -> ranger_motion_gate
  -> /cmd_vel_after_gate -> collision_monitor -> /base/safe_cmd_vel
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def prepare(context):
    if LaunchConfiguration('execute').perform(context) != 'true':
        return [LogInfo(msg='DRY-RUN navigation stack. Components that would start: '
                       'map_server, planner_server(Smac Hybrid-A*), controller_server(MPPI), '
                       'bt_navigator(single_point.xml, no recovery), velocity_smoother, '
                       'twist_mux, ranger_motion_gate, ranger_obstacle_filter, '
                       'collision_monitor. Chain output: /base/safe_cmd_vel (zeros until '
                       'motion_enable). Re-run with execute:=true control_authorized:=true.')]
    if LaunchConfiguration('control_authorized').perform(context) != 'true':
        raise RuntimeError('This start requires fresh explicit control authorization.')

    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory('ranger_navigation'))

    map_yaml = LaunchConfiguration('map_yaml').perform(context)
    if not Path(map_yaml).exists():
        raise RuntimeError(f'map file not found: {map_yaml}')

    nav_params = str(share / 'config' / 'nav2_params.yaml')
    cm_params = str(share / 'config' / 'collision_monitor.yaml')
    mux_params = str(share / 'config' / 'twist_mux.yaml')
    gate_params = str(share / 'config' / 'motion_gate.yaml')
    filt_params = str(share / 'config' / 'obstacle_filter.yaml')
    bt_xml = str(share / 'behavior_trees' / 'single_point.xml')

    common = {'output': 'screen', 'respawn': False, 'emulate_tty': True}

    nodes = [
        # -- plain nodes (not lifecycle managed) ----------------------------
        Node(package='ranger_navigation', executable='obstacle_filter',
             name='ranger_obstacle_filter', parameters=[filt_params],
             remappings=[('cloud_in', '/livox/points_rviz'),
                         ('obstacles_out', '/livox/obstacles')], **common),
        Node(package='ranger_navigation', executable='motion_gate',
             name='ranger_motion_gate', parameters=[gate_params],
             remappings=[('cmd_vel_in', '/cmd_vel_gated_in'),
                         ('cmd_vel_out', '/cmd_vel_after_gate'),
                         ('motion_enable', '/ranger/motion_enable'),
                         ('soft_estop', '/ranger/soft_estop'),
                         ('motion_permitted', '/ranger/motion_permitted'),
                         # 监视过滤器输出（与 CM/局部代价地图同源）：过滤器死亡时
                         # 门禁立即阻断，而不是被上游原始云蒙蔽（对抗审查 major）
                         ('obstacle_cloud', '/livox/obstacles')], **common),
        Node(package='twist_mux', executable='twist_mux', name='twist_mux',
             parameters=[mux_params], remappings=[('cmd_vel_out', '/cmd_vel_mux')], **common),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', parameters=[nav_params],
             remappings=[('cmd_vel', '/cmd_vel_mux'),
                         ('cmd_vel_smoothed', '/cmd_vel_gated_in')], **common),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', parameters=[cm_params], **common),
        # -- lifecycle nodes (managed below) --------------------------------
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[nav_params, {'yaml_filename': map_yaml}], **common),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=[nav_params], **common),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             parameters=[nav_params], remappings=[('cmd_vel', '/cmd_vel_nav')], **common),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             parameters=[nav_params, {'default_nav_to_pose_bt_xml': bt_xml,
                                      'default_nav_through_poses_bt_xml': bt_xml}], **common),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation',
             parameters=[{'use_sim_time': False,
                          'autostart': True,
                          # collision_monitor 放首位：多节点并发启动时 manager 的
                          # get_state 与 on_configure 存在竞态（复现见 06 证据），
                          # 安静期先配置最安全关键的节点
                          'node_names': ['collision_monitor', 'map_server', 'planner_server',
                                         'controller_server', 'bt_navigator',
                                         'velocity_smoother']}], **common),
    ]
    return nodes


def generate_launch_description():
    # 2026-09-28 重录地图（g2p5 二维 8cw1afzl，起点净空 0.326 m 无需副本）
    default_map = ('/home/lab307/ranger_nav/maps/lightning-offline-8cw1afzl'
                   '/data/new_map/map.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('execute', default_value='false', choices=['false', 'true']),
        DeclareLaunchArgument('control_authorized', default_value='false'),
        DeclareLaunchArgument('map_yaml', default_value=default_map),
        OpaqueFunction(function=prepare),
    ])
