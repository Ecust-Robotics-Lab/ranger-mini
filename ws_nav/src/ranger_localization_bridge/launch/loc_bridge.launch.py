"""Compatibility bringup: Lightning adapter plus generic bridge, or external backend.

Starts no chassis driver. All pose/status policy is forwarded explicitly.
Runtime defaults come from config/runtime.yaml; launch args override these.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from ranger_localization_bridge.runtime_config import resolve


def prepare(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    backend = value('localization_backend')
    common = {'map_frame': 'map', 'base_frame': 'base_link',
              'tf_max_age_s': float(value('tf_max_age_s'))}
    bridge = dict(common, adapter_backend='external',
                  odom_max_age_s=float(value('odom_max_age_s')),
                  odom_history_s=float(value('odom_history_s')),
                  odom_max_gap_s=float(value('odom_max_gap_s')),
                  heartbeat_max_age_s=float(value('heartbeat_max_age_s')))
    nodes = [Node(package='ranger_localization_bridge', executable='ranger_localization_bridge',
                  name='ranger_localization_bridge', output='screen', parameters=[bridge])]
    if backend == 'lightning':
        adapter = dict(common, loc_child_frame=value('loc_child_frame'),
            t_base_imu_xyz=yaml.safe_load(value('t_base_imu_xyz')),
            t_base_imu_rpy=yaml.safe_load(value('t_base_imu_rpy')),
            min_confidence=float(value('min_confidence')),
            status_max_age_s=float(value('status_max_age_s')),
            confidence_max_age_s=float(value('confidence_max_age_s')))
        nodes.append(Node(package='ranger_localization_bridge', executable='lightning_adapter',
                          name='ranger_lightning_adapter', output='screen', parameters=[adapter]))
    return nodes


def generate_launch_description():
    cfg = resolve()
    args = [DeclareLaunchArgument('localization_backend', default_value=cfg['LOCALIZATION_BACKEND'],
                                  choices=['lightning', 'standard'])]
    args.append(DeclareLaunchArgument('loc_child_frame', default_value=cfg['LOC_CHILD_FRAME']))
    for key in ['t_base_imu_xyz', 't_base_imu_rpy', 'min_confidence', 'tf_max_age_s',
                'odom_max_age_s', 'odom_history_s', 'odom_max_gap_s', 'heartbeat_max_age_s', 'status_max_age_s', 'confidence_max_age_s']:
        args.append(DeclareLaunchArgument(key, default_value=cfg[key.upper()]))
    return LaunchDescription(args + [OpaqueFunction(function=prepare)])
