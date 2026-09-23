"""Launch the ranger localization bridge (stage 5).

Run AFTER (or alongside) Lightning localization (run_loc_online) and the Ranger
base driver publishing /wheel/odom. This launch file only starts the bridge.

  source scripts/slam_env.sh   # provides ROS 2 + workspaces
  ros2 launch ranger_localization_bridge loc_bridge.launch.py

Extrinsic defaults are the user's 2026-09-23 APPROXIMATE measurements; override
once the base_link-relative mounting is properly measured.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('t_base_imu_xyz', default_value='[0.0, 0.0, 0.556]',
                              description='base_link->imu_link translation (APPROXIMATE, pending measurement)'),
        DeclareLaunchArgument('t_base_imu_rpy', default_value='[0.0, 0.0, 0.0]',
                              description='base_link->imu_link rotation (assumed aligned, pending measurement)'),
        DeclareLaunchArgument('min_confidence', default_value='1.0'),
        Node(
            package='ranger_localization_bridge',
            executable='ranger_localization_bridge',
            name='ranger_localization_bridge',
            output='screen',
            parameters=[{
                't_base_imu_xyz': LaunchConfiguration('t_base_imu_xyz'),
                't_base_imu_rpy': LaunchConfiguration('t_base_imu_rpy'),
                'min_confidence': LaunchConfiguration('min_confidence'),
            }],
        ),
    ])
