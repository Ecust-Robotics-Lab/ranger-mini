import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_csv = os.path.join(
        os.environ.get("ROS_LOG_DIR", os.path.expanduser("~/.ros/log")),
        "timeshare_writer_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv")

    return LaunchDescription([
        DeclareLaunchArgument(
            "lidar_topic", default_value="/livox/lidar",
            description="PointCloud2 topic whose header.stamp carries the scan start"),
        DeclareLaunchArgument(
            "csv", default_value=default_csv,
            description="Per-scan writer log CSV path"),
        DeclareLaunchArgument("timeshare_path", default_value="",
                              description="Per-session shared timestamp file"),
        DeclareLaunchArgument("lidar_reliability", default_value="reliable",
                              description="reliable or best_effort; no automatic fallback"),
        Node(
            package="timeshare_writer",
            executable="timeshare_writer_node",
            name="timeshare_writer",
            output="screen",
            parameters=[
                {"lidar_topic": LaunchConfiguration("lidar_topic")},
                {"csv_path": LaunchConfiguration("csv")},
                {"timeshare_path": LaunchConfiguration("timeshare_path")},
                {"expected_period_ms": 100},
                {"lidar_reliability": LaunchConfiguration("lidar_reliability")},
            ],
        ),
    ])
