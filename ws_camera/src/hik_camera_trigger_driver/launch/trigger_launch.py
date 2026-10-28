import os
from datetime import datetime

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("hik_camera_trigger_driver")
    package_prefix = get_package_prefix("hik_camera_trigger_driver")

    gentl_path = os.path.join(package_prefix, "lib")
    third_party_path = os.path.join(gentl_path, "ThirdParty")

    current_ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    library_paths = [gentl_path, third_party_path]
    if current_ld_library_path:
        library_paths.append(current_ld_library_path)

    default_params = os.path.join(
        package_share, "config", "camera_params_trigger.yaml")
    default_csv = os.path.join(
        os.environ.get("ROS_LOG_DIR", os.path.expanduser("~/.ros/log")),
        "camera_sync_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv")

    return LaunchDescription([
        SetEnvironmentVariable("GENICAM_GENTL64_PATH", gentl_path),
        SetEnvironmentVariable("LD_LIBRARY_PATH", ":".join(library_paths)),
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        DeclareLaunchArgument(
            "params_file", default_value=default_params,
            description="Camera parameter YAML path"),
        DeclareLaunchArgument(
            "exposure_us", default_value="20000.0",
            description="Fixed exposure in us (20000 default, 40000 for static calib)"),
        DeclareLaunchArgument(
            "sync_csv", default_value=default_csv,
            description="Per-frame sync diagnostics CSV path"),
        DeclareLaunchArgument("timeshare_path", default_value="",
                              description="Per-session shared timestamp file"),
        Node(
            name="hik_camera_trigger",
            package="hik_camera_trigger_driver",
            executable="hik_camera_trigger_driver_node",
            parameters=[
                LaunchConfiguration("params_file"),
                {"exposure_time": LaunchConfiguration("exposure_us")},
                {"sync_csv_path": LaunchConfiguration("sync_csv")},
                {"timeshare_path": LaunchConfiguration("timeshare_path")},
                {"image.compressedDepth.disable_pub": True},
                {"front/image.compressedDepth.disable_pub": True},
            ],
            output="screen",
        ),
    ])
