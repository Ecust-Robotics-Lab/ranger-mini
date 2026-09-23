"""Project launch; default is dry-run. Each actual start needs fresh permission."""
import json
from pathlib import Path
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def prepare(context):
    execute = LaunchConfiguration('execute').perform(context)
    if execute != 'true':
        return [LogInfo(msg='DRY-RUN: Mini V2; /wheel/odom; TF disabled; '
                       '/cmd_vel -> /base/safe_cmd_vel. No driver started.')]
    if LaunchConfiguration('confirmed_model').perform(context) != 'ranger_mini_v2':
        raise RuntimeError('Confirm the physical Mini V2 model for this start.')
    if LaunchConfiguration('control_authorized').perform(context) != 'true':
        raise RuntimeError('This start requires fresh explicit control authorization.')
    # Read only: never configure or bring up CAN here.
    result = subprocess.run(['ip', '-json', '-details', 'link', 'show', 'can0'],
                            check=True, capture_output=True, text=True, timeout=5)
    state = json.loads(result.stdout)[0]
    can = state.get('linkinfo', {}).get('info_data', {})
    if ('UP' not in state.get('flags', []) or
            can.get('state') != 'ERROR-ACTIVE' or
            not can.get('bittiming', {}).get('bitrate')):
        raise RuntimeError('can0 must already be UP/ERROR-ACTIVE with known bitrate.')
    return [Node(
        package='ranger_base', executable='ranger_base_node',
        name='ranger_base_node', output='screen', respawn=False,
        parameters=[str(Path(__file__).with_name('base_mini_v2.yaml'))],
        remappings=[('/cmd_vel', '/base/safe_cmd_vel')],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('execute', default_value='false', choices=['false', 'true']),
        DeclareLaunchArgument('confirmed_model', default_value='unconfirmed'),
        DeclareLaunchArgument('control_authorized', default_value='false', choices=['false', 'true']),
        OpaqueFunction(function=prepare),
    ])
