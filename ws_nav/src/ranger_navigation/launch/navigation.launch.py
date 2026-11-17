"""EXACT-MPPI single-point navigation. Dry-run; requires existing FAST/TF/wheel.

No chassis, CAN or sensors are started here. The final safety process owns
/base/safe_cmd_vel and starts unarmed. JAX/planning runs in a separate process.
"""
from pathlib import Path
import importlib.util
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,LogInfo,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def project_root():
    for start in (Path.cwd(),Path(__file__).resolve()):
        for p in (start,*start.parents):
            if (p/'config/fast_livo2/navigation_prep.yaml').is_file():
                return p
    raise RuntimeError('set project_root to the Ranger project checkout')


def prepare(context):
    if LaunchConfiguration('execute').perform(context)!='true':
        return [LogInfo(msg='DRY-RUN: EXACT-MPPI coordinator + independent arbitration/gate/collision monitor. '
                        'Reuses FAST localization, map, wheel and TF; no devices started; motion initially disabled.')]
    if LaunchConfiguration('control_authorized').perform(context)!='true':
        raise RuntimeError('This navigation start requires fresh explicit control authorization.')
    root=Path(LaunchConfiguration('project_root').perform(context)).resolve()
    map_yaml=Path(LaunchConfiguration('map_yaml').perform(context)).resolve()
    # Reuse the existing map/source integrity verifier, without modifying assets.
    spec=importlib.util.spec_from_file_location('fast_navigation_asset',root/'scripts/fast_livo2/navigation/asset.py')
    asset=importlib.util.module_from_spec(spec);spec.loader.exec_module(asset)
    manifest=asset.load_map(map_yaml.parent)
    from ament_index_python.packages import get_package_share_directory
    share=Path(get_package_share_directory('ranger_navigation'))
    cfg=yaml.safe_load((share/'config/exact_navigation.yaml').read_text())
    profile=cfg['clearance_profiles'][LaunchConfiguration('clearance_profile').perform(context)]
    limits=cfg['real_vehicle_first_test'] if LaunchConfiguration('low_speed').perform(context)=='true' else cfg['motion']
    requested_speed=float(LaunchConfiguration('max_speed',default='0').perform(context))
    if not 0 <= requested_speed <= float(cfg['motion']['max_speed']):
        raise RuntimeError('max_speed must be between 0 (profile default) and configured motion limit')
    common={'use_sim_time':LaunchConfiguration('use_sim_time').perform(context)=='true',
            'geometry_config':str(root/'config/fast_livo2/navigation_prep.yaml'),
            'wheel_topic':LaunchConfiguration('wheel_topic').perform(context),
            'clearance':float(profile['clearance']),
            'max_speed':requested_speed or float(limits['max_speed']),'max_yaw_rate':float(limits['max_yaw_rate'])}
    common.update({k:float(cfg['motion'][k]) for k in ('acceleration','yaw_acceleration','min_radius')})
    optimizer=Path(LaunchConfiguration('optimizer_python').perform(context))
    if not optimizer.is_file():
        raise RuntimeError('missing project-local JAX environment: '+str(optimizer))
    nodes=[Node(package='ranger_navigation',executable='exact_navigation',name='ranger_exact_navigation',
                parameters=[common,{'optimizer_python':str(optimizer),'compute_backend':cfg['compute']['backend'],
                    'static_map_reserve':float(profile['static_map_reserve']),
                    'visibility_reserve':float(profile['visibility_reserve']),
                    'segment_yaw_tolerance':float(profile['segment_yaw_tolerance']),
                    'goal_position_tolerance':float(LaunchConfiguration('goal_position_tolerance',default='.15').perform(context)),
                    'goal_yaw_tolerance':float(LaunchConfiguration('goal_yaw_tolerance',default='.15').perform(context)),
                    'prefer_parallel_in_narrow':bool(profile['prefer_parallel_in_narrow']),
                    'samples':cfg['compute']['samples'],'expected_map_uuid':manifest['map_uuid'],
                    **cfg['observation'],
                    'expected_manifest_sha256':manifest['source_manifest_sha256'],
                    'session_root':str(root/'.runtime/exact_navigation'),'map_yaml':str(map_yaml)}],output='screen'),
           Node(package='ranger_navigation',executable='motion_gate',name='ranger_motion_gate',
                parameters=[common],output='screen')]
    if LaunchConfiguration('start_map_server').perform(context)=='true':
        nodes += [Node(package='nav2_map_server',executable='map_server',name='exact_navigation_map',
                       parameters=[{'use_sim_time':common['use_sim_time'],'yaml_filename':str(map_yaml)}],output='screen'),
                  Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='exact_map_lifecycle',
                       parameters=[{'use_sim_time':common['use_sim_time'],'autostart':True,'node_names':['exact_navigation_map']}],output='screen')]
    return nodes


def generate_launch_description():
    root=project_root()
    return LaunchDescription([
        DeclareLaunchArgument('execute',default_value='false',choices=['false','true']),
        DeclareLaunchArgument('control_authorized',default_value='false',choices=['false','true']),
        DeclareLaunchArgument('project_root',default_value=str(root)),
        DeclareLaunchArgument('map_yaml',default_value=str(root/'outputs/fast-livo-demo-navigation-prep-20261106/navigation_map_final/map.yaml')),
        DeclareLaunchArgument('optimizer_python',default_value=str(root/'.runtime/exact_mppi_venv/bin/python3')),
        DeclareLaunchArgument('wheel_topic',default_value='/fast_livo/navigation/wheel/odom'),
        DeclareLaunchArgument('use_sim_time',default_value='true',choices=['false','true']),
        DeclareLaunchArgument('start_map_server',default_value='false',choices=['false','true']),
        DeclareLaunchArgument('low_speed',default_value='true',choices=['false','true']),
        DeclareLaunchArgument('max_speed',default_value='0'),
        DeclareLaunchArgument('goal_position_tolerance',default_value='.15'),
        DeclareLaunchArgument('goal_yaw_tolerance',default_value='.15'),
        DeclareLaunchArgument('clearance_profile',default_value='standard',choices=['standard','narrow']),
        OpaqueFunction(function=prepare)])
