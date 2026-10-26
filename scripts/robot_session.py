#!/usr/bin/env python3
"""Dry-run by default; foreground supervised field/navigation/aux sessions."""
import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import shlex
import sys

from session_manager import Component, read_session, run_guarded, session_live, stop_session

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'ws_nav/src/ranger_localization_bridge'))
from ranger_localization_bridge.runtime_config import resolve


def ros_command(argv, sources=None):
    sources = sources or ['/opt/ros/humble/setup.bash',
                          str(ROOT/'ws_livox/install/setup.bash'), str(ROOT/'ws_nav/install/setup.bash')]
    body = ' && '.join('source '+shlex.quote(p) for p in sources)
    return ['bash', '-c', body+' && exec '+shlex.join([str(a) for a in argv])]


def enabled(env, name, default='1'):
    value = env.get(name, default)
    if value not in ['0', '1']:
        raise ValueError(name+' must be 0 or 1')
    return value == '1'


def build_components(mode, cfg, env):
    specs = [Component('probe', ros_command(['python3', ROOT/'scripts/session_probe.py']), ready_key='probe_live')]
    if mode in ['field', 'lidar'] and (mode == 'lidar' or enabled(env, 'ENABLE_LIDAR')):
        specs.append(Component('lidar', ros_command(['python3', ROOT/'scripts/start_mid360.py',
            '--execute', '--seconds', cfg['LIDAR_SECONDS']], [str(ROOT/'scripts/slam_env.sh')]),
            depends=('probe',), ready_key='lidar', start_requires=('lidar_unclaimed',), resource='livox_lidar'))
    if mode in ['field', 'base'] and (mode == 'base' or enabled(env, 'ENABLE_BASE')):
        specs.append(Component('base', ros_command(['ros2', 'launch', ROOT/'config/base_mini_v2.launch.py',
            'execute:=true', 'confirmed_model:=ranger_mini_v2', 'control_authorized:=true'],
            ['/opt/ros/humble/setup.bash', str(ROOT/'ws_base/install/local_setup.bash')]),
            depends=('probe',), ready_key='odom', start_requires=('odom_unclaimed',), resource='base_control'))
    auxiliary = mode == 'aux' or (mode == 'field' and enabled(env, 'ENABLE_AUX'))
    if auxiliary:
        xyz = json.loads(cfg['T_BASE_IMU_XYZ']); rpy = json.loads(cfg['T_BASE_IMU_RPY'])
        mount_args = ['/opt/ros/humble/lib/tf2_ros/static_transform_publisher']
        for key, value in zip(['x', 'y', 'z', 'roll', 'pitch', 'yaw'], [*xyz, *rpy]):
            mount_args += ['--'+key, str(value)]
        mount_args += ['--frame-id', 'base_link', '--child-frame-id', 'imu_link']
        specs.append(Component('sensor_mount', ros_command(mount_args), depends=('probe',),
                               resource='sensor_mount'))
        specs.append(Component('sensor_tf', ros_command(['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
            '--x', '-0.011', '--y', '-0.02329', '--z', '0.04412',
            '--frame-id', 'imu_link', '--child-frame-id', 'livox_frame']),
            depends=('sensor_mount',), ready_key='sensor_tf', resource='sensor_tf'))
        if enabled(env, 'ENABLE_MAP_PUB', '0'):
            specs.append(Component('visual_map', ros_command(['python3', ROOT/'scripts/publish_loc_map.py',
                '--map-dir', cfg['MAP_DIR']]), depends=('probe',), critical=False,
                start_requires=('map_unclaimed',), resource='map_publisher'))
        if enabled(env, 'ENABLE_DERIVED', '0'):
            specs.append(Component('derived', ros_command(['python3', ROOT/'scripts/mid360_derived.py', '--execute']),
                depends=('probe',), critical=False, resource='derived_cloud'))
        if enabled(env, 'ENABLE_FOXGLOVE'):
            specs.append(Component('foxglove', ros_command(['ros2', 'launch', 'foxglove_bridge',
                'foxglove_bridge_launch.xml']), depends=('probe',), critical=False, resource='foxglove'))
    names = {c.name for c in specs}
    if mode == 'field' and enabled(env, 'ENABLE_LOC') and cfg['LOCALIZATION_BACKEND'] == 'lightning':
        command = [ROOT/'ws_slam/build/lightning/bin/run_loc_online', '--config='+cfg['LOC_CONFIG'],
                   '--tf_child='+cfg['LOC_CHILD_FRAME']]
        command += ['--'+name.lower()+'='+cfg[name] for name in
                    ['INIT_X', 'INIT_Y', 'INIT_Z', 'INIT_QX', 'INIT_QY', 'INIT_QZ', 'INIT_QW']]
        specs.append(Component('localization', ros_command(command, [str(ROOT/'scripts/slam_env.sh')]),
            depends=('probe',)+(('lidar',) if 'lidar' in names else ()), requires=('lidar',),
            ready_key='native_localization', start_requires=('native_unclaimed',), resource='native_localization'))
    names = {c.name for c in specs}
    if mode == 'field' and enabled(env, 'ENABLE_BRIDGE'):
        command = ['ros2', 'launch', 'ranger_localization_bridge', 'loc_bridge.launch.py',
                   'localization_backend:='+cfg['LOCALIZATION_BACKEND']]
        command += [key.lower()+':='+cfg[key] for key in ['T_BASE_IMU_XYZ', 'T_BASE_IMU_RPY',
            'LOC_CHILD_FRAME', 'MIN_CONFIDENCE', 'TF_MAX_AGE_S', 'ODOM_MAX_AGE_S', 'ODOM_HISTORY_S', 'ODOM_MAX_GAP_S',
            'HEARTBEAT_MAX_AGE_S', 'STATUS_MAX_AGE_S', 'CONFIDENCE_MAX_AGE_S']]
        specs.append(Component('bridge', ros_command(command),
            depends=('probe',)+tuple(n for n in ['base','localization'] if n in names),
            requires=('odom',), ready_key='localization', start_requires=('bridge_unclaimed',),
            resource='localization_bridge'))
    if mode == 'navigation':
        specs.append(Component('navigation', ros_command(['ros2', 'launch', 'ranger_navigation',
            'navigation.launch.py', 'execute:=true', 'control_authorized:=true', 'map_yaml:='+cfg['MAP_YAML']]),
            depends=('probe',), requires=('localization', 'lidar', 'sensor_tf'), ready_key='navigation',
            start_requires=('nav_unclaimed',), resource='navigation'))
    return specs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['field', 'navigation', 'aux', 'base', 'lidar'])
    action = parser.add_mutually_exclusive_group()
    action.add_argument('--execute', action='store_true')
    action.add_argument('--plan', action='store_true')
    action.add_argument('--status', action='store_true')
    action.add_argument('--stop', action='store_true')
    parser.add_argument('--control-authorized', action='store_true',
                        help='explicit authorization for this start; never saved or replayed')
    parser.add_argument('--startup-timeout', type=float, default=60.)
    argv = sys.argv[1:]
    if len(argv) > 1 and argv[1] == 'plan':
        argv[1] = '--plan'
    args = parser.parse_args(argv)
    directory = Path(os.environ.get('RANGER_SESSION_ROOT', str(ROOT/'.runtime/sessions')))/args.mode
    try:
        if args.status:
            _, state = read_session(directory)
            print(json.dumps({'session': args.mode, 'live': bool(session_live(state)), 'details': state},
                             ensure_ascii=False, indent=2))
            return 0
        if args.stop:
            ok = stop_session(directory)
            print('session stopped' if ok else 'session stop timed out; inspect state.json')
            return 0 if ok else 2
        if not 0 < args.startup_timeout <= 600:
            raise ValueError('startup timeout must be in (0, 600]')
        cfg = resolve(ROOT)
        specs = build_components(args.mode, cfg, os.environ)
        controls_base = any(c.name == 'base' for c in specs)
        if not args.execute:
            print(json.dumps({'action': 'DRY-RUN', 'session': args.mode,
                'requires_fresh_control_authorization': controls_base or args.mode == 'navigation',
                'map_bundle': cfg['MAP_BUNDLE'], 'components': [asdict(c) for c in specs]},
                ensure_ascii=False, indent=2))
            return 0
        if (controls_base or args.mode == 'navigation') and not args.control_authorized:
            parser.error('this start requires --control-authorized after explicit test authorization (AGENTS.md)')
        os.environ.update(cfg)
        return run_guarded(directory, specs, startup_timeout=args.startup_timeout,
                           fail_on_health_loss=args.mode == 'navigation')
    except (ValueError, RuntimeError, OSError, KeyError, TypeError) as exc:
        parser.exit(2, f'session error: {exc}\n')


if __name__ == '__main__':
    raise SystemExit(main())
