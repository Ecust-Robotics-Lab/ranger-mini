"""Shared, ROS-independent runtime/map resolver used by compatibility entrypoints."""
import argparse
import json
import math
import os
import re
from pathlib import Path
import shlex

import yaml


def project_root():
    if os.environ.get('RANGER_PROJECT_ROOT'):
        return Path(os.environ['RANGER_PROJECT_ROOT']).expanduser().resolve()
    for parent in Path(__file__).resolve().parents:
        if (parent / 'config/runtime.yaml').is_file():
            return parent
    raise ValueError('set RANGER_PROJECT_ROOT to the project directory')


def path_at(root, value):
    p = Path(value).expanduser()
    return (root / p).resolve() if not p.is_absolute() else p.resolve()


def resolve(root=None, environ=None):
    env = dict(os.environ if environ is None else environ)
    # Convert the old raw launch-argument string to validated typed overrides.
    # Unknown arguments previously disappeared silently inside roslaunch.
    legacy_keys = {'t_base_imu_xyz', 't_base_imu_rpy', 'min_confidence',
                   'tf_max_age_s', 'odom_max_age_s', 'odom_history_s', 'odom_max_gap_s', 'heartbeat_max_age_s',
                   'status_max_age_s', 'confidence_max_age_s', 'loc_child_frame'}
    for argument in shlex.split(env.get('BRIDGE_EXTRA_ARGS', '')):
        key, sep, value = argument.partition(':=')
        if not sep or key not in legacy_keys:
            raise ValueError(f'unsupported BRIDGE_EXTRA_ARGS entry: {argument}')
        env[key.upper()] = value
    root = Path(root) if root is not None else project_root()
    runtime_path = path_at(root, env.get('RANGER_RUNTIME_CONFIG', 'config/runtime.yaml'))
    cfg = yaml.safe_load(runtime_path.read_text())
    if cfg['schema_version'] != 1:
        raise ValueError('unsupported runtime schema')
    supported = {'robot': ['ranger_mini_v2'], 'odometry_backend': ['wheel'],
                 'localization_backend': ['lightning', 'standard'],
                 'mapping_backend': ['lightning'], 'terrain_backend': ['disabled']}
    for key, choices in supported.items():
        if cfg[key] not in choices:
            raise ValueError(f'unsupported {key}: {cfg[key]}')
    bundle_path = path_at(root, env.get('MAP_BUNDLE', cfg['map_bundle']))
    bundle = yaml.safe_load(bundle_path.read_text())
    if bundle['schema_version'] != 1 or bundle['frame'] != 'map':
        raise ValueError('unsupported map bundle schema/frame')
    if bundle['localization_backend'] != cfg['localization_backend']:
        raise ValueError('map bundle/backend mismatch')
    paths = {k: path_at(root, bundle[k]) for k in [
        'localization_config', 'localization_map', 'navigation_map', 'calibration', 'source_bag']}
    for name, path in paths.items():
        if not path.exists():
            raise ValueError(f'missing {name}: {path}')
    if not paths['navigation_map'].is_file() or not paths['localization_config'].is_file():
        raise ValueError('navigation_map and localization_config must be files')
    loc_cfg = yaml.safe_load(paths['localization_config'].read_text())
    if cfg['localization_backend'] == 'lightning':
        if path_at(root, loc_cfg['system']['map_path']) != paths['localization_map']:
            raise ValueError('Lightning config references a different native map')
        a = json.loads((paths['localization_map'].parents[1] / 'manifest.json').read_text())
        b = json.loads((paths['navigation_map'].parents[2] / 'manifest.json').read_text())
        if (path_at(root, a['bag']) != paths['source_bag'] or
                path_at(root, b['bag']) != paths['source_bag'] or
                a['bag_inventory']['files'] != b['bag_inventory']['files'] or
                a.get('current_source_commit') != b.get('current_source_commit')):
            raise ValueError('map bundle has incompatible source provenance')
    grid = yaml.safe_load(paths['navigation_map'].read_text())
    if not (paths['navigation_map'].parent / grid['image']).is_file():
        raise ValueError('navigation image missing')
    # Preserve legacy names, but require overrides to select the same bundle.
    for key, expected in [('LOC_CONFIG', paths['localization_config']),
                          ('MAP_YAML', paths['navigation_map']),
                          ('MAP_DIR', paths['navigation_map'].parent)]:
        if key in env and path_at(root, env[key]) != expected:
            raise ValueError(f'{key} conflicts with map bundle; select MAP_BUNDLE instead')
    loc = dict(cfg['localization'])
    loc.setdefault('odom_history_s', 2.0)
    loc.setdefault('odom_max_gap_s', 0.1)
    for key in ['tf_max_age_s', 'odom_max_age_s', 'odom_history_s', 'odom_max_gap_s', 'heartbeat_max_age_s',
                'status_max_age_s', 'confidence_max_age_s', 'min_confidence']:
        if not math.isfinite(float(loc[key])) or float(loc[key]) <= 0:
            raise ValueError(f'{key} must be finite and positive')
    values = {
        'RANGER_RUNTIME_CONFIG': str(runtime_path), 'MAP_BUNDLE': str(bundle_path),
        'LOC_CONFIG': str(paths['localization_config']),
        'MAP_YAML': str(paths['navigation_map']), 'MAP_DIR': str(paths['navigation_map'].parent),
        'LOCALIZATION_BACKEND': cfg['localization_backend'],
        'TF_MAX_AGE_S': str(loc['tf_max_age_s']), 'ODOM_MAX_AGE_S': str(loc['odom_max_age_s']),
        'ODOM_HISTORY_S': str(loc['odom_history_s']), 'ODOM_MAX_GAP_S': str(loc['odom_max_gap_s']),
        'HEARTBEAT_MAX_AGE_S': str(loc['heartbeat_max_age_s']),
        'STATUS_MAX_AGE_S': str(loc['status_max_age_s']),
        'CONFIDENCE_MAX_AGE_S': str(loc['confidence_max_age_s']),
        'MIN_CONFIDENCE': str(loc['min_confidence']),
        'T_BASE_IMU_XYZ': json.dumps(loc['t_base_imu_xyz']),
        'T_BASE_IMU_RPY': json.dumps(loc['t_base_imu_rpy']),
        'LIDAR_SECONDS': str(cfg['session']['lidar_seconds']),
        'LOC_CHILD_FRAME': env.get('LOC_CHILD_FRAME', env.get('TF_CHILD', loc.get('native_pose_frame', 'lightning_imu_pose'))),
        'BRIDGE_EXTRA_ARGS': '',  # overrides above have already been consumed
    }
    pose = cfg['session']['initial_pose']
    if len(pose) != 7 or not all(math.isfinite(v) for v in pose) or sum(v*v for v in pose[3:]) < 1e-12:
        raise ValueError('initial_pose must contain finite xyz and nonzero quaternion')
    for key, value in zip(['INIT_X', 'INIT_Y', 'INIT_Z', 'INIT_QX', 'INIT_QY', 'INIT_QZ', 'INIT_QW'], pose):
        values[key] = str(value)
    # Only operational seed/extrinsic/threshold variables can override defaults.
    for key in list(values):
        if key not in ['RANGER_RUNTIME_CONFIG', 'MAP_BUNDLE', 'LOC_CONFIG', 'MAP_YAML', 'MAP_DIR', 'LOCALIZATION_BACKEND', 'BRIDGE_EXTRA_ARGS'] and key in env:
            values[key] = env[key]
    for key in ['TF_MAX_AGE_S', 'ODOM_MAX_AGE_S', 'ODOM_HISTORY_S', 'ODOM_MAX_GAP_S', 'HEARTBEAT_MAX_AGE_S', 'STATUS_MAX_AGE_S', 'CONFIDENCE_MAX_AGE_S', 'MIN_CONFIDENCE']:
        if not math.isfinite(float(values[key])) or float(values[key]) <= 0:
            raise ValueError(f'invalid {key}')
        values[key] = str(float(values[key]))
    if not 0 < float(values['LIDAR_SECONDS']) <= 3600:
        raise ValueError('LIDAR_SECONDS must be in (0, 3600]')
    values['LIDAR_SECONDS'] = str(float(values['LIDAR_SECONDS']))
    for key in ['T_BASE_IMU_XYZ', 'T_BASE_IMU_RPY']:
        v = yaml.safe_load(values[key])
        if not isinstance(v, list) or len(v) != 3 or not all(type(x) in (int, float) and math.isfinite(x) for x in v):
            raise ValueError(f'invalid {key}')
        values[key] = json.dumps([float(x) for x in v])
    for key in ['INIT_X', 'INIT_Y', 'INIT_Z', 'INIT_QX', 'INIT_QY', 'INIT_QZ', 'INIT_QW']:
        if not math.isfinite(float(values[key])):
            raise ValueError(f'invalid {key}')
        values[key] = str(float(values[key]))
    if sum(float(values[key])**2 for key in ['INIT_QX', 'INIT_QY', 'INIT_QZ', 'INIT_QW']) < 1e-12:
        raise ValueError('initial quaternion is zero')
    native_frame = values['LOC_CHILD_FRAME']
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_/]*', native_frame) or native_frame in ['map', 'odom', 'base_link', 'imu_link', 'livox_frame']:
        raise ValueError('native pose frame must be private, not a physical/public TF frame')
    if float(values['ODOM_HISTORY_S']) < max(float(values['ODOM_MAX_GAP_S']), float(values['TF_MAX_AGE_S'])):
        raise ValueError('odometry history must cover pose age and maximum interpolation gap')
    return values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--shell', action='store_true')
    args = parser.parse_args()
    try:
        values = resolve()
    except (ValueError, KeyError, TypeError, OSError, yaml.YAMLError) as exc:
        parser.exit(2, f'configuration error: {exc}\n')
    if args.shell:
        for key, value in values.items():
            print(f'export {key}={shlex.quote(value)}')
    else:
        print(json.dumps(values, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
