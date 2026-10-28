#!/usr/bin/env python3
"""Managed private backend or explicit offline bridge session; never starts devices."""
import argparse
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
from session_manager import Component, run_guarded, read_session, session_live, stop_session
sys.path.insert(0, str(ROOT/'ws_nav/src/ranger_localization_bridge'))
from ranger_localization_bridge.runtime_config import resolve


def specs(args, metadata):
    backend = ROOT/'ws_fast_livo/backend_install/ranger_fast_livo_backend/lib/ranger_fast_livo_backend/backend_node'
    command = [str(backend), '--ros-args', '--params-file', str(ROOT/'config/fast_livo2/backend.yaml'),
               '-p', 'use_sim_time:='+str(args.sim_time).lower(), '-p', 'metadata_path:='+str(metadata),
               '-p', 'data_root:='+str(ROOT/'.runtime/fast_livo_backend'), '-p', 'registration:='+args.registration]
    components = [Component('monitor', [sys.executable, str(ROOT/'scripts/fast_livo2/backend/monitor.py')], ready_key='probe_live'),
                  Component('backend', command, depends=('monitor',), ready_key='backend_healthy',
                            requires=('probe_live',), resource='fast_livo_backend')]
    if args.offline_bridge:
        cfg = resolve(ROOT)
        adapter_parameters = ['-p', 't_base_imu_xyz:='+cfg['T_BASE_IMU_XYZ'],
                              '-p', 't_base_imu_rpy:='+cfg['T_BASE_IMU_RPY'],
                              '-p', 'sample_max_age:='+cfg['TF_MAX_AGE_S'],
                              '-p', 'heartbeat_max_age:='+cfg['HEARTBEAT_MAX_AGE_S']]
        bridge_parameters = []
        for key, parameter in [('TF_MAX_AGE_S', 'tf_max_age_s'), ('ODOM_MAX_AGE_S', 'odom_max_age_s'),
                               ('ODOM_HISTORY_S', 'odom_history_s'), ('ODOM_MAX_GAP_S', 'odom_max_gap_s'),
                               ('HEARTBEAT_MAX_AGE_S', 'heartbeat_max_age_s')]:
            bridge_parameters += ['-p', parameter+':='+cfg[key]]
        components += [
            Component('adapter', [sys.executable, '-m', 'ranger_localization_bridge.fast_livo_adapter',
                                 '--ros-args', '-p', 'use_sim_time:=true', *adapter_parameters], depends=('backend',), resource='localization_backend'),
            Component('bridge', [sys.executable, '-m', 'ranger_localization_bridge.bridge_node', '--ros-args',
                                '-p', 'adapter_backend:=external', '-p', 'use_sim_time:=true',
                                '-p', 'latch_on_innovation_failure:=true', *bridge_parameters], depends=('adapter',), resource='localization_bridge')]
    return components


def main():
    p = argparse.ArgumentParser(description=__doc__)
    actions = p.add_mutually_exclusive_group()
    actions.add_argument('--execute', action='store_true')
    actions.add_argument('--status', action='store_true')
    actions.add_argument('--stop', action='store_true')
    p.add_argument('--offline-bridge', action='store_true')
    p.add_argument('--sim-time', action=argparse.BooleanOptionalAction, default=True)
    p.add_argument('--domain', type=int, default=187)
    p.add_argument('--registration', choices=['pcl_icp', 'small_gicp', 'small_vgicp'], default='pcl_icp')
    p.add_argument('--source-id', default='unspecified DDS input; archived scan hashes identify the selected data')
    args = p.parse_args()
    if not 0 <= args.domain <= 232:
        p.error('domain must be 0..232')
    if args.offline_bridge and not args.sim_time:
        p.error('public bridge entry is offline only and requires simulated time')
    directory = ROOT/'.runtime/sessions'/('fast_livo_backend_bridge' if args.offline_bridge else 'fast_livo_backend')
    if args.status:
        _, state = read_session(directory)
        print(json.dumps({'live': bool(session_live(state)), 'details': state}, indent=2)); return 0
    if args.stop:
        return 0 if stop_session(directory) else 2
    metadata = directory/'provenance-NEW.json'
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1', PYTHONDONTWRITEBYTECODE='1')
    os.environ['PYTHONPATH'] = str(ROOT/'ws_nav/src/ranger_localization_bridge')+':'+os.environ.get('PYTHONPATH', '')
    os.environ['LD_LIBRARY_PATH'] = str(ROOT/'ws_fast_livo/backend_deps/lib')+':'+os.environ.get('LD_LIBRARY_PATH', '')
    if not args.execute:
        print(json.dumps({'action': 'DRY-RUN', 'domain': args.domain, 'localhost_only': True,
                          'public_tf_enabled': args.offline_bridge, 'components': [asdict(c) for c in specs(args, metadata)]}, indent=2)); return 0
    if session_live(read_session(directory)[1]):
        raise RuntimeError('managed session is already live; inspect status')
    # Read-only DDS occupancy check before any algorithm or public TF component starts.
    import rclpy
    rclpy.init(); probe = rclpy.create_node('ranger_backend_occupancy_probe')
    try:
        deadline = time.monotonic()+2
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=.05)
        topics = ['/fast_livo/backend/estimate']
        if args.offline_bridge:
            topics += ['/localization/estimate', '/navigation_ready', '/tf', '/tf_static', '/cmd_vel', '/wheel/odom']
        occupied = [t for t in topics if probe.get_publishers_info_by_topic(t)]
        if occupied:
            raise RuntimeError('session requires unclaimed outputs (start synthetic wheel after this check): '+', '.join(occupied))
    finally:
        probe.destroy_node(); rclpy.shutdown()
    directory.mkdir(parents=True, exist_ok=True)
    metadata = directory/('provenance-'+str(time.time_ns())+'.json')
    files = ['config/fast_livo2/sources.lock.json', 'config/fast_livo2/ranger_mid360s_hik_trial.yaml',
             'config/runtime.yaml', 'patches/fast_livo2_stage_a.patch', 'config/fast_livo2/backend.yaml']
    backend_source = ROOT/'ws_fast_livo/src/ranger_fast_livo_backend'
    files += [str(f.relative_to(ROOT)) for f in backend_source.rglob('*') if f.suffix in ['.cpp', '.hpp', '.msg', '.srv', '.xml']]
    cfg = resolve(ROOT)
    frontend = Path(cfg['FAST_LIVO_CONFIG'])
    declaration = yaml.safe_load(frontend.read_text())['/**']['ros__parameters']
    metadata.write_text(json.dumps({'git_head': subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip(),
                                   'files_sha256': {f: hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in files},
                                   'frontend_gravity_aligned': False, 'physical_sync': 'PARTIAL',
                                   'T_base_imu_calibration': 'approximate; existing runtime values',
                                   'T_base_imu_xyz': json.loads(cfg['T_BASE_IMU_XYZ']),
                                   'T_base_imu_rpy': json.loads(cfg['T_BASE_IMU_RPY']),
                                   'T_imu_lidar': {'source_config': str(frontend),
                                                   'already_applied_by_frontend': True,
                                                   'declared_extrinsics': declaration['extrin_calib']},
                                   'declared_frontend_config': str(frontend),
                                   'declared_frontend_config_sha256': hashlib.sha256(frontend.read_bytes()).hexdigest(),
                                   'active_frontend_configuration_attested': False,
                                   'source_data_id': args.source_id,
                                   'hardware_started': False}, indent=2)+'\n')
    return run_guarded(directory, specs(args, metadata), startup_timeout=15,
                       fail_on_health_loss=True, health_loss_grace=1.)


if __name__ == '__main__':
    raise SystemExit(main())
