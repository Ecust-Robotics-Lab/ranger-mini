#!/usr/bin/env python3
"""Managed offline C1 entry. Dry-run by default; never starts sensors or a base."""
import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
from session_manager import Component, run_guarded, read_session, session_live, stop_session
sys.path.insert(0, str(ROOT/'ws_nav/src/ranger_localization_bridge'))
from ranger_localization_bridge.runtime_config import resolve


def specs(args):
    cfg = resolve(ROOT)
    extrinsics = ['-p', 't_base_imu_xyz:='+cfg['T_BASE_IMU_XYZ'], '-p', 't_base_imu_rpy:='+cfg['T_BASE_IMU_RPY']]
    command = [str(ROOT/'ws_fast_livo/backend_install/ranger_fast_livo_localizer/lib/ranger_fast_livo_localizer/localizer_node'),
               '--ros-args', '--params-file', str(ROOT/'config/fast_livo2/localizer.yaml'),
               '-p', 'use_sim_time:=true', '-p', 'registration:='+args.registration, *extrinsics]
    if args.archive:
        command += ['-p', 'archive_path:='+str(args.archive.resolve())]
    if args.compatibility:
        command += ['-p', 'compatibility_path:='+str(args.compatibility.resolve())]
    if getattr(args, 'identity_file', None):
        if args.compatibility:
            raise ValueError('choose expected source identities or an offline compatibility declaration')
        identity = json.loads(args.identity_file.read_text())
        if identity.get('schema_version') != 1:
            raise ValueError('unsupported source identity schema')
        for name in ('sensor_configuration_sha256', 'time_contract_sha256', 'frontend_sources_sha256'):
            value = identity[name]
            if len(value) != 64 or any(c not in '0123456789abcdef' for c in value):
                raise ValueError('invalid expected source identity: '+name)
            command += ['-p', name+':='+value]
    if getattr(args, 'global_search', False):
        command += ['--params-file', str(ROOT/'config/fast_livo2/relocalizer.yaml')]
        if args.index:
            command += ['-p', 'global_index_path:='+str(args.index.resolve())]
    if getattr(args, 'tag_relocalization', False) or getattr(args, 'tag_monitor', False) or getattr(args, 'tag_record', None):
        if args.tag_relocalization and not args.global_search:
            raise ValueError('Tag relocalization requires the explicit C2 entry')
        command += ['-p', 'tag_config_path:='+str(args.tag_config.resolve()),
                    '-p', 'enable_tag_relocalization:='+str(args.tag_relocalization).lower(),
                    '-p', 'enable_tag_monitor:='+str(args.tag_monitor).lower()]
        if args.tag_landmarks:
            command += ['-p', 'tag_landmarks_path:='+str(args.tag_landmarks.resolve())]
        if args.tag_record:
            command += ['-p', 'landmark_recording_mode:=c1', '-p', 'tag_record_path:='+str(args.tag_record.resolve())]
    components = [Component('monitor', [sys.executable, str(ROOT/'scripts/fast_livo2/localizer/monitor.py')], ready_key='probe_live'),
                  Component('localizer', command, depends=('monitor',), requires=('probe_live',), ready_key='localizer_alive',
                            resource='fast_livo_backend')]
    if args.offline_bridge:
        parameters = []
        for key, parameter in [('TF_MAX_AGE_S', 'tf_max_age_s'), ('ODOM_MAX_AGE_S', 'odom_max_age_s'),
                               ('ODOM_HISTORY_S', 'odom_history_s'), ('ODOM_MAX_GAP_S', 'odom_max_gap_s'),
                               ('HEARTBEAT_MAX_AGE_S', 'heartbeat_max_age_s')]:
            parameters += ['-p', parameter+':='+cfg[key]]
        components += [Component('adapter', [sys.executable, '-m', 'ranger_localization_bridge.fast_livo_localizer_adapter',
                          '--ros-args', '-p', 'use_sim_time:=true', *extrinsics], depends=('localizer',), resource='localization_backend'),
                       Component('bridge', [sys.executable, '-m', 'ranger_localization_bridge.bridge_node', '--ros-args',
                          '-p', 'adapter_backend:=external', '-p', 'use_sim_time:=true',
                          '-p', 'latch_on_innovation_failure:=true', *parameters], depends=('adapter',), resource='localization_bridge')]
    return components


def main():
    p = argparse.ArgumentParser(description=__doc__)
    action = p.add_mutually_exclusive_group()
    action.add_argument('--execute', action='store_true')
    action.add_argument('--status', action='store_true')
    action.add_argument('--stop', action='store_true')
    p.add_argument('--archive', type=Path)
    p.add_argument('--compatibility', type=Path)
    p.add_argument('--identity-file', type=Path, help='Expected frontend source identities; does not attest physical accuracy')
    p.add_argument('--offline-bridge', action='store_true')
    p.add_argument('--global-search', action='store_true', help='Explicit optional C2; no initial pose required')
    p.add_argument('--index', type=Path, help='Bound C2 index; omission builds an in-memory index')
    p.add_argument('--tag-config', type=Path, default=ROOT/'config/fast_livo2/apriltag_camera.json')
    p.add_argument('--tag-landmarks', type=Path)
    p.add_argument('--tag-relocalization', action='store_true')
    p.add_argument('--tag-monitor', action='store_true')
    p.add_argument('--tag-record', type=Path, help='New C1 confirmed-segment JSONL, no online map edit')
    p.add_argument('--domain', type=int, default=189)
    p.add_argument('--registration', choices=['pcl_icp', 'small_gicp', 'small_vgicp'], default='small_gicp')
    args = p.parse_args()
    if not 1 <= args.domain <= 232:
        p.error('offline domain must be 1..232')
    directory = ROOT/'.runtime/sessions'/('fast_livo_localizer_bridge' if args.offline_bridge else 'fast_livo_localizer')
    if args.global_search:
        directory = ROOT/'.runtime/sessions'/('fast_livo_relocalizer_bridge' if args.offline_bridge else 'fast_livo_relocalizer')
    if args.status:
        _, state = read_session(directory)
        print(json.dumps({'live': bool(session_live(state)), 'details': state}, indent=2))
        return 0
    if args.stop:
        return 0 if stop_session(directory) else 2
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1', PYTHONDONTWRITEBYTECODE='1')
    os.environ['PYTHONPATH'] = str(ROOT/'ws_nav/src/ranger_localization_bridge')+':'+os.environ.get('PYTHONPATH', '')
    os.environ['LD_LIBRARY_PATH'] = str(ROOT/'ws_fast_livo/backend_deps/lib')+':'+os.environ.get('LD_LIBRARY_PATH', '')
    components = specs(args)
    if not args.execute:
        print(json.dumps({'action': 'DRY-RUN', 'domain': args.domain, 'localhost_only': True,
                          'public_tf_enabled': args.offline_bridge, 'components': [asdict(c) for c in components]}, indent=2))
        return 0
    if session_live(read_session(directory)[1]):
        raise RuntimeError('managed C1 session is already live; inspect --status')
    import rclpy
    rclpy.init()
    probe = rclpy.create_node('ranger_localizer_occupancy_probe')
    try:
        deadline = time.monotonic()+2
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=.05)
        topics = ['/fast_livo/backend/estimate', '/fast_livo/localizer/estimate']
        if args.offline_bridge:
            topics += ['/localization/estimate', '/navigation_ready', '/tf', '/tf_static', '/cmd_vel', '/base/safe_cmd_vel', '/wheel/odom']
        occupied = [topic for topic in topics if probe.get_publishers_info_by_topic(topic)]
        if occupied:
            raise RuntimeError('C1 requires unclaimed outputs; start synthetic wheel after entry: '+', '.join(occupied))
    finally:
        probe.destroy_node()
        rclpy.shutdown()
    return run_guarded(directory, components, startup_timeout=20, fail_on_health_loss=True, health_loss_grace=1.)


if __name__ == '__main__':
    raise SystemExit(main())
