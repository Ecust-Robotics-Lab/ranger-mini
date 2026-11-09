#!/usr/bin/env python3
"""Managed private online C1/C2. Uses the existing mapping sensor transport."""
import argparse
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time

import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT/'scripts/fast_livo2'), str(ROOT/'scripts'),
               str(ROOT/'ws_nav/src/ranger_localization_bridge')]
import mapping_session as mapping
from session_manager import JsonHealth, atomic_json, read_session, session_live, run_guarded, stop_session
from ranger_localization_bridge.runtime_config import resolve, validate_fast_livo_config

SESSION = ROOT/'.runtime/sessions/fast_livo_online_localization'
PREFIX = ROOT/'ws_fast_livo/online_localization_install/ranger_fast_livo_localizer'
NODE = PREFIX/'lib/ranger_fast_livo_localizer/localizer_node'
IDENTITIES = ('sensor_configuration_sha256', 'time_contract_sha256', 'frontend_sources_sha256')


def components(cfg, args, folder, identity):
    # Keep transport, devices, discovery checks and process resources identical.
    result = mapping.components(cfg, args, folder)
    localizer = next(c for c in result if c.name == 'backend')
    localizer.name = 'localizer'
    config_root = folder/'config' if (folder/'config/localizer.yaml').exists() else ROOT/'config/fast_livo2'
    localizer.argv = [str(NODE), '--ros-args', '--params-file', str(config_root/'localizer.yaml'),
        '--params-file', str(config_root/'relocalizer.yaml'),
        '-p', 'archive_path:='+str(args.archive.resolve()), '-p', 'use_sim_time:=true',
        '-p', 'online_scheduling:=true', '-p', 'registration:='+args.registration,
        '-p', 'global_search_enabled:='+str(args.mode == 'auto').lower()]
    if args.descriptor:
        localizer.argv += ['-p', 'global_descriptor:='+args.descriptor]
    for key in IDENTITIES:
        localizer.argv += ['-p', key+':='+identity[key]]
    localizer.ready_key = 'localizer_alive'
    localizer.critical = True
    for c in result:
        c.depends = tuple('localizer' if d == 'backend' else d for d in c.depends)
        if c.name == 'probe': c.argv.append('--localization')
        if c.name == 'frontend':
            c.argv = [a.replace('use_sim_time:=false', 'use_sim_time:=true') for a in c.argv]
        if c.name == 'rviz':
            c.argv[c.argv.index('-d')+1] = str(ROOT/'config/fast_livo2/localization.rviz')
    if args.visualization == 'rviz':
        from session_manager import Component
        result.append(Component('view', [sys.executable, str(ROOT/'scripts/fast_livo2/localizer/view.py'),
            '--archive', str(args.archive.resolve())], depends=('localizer',), critical=False,
            resource='fast_livo_localization_view'))
    return result


class Health(JsonHealth):
    def __init__(self, path, folder):
        super().__init__(path)
        self.folder, self.last_print, self.previous = folder, -1e9, None
        atomic_json(path.parent/'localization.json', {'folder': str(folder)})

    def __call__(self):
        checks = super().__call__()
        health = mapping.load(self.path, {})
        estimate = health.get('backend') or {}
        phase = estimate.get('reason', 'waiting_inputs')
        now = time.monotonic()
        if phase != self.previous or now-self.last_print > 10:
            metrics = health.get('backend_metrics', {})
            print(f'LOCALIZATION state={estimate.get("state")} valid={estimate.get("pose_valid", False)} '
                  f'{phase}; processed={metrics.get("processed_scans", 0)} '
                  f'skipped={metrics.get("skipped_scans", 0)} age={metrics.get("sample_age_seconds")}', flush=True)
            self.previous, self.last_print = phase, now
        return checks


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    action = p.add_mutually_exclusive_group()
    action.add_argument('--execute', action='store_true')
    action.add_argument('--dry-run', action='store_true')
    action.add_argument('--status', action='store_true')
    action.add_argument('--stop', action='store_true')
    p.add_argument('--archive', type=Path)
    p.add_argument('--mode', choices=('auto', 'manual'), default='auto')
    p.add_argument('--runtime-config', default=str(ROOT/'config/profiles/fast_livo2_frontend.yaml'))
    p.add_argument('--replay-view', type=Path)
    p.add_argument('--output', type=Path)
    p.add_argument('--seconds', type=float, default=900.)
    p.add_argument('--domain', type=int, default=215)
    p.add_argument('--min-free-gb', type=float, default=20.)
    p.add_argument('--registration', choices=('pcl_icp', 'small_gicp'), default='small_gicp')
    p.add_argument('--descriptor', choices=('radial_pair_distribution_v1', 'radial_pair_multiscale_v2'), default='radial_pair_multiscale_v2')
    p.add_argument('--visualization', choices=('auto', 'rviz', 'none'), default='auto')
    return p


def main():
    p = parser(); args = p.parse_args()
    if args.status:
        run, state = read_session(SESSION)
        info = mapping.load(run/'localization.json', {}) if run else {}
        folder = Path(info['folder']) if info else None
        print(json.dumps({'live': bool(session_live(state)), 'session': state, 'folder': str(folder),
            'health': mapping.load(folder/'localization_health.json') if folder else None,
            'result': mapping.load(folder/'localization_result.json') if folder else None}, indent=2)); return 0
    if args.stop: return 0 if stop_session(SESSION, timeout=90) else 2
    if not args.archive: p.error('--archive is required')
    if not 1 <= args.domain <= 232: p.error('isolated domain must be 1..232')
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 3600: p.error('seconds must be 1..3600')
    if not math.isfinite(args.min_free_gb) or args.min_free_gb < 1: p.error('min-free-gb must be >=1')
    args.visualization = ('rviz' if os.environ.get('DISPLAY') else 'none') if args.visualization == 'auto' else args.visualization
    args.foxglove_port = 8765  # unused by this entry; shared component builder argument
    os.environ['RANGER_RUNTIME_CONFIG'] = args.runtime_config
    cfg = resolve(ROOT); cfg['FAST_LIVO_ROS_DOMAIN_ID'] = str(args.domain)
    validate_fast_livo_config(cfg['FAST_LIVO_CONFIG'])
    frontend = yaml.safe_load(Path(cfg['FAST_LIVO_CONFIG']).read_text())['/**']['ros__parameters']
    if frontend['time_offset']['img_time_offset'] != .1 or not frontend['uav']['gravity_align_en']:
        raise ValueError('online localization requires formal +100ms and initial gravity alignment')
    folder = args.output.expanduser().absolute() if args.output else ROOT/'bags/fast-livo-localization-NEW'
    mapping.environment(cfg, folder)
    os.environ['LD_LIBRARY_PATH'] = str(PREFIX/'lib')+':'+os.environ.get('LD_LIBRARY_PATH', '')
    identity = mapping.frontend_identity(cfg)
    if not args.execute:
        print(json.dumps({'action': 'DRY-RUN', 'output': str(folder), 'mode': args.mode,
            'hardware_started': False, 'would_start_sensors': not bool(args.replay_view),
            'public_tf': False, 'components': [asdict(c) for c in components(cfg, args, folder, identity)]}, indent=2)); return 0
    if session_live(read_session(SESSION)[1]): raise RuntimeError('online localization already active')
    if cfg['FAST_LIVO_LOCALHOST_ONLY'] != '1': raise RuntimeError('requires localhost_only=1')
    if not folder.parent.is_dir() or (args.output and folder.exists()): raise ValueError('output must be new with an existing parent')
    if shutil.disk_usage(folder.parent).free < args.min_free_gb*1e9: raise RuntimeError('insufficient free disk')
    if args.visualization == 'rviz' and not os.environ.get('DISPLAY'): raise ValueError('RViz requires DISPLAY')
    if args.replay_view:
        view = mapping.load(args.replay_view/'view_manifest.json', {})
        if (not (args.replay_view/'metadata.yaml').is_file() or not view.get('payloads_identical')
                or view.get('header_or_point_timestamp_modified') is not False
                or not all(view.get('counts', {}).get(t, 0)>0 for t in ('/livox/lidar', '/livox/imu', '/front/image_sync'))):
            raise ValueError('requires an audited three-sensor header-clock view')
    else:
        mapping.check_sensor_processes(); mapping.check_lidar_host_address(cfg)
    for component in components(cfg, args, folder, identity):
        exe = component.argv[0]
        if exe.startswith('/') and not os.access(exe, os.X_OK): raise RuntimeError('missing executable '+exe)
    if args.output: folder.mkdir()
    else: folder = Path(tempfile.mkdtemp(prefix='fast-livo-localization-', dir=ROOT/'bags'))
    (folder/'sensor_assets').mkdir(); (folder/'config').mkdir()
    source_files = mapping.snapshot_sources(folder, cfg)
    for key in ('FAST_LIVO_CONFIG', 'FAST_LIVO_CAMERA_CONFIG', 'FAST_LIVO_LIDAR_CONFIG'):
        src = Path(cfg[key]); dest = folder/'config'/src.name; shutil.copy2(src, dest); cfg[key] = str(dest)
    shutil.copy2(ROOT/'config/fast_livo2/mapping_fastdds.xml', folder/'config/mapping_fastdds.xml')
    for name in ('localizer.yaml', 'relocalizer.yaml'):
        shutil.copy2(ROOT/'config/fast_livo2'/name, folder/'config'/name)
    atomic_json(folder/'source_identity.json', identity)
    loader = subprocess.run([str(PREFIX/'lib/ranger_fast_livo_localizer/archive_tool'), '--archive',
        str(args.archive.resolve()), '--identity-file', str(folder/'source_identity.json')], capture_output=True, text=True)
    (folder/'archive_loader.log').write_text(loader.stdout+loader.stderr)
    if loader.returncode: raise RuntimeError('map identity/integrity validation failed; see '+str(folder/'archive_loader.log'))
    mapping.environment(cfg, folder)
    os.environ['LD_LIBRARY_PATH'] = str(PREFIX/'lib')+':'+os.environ.get('LD_LIBRARY_PATH', '')
    record_topics = [*mapping.required_topics(args), '/fast_livo/localizer/estimate', '/fast_livo/capture/input_receipts', '/clock']
    atomic_json(folder/'manifest.json', {'schema_version': 1, 'kind': 'online_localization_capture',
        'folder': str(folder), 'seconds': args.seconds, 'mode': args.mode, 'domain': args.domain,
        'topics': mapping.required_topics(args), 'record_topics': record_topics,
        'min_free_bytes': int(args.min_free_gb*1e9), 'managed_finalization': True,
        'preflight_timeout': 90, 'source_files': source_files,
        'replay_view': str(args.replay_view.resolve()) if args.replay_view else None,
        'no_base_or_navigation': True, 'physical_sync_proven': False})
    atomic_json(folder/'provenance.json', {'archive': str(args.archive.resolve()),
        'archive_manifest_sha256': mapping.sha(args.archive/'manifest.json'),
        'hardware_started': not bool(args.replay_view), 'source_identity': identity,
        'files_sha256': {str(f): mapping.sha(f) for f in [NODE, mapping.FRONTEND,
            PREFIX/'lib/libranger_localizer_core.so', *list((folder/'config').iterdir()),
            ROOT/'ws_fast_livo/src/ranger_fast_livo_localizer/src/node.cpp', Path(__file__)]}})
    print('LOCALIZATION_DIR='+str(folder), flush=True)
    try:
        code = run_guarded(SESSION, components(cfg, args, folder, identity),
            health_factory=lambda path: Health(path, folder), startup_timeout=100,
            fail_on_health_loss=True, health_loss_grace=3, cleanup_grace=40,
            prepare_hook=lambda: mapping.backup_frontend_logs(folder))
    finally:
        mapping.restore_stopped_frontend_logs(folder, read_session(SESSION)[1])
    run, state = read_session(SESSION)
    mapping.archive_logs(folder, run, run, folder/'sensor_assets', True)
    recorder = mapping.load(folder/'recorder_result.json', {})
    result = {'workflow_success': code == 0 and recorder.get('exit') == 0 and not recorder.get('error'),
        'session_exit': code, 'session_state': state.get('state'), 'recorder': recorder,
        'localization_acceptance': 'NOT_EVALUATED', 'physical_sync': 'PARTIAL',
        'hardware_started': not bool(args.replay_view)}
    atomic_json(folder/'localization_result.json', result)
    print(json.dumps(result, indent=2)); return 0 if result['workflow_success'] else 2


if __name__ == '__main__':
    try: raise SystemExit(main())
    except (RuntimeError, OSError, ValueError) as exc:
        print('LOCALIZATION ERROR: '+str(exc), file=sys.stderr); raise SystemExit(2)
