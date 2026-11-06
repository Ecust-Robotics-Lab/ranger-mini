#!/usr/bin/env python3
"""Managed FAST-LIVO2 frontend + private B mapping; dry-run unless --execute."""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'scripts'))
from session_manager import (Component, JsonHealth, atomic_json, notify, read_session,
                             run_guarded, same_process, session_live, stop_session)
from online_session import specs as sensor_specs, sensor_environment, check_sensor_processes
from record_session import (TOPICS, check_lidar_host_address, snapshot_sources,
                            archive_logs, sync_summary, quality_report)
from ranger_localization_bridge.runtime_config import resolve, validate_fast_livo_config

SESSION = ROOT/'.runtime/sessions/fast_livo_mapping'
PREFIX = ROOT/'ws_fast_livo/online_install'
BACKEND = PREFIX/'ranger_fast_livo_backend/lib/ranger_fast_livo_backend/backend_node'
RECEIPTS = PREFIX/'ranger_fast_livo_backend/lib/ranger_fast_livo_backend/receipt_probe'
FRONTEND_SOURCE = ROOT/'ws_fast_livo/online_frontend_source'
FRONTEND = ROOT/'ws_fast_livo/online_frontend_install/fast_livo/lib/fast_livo/fastlivo_mapping'
REQUIRED_TOPICS = {**TOPICS, '/fast_livo/deskewed_imu': 'sensor_msgs/msg/PointCloud2'}
EXTRA_TOPICS = ['/fast_livo/backend/estimate', '/fast_livo/backend/health',
                '/fast_livo/backend/optimized_path', '/fast_livo/backend/view/status',
                '/fast_livo/capture/input_receipts', '/clock']


def required_topics(args):
    # Auxiliary raw image/camera_info use host headers in this capture. The
    # audited replay view contains the three original fusion streams only;
    # the complete original capture remains bound by its view manifest.
    return {name: kind for name, kind in REQUIRED_TOPICS.items()
            if not args.replay_view or name not in ('/front/image', '/front/camera_info')}


def backup_frontend_logs(folder):
    root = FRONTEND_SOURCE/'Log'
    backup = folder/'frontend_logs/before'
    backup.mkdir(parents=True)
    files = []
    for name in ('mat_pre.txt', 'mat_out.txt', 'imu.txt'):
        if (root/name).is_file():
            shutil.copy2(root/name, backup/name)
            files.append(name)
    atomic_json(folder/'frontend_logs/backup.json', {'root': str(root), 'files': files,
        'sha256': {name: sha(backup/name) for name in files}})


def restore_frontend_logs(folder):
    info = load(folder/'frontend_logs/backup.json')
    if not info: return
    root, before, after = Path(info['root']), folder/'frontend_logs/before', folder/'frontend_logs/after'
    after.mkdir()
    for name in ('mat_pre.txt', 'mat_out.txt', 'imu.txt'):
        if (root/name).is_file(): shutil.copy2(root/name, after/name)
    for name in info['files']: shutil.copy2(before/name, root/name)
    atomic_json(folder/'frontend_logs/restored.json', {
        'existing_logs_restored': all(sha(root/name) == info['sha256'][name] for name in info['files']),
        'session_logs': str(after)})


def restore_stopped_frontend_logs(folder, state):
    # The guardian is this caller and stays alive after run_guarded returns.
    # Restore once its cleanup is complete and all managed children are gone.
    if state and state.get('guardian_cleanup_complete') and not any(
            same_process(r) for r in [state.get('owner'), *state.get('owned', [])]):
        restore_frontend_logs(folder)


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for chunk in iter(lambda: handle.read(1024*1024), b''): digest.update(chunk)
    return digest.hexdigest()


def load(path, default=None):
    try: return json.loads(Path(path).read_text())
    except (OSError, ValueError): return default


def frontend_identity(cfg):
    """The existing map identity, computed from the actual query frontend."""
    frontend = yaml.safe_load(Path(cfg['FAST_LIVO_CONFIG']).read_text())['/**']['ros__parameters']
    definitions = {
        'sensor_configuration_sha256': {'schema': 1, 'extrin_calib': frontend['extrin_calib'],
            'camera': frontend['camera'], 'preprocess': frontend['preprocess'],
            'camera_driver': yaml.safe_load(Path(cfg['FAST_LIVO_CAMERA_CONFIG']).read_text()),
            'lidar_driver': load(cfg['FAST_LIVO_LIDAR_CONFIG'])},
        'time_contract_sha256': {'schema': 1, 'time_offset': frontend['time_offset'],
            'pose': 'VIO-corrected IMU', 'cloud': 'same-stamp deskewed IMU interval',
            'raw_header_rewritten': False},
        'frontend_sources_sha256': {'schema': 1, 'binary_sha256': sha(FRONTEND),
            'sources_lock_sha256': sha(ROOT/'config/fast_livo2/sources.lock.json'),
            'patch_sha256': sha(ROOT/'patches/fast_livo2_stage_a.patch'),
            'online_io_patch_sha256': sha(ROOT/'patches/fast_livo2_online_io.patch')}}
    return {'schema_version': 1, **{key: hashlib.sha256(json.dumps(value,
        sort_keys=True, separators=(',', ':')).encode()).hexdigest() for key, value in definitions.items()},
        'definitions': definitions, 'physical_sync_validated': False}


def environment(cfg, folder):
    sensor_environment(cfg, folder/'sensor_assets')
    profile=folder/'config/mapping_fastdds.xml'
    if not profile.is_file():profile=ROOT/'config/fast_livo2/mapping_fastdds.xml'
    os.environ.update(RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
        FASTRTPS_DEFAULT_PROFILES_FILE=str(profile), FASTDDS_DEFAULT_PROFILES_FILE=str(profile),
        RMW_FASTRTPS_USE_QOS_FROM_XML='0', RMW_FASTRTPS_PUBLICATION_MODE='SYNCHRONOUS')
    os.environ['PYTHONPATH'] = ':'.join([str(ROOT/'scripts'),
        str(ROOT/'ws_nav/src/ranger_localization_bridge'), os.environ.get('PYTHONPATH', '')])
    os.environ['LD_LIBRARY_PATH'] = ':'.join([str(PREFIX/'ranger_fast_livo_backend/lib'),
        str(ROOT/'ws_fast_livo/backend_deps/lib'), os.environ.get('LD_LIBRARY_PATH', '')])


def components(cfg, args, folder):
    monitor = [sys.executable, str(ROOT/'scripts/fast_livo2/mapping_monitor.py'), '--folder', str(folder)]
    if args.replay_view: monitor.append('--replay')
    probe = Component('probe', monitor, ready_key='probe_live')
    backend = Component('backend', [str(BACKEND), '--ros-args', '--params-file',
        str(folder/'config/backend.yaml'), '-p', 'use_sim_time:=true',
        '-p', 'metadata_path:='+str(folder/'provenance.json'),
        '-p', 'data_root:='+str(folder/'backend_work'), '-p', 'publish_map_view:=true',
        '-p', 'enable_tag_loop:=false', '-p', 'registration:='+args.registration],
        depends=('probe',), start_requires=('backend_unclaimed', 'clock_unclaimed'),
        ready_key='backend_ready', critical=False, resource='fast_livo_backend')
    capture = Component('capture', [sys.executable, str(ROOT/'scripts/fast_livo2/record_worker.py'),
        '--folder', str(folder)], depends=('receipts',), ready_key='capture_recording',
        preflight_requires=('backend_unclaimed', 'clock_unclaimed'), resource='fast_livo_recorder')
    receipts = Component('receipts', [str(RECEIPTS), '--ros-args',
        '-p', 'context:='+str(folder), '-p', 'topics_json:='+json.dumps(json.dumps(required_topics(args),separators=(',',':')))],
        depends=('probe',), ready_key='receipt_probe_live', start_requires=('receipts_unclaimed',),
        resource='fast_livo_capture_receipts')
    if args.replay_view:
        frontend = Component('frontend', [str(FRONTEND),
            '--ros-args', '--params-file', cfg['FAST_LIVO_CONFIG'], '-p', 'use_sim_time:=true',
            '--log-level', 'warn'], depends=('probe',), ready_key='frontend',
            start_requires=('frontend_unclaimed', 'backend_listening'), resource='fast_livo_frontend',
            preflight_requires=('frontend_unclaimed',))
        player = Component('player', [sys.executable, str(ROOT/'scripts/fast_livo2/mapping_replay.py'),
            '--view', str(args.replay_view.resolve()), '--folder', str(folder)], depends=('receipts',),
            start_requires=('frontend_subscribed',), resource='fast_livo_replay', preflight_requires=())
        result = [probe, backend, receipts, capture, frontend, player]
    else:
        sensors = sensor_specs(cfg, 'none')
        sensors = [c for c in sensors if c.name != 'probe']
        next(c for c in sensors if c.name == 'frontend').start_requires += ('backend_listening',)
        next(c for c in sensors if c.name == 'frontend').preflight_requires = ('frontend_unclaimed',)
        next(c for c in sensors if c.name == 'frontend').argv[0] = str(FRONTEND)
        next(c for c in sensors if c.name == 'lidar').start_requires += ('clock_unclaimed',)
        next(c for c in sensors if c.name == 'lidar').depends = ('receipts',)
        result = [probe, backend, receipts, capture, *sensors]
    if args.visualization in ('rviz', 'both'):
        result.append(Component('rviz', ['/opt/ros/humble/lib/rviz2/rviz2', '-d',
            str(ROOT/'config/fast_livo2/mapping.rviz'), '--ros-args', '-p', 'use_sim_time:=true'],
            depends=('backend',), critical=False, resource='fast_livo_rviz'))
    if args.visualization in ('foxglove', 'both'):
        result.append(Component('foxglove', ['/opt/ros/humble/lib/foxglove_bridge/foxglove_bridge',
            '--ros-args', '--params-file', str(ROOT/'config/fast_livo2/mapping_foxglove.yaml'),
            '-p', 'port:='+str(args.foxglove_port)], depends=('probe',), critical=False,
            resource='fast_livo_foxglove'))
    return result


class MappingHealth(JsonHealth):
    def __init__(self, path, folder):
        super().__init__(path)
        self.folder, self.last_print, self.last_state = folder, -1e9, None
        self.announced_ready = False
        atomic_json(self.path.parent/'mapping.json', {'folder': str(folder)})

    def __call__(self):
        checks = super().__call__()
        health = load(self.path, {})
        recording = load(self.folder/'recording.json', {})
        ready = checks.get('capture_recording') and checks.get('frontend') and checks.get('backend_ready')
        phase = recording.get('state')
        state = ('MAPPING' if ready else 'DEGRADED') if phase == 'RECORDING' else (
            phase if phase in ('FINALIZING', 'RECORDED') else 'PREPARING')
        now = time.monotonic()
        if state != self.last_state or now-self.last_print > 10:
            backend = health.get('backend') or {}
            metrics = health.get('backend_metrics', {})
            lag = health.get('frontend_backend_lag_seconds')
            print(f'MAP {state}: 前端={health.get("paired_frames", 0)} 对，'
                  f'图版本={backend.get("graph_version", 0)}，回环事件={backend.get("loop_event_id", 0)}，'
                  f'队列={metrics.get("work_queue_entries", 0)}，延迟={lag} 秒；{backend.get("reason", "等待输入")}', flush=True)
            if ready and not self.announced_ready:
                print('在线建图已就绪；先静止15秒，再人工采集。Ctrl+C 封存地图并停录。', flush=True)
                self.announced_ready = True
            self.last_state, self.last_print = state, now
            atomic_json(self.folder/'mapping_state.json', {'state': state, 'monotonic': now, 'health': health})
            if health:
                with (self.folder/'mapping_metrics.jsonl').open('a') as handle:
                    handle.write(json.dumps(health)+'\n')
        return checks


def verify_snapshot(folder):
    manifest = load(folder/'manifest.json')
    marker = load(folder/'COMMITTED')
    if not manifest or not marker or marker.get('manifest_sha256') != sha(folder/'manifest.json'):
        raise ValueError('snapshot commit marker mismatch')
    for name, digest in manifest['files'].items():
        path = (folder/name).resolve()
        if not path.is_relative_to(folder.resolve()) or sha(path) != digest:
            raise ValueError('snapshot file checksum mismatch: '+name)
    accepted = 0
    for line in (folder/'candidates.jsonl').read_text().splitlines():
        row = json.loads(line)
        if row['decision'] == 'accepted_and_optimized':
            accepted += 1
            if row['graph_error_after_same_factors'] > row['graph_error_before_same_factors']+1e-6:
                raise ValueError('accepted graph objective increased')
    return {'checksummed_files': len(manifest['files']), 'keyframes': len(manifest['keyframes']),
            'loop_factors': sum(f['loop'] for f in manifest['factors']), 'accepted_loop_updates': accepted,
            'map_uuid': manifest['map_uuid'], 'session_id': manifest['session_id'],
            'graph_version': manifest['graph_version']}


def finalize(folder, timeout, run, state, records):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from ranger_fast_livo_backend.srv import FinalizeMapping
    result = {'success': False, 'hardware_started': load(folder/'manifest.json')['owns_sensors']}
    capture_stopped = False
    node = None
    old_profiles={key:os.environ.get(key) for key in ('FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE')}
    def stop_capture():
        nonlocal capture_stopped
        if not capture_stopped:
            notify(records.get('capture'), signal.SIGINT)
            capture_stopped = True
    try:
        print('MAP FINALIZING：封存后端输入，排空已接收队列，再保存地图。', flush=True)
        atomic_json(folder/'mapping_state.json', {'state': 'FINALIZING'})
        health = load(folder/'mapping_health.json', {})
        backend = health.get('backend') or {}
        if not same_process(records.get('backend')) or not backend.get('session_id'):
            raise RuntimeError('backend unavailable; raw recording will be retained')
        # RPC needs only a few small messages. Avoid allocating a large data-plane
        # SHM segment while the source player is tearing down its participant.
        control_profile=folder/'config/mapping_control_fastdds.xml'
        for key in old_profiles:os.environ[key]=str(control_profile)
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('ranger_mapping_finalizer')
        client = node.create_client(FinalizeMapping, '/fast_livo/backend/finalize_mapping')
        if not client.wait_for_service(timeout_sec=5): raise RuntimeError('finalization service unavailable')
        request = FinalizeMapping.Request()
        request.expected_session_id = backend['session_id']
        request.destination = str(folder/'map')
        request.timeout_seconds = float(timeout)
        future = client.call_async(request)
        deadline, progress = time.monotonic()+timeout+120, -1e9
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            current = load(folder/'mapping_health.json', {})
            metrics = current.get('backend_metrics', {})
            if metrics.get('sealed'): stop_capture()
            if time.monotonic()-progress >= 10:
                print('MAP FINALIZING：待配对='+str(metrics.get('pair_cache_entries'))
                      +'，待处理='+str(metrics.get('work_queue_entries'))
                      +'，工作线程='+str(metrics.get('worker_busy'))+'；地图导出可能仍在进行。', flush=True)
                progress = time.monotonic()
            if not same_process(records.get('backend')): raise RuntimeError('backend exited during finalization')
        if not future.done(): raise TimeoutError('map finalization deadline exceeded; partial data preserved')
        response = future.result()
        result.update(success=response.success, reason=response.reason, session_id=response.session_id,
            graph_version=response.graph_version, last_stamp_ns=response.last_stamp_ns,
            processed_pairs=response.processed_pairs, manifest_path=response.manifest_path)
        if not response.success: raise RuntimeError(response.reason)
        result['snapshot'] = verify_snapshot(folder/'map')
    except Exception as exc:
        result.update(success=False, error=str(exc))
    finally:
        stop_capture()
        if node:
            node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        for key,value in old_profiles.items():
            if value is None:os.environ.pop(key,None)
            else:os.environ[key]=value
        deadline = time.monotonic()+32
        while same_process(records.get('capture')) and time.monotonic() < deadline:
            time.sleep(.05)
        result['recorder'] = load(folder/'recorder_result.json', {})
        recorder = result['recorder']
        if recorder.get('exit') != 0 or recorder.get('forced') or recorder.get('error'):
            result.update(success=False, recorder_error='recorder did not finalize cleanly')
        if state.get('reason'): result.update(success=False, session_error=state['reason'])
        atomic_json(folder/'mapping_result.json', result)
    if not result['success']: raise RuntimeError(result.get('error', result.get('recorder_error', 'mapping failed')))
    print('MAP SAVED '+str(folder/'map'), flush=True)


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    action = p.add_mutually_exclusive_group()
    action.add_argument('--execute', action='store_true')
    action.add_argument('--dry-run', action='store_true')
    action.add_argument('--status', action='store_true')
    action.add_argument('--stop', action='store_true')
    p.add_argument('--seconds', type=float, default=900)
    p.add_argument('--label', default='map')
    p.add_argument('--output', type=Path)
    p.add_argument('--runtime-config', default=str(ROOT/'config/profiles/fast_livo2_frontend.yaml'))
    p.add_argument('--registration', choices=('pcl_icp', 'small_gicp', 'small_vgicp'), default='pcl_icp')
    p.add_argument('--visualization', choices=('auto', 'rviz', 'foxglove', 'both', 'none'), default='auto')
    p.add_argument('--foxglove-port', type=int, default=8765)
    p.add_argument('--min-free-gb', type=float, default=20)
    p.add_argument('--finalize-timeout', type=float, default=90.)
    p.add_argument('--replay-view', type=Path, help='Offline header-clock bag view; no sensor drivers')
    p.add_argument('--domain', type=int)
    return p


def main():
    p = parser(); args = p.parse_args()
    if args.status:
        run, state = read_session(SESSION)
        capture = load(run/'mapping.json', load(run/'capture.json', {})) if run else {}
        folder = Path(capture['folder']) if capture.get('folder') else None
        print(json.dumps({'live': bool(session_live(state)), 'session': state,
            'folder': str(folder) if folder else None,
            'mapping': load(folder/'mapping_state.json') if folder else None,
            'result': load(folder/'mapping_result.json') if folder else None}, ensure_ascii=False, indent=2)); return 0
    if args.stop: return 0 if stop_session(SESSION, timeout=400) else 2
    for name, value, minimum, maximum in [('seconds', args.seconds, 1, 3600),
            ('min-free-gb', args.min_free_gb, 1, 10000), ('finalize-timeout', args.finalize_timeout, 1, 180)]:
        if not math.isfinite(value) or not minimum <= value <= maximum: p.error(f'invalid {name}')
    if not args.label or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in args.label):
        p.error('label must use letters, numbers, _ or -')
    if not 1 <= args.foxglove_port <= 65535: p.error('invalid foxglove port')
    if args.domain is not None and not 1 <= args.domain <= 232: p.error('domain must be 1..232')
    if args.visualization == 'auto': args.visualization = 'rviz' if os.environ.get('DISPLAY', '').strip() else 'foxglove'
    os.environ['RANGER_RUNTIME_CONFIG'] = args.runtime_config
    cfg = resolve(ROOT)
    if args.domain is not None: cfg['FAST_LIVO_ROS_DOMAIN_ID'] = str(args.domain)
    if args.replay_view and int(cfg['FAST_LIVO_ROS_DOMAIN_ID']) == 0: p.error('replay requires an isolated domain')
    validate_fast_livo_config(cfg['FAST_LIVO_CONFIG'])
    frontend = yaml.safe_load(Path(cfg['FAST_LIVO_CONFIG']).read_text())['/**']['ros__parameters']
    if frontend['time_offset']['img_time_offset'] != .1: raise ValueError('formal mapping requires img_time_offset +0.1s')
    if not frontend['uav']['gravity_align_en']: raise ValueError('formal mapping requires initial gravity alignment')
    folder = args.output.expanduser().absolute() if args.output else ROOT/'bags'/('fast-livo-map-'+args.label+'-NEW')
    environment(cfg, folder)
    if not args.execute:
        print(json.dumps({'action': 'DRY-RUN', 'output': str(folder),
            'domain': cfg['FAST_LIVO_ROS_DOMAIN_ID'], 'hardware_started': False,
            'would_start_sensors': not bool(args.replay_view), 'seconds': args.seconds,
            'required_topics': required_topics(args), 'record_topics': [*required_topics(args), *EXTRA_TOPICS],
            'components': [asdict(c) for c in components(cfg, args, folder)],
            'public_tf': False, 'tag_enabled': False, 'no_base_or_navigation': True}, indent=2)); return 0
    if session_live(read_session(SESSION)[1]): raise RuntimeError('mapping already active; inspect --status')
    if cfg['FAST_LIVO_LOCALHOST_ONLY'] != '1': raise RuntimeError('mapping requires localhost_only=1')
    if not folder.parent.is_dir(): raise ValueError('output parent must exist')
    if args.output and folder.exists(): raise FileExistsError('refusing to overwrite '+str(folder))
    if shutil.disk_usage(folder.parent).free < args.min_free_gb*1e9: raise RuntimeError('insufficient free disk')
    if not BACKEND.is_file() or not FRONTEND.is_file() or not RECEIPTS.is_file():
        raise RuntimeError('build online frontend/backend: bash scripts/build_fast_livo2_online.sh --execute')
    if args.replay_view:
        if not (args.replay_view/'metadata.yaml').is_file(): raise ValueError('replay view is not a bag')
        view_manifest = load(args.replay_view/'view_manifest.json', {})
        required_sensors = {'/livox/lidar', '/livox/imu', '/front/image_sync'}
        if (not view_manifest.get('payloads_identical') or view_manifest.get('header_or_point_timestamp_modified') is not False
                or not all(view_manifest.get('counts', {}).get(t, 0) > 0 for t in required_sensors)):
            raise ValueError('replay requires an audited header-clock view with all three fusion streams')
    else:
        check_sensor_processes(); check_lidar_host_address(cfg)
    if args.visualization in ('rviz', 'both') and not os.environ.get('DISPLAY', '').strip():
        raise ValueError('RViz requires DISPLAY')
    for component in components(cfg, args, folder):
        exe = component.argv[0]
        if exe.startswith('/') and not os.access(exe, os.X_OK): raise RuntimeError('missing executable: '+exe)
    if args.output: folder.mkdir()
    else: folder = Path(tempfile.mkdtemp(prefix='fast-livo-map-'+args.label+'-', dir=ROOT/'bags'))
    (folder/'sensor_assets').mkdir(); (folder/'config').mkdir()
    source_files = snapshot_sources(folder, cfg)
    shutil.copy2(FRONTEND, folder/'sources/online_fastlivo_mapping')
    source_files.append({'source': str(FRONTEND), 'file': 'sources/online_fastlivo_mapping', 'sha256': sha(FRONTEND)})
    shutil.copy2(FRONTEND_SOURCE/'online_source_manifest.json', folder/'sources/online_source_manifest.json')
    for key in ('FAST_LIVO_CONFIG', 'FAST_LIVO_CAMERA_CONFIG', 'FAST_LIVO_LIDAR_CONFIG'):
        source = Path(cfg[key]); destination = folder/'config'/source.name
        shutil.copy2(source, destination); cfg[key] = str(destination)
    shutil.copy2(ROOT/'config/fast_livo2/backend.yaml', folder/'config/backend.yaml')
    shutil.copy2(ROOT/'config/fast_livo2/mapping_fastdds.xml', folder/'config/mapping_fastdds.xml')
    shutil.copy2(ROOT/'config/fast_livo2/mapping_control_fastdds.xml', folder/'config/mapping_control_fastdds.xml')
    environment(cfg, folder)
    manifest = {'schema_version': 1, 'kind': 'online_mapping_capture', 'folder': str(folder),
        'owns_sensors': not bool(args.replay_view), 'seconds': args.seconds,
        'domain': cfg['FAST_LIVO_ROS_DOMAIN_ID'], 'min_free_bytes': int(args.min_free_gb*1e9),
        'topics': required_topics(args), 'record_topics': [*required_topics(args), *EXTRA_TOPICS],
        'managed_finalization': True, 'preflight_timeout': 90, 'source_files': source_files,
        'no_base_or_navigation': True, 'physical_sync_proven': False,
        'replay_view': str(args.replay_view.resolve()) if args.replay_view else None}
    manifest['dds_environment']={key:os.environ[key] for key in ('RMW_IMPLEMENTATION',
        'FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE',
        'RMW_FASTRTPS_USE_QOS_FROM_XML','RMW_FASTRTPS_PUBLICATION_MODE')}
    atomic_json(folder/'manifest.json', manifest)
    runtime_files = [BACKEND, RECEIPTS, PREFIX/'ranger_fast_livo_backend/lib/libranger_backend_core.so',
        FRONTEND,
        *list((folder/'config').iterdir())]
    source_paths = list((ROOT/'ws_fast_livo/src/ranger_fast_livo_backend').rglob('*.cpp'))
    source_paths += list((ROOT/'ws_fast_livo/src/ranger_fast_livo_backend').rglob('*.hpp'))
    source_paths += list((ROOT/'scripts/fast_livo2').glob('mapping*.py'))
    source_paths += [FRONTEND_SOURCE/'online_source_manifest.json', ROOT/'patches/fast_livo2_online_io.patch']
    provenance = {'source_data_id': str(folder), 'frontend_gravity_aligned': True,
        'gravity_alignment_evidence': 'owned frontend configured gravity_align_en=true; physical accuracy unverified',
        'physical_sync': 'PARTIAL', 'img_time_offset_seconds': .1, 'tag_enabled': False,
        'clock_mode': 'header-clock replay' if args.replay_view else 'fixed lidar-header/steady anchor; no host/wheel synchronization claim',
        'declared_frontend_config': cfg['FAST_LIVO_CONFIG'],
        'declared_frontend_config_sha256': sha(cfg['FAST_LIVO_CONFIG']),
        'active_frontend_configuration_attested': True,
        'attestation_scope': 'owned argv, frozen parameters, derived-source manifest and executable hashes; no physical calibration claim',
        'files_sha256': {str(f): sha(f) for f in [*runtime_files, *source_paths]},
        'T_imu_lidar': {'already_applied_by_frontend': True, 'declared_extrinsics': frontend['extrin_calib']},
        'git_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()}
    identity = frontend_identity(cfg)
    provenance.update({key: identity[key] for key in ('sensor_configuration_sha256', 'time_contract_sha256', 'frontend_sources_sha256')})
    atomic_json(folder/'source_identity.json', identity)
    atomic_json(folder/'provenance.json', provenance)
    print('MAP_DIR='+str(folder), flush=True)
    try:
        code = run_guarded(SESSION, components(cfg, args, folder),
            health_factory=lambda path: MappingHealth(path, folder), startup_timeout=100,
            fail_on_health_loss=True, health_loss_grace=3, cleanup_grace=40,
            shutdown_timeout=args.finalize_timeout+155,
            prepare_hook=lambda: backup_frontend_logs(folder),
            shutdown_hook=lambda run, state, records: finalize(folder, args.finalize_timeout, run, state, records))
    finally:
        restore_stopped_frontend_logs(folder, read_session(SESSION)[1])
    run, state = read_session(SESSION)
    archive_logs(folder, run, run, folder/'sensor_assets', True)
    result = load(folder/'mapping_result.json', {'success': False})
    try:
        from audit_bag import audit
        report = audit(folder/'raw', selected_topics=required_topics(args))
        atomic_json(folder/'audit.json', report)
        result['data_quality'] = quality_report(report, required_topics(args))
        result['sync'] = sync_summary(folder/'sensor_assets')
        if result['data_quality']['failures']: result['success'] = False
    except (Exception, KeyboardInterrupt) as exc:
        result.update(success=False, audit_error=str(exc) or 'audit interrupted; finalized data retained')
    result.update(session_exit=code, session_state=state, absolute_accuracy='NOT_EVALUATED',
        physical_sync='PARTIAL', online_hardware_acceptance='NOT_EVALUATED' if args.replay_view else 'REQUIRES_REVIEW')
    result['success'] = result['success'] and code == 0
    atomic_json(folder/'mapping_result.json', result)
    atomic_json(folder/'mapping_state.json', {'state': 'COMPLETE' if result['success'] else 'FAILED', 'result': result})
    if result['success']: (folder/'MAPPING_COMPLETE').write_text('Map archived and data checks passed; accuracy/physical synchronization unverified.\n')
    print('MAP '+('COMPLETE ' if result['success'] else 'FAILED ')+str(folder), flush=True)
    return 0 if result['success'] else 2


if __name__ == '__main__':
    try: raise SystemExit(main())
    except (RuntimeError, OSError, ValueError) as exc:
        print('MAP ERROR: '+str(exc), file=sys.stderr, flush=True); raise SystemExit(2)
