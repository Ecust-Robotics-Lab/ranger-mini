#!/usr/bin/env python3
"""One-command tri-sensor capture, ownership-aware cleanup, immutable evidence."""
import argparse
from collections import Counter
import csv
from dataclasses import asdict
import hashlib
import ipaddress
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'scripts'))
from session_manager import Component, JsonHealth, atomic_json, read_session, run_guarded, session_live, stop_session
from online_session import check_sensor_processes, sensor_environment, specs
sys.path.insert(0, str(ROOT/'ws_nav/src/ranger_localization_bridge'))
from ranger_localization_bridge.runtime_config import resolve, validate_fast_livo_config

TOPICS = {
    '/livox/lidar': 'sensor_msgs/msg/PointCloud2',
    '/livox/imu': 'sensor_msgs/msg/Imu',
    '/front/image_sync': 'sensor_msgs/msg/Image',
    '/front/image': 'sensor_msgs/msg/Image',
    '/front/camera_info': 'sensor_msgs/msg/CameraInfo',
    '/fast_livo/odometry': 'nav_msgs/msg/Odometry',
}
SESSION_ROOT = Path(os.environ.get('RANGER_CAPTURE_SESSION_ROOT', ROOT/'.runtime/sessions'))
SESSION = SESSION_ROOT/'fast_livo_record'


def check_lidar_host_address(cfg):
    """Read-only check of local bind addresses, without contacting a sensor."""
    path = Path(cfg['FAST_LIVO_LIDAR_CONFIG'])
    config = json.loads(path.read_text())
    required = {entry['host_ip'] for model in config.values() if isinstance(model, dict)
                for entry in model.get('host_net_info', [])}
    if not required:
        raise ValueError('雷达配置缺少 host_net_info.host_ip: '+str(path))
    addresses = json.loads(subprocess.check_output(['ip', '-j', '-4', 'address', 'show'], text=True))
    local = {entry['local'] for interface in addresses for entry in interface.get('addr_info', [])
             if entry.get('family') == 'inet'}
    missing = sorted(address for address in required if not ipaddress.ip_address(address).is_unspecified
                     and address not in local)
    if missing:
        raise RuntimeError('雷达采集网卡地址未就绪：配置要求 '+', '.join(missing)
                           +'；本机当前 IPv4 为 '+', '.join(sorted(local))
                           +'。请检查雷达连接与采集网卡配置；未启动驱动或录包。配置: '+str(path))


def input_problems(topics, publishers, counts):
    """Explain absent streams separately from discovery/type/QoS failures."""
    problems = []
    for topic, kind in topics.items():
        endpoints = publishers.get(topic, [])
        if len(endpoints) != 1:
            problems.append(topic+(' 无发布者' if not endpoints else ' 存在多个发布者'))
        elif endpoints[0]['type'] != kind:
            problems.append(topic+' 消息类型不匹配')
        elif endpoints[0]['reliability'] != 'RELIABLE':
            problems.append(topic+' QoS 不是 RELIABLE')
        elif not counts.get(topic, 0):
            problems.append(topic+' 有发布者但未收到数据')
    return problems


def recorder_command(folder, topics):
    return ['ros2', 'bag', 'record', '-s', 'sqlite3', '-o', str(folder/'raw'),
            '--max-cache-size', '268435456', '--qos-profile-overrides-path',
            str(ROOT/'config/mid360/bag_qos.yaml'), *topics]


def snapshot_sources(folder, cfg):
    """Preserve exact bytes, including profile inheritance and local modifications."""
    paths = {ROOT/'config/runtime.yaml', Path(cfg['RANGER_RUNTIME_CONFIG']),
             ROOT/'config/fast_livo2/sources.lock.json', ROOT/'patches/fast_livo2_stage_a.patch',
             ROOT/'config/mid360/bag_qos.yaml',
             ROOT/'ws_camera/install/hik_camera_trigger_driver/share/hik_camera_trigger_driver/config/camera_info.yaml'}
    paths.update(Path(cfg[k]) for k in ('FAST_LIVO_CONFIG', 'FAST_LIVO_CAMERA_CONFIG',
                                      'FAST_LIVO_LIDAR_CONFIG', 'FAST_LIVO_RVIZ_CONFIG'))
    for base in (ROOT/'config/fast_livo2', ROOT/'config/profiles'):
        paths.update(p for p in base.rglob('*') if p.is_file() and p.suffix in ('.yaml', '.json', '.rviz'))
    paths.update(p for p in (ROOT/'scripts/fast_livo2').rglob('*.py'))
    paths.add(ROOT/'scripts/record_fast_livo2.sh')
    # Attribute the actual local synchronization implementation to this capture.
    paths.update(ROOT/p for p in (
        'ws_camera/src/timeshare_writer/src/timeshare_writer_node.cpp',
        'ws_camera/src/timeshare_writer/CMakeLists.txt',
        'ws_camera/src/timeshare_writer/package.xml',
        'ws_camera/src/hik_camera_trigger_driver/src/hik_camera_node.cpp',
        'ws_camera/install/timeshare_writer/lib/timeshare_writer/timeshare_writer_node',
        'ws_camera/install/hik_camera_trigger_driver/lib/hik_camera_trigger_driver/hik_camera_trigger_driver_node'))
    files = []
    dest = folder/'sources'
    dest.mkdir()
    for i, path in enumerate(sorted(paths)):
        content = path.read_bytes()  # Missing required configuration aborts before devices start.
        target = dest/(str(i)+'_'+path.name)
        target.write_bytes(content)
        files.append({'source': str(path), 'file': str(target.relative_to(folder)),
                      'sha256': hashlib.sha256(content).hexdigest()})
    for name, command in [('git_status.txt', ['git', 'status', '--short']),
                          ('git_diff.patch', ['git', 'diff', '--binary']),
                          ('git_head.txt', ['git', 'rev-parse', 'HEAD']),
                          ('network.txt', ['ip', '-br', 'address'])]:
        (dest/name).write_bytes(subprocess.check_output(command, cwd=ROOT))
    return files


def attached_assets(run):
    state = json.loads((run/'state.json').read_text())
    writer = state['components'].get('writer', {}).get('argv', [])
    value = next((a.split(':=', 1)[1] for a in writer if a.startswith('timeshare_path:=')), None)
    if not value:
        raise RuntimeError('existing session has no attributable timeshare path')
    return Path(value).parent


class CaptureHealth(JsonHealth):
    """Report recording readiness on the foreground terminal without changing the manager."""
    def __init__(self, path, folder):
        super().__init__(path)
        self.folder, self.last = folder, None
        self.last_waiting, self.last_progress = None, float('-inf')

    def __call__(self):
        checks = super().__call__()
        status = None
        try:
            status = json.loads((self.folder/'recording.json').read_text())['state']
            if status != self.last:
                print('CAPTURE '+status+' '+str(self.folder), flush=True)
                if status == 'RECORDING':
                    print('录制已就绪；先静止15秒，再开始人工采集动作。Ctrl+C 停录并归档。', flush=True)
                self.last = status
        except (OSError, ValueError, KeyError):
            pass
        if status is None or status == 'PREFLIGHT':
            try:
                state = json.loads((self.path.parent/'state.json').read_text())
                names = {'probe': '输入检查', 'lidar': '雷达', 'writer': '同步时间',
                         'camera': '相机', 'frontend': '前端', 'capture': '录包器'}
                waiting = tuple(names[name]+'='+info['state'] for name, info in state['components'].items()
                                if name in names and info['state'] != 'READY')
                now = time.monotonic()
                if waiting and (waiting != self.last_waiting or now-self.last_progress >= 10):
                    print('CAPTURE WAITING（尚未录制）：'+', '.join(waiting), flush=True)
                    self.last_waiting, self.last_progress = waiting, now
            except (OSError, ValueError, KeyError):
                pass
        return checks


def quality_report(audit, expected):
    failures, warnings = [], []
    for topic, kind in expected.items():
        info = audit['topics'].get(topic, {})
        if info.get('type') != kind or info.get('count', 0) == 0:
            failures.append('missing_or_wrong_type:'+topic)
        if info.get('nonincreasing', 0):
            failures.append('nonincreasing_header:'+topic)
        periods = info.get('period_ns_p0_p50_p100', [])
        if periods and periods[-1] > 500_000_000:
            warnings.append('gap_over_0.5s:'+topic)
    points = audit.get('point_times', {})
    if not points.get('total', 0) or any(points.get(k, 0) for k in ('before_header', 'nonfinite', 'backwards_steps_inside_scans')):
        failures.append('invalid_lidar_point_times')
    if audit.get('imu', {}).get('nonfinite_messages', 0):
        failures.append('nonfinite_imu')
    return {'data_checks': 'FAIL' if failures else ('PARTIAL' if warnings else 'PASS'),
            'failures': failures, 'warnings': warnings, 'physical_sync': 'PARTIAL',
            'motion_mapping_accuracy': 'NOT_EVALUATED',
            'note': 'Header equality is not physical synchronization; no APE/RPE or loop accuracy is inferred.'}


def archive_logs(folder, own_run, source_run, sensor_dir, owns_sensors):
    dest = folder/'session_logs'
    dest.mkdir()
    for label, source in [('record', own_run), ('sensors', source_run), ('sync', sensor_dir)]:
        if source and Path(source).exists():
            # Never copy the mmap file while a writer may still be alive.
            target = dest/label
            target.mkdir()
            for path in Path(source).iterdir():
                if path.is_file() and path.name != 'timeshare':
                    shutil.copy2(path, target/path.name)
    atomic_json(dest/'provenance.json', {'owns_sensor_session': owns_sensors,
        'source_run': str(source_run) if source_run else None,
        'attached_live_logs_may_be_partial': not owns_sensors})


def sync_summary(sensor_dir):
    """Camera CSV contains all triggered frames, including rejected sync copies."""
    path = sensor_dir/'camera_sync.csv'
    if not path.is_file():return {'available': False}
    statuses = Counter()
    previous = None
    missing = backwards = 0
    with path.open() as handle:
        for row in csv.DictReader(handle):
            statuses[row['status']] += 1
            current = int(row['frame_id'])
            if previous is not None:
                if current <= previous:backwards += 1
                else:missing += max(0, current-previous-1)
            previous = current
    return {'available': True, 'source': str(path), 'status_counts': dict(statuses),
            'hardware_frame_id_missing': missing, 'hardware_frame_id_nonincreasing': backwards,
            'scope': 'entire sensor session; may include time outside this recording',
            'physical_sync_proven': False}


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    action = p.add_mutually_exclusive_group()
    action.add_argument('--execute', action='store_true')
    action.add_argument('--status', action='store_true')
    action.add_argument('--stop', action='store_true')
    mode = p.add_mutually_exclusive_group()
    mode.add_argument('--attach', action='store_true', help='Record an existing managed preview; leave it running')
    mode.add_argument('--start-sensors', action='store_true', help='Own sensors/front-end and stop them after recording')
    p.add_argument('--seconds', type=float, default=60)
    p.add_argument('--label', default='motion')
    p.add_argument('--output', type=Path, help='NEW output directory; parent must already exist')
    p.add_argument('--runtime-config')
    p.add_argument('--visualization', choices=('auto', 'rviz', 'foxglove', 'both', 'none'), default='auto',
                   help='auto: RViz with DISPLAY, otherwise localhost Foxglove')
    p.add_argument('--foxglove-port', type=int, default=8765)
    p.add_argument('--min-free-gb', type=float, default=5)
    p.add_argument('--record-wheel', action='store_true', help='Require an existing wheel publisher; never starts a driver')
    return p


def main():
    p = parser()
    args = p.parse_args()
    requested_visualization = args.visualization
    if args.visualization == 'auto':
        args.visualization = 'rviz' if os.environ.get('DISPLAY', '').strip() else 'foxglove'
    if args.status:
        run, state = read_session(SESSION)
        capture = {}
        if run and (run/'capture.json').exists():
            capture = json.loads((run/'capture.json').read_text())
            folder = Path(capture['folder'])
            for filename in ('recording.json', 'result.json'):
                if (folder/filename).exists():capture[filename] = json.loads((folder/filename).read_text())
        print(json.dumps({'live': bool(session_live(state)), 'session': state, 'capture': capture}, indent=2))
        return 0
    if args.stop:
        return 0 if stop_session(SESSION, timeout=65) else 2
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 3600:p.error('seconds must be 1..3600')
    if not math.isfinite(args.min_free_gb) or args.min_free_gb < 1:p.error('min-free-gb must be >= 1')
    if not 1 <= args.foxglove_port <= 65535:p.error('foxglove-port must be 1..65535')
    if not args.label or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in args.label):
        p.error('label must use letters, numbers, _ or -')
    if args.runtime_config:os.environ['RANGER_RUNTIME_CONFIG'] = args.runtime_config
    cfg = resolve(ROOT)
    validate_fast_livo_config(cfg['FAST_LIVO_CONFIG'])
    preview_run, preview = read_session(SESSION_ROOT/'fast_livo_live')
    live = bool(session_live(preview))
    attach = args.attach or (live and not args.start_sensors)
    topics = dict(TOPICS)
    if args.record_wheel:topics['/wheel/odom'] = 'nav_msgs/msg/Odometry'
    folder = args.output.expanduser().absolute() if args.output else ROOT/'bags'/('fast-livo-'+args.label+'-NEW')
    sensor_environment(cfg, folder/'sensor_assets')
    components = [] if attach else specs(cfg, args.visualization, args.foxglove_port)
    worker = [sys.executable, str(ROOT/'scripts/fast_livo2/record_worker.py'), '--folder', str(folder)]
    if not args.execute:
        print(json.dumps({'action': 'DRY-RUN', 'mode': 'attach' if attach else 'start-sensors',
            'domain': int(cfg['FAST_LIVO_ROS_DOMAIN_ID']), 'localhost_only': cfg['FAST_LIVO_LOCALHOST_ONLY'],
            'seconds': args.seconds, 'output': str(folder), 'topics': topics,
            'recorder': recorder_command(folder, topics), 'components': [asdict(c) for c in components],
            'visualization': 'existing session' if attach else args.visualization,
            'requested_visualization': requested_visualization,
            'no_base_or_navigation': True, 'physical_sync_proven': False}, indent=2))
        return 0
    if session_live(read_session(SESSION)[1]):raise RuntimeError('recorder already active; use --status/--stop')
    if args.start_sensors and live:raise RuntimeError('preview already active; use --attach or stop it explicitly')
    if attach and not live:raise RuntimeError('--attach requires a live managed FAST preview')
    if cfg['FAST_LIVO_LOCALHOST_ONLY'] != '1':raise RuntimeError('capture requires localhost_only=1')
    if not folder.parent.is_dir():raise RuntimeError('output parent must exist')
    if args.output and folder.exists():raise FileExistsError('refusing to overwrite '+str(folder))
    if shutil.disk_usage(folder.parent).free < args.min_free_gb*1e9:raise RuntimeError('insufficient free disk')
    if not attach:
        check_sensor_processes()
        for component in components:
            executable = component.argv[0]
            if executable.startswith('/') and not os.access(executable, os.X_OK):
                raise RuntimeError('missing executable: '+executable)
        if args.visualization in ('rviz', 'both') and not os.environ.get('DISPLAY', '').strip():
            p.error('RViz requires DISPLAY; use --visualization auto, foxglove or none')
        if any(component.name == 'lidar' for component in components):
            check_lidar_host_address(cfg)
    if args.output:folder.mkdir()
    else:folder = Path(tempfile.mkdtemp(prefix='fast-livo-'+args.label+'-', dir=ROOT/'bags'))
    print('CAPTURE_DIR='+str(folder), flush=True)
    print('VISUALIZATION='+('existing session' if attach else args.visualization), flush=True)
    print('CAPTURE PREPARING（尚未录制）：等待雷达、相机和前端数据就绪。', flush=True)
    (folder/'sensor_assets').mkdir()
    sensor_environment(cfg, folder/'sensor_assets')
    manifest = {'schema_version': 1, 'created_unix_ns': time.time_ns(), 'folder': str(folder),
        'owns_sensors': not attach, 'mode': 'attach' if attach else 'start-sensors',
        'seconds': args.seconds, 'min_free_bytes': int(args.min_free_gb*1e9), 'topics': topics,
        'domain': int(cfg['FAST_LIVO_ROS_DOMAIN_ID']), 'physical_sync_proven': False,
        'visualization': args.visualization if not attach else 'existing',
        'requested_visualization': requested_visualization,
        'no_base_or_navigation': True, 'source_files': snapshot_sources(folder, cfg)}
    source_run = preview_run if attach else None
    sensor_dir = attached_assets(preview_run) if attach else folder/'sensor_assets'
    if attach:
        # Capture exact running argv/configuration, not only today's declared profile.
        manifest['attached_session'] = preview
        for component in ('lidar', 'camera', 'frontend'):
            argv = preview['components'][component]['argv']
            config = next((argv[i+1] for i, a in enumerate(argv[:-1]) if a == '--params-file'), None)
            if component == 'lidar':config = next(a.split(':=',1)[1] for a in argv if a.startswith('user_config_path:='))
            expected = cfg[{'lidar':'FAST_LIVO_LIDAR_CONFIG','camera':'FAST_LIVO_CAMERA_CONFIG','frontend':'FAST_LIVO_CONFIG'}[component]]
            if config != expected:raise RuntimeError('attached config differs from selected profile: '+component)
        state_env = json.loads((sensor_dir/'manifest.json').read_text())
        if state_env['domain'] != manifest['domain']:raise RuntimeError('attached preview uses another ROS domain')
    atomic_json(folder/'manifest.json', manifest)
    components = [] if attach else specs(cfg, args.visualization, args.foxglove_port)
    worker = [sys.executable, str(ROOT/'scripts/fast_livo2/record_worker.py'), '--folder', str(folder)]
    components.append(Component('capture', worker, depends=() if attach else ('frontend',),
                                resource='fast_livo_recorder'))
    code = run_guarded(SESSION, components, health_factory=lambda path: CaptureHealth(path, folder),
                       startup_timeout=90, fail_on_health_loss=True, health_loss_grace=2, cleanup_grace=40)
    own_run, state = read_session(SESSION)
    if source_run is None:source_run = own_run
    result = json.loads((folder/'recorder_result.json').read_text()) if (folder/'recorder_result.json').exists() else {}
    if result.get('error'):
        result['recorder_error'] = result['error']
    result.update(session_exit=code, session_state=state, folder=str(folder), capture_complete=False)
    try:
        archive_logs(folder, own_run, source_run, sensor_dir, not attach)
        result['sync'] = sync_summary(sensor_dir)
        if code or result.get('exit') != 0 or result.get('forced') or not (folder/'raw/metadata.yaml').exists():
            raise RuntimeError('session or recorder failed; preserved all partial data')
        from audit_bag import audit
        report = audit(folder/'raw')
        atomic_json(folder/'audit.json', report)
        result['quality'] = quality_report(report, topics)
        sync = result['sync']
        if not sync['available']:
            result['quality']['warnings'].append('sync_csv_unavailable')
        elif sync['hardware_frame_id_missing'] or sync['hardware_frame_id_nonincreasing'] or any(
                count for status, count in sync['status_counts'].items() if status != 'OK'):
            result['quality']['warnings'].append('camera_sync_anomalies_in_sensor_session')
        if result['quality']['warnings'] and not result['quality']['failures']:
            result['quality']['data_checks'] = 'PARTIAL'
        if result['quality']['failures']:raise RuntimeError('recorded input validation failed')
        result['capture_complete'] = True
    except Exception as exc:
        result['error'] = str(exc)
        code = 2
    atomic_json(folder/'result.json', result)
    if not result['capture_complete']:
        reason = state.get('reason') if state else None
        print('CAPTURE ERROR：录制未完成；'+(reason or result.get('error', '未知错误'))
              +('；输入诊断：'+result['recorder_error'] if result.get('recorder_error') else '')
              +'；诊断日志: '+str(folder/'session_logs'), file=sys.stderr, flush=True)
    if result['capture_complete']:
        (folder/'CAPTURE_COMPLETE').write_text('Recording finalized and data checks passed; physical sync/SLAM accuracy remain unverified.\n')
    print(json.dumps(result, indent=2), flush=True)
    return code


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError, ValueError) as exc:
        print('CAPTURE ERROR：'+str(exc), file=sys.stderr, flush=True)
        raise SystemExit(2)
