#!/usr/bin/env python3
"""Five independent user-driven P0 closure trials, with guarded capture cleanup."""
import argparse
import csv
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import select
import shutil
import signal
import subprocess
import sys
import tempfile
import time

import yaml
import analysis

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from session_manager import identity, notify, same_process, read_session, session_live


def validate(cfg):
    if cfg['formal_trials'] != 5 or len(cfg['route_labels']) != 5:
        raise ValueError('本实验固定五次正式实验')
    if cfg['registration'] not in ('pcl_icp', 'small_gicp', 'small_vgicp'):
        raise ValueError('invalid registration')
    if not 1 <= cfg['domain'] <= 232:
        raise ValueError('invalid isolated ROS domain')
    for key in ('warmup_seconds', 'start_window_seconds', 'end_hold_seconds',
                'maximum_trial_seconds', 'startup_timeout_seconds', 'min_free_gb'):
        if not math.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError('invalid '+key)
    if cfg['maximum_trial_seconds'] > 3600:
        raise ValueError('maximum duration exceeds managed mapping limit')
    for bounds in cfg['end_windows_seconds'].values():
        if not 0 < bounds[0] < bounds[1] <= cfg['end_hold_seconds']:
            raise ValueError('invalid fixed endpoint window')


def command(cfg, trial):
    return [sys.executable, str(HERE/'mapping_entry.py'), '--execute',
            '--output', str(trial/'capture'), '--seconds', str(cfg['maximum_trial_seconds']),
            '--registration', cfg['registration'], '--domain', str(cfg['domain']),
            '--visualization', 'none', '--min-free-gb', str(cfg['min_free_gb'])]


def source_files():
    files = [p for p in HERE.iterdir() if p.suffix in ('.py', '.sh', '.json')]
    files += [ROOT/p for p in (
        'config/runtime.yaml', 'config/profiles/fast_livo2_frontend.yaml',
        'config/fast_livo2/ranger_mid360s_hik_trial.yaml', 'config/fast_livo2/backend.yaml',
        'config/fast_livo2/navigation_prep.yaml', 'config/fast_livo2/camera_live.yaml',
        'config/fast_livo2/mid360s_live.json', 'config/fast_livo2/mapping_fastdds.xml',
        'config/fast_livo2/mapping_control_fastdds.xml',
        'scripts/fast_livo2/mapping_session.py', 'scripts/fast_livo2/mapping_monitor.py',
        'scripts/fast_livo2/record_session.py', 'scripts/fast_livo2/record_worker.py',
        'scripts/fast_livo2/online_session.py', 'scripts/session_manager.py',
        'ws_fast_livo/online_frontend_source/online_source_manifest.json',
        'ws_fast_livo/online_frontend_install/fast_livo/lib/fast_livo/fastlivo_mapping',
        'ws_fast_livo/online_install/ranger_fast_livo_backend/lib/ranger_fast_livo_backend/backend_node',
        'ws_fast_livo/online_install/ranger_fast_livo_backend/lib/libranger_backend_core.so',
        'ws_camera/latency_install/hik_camera_trigger_driver/lib/hik_camera_trigger_driver/hik_camera_trigger_driver_node',
        'ws_camera/latency_install/hik_camera_trigger_driver/lib/libhik_camera_trigger_driver.so')]
    return sorted(set(files))


def hashes():
    return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in source_files()}


def verify_frozen(run):
    current, frozen = hashes(), analysis.load(run/'source_hashes.json')
    if current == frozen:
        return
    # Evaluator and session selection updates can reuse existing captures.
    # Per-trial acquisition functions, configuration and binaries stay frozen.
    # Keep the original hashes and record the orchestration/reporting revision.
    import ast
    prefix = str(HERE.relative_to(ROOT))+'/'
    changed = {k for k in set(current) | set(frozen) if current.get(k) != frozen.get(k)}
    old_runner = run/'experiment_sources/suite.py'
    allowed = {prefix+'analysis.py', prefix+'suite.py'}
    def acquisition_ast(source):
        tree = ast.parse(source)
        orchestration = {'verify_frozen', 'select_run', 'main'}
        tree.body = [n for n in tree.body if not isinstance(n, ast.FunctionDef) or n.name not in orchestration]
        return ast.dump(tree, include_attributes=False)
    runner_unchanged = (old_runner.exists() and
        hashlib.sha256(old_runner.read_bytes()).hexdigest() == frozen.get(prefix+'suite.py') and
        acquisition_ast(old_runner.read_text()) == acquisition_ast((HERE/'suite.py').read_text()))
    if changed <= allowed and runner_unchanged:
        revisions = run/'analysis_revisions'
        revisions.mkdir(exist_ok=True)
        revision = {'scope': 'evaluator_and_session_selection_only', 'acquisition_unchanged': True,
                    'changes': {k: {'original_sha256': frozen.get(k), 'current_sha256': current.get(k)} for k in sorted(changed)}}
        key = hashlib.sha256(json.dumps(revision, sort_keys=True).encode()).hexdigest()[:16]
        path = revisions/(key+'.json')
        if not path.exists():
            analysis.save(path, revision)
        return
    raise RuntimeError('采集源码/参数/二进制已变化；本组冻结配置不再匹配。保留原结果，请创建新实验组。')


def select_run(runs, resume=None, start_new=False):
    """Prefer the most recent group with progress over accidentally empty groups."""
    runs = Path(runs).resolve()
    if resume is not None:
        selected = Path(resume).resolve()
        if selected.parent != runs:
            raise ValueError('resume directory must be inside this experiment runs directory')
        if not (selected/'progress.json').is_file():
            raise ValueError('resume directory has no experiment progress')
        return selected
    if start_new or not runs.is_dir():
        return None
    candidates = []
    for directory in sorted(runs.iterdir(), reverse=True):
        if not directory.is_dir():
            continue
        progress = analysis.load(directory/'progress.json', {})
        if progress.get('planned_trials') == 5 and isinstance(progress.get('trials'), dict):
            candidates.append((directory, progress))
    for directory, _ in candidates:
        if same_process(analysis.load(directory/'suite_process.json')):
            return directory
    populated = [directory for directory, progress in candidates if progress['trials']]
    return populated[0] if populated else candidates[0][0] if candidates else None


def assert_alive(proc):
    code = proc.poll()
    if code is not None:
        raise RuntimeError(f'在线采集已退出（{code}），查看本轮 mapping_console.log')


def wait_seconds(seconds, proc, message):
    print(message, flush=True)
    end, next_notice = time.monotonic()+seconds, time.monotonic()+10
    while time.monotonic() < end:
        assert_alive(proc)
        if time.monotonic() >= next_notice:
            print(f'  还需约 {max(0, math.ceil(end-time.monotonic()))} 秒。', flush=True)
            next_notice += 10
        time.sleep(.1)


def prompt(text, proc=None):
    print(text, flush=True)
    while True:
        if proc:
            assert_alive(proc)
        if select.select([sys.stdin], [], [], .2)[0]:
            line = sys.stdin.readline()
            if not line:
                raise KeyboardInterrupt('terminal closed')
            return line.strip()


def wait_ready(trial, proc, cfg):
    deadline, notice = time.monotonic()+cfg['startup_timeout_seconds'], 0.
    while time.monotonic() < deadline:
        assert_alive(proc)
        health = analysis.load(trial/'capture/mapping_health.json', {})
        observer = analysis.load(trial/'capture/closure_observer.json', {})
        checks, counts = health.get('checks', {}), observer.get('counts', {})
        clock_error = health.get('clock', {}).get('error')
        if clock_error:
            raise RuntimeError('本轮时钟失败：'+clock_error+'；必须新会话，不能继续等待或中途重对时')
        if health.get('backend_metrics', {}).get('invalid_messages', 0):
            raise RuntimeError('后端已拒绝非法输入：'+str(health.get('backend', {}).get('reason')))
        if (all(checks.get(k) for k in ('frontend', 'backend_ready', 'capture_recording', 'clock_ok'))
                and observer.get('anchor') and counts.get('frontend', 0) >= 5 and counts.get('online', 0) >= 5
                and time.monotonic_ns()-observer.get('monotonic_ns', 0) < 1e9):
            return
        if time.monotonic() >= notice:
            backend = health.get('backend') or {}
            print('等待就绪：时钟='+str(checks.get('clock_ready', False))
                  +'，相机='+str(checks.get('camera', False))
                  +'，前端配对='+str(health.get('paired_frames', 0))
                  +'，后端='+str(backend.get('reason', '等待输入'))
                  +'，录包='+str(checks.get('capture_recording', False)), flush=True)
            notice = time.monotonic()+10
        time.sleep(.2)
    raise RuntimeError('启动超时；原始日志保留。本轮不计为成功实验。')


def stop_owned(proc, record):
    if proc.poll() is not None:
        return proc.returncode
    if not same_process(record):
        raise RuntimeError('采集进程身份不匹配，未发停止信号')
    notify(record, signal.SIGTERM)
    print('正在封存本轮地图、停止录包和本轮拥有的传感器；请等待。', flush=True)
    deadline, notice = time.monotonic()+400, time.monotonic()+10
    while proc.poll() is None and time.monotonic() < deadline:
        if time.monotonic() >= notice:
            print('仍在排空/导出/核验；详情写入 mapping_console.log。', flush=True)
            notice = time.monotonic()+10
        time.sleep(.2)
    if proc.poll() is None:
        raise RuntimeError('本轮受管进程仍未退出；保留其身份记录，不启动下一轮。')
    return proc.returncode


def summarize(run, progress):
    lines = ['# 五次 P0 闭环实验', '',
             '短/中/长是人工选择的路线标签；各次行程不同，逐次报告，不混算固定路线重复性。', '',
             '| 次数 | 路线 | 状态 | 前端估算路程 m | 结果 |', '|---|---|---|---:|---|']
    csv_rows = []
    for index in range(1, 6):
        entry = progress['trials'].get(str(index), {})
        directory = Path(entry['directory']) if entry.get('directory') else None
        latest = analysis.load(directory/'latest_analysis.json', {}) if directory else {}
        metrics = analysis.load(Path(latest['directory'])/'metrics.json', {}) if latest else {}
        distance = metrics.get('estimated_frontend_distance_m')
        distance_text = f'{distance:.3f}' if distance is not None else '—'
        report = (str((Path(latest['directory'])/'REPORT.md').relative_to(run)) if latest else '')
        link = f'[报告]({report})' if report else '—'
        lines.append(f'| {index} | {entry.get("route_label", "—")} | {entry.get("status", "PENDING")} | {distance_text} | {link} |')
        for name, values in metrics.get('trajectories', {}).items():
            for label, value in values['metrics'].items():
                csv_rows.append([index, entry.get('route_label'), entry.get('status'), name, label, distance,
                                 value.get('position_error_m'), value.get('yaw_error_deg'),
                                 value.get('relative_drift_rate_percent'), value.get('estimated_relative_drift_rate_percent')])
    (run/'SUMMARY.md').write_text('\n'.join(lines)+'\n')
    with (run/'summary_metrics.csv').open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['trial', 'route', 'status', 'trajectory', 'window', 'estimated_distance_m',
                         'position_error_m', 'yaw_error_deg', 'measured_distance_drift_percent', 'estimated_distance_drift_percent'])
        writer.writerows(csv_rows)
    analysis.save(run/'progress.json', progress)


def run_trial(run, index, cfg, progress):
    default = cfg['route_labels'][index-1]
    route = prompt(f'\n第 {index}/5 次：车仍在 P0，中心和车头已对齐时，回车启动。路线标签 [{default}]（可输入新标签）：') or default
    verify_frozen(run)
    if shutil.disk_usage(run).free < cfg['min_free_gb']*1e9:
        raise RuntimeError('可用磁盘空间不足，未启动设备')
    previous = progress['trials'].get(str(index), {})
    attempts = list(previous.get('attempts', []))
    trial = run/f'trial_{index:02d}_attempt_{len(attempts)+1:02d}'
    trial.mkdir()
    attempts.append(str(trial))
    entry = {'directory': str(trial), 'route_label': route, 'status': 'STARTING', 'attempts': attempts}
    progress['trials'][str(index)] = entry
    summarize(run, progress)
    analysis.save(trial/'config.json', cfg)
    shutil.copyfile(run/'calibration.json', trial/'calibration.json')
    analysis.save(trial/'trial.json', {'index': index, 'route_label': route,
        'p0_start_operator_confirmed': True, 'p0_return_operator_confirmed': False})
    events = {'boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip()}
    cmd = command(cfg, trial)
    analysis.save(trial/'command.json', cmd)
    proc, record = None, None
    try:
        with (trial/'mapping_console.log').open('xb') as log:
            proc = subprocess.Popen(cmd, cwd=ROOT, stdin=subprocess.DEVNULL,
                                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            record = identity(proc.pid)
            analysis.save(trial/'mapping_process.json', record)
            wait_ready(trial, proc, cfg)
            wait_seconds(cfg['warmup_seconds'], proc, '已就绪。请在 P0 保持静止，进行 15 秒预热。')
            prompt('请仍在 P0 保持静止。按回车后再静止 3 秒，听到出发提示再驾驶：', proc)
            events['start_window'] = [time.monotonic_ns(), None]
            wait_seconds(cfg['start_window_seconds'], proc, '正在记录起点静止窗口……')
            events['start_window'][1] = time.monotonic_ns()
            events['route_start_ns'] = events['start_window'][1]
            analysis.save(trial/'events.json', events)
            entry['status'] = 'DRIVING'
            summarize(run, progress)
            print('\a可以出发。请人工驾驶，完成本轮后回到 P0。', flush=True)
            prompt('底盘中心和车头均已按物理标记复位、车已停稳后，按回车：', proc)
            events['returned_ns'] = time.monotonic_ns()
            analysis.save(trial/'events.json', events)
            meta = analysis.load(trial/'trial.json')
            meta['p0_return_operator_confirmed'] = True
            analysis.save(trial/'trial.json', meta)
            entry['status'] = 'END_HOLD'
            summarize(run, progress)
            wait_seconds(cfg['end_hold_seconds'], proc, '已标记返回 P0。保持不动 15 秒，自动采集即时与等待后结果。')
            events['end_hold_complete_ns'] = time.monotonic_ns()
            analysis.save(trial/'events.json', events)
            entry['status'] = 'FINALIZING'
            summarize(run, progress)
            code = stop_owned(proc, record)
        analysis.save(trial/'capture_exit.json', {'exit_code': code})
        evaluation, result_dir = analysis.analyze(trial)
        entry['status'] = 'COMPLETE' if evaluation['status'] == 'PASS_DATA' and code == 0 else 'PARTIAL'
        print('\n本轮结果：'+str(result_dir/'REPORT.md'), flush=True)
        for name, data in evaluation['trajectories'].items():
            m = data['metrics']['settled']
            if m['available']:
                print(f'  {name}: {m["position_error_m"]*100:.2f} cm / {m["yaw_error_deg"]:.2f}°', flush=True)
            else:
                print('  '+name+': 有效采样不足，未补值。', flush=True)
        if code:
            raise RuntimeError('本轮采集/封存检查失败；保留结果并停止实验组。')
    except BaseException as exc:
        entry['status'] = 'FAILED'
        entry['error'] = str(exc) or type(exc).__name__
        analysis.save(trial/'failure.json', {'error': entry['error'], 'time_ns': time.time_ns()})
        raise
    finally:
        try:
            if proc and proc.poll() is None:
                stop_owned(proc, record)
        finally:
            summarize(run, progress)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    action = p.add_mutually_exclusive_group()
    action.add_argument('--dry-run', action='store_true')
    action.add_argument('--status', action='store_true')
    action.add_argument('--stop', action='store_true')
    action.add_argument('--analyze', type=Path, metavar='TRIAL_DIR')
    p.add_argument('--length-m', type=float, help='independent measured length, only with --analyze')
    selection = p.add_mutually_exclusive_group()
    selection.add_argument('--resume', type=Path, metavar='RUN_DIR')
    selection.add_argument('--new', action='store_true', help='explicitly create a fresh five-trial group')
    args = p.parse_args()
    if args.length_m is not None and not args.analyze:
        p.error('--length-m applies to one --analyze trial only')
    if args.analyze:
        result, out = analysis.analyze(args.analyze.resolve(), args.length_m)
        print(out)
        return 0 if result['status'] == 'PASS_DATA' else 2
    runs = HERE/'runs'
    selected = select_run(runs, args.resume, args.new)
    if args.status or args.stop:
        run = selected
        record = analysis.load(run/'suite_process.json') if run else None
        if args.stop:
            notify(record, signal.SIGTERM)
            print('已向当前实验组的匹配进程请求受管停止。' if same_process(record) else '没有存活的本实验进程。')
        else:
            print(json.dumps({'live': same_process(record), 'directory': str(run) if run else None,
                              'progress': analysis.load(run/'progress.json') if run else None}, ensure_ascii=False, indent=2))
        return 0
    cfg = analysis.load(selected/'config.json' if selected else HERE/'config.json')
    validate(cfg)
    if args.dry_run:
        progress = analysis.load(selected/'progress.json') if selected else {'trials': {}}
        next_trial = next((i for i in range(1, 6) if progress['trials'].get(str(i), {}).get('status')
                           not in ('COMPLETE', 'PARTIAL')), None)
        attempt = len(progress['trials'].get(str(next_trial), {}).get('attempts', []))+1
        example_trial = (selected or runs/'NEW')/f'trial_{next_trial:02d}_attempt_{attempt:02d}' if next_trial else None
        print(json.dumps({'action': 'DRY-RUN', 'hardware_started': False, 'configuration': cfg,
            'selection': 'resume' if selected else 'new', 'selected_run': str(selected) if selected else None,
            'next_trial': next_trial,
            'would_start': ['LiDAR', 'camera trigger/writer', 'latency camera', 'FAST-LIVO2', 'B backend', 'recorder', 'read-only observer'],
            'base_driver_started': False, 'velocity_commands': False, 'public_tf': False,
            'command_example': command(cfg, example_trial) if example_trial else None,
            'source_hashes': hashes()}, ensure_ascii=False, indent=2))
        return 0
    if not sys.stdin.isatty():
        raise RuntimeError('请在交互终端运行；需要人工标记 P0 出发与返回')
    runs.mkdir(exist_ok=True)
    lock = (runs/'.suite.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError('已有本实验组在运行；请使用 --status')
    if session_live(read_session(ROOT/'.runtime/sessions/fast_livo_mapping')[1]):
        raise RuntimeError('已有在线建图会话，保留其运行状态；请先协调会话归属')
    if selected:
        run = selected
        cfg, progress = analysis.load(run/'config.json'), analysis.load(run/'progress.json')
        validate(cfg)
        verify_frozen(run)
    else:
        run = Path(tempfile.mkdtemp(prefix=time.strftime('%Y%m%d_%H%M%S')+'-', dir=runs))
        progress = {'schema_version': 1, 'planned_trials': 5, 'trials': {}, 'state': 'PREPARED'}
        analysis.save(run/'config.json', cfg)
        calibration = yaml.safe_load((ROOT/'config/fast_livo2/navigation_prep.yaml').read_text())
        analysis.save(run/'calibration.json', calibration)
        analysis.save(run/'source_hashes.json', hashes())
        sources = run/'experiment_sources'
        sources.mkdir()
        for path in HERE.iterdir():
            if path.is_file() and path.suffix in ('.py', '.sh', '.json', '.md'):
                shutil.copy2(path, sources/path.name)
        (run/'git_head.txt').write_bytes(subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT))
        (run/'git_status.txt').write_bytes(subprocess.check_output(['git', 'status', '--short'], cwd=ROOT))
    analysis.save(runs/'latest.json', {'directory': str(run)})
    analysis.save(run/'suite_process.json', identity(os.getpid()))
    def interrupt(*_):
        raise KeyboardInterrupt('requested stop')
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupt)
    print('实验组：'+str(run)+'\n共五次；仅 P0。人工驾驶，脚本不启动底盘驱动、不发送速度。', flush=True)
    if selected:
        completed = sum(entry.get('status') in ('COMPLETE', 'PARTIAL') for entry in progress['trials'].values())
        if completed == 5:
            print('本组五次已全部完成。查看 SUMMARY.md；如需新一组，请运行原命令加 --new。', flush=True)
            progress['state'] = 'FINISHED'
            summarize(run, progress)
            lock.close()
            return 0
        print(f'已自动读取进度：完成 {completed}/5 次，跳过已完成轮次。', flush=True)
    progress['state'] = 'RUNNING'
    try:
        for index in range(1, 6):
            if progress['trials'].get(str(index), {}).get('status') not in ('COMPLETE', 'PARTIAL'):
                run_trial(run, index, cfg, progress)
        progress['state'] = 'FINISHED'
    except BaseException:
        progress['state'] = 'STOPPED'
        raise
    finally:
        summarize(run, progress)
        lock.close()
        print('汇总：'+str(run/'SUMMARY.md'), flush=True)
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('\n实验已中断，已完成和失败数据均保留。用 --resume RUN_DIR 继续未完成次数。', flush=True)
        raise SystemExit(130)
    except (RuntimeError, OSError, ValueError) as exc:
        print('EXPERIMENT ERROR: '+str(exc), file=sys.stderr, flush=True)
        raise SystemExit(2)
