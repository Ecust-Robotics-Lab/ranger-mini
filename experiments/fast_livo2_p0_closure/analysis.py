#!/usr/bin/env python3
"""P0 closure evaluation; no trajectory fitting, scale fitting or device access."""
import argparse
import bisect
import csv
import hashlib
import json
import math
import sqlite3
import time
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent


def load(path, default=None):
    try:
        return json.loads(Path(path).read_text())
    except FileNotFoundError:
        return default


def save(path, data):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def jsonl(path):
    if not Path(path).exists():
        return []
    with Path(path).open() as handle:
        return [json.loads(line) for line in handle if line.strip()]


def matrix(pose):
    xyz, q = np.asarray(pose['xyz'], dtype=float), np.asarray(pose['xyzw'], dtype=float)
    if xyz.shape != (3,) or q.shape != (4,) or not np.isfinite(np.r_[xyz, q]).all():
        raise ValueError('invalid/nonfinite pose')
    norm = np.linalg.norm(q)
    if abs(norm - 1) > .01:
        raise ValueError('invalid quaternion norm')
    x, y, z, w = q / norm
    out = np.eye(4)
    out[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                   [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                   [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    out[:3, 3] = xyz
    return out


def base_extrinsic(calibration):
    c = calibration['installation']
    r, p, y = c['t_base_imu_rpy']
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    t = np.eye(4)
    t[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                   [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    t[:3, 3] = c['t_base_imu_xyz']
    return t


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw(t):
    return math.atan2(t[1, 0], t[0, 0])


def raw_frontend(capture, observer, anchor):
    """Decode only Odometry from the raw SQLite bag; never deserialize images/clouds."""
    bags = sorted((capture / 'raw').glob('*.db3'))
    if not bags:
        return [r for r in observer if r['kind'] == 'frontend'], 'observer_best_effort_fallback'
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    result = []
    for bag in bags:
        with sqlite3.connect(bag.resolve().as_uri() + '?mode=ro', uri=True) as conn:
            sql = ('SELECT m.data FROM messages m JOIN topics t ON m.topic_id=t.id '
                   'WHERE t.name=? ORDER BY m.timestamp,m.id')
            for (data,) in conn.execute(sql, ('/fast_livo/odometry',)):
                m = deserialize_message(data, Odometry)
                if m.header.frame_id != 'fast_livo_local' or m.child_frame_id != 'fast_livo_imu':
                    raise ValueError('frontend frame contract mismatch')
                stamp = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
                p = m.pose.pose
                result.append({'kind': 'frontend', 'stamp_ns': stamp,
                    'sample_steady_ns': stamp - int(anchor[0]) + int(anchor[1]),
                    'pose': {'xyz': [p.position.x, p.position.y, p.position.z],
                             'xyzw': [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]}})
    return result, 'raw_bag_odometry'


def unique_frontend(rows):
    found = {}
    for r in rows:
        if r.get('sample_steady_ns') is not None:
            found.setdefault(int(r['stamp_ns']), r)
    return sorted(found.values(), key=lambda r: r['stamp_ns'])


def online_rows(rows, cfg):
    # First publication of each sampling instant is retained. Later heartbeats
    # cannot replace an originally invalid/unavailable output.
    found = {}
    for row in rows:
        if row['kind'] != 'online':
            continue
        key = (row.get('session_id'), row['stamp_ns'], row.get('scan_id'))
        found.setdefault(key, row)
    result = []
    for r in found.values():
        age = r.get('sample_age_s')
        if (r.get('pose_valid') and r.get('state') == 1 and r.get('sample_steady_ns') is not None
                and age is not None and -.05 <= age <= cfg['maximum_sample_age_seconds']):
            result.append(r)
    return sorted(result, key=lambda r: r['sample_steady_ns'])


def optimized_rows(frontend, archive):
    frames = sorted(archive['keyframes'], key=lambda k: k['stamp_ns'])
    stamps = [k['stamp_ns'] for k in frames]
    corrections = [matrix(k['X_map_imu']) @ np.linalg.inv(matrix(k['U_local_imu'])) for k in frames]
    result = []
    for row in frontend:
        k = bisect.bisect_right(stamps, row['stamp_ns']) - 1
        if k < 0:
            continue
        result.append({**row, 'T_imu': corrections[k] @ matrix(row['pose']),
                       'reference_keyframe_id': frames[k]['id']})
    return result


def window(rows, bounds, inv_base_imu, cfg, online=False):
    begin, end = bounds
    chosen = [r for r in rows if begin <= r['sample_steady_ns'] <= end
              and (not online or r['received_steady_ns'] <= end)]
    chosen.sort(key=lambda r: r['sample_steady_ns'])
    span = (chosen[-1]['sample_steady_ns'] - chosen[0]['sample_steady_ns']) / 1e9 if len(chosen) > 1 else 0
    gaps = np.diff([r['sample_steady_ns'] for r in chosen]) / 1e9
    max_gap = float(max(gaps, default=0))
    coverage = span / ((end - begin) / 1e9)
    valid = (len(chosen) >= cfg['minimum_window_samples'] and coverage >= cfg['minimum_window_coverage']
             and max_gap <= cfg['maximum_window_gap_seconds'])
    output = {'samples': len(chosen), 'coverage': coverage, 'maximum_gap_s': max_gap,
              'valid': valid, 'bounds_steady_ns': list(bounds)}
    if not chosen:
        return output
    poses = [(r['T_imu'] if 'T_imu' in r else matrix(r['pose'])) @ inv_base_imu for r in chosen]
    positions = np.asarray([t[:3, 3] for t in poses])
    angles = np.asarray([yaw(t) for t in poses])
    unwrapped = np.unwrap(angles)
    deltas = np.linalg.norm(np.diff(positions[:, :2], axis=0), axis=1)
    yaw_deltas = np.abs(np.diff(unwrapped)) * 180 / math.pi
    jump_m, jump_deg = float(max(deltas, default=0)), float(max(yaw_deltas, default=0))
    output.update(position_xyz=np.median(positions, axis=0).tolist(),
                  yaw_rad=wrap(float(np.median(unwrapped))),
                  position_span_m=float(np.linalg.norm(np.ptp(positions[:, :2], axis=0))),
                  yaw_span_deg=float(np.ptp(unwrapped) * 180 / math.pi),
                  maximum_step_m=jump_m, maximum_step_deg=jump_deg,
                  jump_detected=jump_m > cfg['position_jump_m'] or jump_deg > cfg['yaw_jump_degrees'])
    return output


def path_length(rows, begin, end, inv_base_imu, spacing):
    points = [(matrix(r['pose']) @ inv_base_imu)[:2, 3] for r in rows
              if begin <= r['sample_steady_ns'] <= end]
    if len(points) < 2:
        return None
    previous, total = points[0], 0.
    for point in points[1:]:
        distance = float(np.linalg.norm(point - previous))
        if distance >= spacing:
            total += distance
            previous = point
    total += float(np.linalg.norm(points[-1] - previous))
    return total if total > .1 else None


def closure(start, end, estimated_length, measured_length):
    if not start['valid'] or not end['valid']:
        return {'available': False, 'reason': 'insufficient_fresh_window_samples'}
    d = np.asarray(end['position_xyz']) - np.asarray(start['position_xyz'])
    e = float(np.linalg.norm(d[:2]))
    angle = abs(wrap(end['yaw_rad'] - start['yaw_rad'])) * 180 / math.pi
    return {'available': True, 'position_error_m': e, 'yaw_error_deg': angle,
            'delta_xyz_m': d.tolist(),
            'relative_drift_rate_percent': e / measured_length * 100 if measured_length else None,
            'estimated_relative_drift_rate_percent': e / estimated_length * 100 if estimated_length else None,
            'window_jump_detected': start.get('jump_detected', False) or end.get('jump_detected', False)}


def export_csv(path, rows, inv_base_imu, origin_ns):
    with path.open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['stamp_ns', 'sample_steady_ns', 'received_steady_ns', 'relative_time_s',
                         'x_base_m', 'y_base_m', 'z_base_m', 'yaw_base_deg', 'graph_version'])
        for r in rows:
            t = (r['T_imu'] if 'T_imu' in r else matrix(r['pose'])) @ inv_base_imu
            writer.writerow([r['stamp_ns'], r['sample_steady_ns'], r.get('received_steady_ns', ''),
                (r['sample_steady_ns'] - origin_ns)/1e9, *t[:3, 3], math.degrees(yaw(t)), r.get('graph_version', '')])


def plot(result_dir, series, inv_base_imu, events):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    # Shared start transform, not independently fitted trajectory alignments.
    raw = series['frontend']
    first = next(r for r in raw if r['sample_steady_ns'] >= events['start_window'][0])
    common = np.linalg.inv(matrix(first['pose']) @ inv_base_imu)
    for label, rows in series.items():
        rows = [r for r in rows if events['start_window'][0] <= r['sample_steady_ns'] <= events['returned_ns']+15*10**9]
        if not rows:
            continue
        poses = [common @ (r['T_imu'] if 'T_imu' in r else matrix(r['pose'])) @ inv_base_imu for r in rows]
        p = np.array([t[:2, 3] for t in poses])
        sec = np.array([(r['sample_steady_ns']-events['route_start_ns'])/1e9 for r in rows])
        axes[0].plot(p[:, 0], p[:, 1], label=label, linewidth=1)
        axes[1].plot(sec, np.linalg.norm(p, axis=1), label=label, linewidth=1)
    axes[0].scatter([0], [0], marker='x', c='black', label='P0 reference')
    axes[0].set(xlabel='x (m)', ylabel='y (m)', title='Shared start frame; no trajectory fitting')
    axes[0].axis('equal')
    axes[1].axvline((events['returned_ns']-events['route_start_ns'])/1e9, color='black', linestyle='--')
    axes[1].set(xlabel='Time since departure (s)', ylabel='Estimated distance from P0 (m)', title='Return and online correction')
    for ax in axes:
        ax.grid(True, alpha=.25)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(result_dir / 'trajectories.png', dpi=160)
    plt.close(fig)


def analyze(trial, measured_length=None):
    trial = Path(trial)
    cfg, events = load(trial/'config.json'), load(trial/'events.json')
    if not events or 'returned_ns' not in events:
        raise ValueError('trial has no confirmed physical return to P0')
    measured_length = measured_length if measured_length is not None else cfg.get('independent_distance_m')
    if measured_length is not None and (not math.isfinite(measured_length) or measured_length <= 0):
        raise ValueError('measured distance must be positive and finite')
    capture = trial/'capture'
    observer = jsonl(capture/'closure_samples.jsonl')
    observer_status = load(capture/'closure_observer.json', {})
    anchor = observer_status.get('anchor')
    if not anchor:
        raise ValueError('missing session-specific sensor/steady clock anchor')
    frontend, source = raw_frontend(capture, observer, anchor)
    frontend = unique_frontend(frontend)
    inv_base = np.linalg.inv(base_extrinsic(load(trial/'calibration.json')))
    manifest_path = capture/'map/manifest.json'
    archive, archive_error = None, None
    if manifest_path.exists():
        committed = load(capture/'map/COMMITTED', {})
        if committed.get('manifest_sha256') != hashlib.sha256(manifest_path.read_bytes()).hexdigest():
            raise ValueError('final graph manifest checksum mismatch')
        archive = load(manifest_path)
    else:
        archive_error = 'final map archive unavailable'
    # Discovery establishes a new B session before the user marks the start.
    # Its earlier stamp=0/INITIALIZING heartbeats are not an experimental reset.
    # Preserve strict session checks throughout the actual measurement interval,
    # including invalid outputs; records without receipt provenance fail closed.
    measurement_begin = events['start_window'][0]
    measurement_end = events.get('end_hold_complete_ns', events['returned_ns']+int(cfg['end_hold_seconds']*1e9))
    relevant_online = [r for r in observer if r['kind'] == 'online' and
                       (r.get('received_steady_ns') is None or
                        measurement_begin <= r['received_steady_ns'] <= measurement_end)]
    sessions = {r.get('session_id') for r in relevant_online if r.get('session_id')}
    if len(sessions) != 1 or (archive and sessions != {archive['session_id']}):
        raise ValueError('online/final graph session mismatch or reset')
    online = online_rows(observer, cfg)
    optimized = optimized_rows(frontend, archive) if archive else []
    series = {'frontend': frontend, 'optimized_history': optimized, 'online_backend': online}
    estimated = path_length(frontend, events['route_start_ns'], events['returned_ns'], inv_base, cfg['path_spacing_m'])
    results = {}
    for name, rows in series.items():
        start = window(rows, events['start_window'], inv_base, cfg, name == 'online_backend')
        ends = {label: window(rows, [events['returned_ns']+int(a*1e9), events['returned_ns']+int(b*1e9)],
                             inv_base, cfg, name == 'online_backend')
                for label, (a, b) in cfg['end_windows_seconds'].items()}
        results[name] = {'start': start, 'end': ends,
                        'metrics': {label: closure(start, end, estimated, measured_length) for label, end in ends.items()}}
    raw_start = results['frontend']['start']
    stationary = (raw_start.get('valid', False) and raw_start.get('position_span_m', 1e9) <= cfg['start_motion_span_m']
                  and raw_start.get('yaw_span_deg', 1e9) <= cfg['start_motion_span_degrees'])
    mapping_result = load(capture/'mapping_result.json', {})
    route_frontend = {r['stamp_ns'] for r in frontend
                      if events['route_start_ns'] <= r['sample_steady_ns'] <= events['returned_ns']}
    route_online = [r for r in online if events['route_start_ns'] <= r['sample_steady_ns'] <= events['returned_ns']
                    and r['received_steady_ns'] <= events['returned_ns']]
    matched_stamps = {r['stamp_ns'] for r in route_online} & route_frontend
    valid_times = sorted([events['route_start_ns'], *[r['sample_steady_ns'] for r in route_online], events['returned_ns']])
    loop_events = archive.get('loop_event_id', 0) if archive else 0
    all_available = all(m['available'] for value in results.values() for m in value['metrics'].values())
    status = ('PASS_DATA' if mapping_result.get('success') and all_available and stationary
              and source == 'raw_bag_odometry' else 'PARTIAL')
    evaluation = {'schema_version': 1, 'status': status, 'accuracy_acceptance': 'NOT_PREDEFINED',
        'analysis_source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'measurement_session_ids': sorted(sessions),
        'ignored_prestart_session_ids': sorted({r['session_id'] for r in observer if r['kind'] == 'online'
            and r.get('session_id') and r.get('received_steady_ns', measurement_begin) < measurement_begin}-sessions),
        'scope': 'frontend and B online mapping; not C1/C2 relocalization',
        'route_label': load(trial/'trial.json', {}).get('route_label'), 'frontend_source': source,
        'physical_P0_return': 'OPERATOR_CONFIRMED_NOT_INDEPENDENTLY_TRACKED',
        'start_stationary_check': stationary, 'independent_distance_m': measured_length,
        'estimated_frontend_distance_m': estimated, 'distance_estimate_spacing_m': cfg['path_spacing_m'],
        'rate_basis': 'independently_measured_distance' if measured_length else 'estimated_frontend_distance',
        'physical_sync_verified': False, 'base_extrinsic_field_calibrated': False,
        'graph_version': archive.get('graph_version') if archive else None,
        'accepted_loop_events': loop_events,
        'loop_factor_count': sum(bool(f.get('loop')) for f in archive.get('factors', [])) if archive else 0,
        'archive_error': archive_error, 'mapping_success': bool(mapping_result.get('success')),
        'online_observer_messages': len([r for r in observer if r['kind'] == 'online']),
        'online_fresh_unique_samples': len(online), 'trajectories': results}
    evaluation.update(online_route_sample_coverage=len(matched_stamps)/len(route_frontend) if route_frontend else None,
                      online_longest_route_sample_gap_s=max(np.diff(valid_times), default=0)/1e9)
    # Every reanalysis writes a NEW directory, retaining previous evidence.
    out = trial/'analysis'/('result-'+str(time.time_ns()))
    out.mkdir(parents=True)
    save(out/'metrics.json', evaluation)
    with (out/'metrics.csv').open('w', newline='') as handle:
        w = csv.writer(handle)
        w.writerow(['trajectory', 'window', 'position_error_m', 'yaw_error_deg', 'relative_drift_percent',
                    'estimated_relative_drift_percent', 'jump_detected'])
        for name, value in results.items():
            for label, m in value['metrics'].items():
                w.writerow([name, label, m.get('position_error_m'), m.get('yaw_error_deg'),
                            m.get('relative_drift_rate_percent'), m.get('estimated_relative_drift_rate_percent'),
                            m.get('window_jump_detected')])
    for name, rows in series.items():
        export_csv(out/(name+'.csv'), rows, inv_base, events['route_start_ns'])
    if raw_start.get('samples', 0):
        plot(out, series, inv_base, events)
    lines = ['# P0 闭环结果', '', '数据状态：'+status+'（不代表精度验收通过）', '',
             '路线：'+str(evaluation['route_label']),
             '已接受回环事件：'+str(loop_events),
             '共同参考路程：'+(f'{measured_length:.3f} m，独立测量' if measured_length else
                                      f'{estimated:.3f} m，原始前端估算' if estimated else '不可用'), '',
             '| 轨迹 | 窗口 | 位置误差 cm | 航向误差 ° | 漂移率 % |',
             '|---|---|---:|---:|---:|']
    for name, value in results.items():
        for label, m in value['metrics'].items():
            if m['available']:
                rate = m['relative_drift_rate_percent'] if measured_length else m['estimated_relative_drift_rate_percent']
                rate_text = f'{rate:.4f}' if rate is not None else 'N/A'
                lines.append(f'| {name} | {label} | {m["position_error_m"]*100:.3f} | {m["yaw_error_deg"]:.3f} | {rate_text} |')
            else:
                lines.append(f'| {name} | {label} | N/A | N/A | N/A |')
    lines += ['', '无实测路程时，表中漂移率均为按前端估算路程归一化；独立路程漂移率字段保持 null。',
              '没有接受回环时，本轮只测得未发生回环修正的在线返回误差。',
              'P0 物理复位由操作员确认；结果受复位重复性和工作外参误差影响。',
              'optimized_history 为最终冻结图校正的全采样轨迹，非所有采样点独立参与优化。',
              'online_backend 保留当时首次发布的有效新鲜采样；缺输出不补值。',
              '窗口跨度、缺口、静止范围、跳变见 metrics.json；无外部全程真值，不报告 APE/RPE。', '']
    (out/'REPORT.md').write_text('\n'.join(lines))
    save(trial/'latest_analysis.json', {'directory': str(out), 'status': status})
    return evaluation, out


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('trial', type=Path)
    p.add_argument('--length-m', type=float)
    args = p.parse_args()
    result, directory = analyze(args.trial, args.length_m)
    print(directory)
    raise SystemExit(0 if result['status'] == 'PASS_DATA' else 2)
