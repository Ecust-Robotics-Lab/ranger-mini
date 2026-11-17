#!/usr/bin/env python3
"""Read-only bag/archived-factor ablations; fixed acquisition, no endpoint factors."""
import argparse
import bisect
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'fast_livo2_p0_closure'))
import analysis as a

VARIANTS = {'baseline': {}, 'no_loops': {'no_loops': True},
            'thin': {'thin': True}, 'odom_half': {'odom_sigma_scale': .5},
            'thin_odom_half': {'thin': True, 'odom_sigma_scale': .5},
            'cauchy': {'robust_kernel': 'cauchy'}}


def dataset(trial):
    capture = trial / 'capture'
    obs = a.jsonl(capture / 'closure_samples.jsonl')
    anchor = a.load(capture / 'clock_anchor.json')['anchor']
    raw, source = a.raw_frontend(capture, obs, anchor)
    assert source == 'raw_bag_odometry'
    return a.unique_frontend(raw), obs, a.load(capture / 'map/manifest.json')


def evaluate(trial, raw, keyframes, causal=None):
    events, cfg = a.load(trial / 'events.json'), a.load(trial / 'config.json')
    inv_base = np.linalg.inv(a.base_extrinsic(a.load(trial / 'calibration.json')))
    length = a.path_length(raw, events['route_start_ns'], events['returned_ns'], inv_base, cfg['path_spacing_m'])
    history = a.optimized_rows(raw, {'keyframes': keyframes})
    series = {'frontend': raw, 'history': history}
    if causal is not None:
        series['causal_offline'] = causal
    result = {}
    for name, rows in series.items():
        start = a.window(rows, events['start_window'], inv_base, cfg)
        settled = a.window(rows, [events['returned_ns'] + 11 * 10**9, events['returned_ns'] + 13 * 10**9], inv_base, cfg)
        immediate = a.window(rows, [events['returned_ns'] + 10**9, events['returned_ns'] + 3 * 10**9], inv_base, cfg)
        result[name] = {'settled': a.closure(start, settled, length, None),
                        'immediate': a.closure(start, immediate, length, None)}
    # Deformation is a diagnostic, NOT accuracy or RPE without ground truth.
    corrections = [a.matrix(k['X_map_imu']) @ np.linalg.inv(a.matrix(k['U_local_imu'])) for k in keyframes]
    result['graph_deformation'] = {
        'maximum_keyframe_displacement_m': max(np.linalg.norm(a.matrix(k['X_map_imu'])[:3, 3] - a.matrix(k['U_local_imu'])[:3, 3]) for k in keyframes),
        'maximum_adjacent_correction_translation_m': max((np.linalg.norm(x[:3, 3] - y[:3, 3]) for x, y in zip(corrections, corrections[1:])), default=0)}
    return result


def causal_rows(raw, replay):
    # Apply only updates causally available after processing that scan. No future graph version.
    by_key = {len(s['corrections']) - 1: s for s in replay['states']}
    frames = replay['keyframes']; stamps = [k['stamp_ns'] for k in frames]
    result = []
    for r in raw:
        k = bisect.bisect_right(stamps, r['stamp_ns']) - 1
        if k >= 0:
            result.append({**r, 'T_imu': a.matrix(by_key[k]['corrections'][k]) @ a.matrix(r['pose'])})
    return result


def audit_online(raw, obs, manifest, replay):
    by_stamp = {r['stamp_ns']: r for r in raw}
    states = {r['original_version']: r for r in replay['states']}
    errors, ages, stamps_missing = [], [], 0
    seen = set()
    for r in obs:
        if r['kind'] != 'online' or not r.get('pose_valid') or r.get('session_id') != manifest['session_id']:
            continue
        key = (r['stamp_ns'], r['scan_id'])
        if key in seen:
            continue
        seen.add(key)
        if r['stamp_ns'] not in by_stamp:
            stamps_missing += 1
            continue
        c = states[r['graph_version']]['corrections'][r['reference_keyframe_id']]
        predicted = a.matrix(c) @ a.matrix(by_stamp[r['stamp_ns']]['pose'])
        actual = a.matrix(r['pose'])
        errors.append(float(np.max(np.abs(predicted - actual))))
        ages.append(r['sample_age_s'])
    return {'matched_first_publications': len(errors), 'missing_exact_stamps': stamps_missing,
            'maximum_pose_matrix_error': max(errors, default=None),
            'first_publication_age_median_s': float(np.median(ages)),
            'first_publication_age_p95_s': float(np.percentile(ages, 95))}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--group', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--binary', type=Path, default=HERE / 'build/graph_ablation')
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    a.save(args.output / 'protocol.json', {'variants': VARIANTS,
        'development_trials': [1, 2], 'reserved_candidate_check': [4], 'partial_diagnostic_only': [3, 5],
        'note': 'All baseline endpoints previously seen. Trial 4 is a reserved candidate check, not a blind test. No endpoint factors or per-trial parameter fitting.'})
    result = []
    for trial in sorted(args.group.glob('trial_*_attempt_01')):
        raw, obs, manifest = dataset(trial)
        row = {'trial': int(trial.name.split('_')[1]), 'variants': {}, 'manifest_sha256': hashlib.sha256((trial / 'capture/map/manifest.json').read_bytes()).hexdigest()}
        folder = args.output / trial.name; folder.mkdir()
        for name, options in VARIANTS.items():
            a.save(folder / (name + '_options.json'), options)
            output = folder / (name + '.json')
            command = [str(args.binary), str(trial / 'capture/map'), str(folder / (name + '_options.json')), str(output)]
            subprocess.run(command, check=True)
            replay = a.load(output)
            metrics = evaluate(trial, raw, replay['keyframes'], causal_rows(raw, replay))
            metrics.update(loop_factors=replay['loop_factors'], redundant_events=sum(d['redundant'] for d in replay['decisions']),
                           rejected=replay['rejected'], objective_increases=replay['objective_increases'])
            if name == 'baseline':
                metrics['online_audit'] = audit_online(raw, obs, manifest, replay)
                metrics['final_pose_matrix_difference'] = replay['original_final_pose_matrix_difference']
            row['variants'][name] = metrics
        result.append(row)
        a.save(args.output / 'results.json', result)
        print(json.dumps(row), flush=True)


if __name__ == '__main__':
    main()
