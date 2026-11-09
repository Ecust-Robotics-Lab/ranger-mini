#!/usr/bin/env python3
"""Read-only fixed-landmark trajectory ablations; never calibrates or writes a map."""
import argparse
import importlib.util
import json
from pathlib import Path
import re
import time

import numpy as np

spec = importlib.util.spec_from_file_location('tag_consistency', Path(__file__).with_name('analyze_capture_consistency.py'))
analysis = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analysis)
common = analysis.common


def local_split(stamps, origin, block_ns=5_000_000_000):
    """First/last portions of each block, with a 0.5 s temporal gap."""
    groups = {}
    for i, stamp in enumerate(stamps):
        block, within = divmod(stamp-origin, block_ns)
        group = groups.setdefault(block, {'train': [], 'heldout': []})
        if 250_000_000 <= within < 2_000_000_000:
            if not group['train'] or stamp-stamps[group['train'][-1]] >= 200_000_000:
                group['train'].append(i)
        elif 2_500_000_000 <= within < block_ns-250_000_000:
            group['heldout'].append(i)
    return {b: g for b, g in groups.items() if len(g['train']) >= 4 and len(g['heldout']) >= 4}


def fit_one(cameras, observations, model, train, heldout):
    seeds, modes = analysis.seed_modes(cameras, observations, train)
    fits = [analysis.fit_bundle(seed, cameras, observations, model, train, heldout) for seed in seeds]
    best = min(fits, key=lambda f: f['training_huber_coordinate_cost_mean'])
    return best, modes, len(fits)


def diagnose(cameras, observations, model, origin):
    stamps = [r['stamp_ns'] for r in observations]
    train, heldout, purged = analysis.split_indices(stamps, origin)
    analysis.require(len(train) >= 4 and len(heldout) >= 4, 'insufficient diagnostic split')
    fit, modes, seed_count = fit_one(cameras, observations, model, train, heldout)
    blocks = []
    for block, group in local_split(stamps, origin).items():
        selected = group['train']+group['heldout']
        subobs, subcam = [observations[i] for i in selected], [cameras[i] for i in selected]
        n = len(group['train'])
        local, _, _ = fit_one(subcam, subobs, model, list(range(n)), list(range(n, len(selected))))
        global_errors = [fit['frame_errors'][i]['rmse_px'] for i in group['heldout']]
        blocks.append({'block_index': block, 'start_seconds': block*5.,
                       'train_images': [observations[i]['index'] for i in group['train']],
                       'heldout_images': [observations[i]['index'] for i in group['heldout']],
                       'global_on_local_heldout_px': common.quantiles(global_errors),
                       'local_heldout_px': local['holdout_rmse_px'], 'local_pose': local['pose'],
                       'note': 'local fit changes only the Tag pose; diagnostic of time-varying inconsistency, not a map'})
    return {'global': fit, 'training_modes': modes, 'seeds': seed_count,
            'train_images': [observations[i]['index'] for i in train],
            'heldout_images': [observations[i]['index'] for i in heldout],
            'purged_images': [observations[i]['index'] for i in purged], 'local_blocks': blocks}


def load_native(path):
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    connection = common.open_readonly(path)
    try:
        topic = connection.execute("SELECT id FROM topics WHERE name='/fast_livo/odometry'").fetchone()[0]
        return common.pose_records(connection, topic, deserialize_message, Odometry)
    finally:
        connection.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--observations', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--cases', type=Path, required=True)
    parser.add_argument('--cohort', type=Path, help='Exact previously selected image indices; unavailable support is an error')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not args.execute:
        print(json.dumps({'action': 'DRY-RUN', 'output': str(args.output)}))
        return 0
    started = time.monotonic()
    cases = json.loads(args.cases.read_text())
    config = json.loads(args.config.read_text())
    observations = analysis.rows(args.observations)
    paths = [args.observations, args.config, args.cases]+[Path(c['path']) for c in cases]
    if args.cohort:
        paths.append(args.cohort)
    hashes = {str(p.resolve()): common.sha_file(p) for p in paths}
    datasets = {}
    for case in cases:
        analysis.require(re.fullmatch(r'[A-Za-z0-9_-]+', case['name']), 'invalid case name')
        analysis.require(case['name'] not in datasets, 'duplicate case name')
        analysis.require(case['format'] in ('native_db', 'diagnostic_states'), 'unsupported trajectory format')
        records = load_native(case['path']) if case['format'] == 'native_db' else analysis.rows(case['path'])
        history = common.PoseHistory(records)
        analysis.require(history.monotonic, 'trajectory contains duplicate/regressed epoch')
        cameras, supports = analysis.associate(history, observations, common.matrix(config['T_imu_camera']), case['offset_ns'])
        datasets[case['name']] = (cameras, supports)
    indices = [i for i, row in enumerate(observations) if not row['rejection_reasons']
               and all(d[1][i]['available'] for d in datasets.values())]
    if args.cohort:
        requested = json.loads(args.cohort.read_text())
        available = {observations[i]['index']: i for i in indices}
        analysis.require(requested == sorted(set(requested)) and all(i in available for i in requested),
                         'cohort has duplicate, unsorted, or unsupported image identities')
        indices = [available[i] for i in requested]
    cohort = [observations[i] for i in indices]
    analysis.require(len(cohort) >= 20, 'insufficient common cohort')
    args.output.mkdir(parents=True, exist_ok=False)
    results = []
    for case in cases:
        cameras = [datasets[case['name']][0][i] for i in indices]
        result = diagnose(cameras, cohort, config['camera'], observations[0]['stamp_ns'])
        result['case'] = case
        (args.output/(case['name']+'.json')).write_text(json.dumps(result, indent=2)+'\n')
        results.append({'case': case, 'global_holdout_px': result['global']['holdout_rmse_px'],
                        'global_training_cost': result['global']['training_huber_coordinate_cost_mean'],
                        'initial_threshold_pass': result['global']['frames_passing_initial_3px_rms_6px_max'],
                        'local_blocks': len(result['local_blocks']),
                        'median_local_heldout_p95_px': float(np.median([b['local_heldout_px']['p95'] for b in result['local_blocks']]))})
        print(json.dumps(results[-1]), flush=True)
    summary = {'common_cohort': len(cohort), 'source_sha256': hashes, 'cases': results,
               'wall_seconds': time.monotonic()-started, 'diagnostic_only': True,
               'physical_time_verified': False, 'real_VALID_landmark_created': False,
               'limitations': ['same-capture temporal holdout, not independent visits',
                              'local block fit changes landmark origin; never combines these poses into one landmark',
                              'LIO ablation has no image fusion and does not replace the three-sensor frontend',
                              'differences are conditional on fixed calibration and do not identify a unique error source']}
    analysis.require(all(common.sha_file(p) == hashes[str(p.resolve())] for p in paths), 'input changed')
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
