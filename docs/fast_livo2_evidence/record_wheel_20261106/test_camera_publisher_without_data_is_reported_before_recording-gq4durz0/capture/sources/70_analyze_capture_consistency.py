#!/usr/bin/env python3
"""Offline timing and fixed-local-Tag diagnostics, never a clock calibration/map."""
import argparse
import bisect
from collections import Counter
import csv
import importlib.util
import json
from pathlib import Path
import time

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

spec = importlib.util.spec_from_file_location('tag_assessment', Path(__file__).with_name('assess_landmark.py'))
geometry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(geometry)
common = geometry.common


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def rows(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line]


def analyze_sync(camera, writer, lidar_stamps):
    """Use same-host steady arrivals; do not turn arrival age into exposure offset."""
    by_seq = {2*int(row['seq']): row for row in writer}
    arrivals = [int(row['arrival_ns']) for row in writer]
    require(all(b > a for a, b in zip(arrivals, arrivals[1:])), 'writer arrival epoch not monotonic')
    writer_stamps = {int(row['scan_start_ns']) for row in writer}
    missing = sorted(set(lidar_stamps)-writer_stamps)
    first, last = min(lidar_stamps), max(lidar_stamps)
    output, failures, ages, read_latencies = [], [], [], []
    latest_deltas = Counter()
    for i, row in enumerate(camera):
        seq, low, arrival = int(row['ts_seq']), int(row['ts_low_ns']), int(row['arrival_ns'])
        require(seq in by_seq and int(by_seq[seq]['scan_start_ns']) == low, 'camera/writer identity mismatch')
        j = bisect.bisect_right(arrivals, arrival)-1
        require(j >= 0, 'camera precedes writer history')
        consumed = by_seq[seq]
        age = (arrival-int(consumed['arrival_ns']))*1e-6
        ages.append(age)
        latest_deltas[seq//2-int(writer[j]['seq'])] += 1
        record = {'frame_id': int(row['frame_id']), 'status': row['status'], 'shared_seq': seq,
                  'shared_stamp_ns': low, 'camera_arrival_ns': arrival, 'writer_age_at_camera_arrival_ms': age,
                  'latest_writer_sequence_at_camera_arrival': int(writer[j]['seq']),
                  'inside_recorded_lidar_range': first <= low <= last}
        if int(row['publish_ns']) > 0:
            read_latencies.append((int(row['publish_ns'])-arrival)*1e-6)
        if row['status'] != 'OK':
            record['same_stamp_as_previous'] = i > 0 and low == int(camera[i-1]['ts_low_ns'])
            record['same_seq_as_previous'] = i > 0 and seq == int(camera[i-1]['ts_seq'])
            next_stamp = int(writer[j+1]['scan_start_ns']) if j+1 < len(writer) else None
            record['following_writer_stamp_gap_ms'] = (next_stamp-low)*1e-6 if next_stamp else None
            record['bag_scans_absent_from_writer_in_following_gap'] = [t for t in missing if next_stamp and low < t < next_stamp]
            failures.append(record)
        output.append(record)
    dev = [int(row['dev_ts_ticks']) for row in camera]
    frame_ids = [int(row['frame_id']) for row in camera]
    x = np.asarray([v-dev[0] for v in dev], dtype=float)
    y = np.asarray([int(row['arrival_ns'])-int(camera[0]['arrival_ns']) for row in camera], dtype=float)
    slope, intercept = np.linalg.lstsq(np.column_stack([x, np.ones_like(x)]), y, rcond=None)[0]
    return {'camera_frames': len(camera), 'writer_samples': len(writer), 'recorded_lidar_scans': len(lidar_stamps),
            'recorded_lidar_scans_missing_from_writer': missing,
            'writer_age_at_camera_arrival_ms': common.quantiles(ages),
            'camera_read_publish_after_arrival_ms': common.quantiles(read_latencies),
            'consumed_minus_latest_writer_sequence_at_frame_arrival': dict(latest_deltas),
            'camera_frame_ids_contiguous': all(b == a+1 for a, b in zip(frame_ids, frame_ids[1:])),
            'device_clock_monotonic': all(b > a for a, b in zip(dev, dev[1:])),
            'empirical_device_tick_to_steady_arrival_ns_per_tick': float(slope),
            'device_to_arrival_affine_residual_abs_ms': common.quantiles(np.abs(y-(slope*x+intercept))*1e-6),
            'anomalies': failures, 'physical_exposure_offset_identified': False}, output


def projected_errors(transform, inverse_cameras, observations, model):
    """One projectPoints call for all cameras, preserving raw Brown pixel domain."""
    points = geometry.object_points(observations[0]['observation']['size_m'])
    world = points@transform[:3, :3].T+transform[:3, 3]
    camera_points = np.einsum('nij,kj->nki', inverse_cameras[:, :3, :3], world)+inverse_cameras[:, None, :3, 3]
    projected, _ = cv2.projectPoints(camera_points.reshape(-1, 3), np.zeros(3), np.zeros(3),
                                    np.asarray(model['K']).reshape(3, 3), np.asarray(model['D']))
    observed = np.asarray([row['observation']['corners'] for row in observations])
    return projected.reshape(-1, 4, 2)-observed, np.all(camera_points[:, :, 2] > 1e-6, axis=1)


def split_indices(stamps, origin, period_ns=5_000_000_000, purge_ns=500_000_000, thin_ns=500_000_000):
    """Disjoint temporal blocks, boundary purge and bounded training frame density."""
    train, holdout, purged = [], [], []
    previous = None
    for i, stamp in enumerate(stamps):
        block, within = divmod(stamp-origin, period_ns)
        if within < purge_ns or within >= period_ns-purge_ns:
            purged.append(i)
        elif block % 3 == 2:
            holdout.append(i)
        elif previous is None or stamp-previous >= thin_ns:
            train.append(i)
            previous = stamp
    return train, holdout, purged


def fit_bundle(seed, cameras, observations, model, train, holdout, limit=100):
    inverse_cameras = np.linalg.inv(np.asarray(cameras))
    selected = np.asarray(train)
    require(len(selected) >= 4, 'insufficient fixed-pose training support')

    def decode(x):
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_rotvec(x[3:]).as_matrix()
        transform[:3, 3] = x[:3]
        return transform

    def residual(x):
        errors, positive = projected_errors(decode(x), inverse_cameras[selected], [observations[i] for i in selected], model)
        errors[~positive] = 1e4
        return errors.ravel()

    initial = np.r_[seed[:3, 3], Rotation.from_matrix(seed[:3, :3]).as_rotvec()]
    fit = least_squares(residual, initial, loss='huber', f_scale=1., max_nfev=limit)
    transform = decode(fit.x)
    errors, positive = projected_errors(transform, inverse_cameras, observations, model)
    lengths = np.linalg.norm(errors, axis=2)
    rms, maxima = np.sqrt(np.mean(lengths**2, axis=1)), np.max(lengths, axis=1)
    training_residual = residual(fit.x)
    # Huber coordinate cost is the optimizer objective, not a calibrated likelihood.
    loss = np.where(np.abs(training_residual) <= 1., .5*training_residual**2, np.abs(training_residual)-.5)
    return {'pose': common.pose_json(transform), 'optimizer_success': bool(fit.success), 'evaluations': fit.nfev,
            'training_huber_coordinate_cost_mean': float(np.mean(loss)),
            'train_rmse_px': common.quantiles(rms[train]), 'holdout_rmse_px': common.quantiles(rms[holdout]),
            'all_cohort_rmse_px': common.quantiles(rms), 'all_cohort_maximum_corner_px': common.quantiles(maxima),
            'frames_passing_initial_3px_rms_6px_max': int(np.sum(positive & (rms <= 3.) & (maxima <= 6.))),
            'negative_depth_frames': int(np.sum(~positive)),
            'frame_errors': [{'image_index': row['index'], 'stamp_ns': row['stamp_ns'], 'rmse_px': float(r),
                              'maximum_px': float(m), 'positive_depth': bool(p)}
                             for row, r, m, p in zip(observations, rms, maxima, positive)],
            'trajectory_optimized': False, 'extrinsic_optimized': False, 'clock_calibrated': False}


def associate(history, observations, E, offset):
    cameras, associations = [], []
    for row in observations:
        record = history.lookup(row['stamp_ns']+offset)
        associations.append(record)
        cameras.append(common.matrix(record['pose'])@E if record['available'] else None)
    return cameras, associations


def seed_modes(cameras, observations, train):
    candidates = []
    for i in train:
        for h in observations[i]['observation']['hypotheses']:
            candidates.append({'T': cameras[i]@common.matrix(h['T_camera_tag']),
                               'observation_id': observations[i]['observation']['observation_id']})
    modes = geometry.cluster(candidates)
    return [geometry.center([row['T'] for row in mode]) for mode in modes[:4]], len(modes)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--screen', type=Path, required=True)
    parser.add_argument('--fixedness', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not args.execute:
        print(json.dumps({'action': 'DRY-RUN', 'scope': 'diagnostic only; no clock/config/map update', 'output': str(args.output)}))
        return 0
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry

    args.output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    capture, screen = args.capture.resolve(), args.screen.resolve()
    screened = json.loads((screen/'summary.json').read_text())
    declaration = json.loads(args.fixedness.read_text())
    require(declaration['fixed_during_capture'] and declaration['size_m'] == .140, 'new fixed-tag declaration missing')
    require(screened['sizes']['sizes_m']['1'] == .140 and screened['identities'].keys() == {'tag36h11:1'}, 'wrong screened Tag identity')
    database = capture/'raw/raw_0.db3'
    require(common.sha_file(database) == screened['source']['sha256'], 'raw database SHA mismatch')
    manifest = json.loads((capture/'manifest.json').read_text())
    archived = next(row for row in manifest['source_files'] if row['source'].endswith('/apriltag_camera.json'))
    config_path = capture/archived['file']
    require(common.sha_file(config_path) == archived['sha256'], 'archived calibration SHA mismatch')
    config = json.loads(config_path.read_text())
    require(config['camera']['pixel_domain'] == 'raw' and config['camera']['distortion_model'] == 'plumb_bob', 'camera model unsupported')
    connection = common.open_readonly(database)
    topics = {name: key for key, name in connection.execute('SELECT id,name FROM topics')}
    native = common.pose_records(connection, topics['/fast_livo/odometry'], deserialize_message, Odometry)
    lidar = [common.stamp(deserialize_message(data, PointCloud2)) for data, in connection.execute(
        'SELECT data FROM messages WHERE topic_id=? ORDER BY id', (topics['/livox/lidar'],))]
    connection.close()
    camera_csv = list(csv.DictReader((capture/'sensor_assets/camera_sync.csv').open()))
    writer_csv = list(csv.DictReader((capture/'sensor_assets/timeshare_writer.csv').open()))
    timing, timing_rows = analyze_sync(camera_csv, writer_csv, lidar)
    (args.output/'timing.json').write_text(json.dumps(timing, indent=2)+'\n')
    (args.output/'camera_writer_associations.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in timing_rows))
    observations = rows(screen/'observations.jsonl')
    require(len(observations) == screened['counts']['observations'] and all(not row['rejection_reasons'] for row in observations), 'screen observations inconsistent')
    history = common.PoseHistory(native)
    require(history.monotonic, 'local trajectory crosses an epoch')
    E = common.matrix(config['T_imu_camera'])
    period = int(np.median(np.diff([row['stamp_ns'] for row in rows(screen/'frames.jsonl')])))
    offsets = [i*(period//4) for i in range(-8, 9)]
    datasets = {offset: associate(history, observations, E, offset) for offset in offsets}
    common_indices = [i for i in range(len(observations)) if all(dataset[1][i]['available'] for dataset in datasets.values())]
    require(len(common_indices) >= 20, 'insufficient common cohort across offset sensitivity grid')
    cohort = [observations[i] for i in common_indices]
    origin = observations[0]['stamp_ns']
    train, holdout, purged = split_indices([row['stamp_ns'] for row in cohort], origin)
    require(len(holdout) >= 4, 'insufficient temporally separated holdout support')
    split = {'common_observation_indices': common_indices, 'common_image_indices': [row['index'] for row in cohort],
             'train_image_indices': [cohort[i]['index'] for i in train], 'holdout_image_indices': [cohort[i]['index'] for i in holdout],
             'boundary_purged_image_indices': [cohort[i]['index'] for i in purged],
             'block_seconds': 5, 'purge_each_side_seconds': .5, 'training_min_spacing_seconds': .5,
             'holdout_rule': 'block_index modulo 3 equals 2; same capture, not independent visits'}
    (args.output/'split.json').write_text(json.dumps(split, indent=2)+'\n')
    scores, baseline = [], None
    for offset in offsets:
        cameras = [datasets[offset][0][i] for i in common_indices]
        seeds, total_modes = seed_modes(cameras, cohort, train)
        fits = [fit_bundle(seed, cameras, cohort, config['camera'], train, holdout) for seed in seeds]
        # Seed/offset ranking uses training objective only. Holdout is never fitted.
        best = min(fits, key=lambda fit: fit['training_huber_coordinate_cost_mean'])
        scored = {'assumed_offset_ns': offset, 'offset_is_calibrated': False, 'common_cohort_count': len(cohort),
                  'training_pose_modes': total_modes, 'seed_budget': 4, 'evaluated_seeds': len(seeds),
                  'fits': [{k: v for k, v in fit.items() if k != 'frame_errors'} for fit in fits],
                  'training_selected_seed': fits.index(best)}
        (args.output/f'fit_offset_{offset}.json').write_text(json.dumps({'score': scored, 'training_selected_fit': best}, indent=2)+'\n')
        scores.append(scored)
        if offset == 0:
            baseline = best
    best_row = min(scores, key=lambda row: row['fits'][row['training_selected_seed']]['training_huber_coordinate_cost_mean'])
    best_fit = best_row['fits'][best_row['training_selected_seed']]
    zero_cameras, zero_support = datasets[0]
    positions = [zero_cameras[i]@common.matrix(observations[i]['observation']['hypotheses'][0]['T_camera_tag'])
                 for i in range(len(observations)) if zero_cameras[i] is not None]
    center = geometry.center(positions)
    (args.output/'offset_sensitivity.json').write_text(json.dumps(scores, indent=2)+'\n')
    result = {'kind': 'conditional_fixed_local_tag_and_software_timing_not_registration',
              'source_database_sha256': screened['source']['sha256'], 'fixedness_declaration': declaration,
              'source_files_sha256': {str(p): common.sha_file(p) for p in [screen/'summary.json', screen/'observations.jsonl', screen/'frames.jsonl', config_path, args.fixedness]},
              'timing': timing, 'observations': len(observations), 'zero_offset_supported': sum(a['available'] for a in zero_support),
              'common_offset_cohort': len(cohort), 'training_count': len(train), 'holdout_count': len(holdout),
              'time_offset_grid_ns': offsets, 'zero_offset_fit': {k: v for k, v in baseline.items() if k != 'frame_errors'},
              'diagnostic_training_minimum_offset_ns': best_row['assumed_offset_ns'], 'diagnostic_training_minimum_fit': best_fit,
              'zero_offset_primary_local_tag_position_spread_m': common.quantiles([np.linalg.norm(p[:3, 3]-center[:3, 3]) for p in positions]),
              'wall_seconds': time.monotonic()-started, 'physical_time_verified': False, 'clock_mapping_changed': False,
              'real_VALID_landmark_created': False, 'map_pose_estimated': False,
              'limitations': ['One capture; temporal holdout is not independent-visit evidence',
                              'Local trajectory may contain drift and is not independent of images',
                              'Fixed extrinsics and pixel model; error sources cannot be individually identified',
                              'Offset sweep is diagnostic, not a physical exposure calibration',
                              'Pose modes beyond four-seed budget remain untested; no uniqueness claim']}
    require(common.sha_file(database) == screened['source']['sha256'], 'source database changed')
    (args.output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'timing'}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
