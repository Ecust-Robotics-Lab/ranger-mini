#!/usr/bin/env python3
"""Full read-only AprilTag extraction and header-clock pose association, no ROS graph."""
import argparse
import bisect
from collections import Counter, defaultdict
import hashlib
import json
import math
import os
from pathlib import Path
import select
import sqlite3
import subprocess
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

ROOT = Path(__file__).resolve().parents[3]


def sha_file(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(8*1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def stamp(message):
    return message.header.stamp.sec*10**9+message.header.stamp.nanosec


def quantiles(values):
    values = [float(v) for v in values if v is not None and math.isfinite(v)]
    if not values:
        return {'count': 0}
    return dict(count=len(values), minimum=min(values), p50=float(np.percentile(values, 50)),
                p95=float(np.percentile(values, 95)), maximum=max(values))


def matrix(pose):
    out = np.eye(4)
    out[:3, :3] = Rotation.from_quat(pose['xyzw']).as_matrix()
    out[:3, 3] = pose['xyz']
    return out


def pose_json(transform):
    return {'xyz': transform[:3, 3].tolist(), 'xyzw': Rotation.from_matrix(transform[:3, :3]).as_quat().tolist()}


class PoseHistory:
    """One immutable capture sequence; never merges raw and replay coordinates."""
    def __init__(self, records, gap=.15, max_speed=10., max_angular_speed=6.):
        self.records = records
        self.stamps = [row['stamp_ns'] for row in records]
        self.gap, self.max_speed, self.max_angular_speed = gap, max_speed, max_angular_speed
        self.monotonic = all(b > a for a, b in zip(self.stamps, self.stamps[1:]))

    def lookup(self, ns):
        if not self.monotonic:
            return {'available': False, 'reason': 'nonmonotonic_or_duplicate_pose_epoch'}
        position = bisect.bisect_left(self.stamps, ns)
        if position < len(self.stamps) and self.stamps[position] == ns:
            row = self.records[position]
            return {'available': True, 'kind': 'exact', 'pose': row['pose'],
                    'support_stamps_ns': [ns], 'support_message_ids': [row['message_id']], 'alpha': 0.}
        if position == 0 or position == len(self.stamps):
            return {'available': False, 'reason': 'no_bracket_no_extrapolation'}
        a, b = self.records[position-1:position+1]
        interval_ns = b['stamp_ns']-a['stamp_ns']
        dt = interval_ns*1e-9
        if dt > self.gap:
            return {'available': False, 'reason': 'pose_gap_excessive', 'gap_ns': interval_ns}
        translation = np.asarray(b['pose']['xyz'])-np.asarray(a['pose']['xyz'])
        qa, qb = a['pose']['xyzw'], b['pose']['xyzw']
        angle = (Rotation.from_quat(qa).inv()*Rotation.from_quat(qb)).magnitude()
        if np.linalg.norm(translation)/dt > self.max_speed or angle/dt > self.max_angular_speed:
            return {'available': False, 'reason': 'pose_interval_motion_excessive'}
        alpha = (ns-a['stamp_ns'])/interval_ns  # subtract integer ns before conversion
        q = Slerp([0., 1.], Rotation.from_quat([qa, qb]))([alpha]).as_quat()[0]
        p = np.asarray(a['pose']['xyz'])+alpha*translation
        return {'available': True, 'kind': 'bracketed_slerp', 'pose': {'xyz': p.tolist(), 'xyzw': q.tolist()},
                'support_stamps_ns': [a['stamp_ns'], b['stamp_ns']],
                'support_message_ids': [a['message_id'], b['message_id']], 'alpha': alpha,
                'gap_ns': interval_ns, 'interval_speed_mps': float(np.linalg.norm(translation)/dt),
                'interval_angular_speed_radps': float(angle/dt)}

    def summary(self):
        return {'count': len(self.records), 'strictly_monotonic': self.monotonic,
                'first_stamp_ns': self.stamps[0] if self.stamps else None,
                'last_stamp_ns': self.stamps[-1] if self.stamps else None,
                'interval_ms': quantiles([(b-a)*1e-6 for a, b in zip(self.stamps, self.stamps[1:])])}


def pose_records(connection, topic, deserialize, odometry_type):
    records = []
    for message_id, record_ns, data in connection.execute(
            'SELECT id,timestamp,data FROM messages WHERE topic_id=? ORDER BY id', (topic,)):
        message = deserialize(data, odometry_type)
        p, q = message.pose.pose.position, message.pose.pose.orientation
        values = np.asarray([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        if not np.isfinite(values).all() or abs(np.linalg.norm(values[3:])-1.) > 1e-3:
            raise ValueError('invalid odometry pose at message '+str(message_id))
        if message.header.frame_id != 'fast_livo_local' or message.child_frame_id != 'fast_livo_imu':
            raise ValueError('odometry body/frame contract mismatch')
        records.append({'message_id': message_id, 'record_ns': record_ns, 'stamp_ns': stamp(message),
                        'pose': {'xyz': values[:3].tolist(), 'xyzw': (values[3:]/np.linalg.norm(values[3:])).tolist()}})
    return records


def camera_model(message):
    return {'width': message.width, 'height': message.height, 'frame': message.header.frame_id,
            'distortion_model': message.distortion_model, 'K': list(message.k), 'D': list(message.d),
            'R': list(message.r), 'P': list(message.p), 'binning_x': message.binning_x, 'binning_y': message.binning_y,
            'roi': [message.roi.x_offset, message.roi.y_offset, message.roi.width, message.roi.height, message.roi.do_rectify]}


def model_matches(model, configured):
    return (all(model[key] == configured[key] for key in ['width', 'height', 'frame', 'distortion_model'])
            and all(np.allclose(model[key], configured[key], atol=1e-9, rtol=0) for key in ['K', 'D', 'R'])
            and np.allclose(np.asarray(model['P']).reshape(3, 4)[:, :3], np.asarray(configured['K']).reshape(3, 3), atol=1e-9, rtol=0)
            and np.allclose(np.asarray(model['P']).reshape(3, 4)[:, 3], 0, atol=1e-9, rtol=0)
            and model['binning_x'] in [0, 1] and model['binning_y'] in [0, 1] and not any(model['roi']))


def request_frame(process, request, pixels, timeout=15.):
    process.stdin.write((json.dumps(request, separators=(',', ':'))+'\n').encode())
    process.stdin.write(pixels)
    process.stdin.flush()
    if not select.select([process.stdout], [], [], timeout)[0]:
        raise TimeoutError('detector response timeout')
    response = process.stdout.readline(1024*1024)
    if not response or not response.endswith(b'\n'):
        raise RuntimeError('detector missing/truncated/oversized response; inspect detector.log')
    parsed = json.loads(response)
    if parsed['request_id'] != request['request_id']:
        raise RuntimeError('detector request/response identity mismatch')
    return parsed


def open_readonly(path):
    connection = sqlite3.connect(Path(path).resolve().as_uri()+'?mode=ro', uri=True)
    connection.execute('PRAGMA query_only=ON')
    return connection


def run(args):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image, CameraInfo
    from nav_msgs.msg import Odometry

    args.output.mkdir(parents=True, exist_ok=False)
    source_stat = args.bag.stat()
    config = json.loads(args.config.read_text())
    sizes = json.loads(args.sizes.read_text())
    if sizes['family'] != 'tag36h11':
        raise ValueError('this capture audit explicitly uses tag36h11; family mismatch')
    before = {'path': str(args.bag.resolve()), 'bytes': source_stat.st_size, 'mtime_ns': source_stat.st_mtime_ns,
              'sha256': sha_file(args.bag)}
    source_id = 'bag-'+before['sha256']
    config_sha, sizes_sha = sha_file(args.config), sha_file(args.sizes)
    connection = open_readonly(args.bag)
    topics = {name: (topic_id, kind) for topic_id, name, kind in connection.execute('SELECT id,name,type FROM topics')}
    total = connection.execute('SELECT count(*) FROM messages WHERE topic_id=?', (topics['/front/image_sync'][0],)).fetchone()[0]
    raw_poses = pose_records(connection, topics['/fast_livo/odometry'][0], deserialize_message, Odometry)
    history = PoseHistory(raw_poses, config.get('history_gap', .15), config.get('max_speed', 10.), config.get('max_angular_speed', 6.))
    replay = open_readonly(args.native)
    replay_topic = replay.execute("SELECT id FROM topics WHERE name='/fast_livo/odometry'").fetchone()[0]
    replay_poses = pose_records(replay, replay_topic, deserialize_message, Odometry)
    replay_history = PoseHistory(replay_poses, config.get('history_gap', .15))
    replay.close()
    native_identity = {'path': str(args.native.resolve()), 'sha256': sha_file(args.native)}
    for filename, records in [('raw_odometry.jsonl', raw_poses), ('replay_odometry.jsonl', replay_poses)]:
        with (args.output/filename).open('w') as stream:
            for row in records:
                stream.write(json.dumps(row)+'\n')

    models, info_by_stamp = {}, defaultdict(list)
    for message_id, record_ns, data in connection.execute('SELECT id,timestamp,data FROM messages WHERE topic_id=? ORDER BY id', (topics['/front/camera_info'][0],)):
        message = deserialize_message(data, CameraInfo)
        model = camera_model(message)
        identity = hashlib.sha256(json.dumps(model, sort_keys=True).encode()).hexdigest()
        models[identity] = model
        info_by_stamp[stamp(message)].append({'message_id': message_id, 'record_ns': record_ns,
                                            'header_stamp_ns': stamp(message), 'model_sha256': identity})
    raw_images = defaultdict(list)
    raw_count = 0
    # Pixel equality establishes the copied image identity across clock domains;
    # host CameraInfo is never associated by nearest timestamp to a 2020 stamp.
    for message_id, record_ns, data in connection.execute('SELECT id,timestamp,data FROM messages WHERE topic_id=? ORDER BY id', (topics['/front/image'][0],)):
        message = deserialize_message(data, Image)
        digest = hashlib.sha256(bytes(message.data)).hexdigest()
        raw_images[digest].append({'message_id': message_id, 'record_ns': record_ns, 'stamp_ns': stamp(message),
                                   'width': message.width, 'height': message.height, 'encoding': message.encoding,
                                   'step': message.step, 'frame': message.header.frame_id})
        raw_count += 1
    (args.output/'camera_models.json').write_text(json.dumps(models, indent=2)+'\n')

    command = [str(args.detector), '--stream', str(args.config.resolve()), 'tag36h11', json.dumps(sizes['sizes_m'], separators=(',', ':'))]
    detector_log = (args.output/'detector.log').open('wb')
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=detector_log)
    counters = Counter()
    identities = Counter()
    timing, metric_timing, hypothesis_errors, associations = [], [], [], Counter()
    sync_stamps, positive_indices, rows = [], [], []
    previous = {}
    E = matrix(config['T_imu_camera'])
    start = time.monotonic()
    frames_stream = (args.output/'frames.jsonl').open('w')
    observations_stream = (args.output/'observations.jsonl').open('w')
    transitions_stream = (args.output/'hypothesis_transitions.jsonl').open('w')
    try:
        for index, (message_id, record_ns, data) in enumerate(connection.execute('SELECT id,timestamp,data FROM messages WHERE topic_id=? ORDER BY id', (topics['/front/image_sync'][0],))):
            message = deserialize_message(data, Image)
            pixels = bytes(message.data)
            pixel_sha = hashlib.sha256(pixels).hexdigest()
            ns = stamp(message)
            effective = ns+int(config['offset_ns'])
            sync_stamps.append(ns)
            copied_from = raw_images.get(pixel_sha, [])
            camera_infos = info_by_stamp.get(copied_from[0]['stamp_ns'], []) if len(copied_from) == 1 else []
            camera_info = camera_infos[0] if len(camera_infos) == 1 else None
            matches = bool(camera_info and model_matches(models[camera_info['model_sha256']], config['camera']))
            image_model_ok = (message.width == config['camera']['width'] and message.height == config['camera']['height']
                              and message.header.frame_id == config['camera']['frame'])
            if len(copied_from) == 1:
                counters['unique_raw_copy_matches'] += 1
            if matches and image_model_ok:
                counters['camera_model_matches'] += 1
            raw_association = history.lookup(effective)
            replay_association = replay_history.lookup(effective)
            associations['raw:'+raw_association.get('kind', raw_association.get('reason'))] += 1
            associations['replay:'+replay_association.get('kind', replay_association.get('reason'))] += 1
            request = {'schema_version': 1, 'request_id': index, 'payload_bytes': len(pixels),
                       'width': message.width, 'height': message.height, 'step': message.step, 'encoding': message.encoding,
                       'stamp_ns': ns, 'context': source_id, 'image_id': source_id+':'+str(message_id),
                       'image_reference': str(args.bag.resolve())+'#/front/image_sync/message/'+str(message_id)}
            response = request_frame(process, request, pixels)
            frame = {'image_index': index, 'message_id': message_id, 'record_ns': record_ns, 'image_stamp_ns': ns,
                     'effective_stamp_ns': effective, 'message_sha256': hashlib.sha256(data).hexdigest(), 'pixel_sha256': pixel_sha,
                     'raw_copy_matches': copied_from, 'camera_info': camera_info, 'camera_model_matches': matches and image_model_ok,
                     'raw_pose_association': raw_association, 'replay_pose_association': replay_association,
                     'result': response, 'candidate_eligible': False, 'reason': 'passive_fullbag_audit_no_global_submission'}
            counters['processed_images'] += 1
            if not response['success']:
                counters['detector_errors'] += 1
            timing.append(response.get('processing_ms'))
            detections = response.get('observations', [])
            if detections:
                counters['images_with_detections'] += 1
                positive_indices.append(index)
                metric_timing.append(response.get('processing_ms'))
            key_counts = Counter((o['family'], o['id']) for o in detections)
            for observation in detections:
                key = observation['family']+':'+str(observation['id'])
                identities[key] += 1
                reasons = []
                if key_counts[(observation['family'], observation['id'])] > 1:
                    reasons.append('same_image_duplicate_physical_id')
                if not matches or not image_model_ok:
                    reasons.append('camera_model_not_bound')
                if observation['hamming'] > config.get('max_hamming', 0):
                    reasons.append('hamming')
                if observation['decision_margin'] < config.get('min_margin', 0.):
                    reasons.append('decision_margin')
                if observation['quality']['area_pixels'] < config.get('min_area', 16.):
                    reasons.append('area')
                if observation['quality']['border_distance_pixels'] < 0:
                    reasons.append('corner_outside_image')
                if observation['size_m'] <= 0:
                    reasons.append('unknown_metric_size')
                hypotheses = observation['hypotheses']
                eligible = [h['hypothesis_id'] for h in hypotheses if h['pixel_rmse'] <= config.get('pixel_rmse', 3.)
                            and h['pixel_max'] <= config.get('pixel_max', 6.)]
                if not eligible:
                    reasons.append('no_hypothesis_passes_pixel_limits')
                counters['observations'] += 1
                counters['observations_with_'+str(len(hypotheses))+'_hypotheses'] += 1
                if not reasons:
                    counters['metric_quality_eligible_observations'] += 1
                counters['quality_eligible_branches'] += len(eligible) if not reasons else 0
                for h in hypotheses:
                    hypothesis_errors.append(h['pixel_rmse'])
                local_hypotheses = []
                if raw_association['available']:
                    X = matrix(raw_association['pose'])
                    for h in hypotheses:
                        local_hypotheses.append({'hypothesis_id': h['hypothesis_id'],
                                                 'T_raw_local_tag': pose_json(X@E@matrix(h['T_camera_tag']))})
                row = {'image_index': index, 'message_id': message_id, 'observation': observation,
                       'metric_quality_eligible': not reasons, 'quality_reasons': reasons,
                       'eligible_hypothesis_ids': eligible, 'raw_pose_association': raw_association,
                       'replay_pose_association': replay_association,
                       'conditional_local_tag_hypotheses': local_hypotheses,
                       'time_assumption': 'effective=header+configured_offset; exposure phase unverified',
                       'physical_time_verified': False, 'fixed_deployment_validated': False,
                       'candidate_eligible': False, 'global_pose_submitted': False}
                old = previous.get(key)
                if old and local_hypotheses and 0 < effective-old['stamp_ns'] <= 1_000_000_000:
                    combinations = []
                    for before_h in old['hypotheses']:
                        a = matrix(before_h['T_raw_local_tag'])
                        for after_h in local_hypotheses:
                            b = matrix(after_h['T_raw_local_tag'])
                            combinations.append({'from_hypothesis': before_h['hypothesis_id'], 'to_hypothesis': after_h['hypothesis_id'],
                                                 'translation_delta_m': float(np.linalg.norm(a[:3, 3]-b[:3, 3])),
                                                 'rotation_delta_rad': float((Rotation.from_matrix(a[:3, :3]).inv()*Rotation.from_matrix(b[:3, :3])).magnitude())})
                    transitions_stream.write(json.dumps({'family_id': key, 'previous_image': old['image_index'], 'image_index': index,
                                                        'interval_ns': effective-old['stamp_ns'], 'combinations': combinations,
                                                        'branch_selected': False, 'fixedness_or_accuracy_proven': False})+'\n')
                previous[key] = {'stamp_ns': effective, 'image_index': index, 'hypotheses': local_hypotheses}
                observations_stream.write(json.dumps(row)+'\n')
                rows.append(row)
            frames_stream.write(json.dumps(frame)+'\n')
            if index % 100 == 0 or index+1 == total:
                frames_stream.flush()
                print(json.dumps({'processed': index+1, 'total': total, 'detections': counters['observations'],
                                  'wall_seconds': time.monotonic()-start}), flush=True)
        process.stdin.close()
        detector_exit = process.wait(timeout=15)
        if detector_exit != 0:
            raise RuntimeError('detector exited '+str(detector_exit))
    finally:
        for stream in [frames_stream, observations_stream, transitions_stream]:
            stream.close()
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        detector_log.close()
        connection.close()

    intervals = []
    for index in positive_indices:
        if not intervals or index > intervals[-1]['last_image_index']+1:
            intervals.append({'first_image_index': index, 'last_image_index': index,
                              'first_stamp_ns': sync_stamps[index], 'last_stamp_ns': sync_stamps[index], 'frames': 1})
        else:
            intervals[-1].update(last_image_index=index, last_stamp_ns=sync_stamps[index], frames=intervals[-1]['frames']+1)
    after_stat = args.bag.stat()
    after = {'bytes': after_stat.st_size, 'mtime_ns': after_stat.st_mtime_ns, 'sha256': sha_file(args.bag)}
    integrity = all(before[key] == after[key] for key in ['bytes', 'mtime_ns', 'sha256'])
    summary = {'schema_version': 1, 'kind': 'passive_apriltag_fullbag_audit', 'source_before': before, 'source_after': after,
               'source_sha256_unchanged': integrity, 'hardware_started': False, 'ros_graph_started': False,
               'source_open_mode': 'SQLite mode=ro and PRAGMA query_only=ON', 'detector_argv': command,
               'detector_exit_code': detector_exit, 'detector_sha256': sha_file(args.detector),
               'config_sha256': config_sha, 'sizes_sha256': sizes_sha, 'measured_sizes': sizes,
               'configured_time_verified': config['time_verified'], 'raw_image_count': raw_count,
               'camera_model_versions': len(models), 'raw_pose_history': history.summary(),
               'replay_pose_history': replay_history.summary(), 'replay_identity': native_identity,
               'counts': dict(counters), 'family_ids': dict(identities), 'pose_associations': dict(associations),
               'sync_stamp_strictly_monotonic': all(b > a for a, b in zip(sync_stamps, sync_stamps[1:])),
               'detection_intervals': intervals, 'detector_ms': quantiles(timing), 'positive_frame_detector_ms': quantiles(metric_timing),
               'all_hypothesis_pixel_rmse': quantiles(hypothesis_errors), 'processing_wall_seconds': time.monotonic()-start,
               'absolute_accuracy_validated': False, 'physical_synchronization_validated': False,
               'cross_session_validated': False, 'real_VALID_landmark_created': False,
               'global_localization_or_loop_submitted': False,
               'limitations': ['No frame-by-frame truth labels: detection frequency is not recall or false-positive rate',
                               'Header-clock interpolation is not physical exposure synchronization',
                               'Raw and replay pose coordinate systems are kept separate',
                               'Conditional local Tag transforms do not establish fixedness or map accuracy']}
    summary['passed'] = (integrity and counters['processed_images'] == total and not counters['detector_errors']
                         and counters['unique_raw_copy_matches'] == total and counters['camera_model_matches'] == total)
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    manifest = {'schema_version': 1, 'files': {str(p.relative_to(args.output)): sha_file(p) for p in args.output.iterdir() if p.is_file()}}
    (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    if summary['passed']:
        (args.output/'COMMITTED').write_text(json.dumps({'manifest_sha256': sha_file(args.output/'manifest.json')})+'\n')
    else:
        (args.output/'FAILED.json').write_text(json.dumps({'reason': 'one or more fullbag integrity/model checks failed'})+'\n')
    print(json.dumps({'passed': summary['passed'], 'counts': summary['counts'], 'family_ids': summary['family_ids']}), flush=True)
    return 0 if summary['passed'] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--bag', type=Path, default=ROOT/'bags/fast-livo-motion-w5phdosj/raw/raw_0.db3')
    parser.add_argument('--native', type=Path, default=ROOT/'outputs/fast-livo-demo-20261028-fHJTbGCU/frontend/native_frames/native_frames_0.db3')
    parser.add_argument('--config', type=Path, default=ROOT/'config/fast_livo2/apriltag_camera.json')
    parser.add_argument('--sizes', type=Path, default=ROOT/'config/fast_livo2/apriltag_sizes.json')
    parser.add_argument('--detector', type=Path, default=ROOT/'ws_fast_livo/backend_install/ranger_fast_livo_backend/lib/ranger_fast_livo_backend/tag_image_tool')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output = args.output.resolve()
    if not args.execute:
        print(json.dumps({'action': 'DRY-RUN', 'bag': str(args.bag), 'output': str(args.output), 'all_images': True,
                          'hardware_started': False, 'ros_graph_started': False}))
        return 0
    try:
        return run(args)
    except Exception as error:
        if args.output.is_dir() and not (args.output/'COMMITTED').exists():
            (args.output/'FAILED.json').write_text(json.dumps({'reason': type(error).__name__+': '+str(error)})+'\n')
        raise


if __name__ == '__main__':
    raise SystemExit(main())
