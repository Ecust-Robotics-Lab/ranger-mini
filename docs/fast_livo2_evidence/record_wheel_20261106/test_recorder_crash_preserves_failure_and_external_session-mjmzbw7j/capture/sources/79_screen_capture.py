#!/usr/bin/env python3
"""Read-only capture screening; no ROS graph, map registration or replay trajectory."""
import argparse
from collections import Counter
import csv
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import time

import cv2
import numpy as np

spec = importlib.util.spec_from_file_location('tag_extract', Path(__file__).with_name('extract_bag.py'))
common = importlib.util.module_from_spec(spec)
spec.loader.exec_module(common)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--sizes', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not args.execute:
        print(json.dumps({'action': 'DRY-RUN', 'capture': str(args.capture), 'output': str(args.output), 'hardware_started': False}))
        return 0
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image, CameraInfo
    from nav_msgs.msg import Odometry

    args.output.mkdir(parents=True, exist_ok=False)
    capture = args.capture.resolve()
    manifest = json.loads((capture/'manifest.json').read_text())
    assert (capture/'CAPTURE_COMPLETE').is_file(), 'capture not finalized'
    # Only archived source bytes are used as the camera model.
    for row in manifest['source_files']:
        assert common.sha_file(capture/row['file']) == row['sha256'], 'source snapshot changed: '+row['file']
    archived = next(row for row in manifest['source_files'] if row['source'].endswith('/apriltag_camera.json'))
    config_path = capture/archived['file']
    config = json.loads(config_path.read_text())
    sizes = json.loads(args.sizes.read_text())
    assert any(row['sha256'] == common.sha_file(args.sizes) for row in manifest['source_files']), 'size declaration not archived'
    database = capture/'raw/raw_0.db3'
    before = {'sha256': common.sha_file(database), 'bytes': database.stat().st_size, 'mtime_ns': database.stat().st_mtime_ns}
    connection = common.open_readonly(database)
    topics = {name: key for key, name in connection.execute('SELECT id,name FROM topics')}
    count = connection.execute('SELECT count(*) FROM messages WHERE topic_id=?', (topics['/front/image_sync'],)).fetchone()[0]
    poses = common.pose_records(connection, topics['/fast_livo/odometry'], deserialize_message, Odometry)
    history = common.PoseHistory(poses)
    models = [common.camera_model(deserialize_message(data, CameraInfo)) for data, in connection.execute(
        'SELECT data FROM messages WHERE topic_id=? ORDER BY id', (topics['/front/camera_info'],))]
    assert models and all(common.model_matches(model, config['camera']) for model in models), 'recorded CameraInfo mismatch'
    audit = json.loads((capture/'audit.json').read_text())
    first = audit['topics']['/front/image_sync']['first_header_ns']
    last = audit['topics']['/front/image_sync']['last_header_ns']
    csv_rows = list(csv.DictReader((capture/'sensor_assets/camera_sync.csv').open()))
    bad = []
    for i, row in enumerate(csv_rows):
        if row['status'] == 'OK':
            continue
        entry = dict(row)
        entry['from_first_sync_s'] = (int(row['ts_low_ns'])-first)*1e-9
        entry['inside_recorded_sync_stamp_range'] = first <= int(row['ts_low_ns']) <= last
        entry['same_as_previous_shared_stamp'] = i > 0 and row['ts_low_ns'] == csv_rows[i-1]['ts_low_ns']
        entry['same_as_previous_shared_seq'] = i > 0 and row['ts_seq'] == csv_rows[i-1]['ts_seq']
        bad.append(entry)
    root = Path(__file__).resolve().parents[3]
    detector = root/'ws_fast_livo/backend_install/ranger_fast_livo_backend/lib/ranger_fast_livo_backend/tag_image_tool'
    env = dict(os.environ)
    env['LD_LIBRARY_PATH'] = ':'.join([str(root/'ws_fast_livo/backend_install/ranger_fast_livo_backend/lib'),
                                     str(root/'ws_fast_livo/apriltag_deps/lib'),
                                     str(root/'ws_fast_livo/backend_deps/lib'), env.get('LD_LIBRARY_PATH', '')])
    log = (args.output/'detector.log').open('wb')
    process = subprocess.Popen([str(detector), '--stream', str(config_path), sizes['family'], json.dumps(sizes['sizes_m'])],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=log, env=env)
    counts, identities, associations = Counter(), Counter(), Counter()
    margins, residuals, ranges, times, positive_stamps, positive_cameras = [], [], [], [], [], []
    source = 'bag-'+before['sha256']
    begin = time.monotonic()
    try:
        with (args.output/'frames.jsonl').open('w') as frames, (args.output/'observations.jsonl').open('w') as observations:
            for index, (message_id, data) in enumerate(connection.execute('SELECT id,data FROM messages WHERE topic_id=? ORDER BY id', (topics['/front/image_sync'],))):
                message = deserialize_message(data, Image)
                ns = common.stamp(message)
                assert message.width == config['camera']['width'] and message.height == config['camera']['height'] and message.header.frame_id == config['camera']['frame'], 'image model mismatch'
                payload = bytes(message.data)
                request = {'schema_version': 1, 'request_id': index, 'payload_bytes': len(payload), 'width': message.width,
                           'height': message.height, 'step': message.step, 'encoding': message.encoding,
                           'stamp_ns': ns, 'context': source, 'image_id': source+':'+str(message_id),
                           'image_reference': str(database)+'#/front/image_sync/message/'+str(message_id)}
                response = common.request_frame(process, request, payload)
                assert response['success'], response.get('reason')
                association = history.lookup(ns)
                counts['images'] += 1
                detections = response.get('observations', [])
                counts['images_with_detections'] += bool(detections)
                frames.write(json.dumps({'index': index, 'message_id': message_id, 'stamp_ns': ns,
                                         'detections': len(detections), 'local_pose_association': association,
                                         'processing_ms': response.get('processing_ms')})+'\n')
                times.append(response.get('processing_ms'))
                duplicates = Counter((o['family'], o['id']) for o in detections)
                if detections:
                    positive_stamps.append(ns)
                    if association['available']:
                        positive_cameras.append(common.matrix(association['pose'])@common.matrix(config['T_imu_camera']))
                for observation in detections:
                    identities[observation['family']+':'+str(observation['id'])] += 1
                    hypotheses = observation['hypotheses']
                    eligible = [h['hypothesis_id'] for h in hypotheses if h['pixel_rmse'] <= config['pixel_rmse'] and h['pixel_max'] <= config['pixel_max']]
                    reasons = []
                    if duplicates[(observation['family'], observation['id'])] != 1: reasons.append('same_image_duplicate_id')
                    if observation['hamming'] > config['max_hamming']: reasons.append('hamming')
                    if observation['decision_margin'] < config['min_margin']: reasons.append('margin')
                    if observation['quality']['area_pixels'] < config['min_area']: reasons.append('area')
                    if observation['quality']['border_distance_pixels'] < 0: reasons.append('boundary')
                    if observation['size_m'] <= 0: reasons.append('unknown_size')
                    if not eligible: reasons.append('no_hypothesis_passes_pixel_limits')
                    counts['observations'] += 1
                    counts['observations_with_'+str(len(hypotheses))+'_hypotheses'] += 1
                    counts['quality_eligible_observations'] += not reasons
                    associations[association.get('kind', association.get('reason'))] += 1
                    margins.append(observation['decision_margin'])
                    residuals.extend(h['pixel_rmse'] for h in hypotheses)
                    ranges.extend(float(np.linalg.norm(h['T_camera_tag']['xyz'])) for h in hypotheses)
                    observations.write(json.dumps({'index': index, 'stamp_ns': ns, 'observation': observation,
                                                   'eligible_hypothesis_ids': eligible, 'rejection_reasons': reasons,
                                                   'local_pose_association': association})+'\n')
                if index in {0, count//2, count-1}:
                    assert message.encoding == 'rgb8'
                    image = np.ndarray((message.height, message.width, 3), np.uint8, buffer=payload, strides=(message.step, 3, 1))
                    assert cv2.imwrite(str(args.output/f'original_{index}.png'), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        process.stdin.close()
        assert process.wait(timeout=10) == 0, 'detector failed'
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=5)
        log.close()
        connection.close()
    after = {'sha256': common.sha_file(database), 'bytes': database.stat().st_size, 'mtime_ns': database.stat().st_mtime_ns}
    assert after == before, 'source database changed'
    baseline = max((float(np.linalg.norm(a[:3, 3]-b[:3, 3])) for a in positive_cameras for b in positive_cameras), default=0)
    result = {'kind': 'read_only_capture_screening_not_registration', 'capture': str(capture), 'source': before,
              'source_unchanged': True, 'hardware_started': False, 'ros_graph_started': False,
              'camera_model_matches_all_recorded_CameraInfo': True, 'sizes': sizes, 'counts': dict(counts),
              'identities': dict(identities), 'positive_local_pose_associations': dict(associations),
              'positive_time_span_s': (max(positive_stamps)-min(positive_stamps))*1e-9 if positive_stamps else None,
              'maximum_conditional_camera_baseline_m': baseline, 'decision_margin': common.quantiles(margins),
              'hypothesis_pixel_rmse_px': common.quantiles(residuals), 'camera_range_m': common.quantiles(ranges),
              'processing_ms': common.quantiles(times), 'detector_wall_s': time.monotonic()-begin,
              'sync_anomalies': bad, 'physical_synchronization_verified': False, 'real_VALID_landmark_created': False,
              'limitations': ['No old map/C1 or old replay trajectory used', 'Zero-offset local association is conditional, not physically verified',
                              'Detection frame fraction is not recall; pose baseline is not independent truth', 'No fixed-landmark fit or cross-visit recovery evaluated']}
    (args.output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
