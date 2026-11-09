#!/usr/bin/env python3
"""Summarize existing extraction, recorded sync logs and bounded image evidence."""
import argparse
from collections import Counter, defaultdict
import csv
import importlib.util
import json
from pathlib import Path
import sqlite3

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('tag_extraction', Path(__file__).with_name('extract_bag.py'))
extraction = importlib.util.module_from_spec(spec)
spec.loader.exec_module(extraction)


def read_lines(path):
    with Path(path).open() as stream:
        return [json.loads(line) for line in stream]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--extraction', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--capture', type=Path, default=ROOT/'bags/fast-livo-motion-w5phdosj')
    args = parser.parse_args()
    if not args.execute:
        print(json.dumps({'action': 'DRY-RUN', 'extraction': str(args.extraction), 'output': str(args.output)}))
        return 0
    args.output.mkdir(parents=True, exist_ok=False)
    frames = read_lines(args.extraction/'frames.jsonl')
    observations = read_lines(args.extraction/'observations.jsonl')
    transitions = read_lines(args.extraction/'hypothesis_transitions.jsonl')
    summary = json.loads((args.extraction/'summary.json').read_text())
    with (args.capture/'sensor_assets/camera_sync.csv').open() as stream:
        camera_rows = list(csv.DictReader(stream))
    with (args.capture/'sensor_assets/timeshare_writer.csv').open() as stream:
        writer_rows = list(csv.DictReader(stream))
    cameras, writers = defaultdict(list), defaultdict(list)
    for row in camera_rows:
        if row['status'] == 'OK':
            cameras[int(row['ts_low_ns'])].append(row)
    for row in writer_rows:
        writers[int(row['scan_start_ns'])].append(row)

    timing_rows, combined = [], []
    expected_sequence, unique_csv = 0, 0
    for frame in frames:
        ns = frame['image_stamp_ns']
        camera = cameras[ns][0] if len(cameras[ns]) == 1 else None
        writer = writers[ns][0] if len(writers[ns]) == 1 else None
        row = {'image_index': frame['image_index'], 'stamp_ns': ns, 'camera_csv_matches': len(cameras[ns]),
               'writer_csv_matches': len(writers[ns]), 'physical_exposure_phase_proven': False}
        if camera and writer:
            unique_csv += 1
            relation = int(camera['ts_seq']) == 2*int(writer['seq'])
            expected_sequence += relation
            row.update(camera_frame_id=int(camera['frame_id']), device_timestamp_ticks=int(camera['dev_ts_ticks']),
                       camera_sequence=int(camera['ts_seq']), writer_sequence=int(writer['seq']), v2_sequence_relation_ok=relation,
                       camera_arrival_after_writer_ms=(int(camera['arrival_ns'])-int(writer['arrival_ns']))*1e-6,
                       camera_publish_after_arrival_ms=(int(camera['publish_ns'])-int(camera['arrival_ns']))*1e-6)
        if frame['raw_copy_matches']:
            raw = frame['raw_copy_matches'][0]
            row['bag_record_after_raw_header_ms'] = (frame['record_ns']-raw['stamp_ns'])*1e-6
        raw_stamps = summary['raw_pose_history']
        row['seconds_from_first_image'] = (ns-frames[0]['image_stamp_ns'])*1e-9
        timing_rows.append(row)
        combined.append({'image_index': frame['image_index'], 'stamp_ns': ns, 'detections': len(frame['result']['observations']),
                         'raw_pose_association': frame['raw_pose_association'].get('kind', frame['raw_pose_association'].get('reason')),
                         'replay_pose_association': frame['replay_pose_association'].get('kind', frame['replay_pose_association'].get('reason'))})
    with (args.output/'time_associations.jsonl').open('w') as stream:
        for row in timing_rows:
            stream.write(json.dumps(row)+'\n')
    with (args.output/'frame_table.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(combined[0]))
        writer.writeheader()
        writer.writerows(combined)

    positive_indices = [row['image_index'] for row in observations]
    holes = sorted(set(range(min(positive_indices), max(positive_indices)+1))-set(positive_indices)) if positive_indices else []
    pixel_rows = []
    for row in observations:
        observation = row['observation']
        for hypothesis in observation['hypotheses']:
            pixel_rows.append({'image_index': row['image_index'], 'family': observation['family'], 'id': observation['id'],
                               'hypothesis_id': hypothesis['hypothesis_id'], 'pixel_rmse': hypothesis['pixel_rmse'],
                               'pixel_max': hypothesis['pixel_max'], 'margin': observation['decision_margin'],
                               'area_pixels': observation['quality']['area_pixels'],
                               'camera_distance_m': float(np.linalg.norm(hypothesis['T_camera_tag']['xyz'])),
                               'quality_eligible': hypothesis['hypothesis_id'] in row['eligible_hypothesis_ids']})
    with (args.output/'hypothesis_quality.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(pixel_rows[0]) if pixel_rows else ['image_index'])
        writer.writeheader()
        writer.writerows(pixel_rows)

    transition_summary = {}
    for key in ['translation_delta_m', 'rotation_delta_rad']:
        # All four pairs remain in the JSONL. This minimum is descriptive only,
        # not a branch label, selection, fixed-landmark or accuracy criterion.
        minimums = [min(c[key] for c in row['combinations']) for row in transitions if row['combinations']]
        transition_summary['minimum_over_all_pairs_'+key] = extraction.quantiles(minimums)
    analysis = {'schema_version': 1, 'extraction_summary_sha256': extraction.sha_file(args.extraction/'summary.json'),
                'csv_sources_sha256': {str(args.capture/'sensor_assets'/name): extraction.sha_file(args.capture/'sensor_assets'/name)
                                       for name in ['camera_sync.csv', 'timeshare_writer.csv']},
                'all_frames': len(frames), 'images_with_detections': summary['counts'].get('images_with_detections', 0),
                'detection_frame_fraction': summary['counts'].get('images_with_detections', 0)/len(frames),
                'detection_fraction_is_not_recall': True, 'indices_without_detection_inside_positive_window': holes,
                'positive_window_seconds': (max(frames[i]['image_stamp_ns'] for i in positive_indices)-min(frames[i]['image_stamp_ns'] for i in positive_indices))*1e-9 if positive_indices else None,
                'positive_raw_pose_available': sum(row['raw_pose_association']['available'] for row in observations),
                'positive_replay_pose_available': sum(row['replay_pose_association']['available'] for row in observations),
                'observations_passing_margin_50': sum(row['observation']['decision_margin'] > 50 for row in observations),
                'decision_margin': extraction.quantiles([row['observation']['decision_margin'] for row in observations]),
                'pixel_rmse_by_per_frame_hypothesis_index': {str(i): extraction.quantiles([p['pixel_rmse'] for p in pixel_rows if p['hypothesis_id'] == i]) for i in [0, 1]},
                'camera_range_m': extraction.quantiles([row['camera_distance_m'] for row in pixel_rows]),
                'camera_csv_status_counts': dict(Counter(row['status'] for row in camera_rows)),
                'unique_camera_and_writer_csv_matches': unique_csv, 'v2_sequence_relation_matches': expected_sequence,
                'camera_arrival_after_writer_ms': extraction.quantiles([row.get('camera_arrival_after_writer_ms') for row in timing_rows]),
                'camera_publish_after_arrival_ms': extraction.quantiles([row.get('camera_publish_after_arrival_ms') for row in timing_rows]),
                'bag_record_after_raw_header_ms': extraction.quantiles([row.get('bag_record_after_raw_header_ms') for row in timing_rows]),
                'conditional_local_transitions': transition_summary, 'branch_selection_performed': False,
                'physical_synchronization_validated': False, 'fixed_deployment_validated': False,
                'absolute_accuracy_validated': False, 'real_VALID_landmark_created': False,
                'limitations': ['CSV arrival/publish times describe software timing, not exposure or trigger-edge metrology',
                                'IPPE hypothesis indices are local to each image, not persistent branch identities',
                                'Small minimum transition error over alternative pairs is not proof of the correct branch']}

    # Preserve at most six original images as diagnostic evidence, without overlays.
    if positive_indices:
        selected = sorted(set([positive_indices[0], positive_indices[len(positive_indices)//2], positive_indices[-1], *holes[:3]]))[:6]
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Image
        import cv2
        connection = extraction.open_readonly(args.capture/'raw/raw_0.db3')
        (args.output/'images').mkdir()
        for index in selected:
            row = frames[index]
            (data,) = connection.execute('SELECT data FROM messages WHERE id=?', (row['message_id'],)).fetchone()
            message = deserialize_message(data, Image)
            values = np.frombuffer(bytes(message.data), dtype=np.uint8).reshape(message.height, message.step)
            image = values[:, :message.width*3].reshape(message.height, message.width, 3)
            if message.encoding == 'rgb8':
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            elif message.encoding != 'bgr8':
                raise ValueError('unsupported preview encoding')
            if not cv2.imwrite(str(args.output/'images'/f'image_{index}.png'), image):
                raise OSError('preview image write failed')
        connection.close()
        analysis['preserved_original_image_indices'] = selected

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    first = frames[0]['image_stamp_ns']
    seconds = [(row['image_stamp_ns']-first)*1e-9 for row in frames]
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), constrained_layout=True)
    axes[0].plot(seconds, [len(row['result']['observations']) for row in frames], linewidth=.8)
    axes[0].set(ylabel='Detections per image', title='Full capture: passive Tag36h11 extraction (1367 images)')
    for h in [0, 1]:
        points = [p for p in pixel_rows if p['hypothesis_id'] == h]
        axes[1].plot([(frames[p['image_index']]['image_stamp_ns']-first)*1e-9 for p in points], [p['pixel_rmse'] for p in points], '.-', label=f'Per-image hypothesis {h}')
    axes[1].axhline(3., color='gray', linestyle='--', label='Current trial RMS limit')
    axes[1].set(ylabel='Raw-pixel reprojection RMS (px)')
    axes[1].legend(loc='upper right')
    axes[2].plot(seconds, [row.get('camera_arrival_after_writer_ms', np.nan) for row in timing_rows], linewidth=.7)
    axes[2].set(xlabel='Seconds from first sync image header', ylabel='Camera arrival - writer arrival (ms)',
                title='Recorded software timing only; physical exposure phase unverified')
    for axis in axes:
        axis.grid(alpha=.25)
    fig.savefig(args.output/'overview.png', dpi=160)
    plt.close(fig)
    (args.output/'analysis.json').write_text(json.dumps(analysis, indent=2)+'\n')
    print(json.dumps(analysis, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
