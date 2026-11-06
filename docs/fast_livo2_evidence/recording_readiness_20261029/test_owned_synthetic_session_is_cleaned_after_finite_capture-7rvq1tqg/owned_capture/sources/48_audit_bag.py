#!/usr/bin/env python3
"""Read-only image/model/time inventory and bounded detector sampling; no ROS graph."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute', action='store_true')
    p.add_argument('--bag', type=Path, default=ROOT/'bags/fast-livo-motion-w5phdosj/raw/raw_0.db3')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--samples', type=int, default=12)
    p.add_argument('--sizes-json', default='{}', help='Explicit measured tag boundary sizes in metres')
    args = p.parse_args()
    if not args.execute:
        print(json.dumps({'action': 'DRY-RUN', 'bag': str(args.bag), 'output': str(args.output), 'samples': args.samples}))
        return 0
    if not 1 <= args.samples <= 100:
        p.error('samples must be 1..100')
    import cv2
    import numpy as np
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image, CameraInfo
    from nav_msgs.msg import Odometry
    args.output.mkdir(parents=True, exist_ok=False)
    before = args.bag.stat()
    connection = sqlite3.connect(args.bag.resolve().as_uri()+'?mode=ro', uri=True)
    topics = {name: (topic, kind) for topic, name, kind in connection.execute('SELECT id,name,type FROM topics')}
    result = {'hardware_started': False, 'raw_open_mode': 'sqlite mode=ro', 'bag': str(args.bag.resolve()),
              'topics': {}, 'detector_samples': [], 'scope': 'sampled image inventory, not labeled recall or absolute pose acceptance'}
    for name, (topic, kind) in topics.items():
        count = connection.execute('SELECT count(*) FROM messages WHERE topic_id=?', (topic,)).fetchone()[0]
        result['topics'][name] = {'type': kind, 'count': count}

    def stamp(message):
        return message.header.stamp.sec*10**9+message.header.stamp.nanosec

    def first(name, cls):
        row = connection.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY id LIMIT 1', (topics[name][0],)).fetchone()
        return deserialize_message(row[0], cls)

    info = first('/front/camera_info', CameraInfo)
    image = first('/front/image_sync', Image)
    raw = first('/front/image', Image)
    result['first_headers'] = {'camera_info_ns': stamp(info), 'raw_image_ns': stamp(raw), 'sync_image_ns': stamp(image),
                               'sync_minus_info_ns': stamp(image)-stamp(info)}
    result['image'] = {'width': image.width, 'height': image.height, 'encoding': image.encoding,
                       'step': image.step, 'frame': image.header.frame_id}
    result['camera_info'] = {'width': info.width, 'height': info.height, 'distortion_model': info.distortion_model,
                            'K': list(info.k), 'D': list(info.d), 'R': list(info.r), 'P': list(info.p),
                            'binning_x': info.binning_x, 'binning_y': info.binning_y,
                            'roi': {'x': info.roi.x_offset, 'y': info.roi.y_offset, 'width': info.roi.width, 'height': info.roi.height}}
    stamps = [stamp(deserialize_message(data, Odometry)) for (data,) in connection.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY id', (topics['/fast_livo/odometry'][0],))]
    diffs = np.diff(np.asarray(stamps, dtype=np.int64))
    result['odometry'] = {'count': len(stamps), 'first_stamp_ns': stamps[0], 'last_stamp_ns': stamps[-1],
                          'median_interval_ns': int(np.median(diffs)), 'nonpositive_intervals': int(np.sum(diffs <= 0))}
    image_topic = topics['/front/image_sync'][0]
    ids = [row[0] for row in connection.execute('SELECT id FROM messages WHERE topic_id=? ORDER BY id', (image_topic,))]
    indices = sorted(set(int(i) for i in np.linspace(0, len(ids)-1, args.samples)))
    tool = ROOT/'ws_fast_livo/backend_install/ranger_fast_livo_backend/lib/ranger_fast_livo_backend/tag_image_tool'
    for index in indices:
        (data,) = connection.execute('SELECT data FROM messages WHERE id=?', (ids[index],)).fetchone()
        message = deserialize_message(data, Image)
        channels = 1 if message.encoding == 'mono8' else 3
        values = np.frombuffer(bytes(message.data), dtype=np.uint8).reshape(message.height, message.step)
        pixels = values[:, :message.width*channels].reshape(message.height, message.width, channels)
        if message.encoding == 'rgb8':
            pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
        elif message.encoding not in ['mono8', 'bgr8']:
            raise ValueError('unsupported audit image encoding '+message.encoding)
        filename = args.output/f'image_{index}.png'
        if not cv2.imwrite(str(filename), pixels):
            raise OSError('audit image write failed')
        command = [str(tool), str(ROOT/'config/fast_livo2/apriltag_camera.json'), str(filename), str(stamp(message)), 'tag36h11', args.sizes_json]
        completed = subprocess.run(command, capture_output=True, text=True, timeout=15)
        row = {'image_index': index, 'message_id': ids[index], 'stamp_ns': stamp(message), 'message_sha256': hashlib.sha256(data).hexdigest(),
               'argv': command, 'exit_code': completed.returncode, 'stderr': completed.stderr}
        if completed.returncode == 0:
            row['result'] = json.loads(completed.stdout)
        result['detector_samples'].append(row)
    connection.close()
    after = args.bag.stat()
    result['raw_stat_unchanged'] = (before.st_size, before.st_mtime_ns) == (after.st_size, after.st_mtime_ns)
    result['detected_tags_in_samples'] = sum(len(row.get('result', {}).get('observations', [])) for row in result['detector_samples'])
    result['independent_tag_dataset_available'] = False
    (args.output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({key: result[key] for key in ['raw_stat_unchanged', 'detected_tags_in_samples', 'first_headers', 'odometry']}, indent=2))
    return 0 if result['raw_stat_unchanged'] and all(row['exit_code'] == 0 for row in result['detector_samples']) else 1


if __name__ == '__main__':
    raise SystemExit(main())
