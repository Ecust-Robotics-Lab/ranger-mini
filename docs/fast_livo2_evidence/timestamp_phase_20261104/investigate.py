#!/usr/bin/env python3
"""Read-only capture/arrival audit. Arrival delays never identify exposure time."""
import argparse
import bisect
import csv
import hashlib
import json
from pathlib import Path
import sqlite3
import struct

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8*1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def quantiles(values):
    a = np.asarray(values, dtype=float)
    return dict(count=len(a), minimum=float(a.min()), p50=float(np.median(a)),
                p95=float(np.percentile(a, 95)), maximum=float(a.max()))


def header(prefix):
    assert prefix[:4] == b'\x00\x01\x00\x00', 'unexpected CDR encoding'
    sec, ns = struct.unpack_from('<iI', prefix, 4)
    assert 0 <= ns < 10**9
    return sec*10**9+ns


def audit(name):
    folder = ROOT/'bags'/name
    db = folder/'raw/raw_0.db3'
    before = db.stat()
    digest = sha(db)
    con = sqlite3.connect(db.resolve().as_uri()+'?mode=ro', uri=True)
    con.execute('PRAGMA query_only=ON')
    topics = con.execute('SELECT id,name FROM topics').fetchall()
    records = {}
    for tid, topic in topics:
        records[topic] = [(mid, receive, header(prefix)) for mid, receive, prefix in
                         con.execute('SELECT id,timestamp,substr(data,1,32) FROM messages WHERE topic_id=? ORDER BY id', (tid,))]
    imu_delays = [received-stamp for _, received, stamp in records['/livox/imu']]
    anchor = imu_delays[0]
    imu_reference = anchor+int(np.median([v-anchor for v in imu_delays]))
    result = {'capture': name, 'db_sha256': digest, 'db_bytes': before.st_size,
              'db_mtime_ns': before.st_mtime_ns, 'imu_record_minus_header_reference_ns': imu_reference,
              'arrival_is_not_exposure_time': True, 'physical_time_verified': False, 'topics': {}}
    for topic, rows in records.items():
        result['topics'][topic] = {'messages': len(rows), 'first_header_ns': rows[0][2],
            'first_record_ns': rows[0][1], 'strictly_increasing_headers': all(b[2] > a[2] for a, b in zip(rows, rows[1:])),
            'record_lag_relative_to_imu_reference_ms': quantiles([(received-stamp-imu_reference)*1e-6 for _, received, stamp in rows])}
    tid = next(tid for tid, topic in topics if topic == '/livox/lidar')
    spans, first_offsets = [], []
    for data, in con.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY id', (tid,)):
        msg = deserialize_message(data, PointCloud2)
        field = next(f for f in msg.fields if f.name == 'timestamp')
        assert field.datatype == 8 and field.count == 1 and not msg.is_bigendian
        times = np.ndarray((msg.height, msg.width), dtype='<f8', buffer=msg.data,
                           offset=field.offset, strides=(msg.row_step, msg.point_step))
        assert times.size and np.isfinite(times).all()
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        first_offsets.append(int(times.min())-stamp)
        spans.append((int(times.max())-stamp)*1e-6)
    result['scan_last_point_minus_header_ms'] = quantiles(spans)
    result['scan_first_point_minus_header_ns'] = quantiles(first_offsets)
    con.close()
    with (folder/'sensor_assets/camera_sync.csv').open() as stream:
        cameras = list(csv.DictReader(stream))
    with (folder/'sensor_assets/timeshare_writer.csv').open() as stream:
        writers = list(csv.DictReader(stream))
    arrivals = [int(r['arrival_ns']) for r in writers]
    assert all(b > a for a, b in zip(arrivals, arrivals[1:]))
    writers_by_seq = {2*int(r['seq']): r for r in writers}
    ages, sequence_deltas = [], []
    for camera in cameras:
        writer = writers_by_seq[int(camera['ts_seq'])]
        assert int(writer['scan_start_ns']) == int(camera['ts_low_ns'])
        j = bisect.bisect_right(arrivals, int(camera['arrival_ns']))-1
        assert j >= 0
        sequence_deltas.append(int(camera['ts_seq'])//2-int(writers[j]['seq']))
        ages.append((int(camera['arrival_ns'])-int(writer['arrival_ns']))*1e-6)
    result['camera_csv_frames'] = len(cameras)
    result['camera_csv_statuses'] = {status: sum(r['status'] == status for r in cameras) for status in sorted({r['status'] for r in cameras})}
    result['camera_consumed_minus_latest_writer_sequence'] = {str(v): sequence_deltas.count(v) for v in sorted(set(sequence_deltas))}
    result['writer_age_at_camera_sdk_arrival_ms'] = quantiles(ages)
    result['camera_exposure_us_from_session_log'] = 20000
    assert 'exposure=20000.0 us' in (folder/'session_logs/sensors/camera.log').read_text()
    assert db.stat().st_mtime_ns == before.st_mtime_ns and db.stat().st_size == before.st_size
    assert sha(db) == digest, 'original bag changed'
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute', action='store_true')
    args = p.parse_args()
    if not args.execute:
        print('DRY-RUN: read two original bags; write new evidence only')
        return
    captures = [('turn', 'fast-livo-tag140_turn_03-bk8_pjyy'), ('straight', 'fast-livo-tag140_straight_04-ryzykq2l')]
    results = []
    for key, name in captures:
        result = audit(name)
        with (HERE/(key+'_timing.json')).open('x') as stream:
            json.dump(result, stream, indent=2, ensure_ascii=False)
            stream.write('\n')
        results.append(result)
        print(json.dumps({'capture': name, 'scan_span_ms': result['scan_last_point_minus_header_ms'],
                          'relative_lidar_arrival_ms': result['topics']['/livox/lidar']['record_lag_relative_to_imu_reference_ms'],
                          'relative_sync_image_arrival_ms': result['topics']['/front/image_sync']['record_lag_relative_to_imu_reference_ms'],
                          'latest_writer': result['camera_consumed_minus_latest_writer_sequence']}, ensure_ascii=False), flush=True)
    with (HERE/'timing_summary.json').open('x') as stream:
        json.dump(results, stream, indent=2, ensure_ascii=False)
        stream.write('\n')


if __name__ == '__main__':
    main()
