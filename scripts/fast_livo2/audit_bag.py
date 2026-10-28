#!/usr/bin/env python3
"""Audit original SQLite bag headers/point times without rewriting messages."""
import argparse
import json
import sqlite3
import struct
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, Image, Imu


def header(data):
    if data[:4] != b'\x00\x01\x00\x00':
        raise ValueError('Only little-endian CDR is supported')
    sec, ns, length = struct.unpack_from('<iII', data, 4)
    if not 0 <= ns < 10**9 or length > len(data)-16:
        raise ValueError('Invalid CDR header')
    return sec*10**9+ns, data[16:16+length-1].decode()


def audit(bag):
    dbs = sorted(bag.glob('*.db3'))
    if len(dbs) != 1:
        raise ValueError('This audit requires one SQLite database')
    with sqlite3.connect(dbs[0].resolve().as_uri()+'?mode=ro', uri=True) as conn:
        topics = conn.execute('SELECT id,name,type FROM topics').fetchall()
        report = {'bag': str(bag.resolve()), 'topics': {}, 'physical_sync_proven': False}
        times = {}
        for topic_id, name, kind in topics:
            values, frames, delays = [], set(), []
            # Read just the CDR header even for multi-megabyte images.
            for stamp, prefix in conn.execute('SELECT timestamp,substr(data,1,256) FROM messages WHERE topic_id=? ORDER BY timestamp,id', (topic_id,)):
                t, frame = header(prefix)
                values.append(t); frames.add(frame); delays.append(stamp-t)
            arr = np.array(values, dtype=np.int64)
            delta = np.diff(arr)
            times[name] = set(values)
            report['topics'][name] = {'type': kind, 'count': len(values), 'frames': sorted(frames),
                'first_header_ns': values[0] if values else None,
                'last_header_ns': values[-1] if values else None,
                'nonincreasing': int(np.count_nonzero(delta <= 0)),
                'period_ns_p0_p50_p100': np.percentile(delta,[0,50,100]).tolist() if len(delta) else [],
                'record_minus_header_ns_p0_p50_p100': np.percentile(delays,[0,50,100]).tolist() if delays else []}
            if kind == 'sensor_msgs/msg/PointCloud2' and name == '/livox/lidar':
                total = negative = nonfinite = reversed_steps = last_not_max = 0
                minima, maxima = [], []
                for (data,) in conn.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp,id',(topic_id,)):
                    msg = deserialize_message(data, PointCloud2)
                    fields = {f.name:f for f in msg.fields}
                    if fields['timestamp'].datatype != 8 or msg.is_bigendian:
                        raise ValueError('Unexpected point timestamp format')
                    points = np.ndarray(shape=(msg.height,msg.width),dtype='<f8',buffer=msg.data,
                                        offset=fields['timestamp'].offset,strides=(msg.row_step,msg.point_step)).reshape(-1)
                    if not len(points):
                        raise ValueError('Empty Livox cloud')
                    offsets = points-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec)
                    total += len(offsets); negative += int(np.count_nonzero(offsets < 0))
                    nonfinite += int(np.count_nonzero(~np.isfinite(offsets)))
                    reversed_steps += int(np.count_nonzero(np.diff(offsets)<0))
                    last_not_max += int(offsets[-1] != offsets.max())
                    minima.append(float(offsets.min())); maxima.append(float(offsets.max()))
                report['point_times'] = {'encoding':'absolute nanoseconds FLOAT64', 'total':total,
                    'before_header':negative,'nonfinite':nonfinite,'backwards_steps_inside_scans':reversed_steps,
                    'scans_last_point_not_latest':last_not_max,
                    'min_offset_ns':min(minima) if minima else None,'max_offset_ns':max(maxima) if maxima else None,
                    'precision_note':'FLOAT64 epoch ns has approximately 256 ns spacing; input unchanged'}
            elif kind == 'sensor_msgs/msg/Image':
                row=conn.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp,id LIMIT 1',(topic_id,)).fetchone()
                if row:
                    msg=deserialize_message(row[0],Image)
                    report['topics'][name]['image']={'width':msg.width,'height':msg.height,'encoding':msg.encoding,'step':msg.step}
            elif kind == 'sensor_msgs/msg/Imu':
                acc=[]; gyro=[]; nonfinite = 0
                for (data,) in conn.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp,id',(topic_id,)):
                    msg=deserialize_message(data,Imu)
                    values = [msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z,
                              msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z]
                    nonfinite += int(not np.isfinite(values).all())
                    acc.append(np.linalg.norm([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]))
                    gyro.append(np.linalg.norm([msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z]))
                report['imu']={'acceleration_norm_p0_p50_p100':np.percentile(acc,[0,50,100]).tolist() if acc else [],
                               'angular_speed_p0_p50_p100':np.percentile(gyro,[0,50,100]).tolist() if gyro else [],
                               'nonfinite_messages': nonfinite}
        report['exact_image_lidar_headers']=len(times.get('/front/image_sync',set()) & times.get('/livox/lidar',set()))
        report['interpretation']='Header equality verifies copied timestamps only; physical trigger/scan/exposure phase remains unproven.'
    return report


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag',type=Path);p.add_argument('--out',type=Path,required=True)
    args=p.parse_args()
    result=audit(args.bag)
    with args.out.open('x') as f:
        json.dump(result,f,ensure_ascii=False,indent=2);f.write('\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))
