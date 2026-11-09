#!/usr/bin/env python3
"""Freeze common query clouds; reference poses are evaluation-only estimates."""
import argparse
import bisect
import json
from pathlib import Path
import sqlite3
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from ranger_fast_livo_localizer.msg import LocalizerEstimate


def ns(m):
    return m.header.stamp.sec*10**9+m.header.stamp.nanosec


def matrix(p):
    a = np.eye(4)
    a[:3, :3] = Rotation.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]).as_matrix()
    a[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return a


def pose(a):
    return {'xyz': a[:3, 3].tolist(), 'xyzw': Rotation.from_matrix(a[:3, :3]).as_quat().tolist()}


def cloud(m):
    offsets = {f.name: f.offset for f in m.fields}
    return np.column_stack([np.ndarray((m.width*m.height,), dtype='<f4', buffer=m.data,
                                      offset=offsets[n], strides=(m.point_step,)) for n in ('x', 'y', 'z', 'intensity')])


def pcd(path, points):
    header = ('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n'
              'TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n') % (len(points), len(points))
    with path.open('xb') as f:
        f.write(header.encode()); f.write(np.asarray(points, dtype='<f4').tobytes())


def main():
    p = argparse.ArgumentParser(); p.add_argument('run', type=Path); p.add_argument('output', type=Path)
    args = p.parse_args(); args.output.mkdir(parents=True, exist_ok=False)
    db = next((args.run/'raw').glob('*.db3'))
    con = sqlite3.connect('file:'+str(db.resolve())+'?mode=ro', uri=True)
    topics = dict(con.execute('select name,id from topics'))
    def messages(topic, cls):
        for (data,) in con.execute('select data from messages where topic_id=? order by timestamp,id', (topics[topic],)):
            yield deserialize_message(data, cls)
    odom = {ns(m): matrix(m.pose.pose) for m in messages('/fast_livo/odometry', Odometry)}
    accepted = {}
    for m in messages('/fast_livo/localizer/estimate', LocalizerEstimate):
        if m.pose_valid and ns(m) in odom:
            accepted[ns(m)] = matrix(m.pose)@np.linalg.inv(odom[ns(m)])
    stamps = sorted(accepted); assert stamps
    t0 = min(odom); recent = []; cases = []; next_sample = 0
    for m in messages('/fast_livo/deskewed_imu', PointCloud2):
        stamp = ns(m)
        if stamp not in odom: continue
        recent.append((stamp, cloud(m))); recent = recent[-8:]
        t = (stamp-t0)*1e-9
        if len(recent)<8 or t<next_sample: continue
        next_sample = t+2.
        j = bisect.bisect_left(stamps, stamp)
        ref = min(stamps[max(0,j-1):min(len(stamps),j+1)], key=lambda x: abs(x-stamp))
        u = odom[stamp]; x = accepted[ref]@u
        for frames in (2,8):
            chunks = []
            for old_stamp, points in recent[-frames:]:
                transform = np.linalg.inv(u)@odom[old_stamp]
                out = points.copy(); out[:,:3] = points[:,:3]@transform[:3,:3].T+transform[:3,3]
                chunks.append(out)
            name = f'query_{stamp}_{frames}.pcd'; pcd(args.output/name, np.concatenate(chunks))
            cases.append({'id': name, 'path': name, 'stamp_ns': stamp, 't': t, 'frames': frames,
                          'split': 'development' if t<60 else 'heldout_same_visit', 'U': pose(u),
                          'reference_X': pose(x), 'reference_stamp_ns': ref,
                          'reference_propagation_seconds': abs(ref-stamp)*1e-9})
    result = {'schema_version': 1, 'source_run': str(args.run.resolve()), 'cases': cases,
              'reference_kind': 'estimated_baseline_map_local_correction_propagated_with_frontend',
              'ground_truth': False, 'reference_used_by_retrievers': False,
              'limitations': ['One independent capture only; heldout is same visit.',
                              'Before first baseline VALID, reference is backward propagated; not external truth.']}
    (args.output/'cases.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({'cases': len(cases), 'unique_query_times': len(cases)//2, 'output': str(args.output)}))


if __name__ == '__main__': main()
