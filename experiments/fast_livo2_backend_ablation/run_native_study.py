#!/usr/bin/env python3
"""Verify archived keyframes against raw bag payloads, then rerun registration/PGO."""
import argparse
import hashlib
import json
import sqlite3
import subprocess
from pathlib import Path
import numpy as np
import time
from run_factor_study import a, dataset, evaluate, causal_rows, HERE

VARIANTS = {'pcl_baseline': {}, 'gicp': {'registration': 'small_gicp'},
            'gicp_odom_half': {'registration': 'small_gicp', 'odom_sigma_scale': .5},
            'pcl_cauchy': {'robust_kernel': 'cauchy'}}


def verify_payloads(trial, manifest, raw):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    expected = {k['stamp_ns']: manifest['files'][k['directory'] + '/scan.cdr'] for k in manifest['keyframes']}
    matched = set()
    for bag in sorted((trial / 'capture/raw').glob('*.db3')):
        with sqlite3.connect(bag.resolve().as_uri() + '?mode=ro', uri=True) as conn:
            for (payload,) in conn.execute('SELECT m.data FROM messages m JOIN topics t ON t.id=m.topic_id WHERE t.name=?', ('/fast_livo/deskewed_imu',)):
                msg = deserialize_message(payload, PointCloud2)
                stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
                if stamp in expected:
                    key = next(k for k in manifest['keyframes'] if k['stamp_ns'] == stamp)
                    archived = (trial / 'capture/map' / key['directory'] / 'scan.cdr').read_bytes()
                    assert hashlib.sha256(archived).hexdigest() == expected[stamp]
                    # ROS CDR padding can differ after node reserialization; compare all decoded fields.
                    assert msg == deserialize_message(archived, PointCloud2), 'raw/archive cloud fields differ'
                    matched.add(stamp)
    assert matched == set(expected), 'missing native keyframe cloud'
    odom = {r['stamp_ns']: r for r in raw}
    for k in manifest['keyframes']:
        assert np.max(np.abs(a.matrix(k['U_local_imu']) - a.matrix(odom[k['stamp_ns']]['pose']))) < 1e-10
    return {'identical_decoded_raw_clouds': len(matched), 'exact_raw_odometry_poses': len(matched)}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--group', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--binary', type=Path, default=HERE / 'build/backend_ablation')
    p.add_argument('--path-study', action='store_true')
    args = p.parse_args(); args.output.mkdir(parents=True, exist_ok=False)
    variants = {f'path_{t}m': {'min_loop_odom_path_m': t} for t in (0,3,5,10)} if args.path_study else VARIANTS
    a.save(args.output / 'protocol.json', {'variants': variants,
        'scope': 'native keyframe inputs verified by every decoded field against raw bag (CDR padding can differ); actual Backend/registration/Graph; non-keyframe propagation offline, no DDS latency claim'})
    results = []
    for trial in sorted(args.group.glob('trial_*_attempt_01')):
        raw, obs, manifest = dataset(trial)
        row = {'trial': int(trial.name.split('_')[1]), 'input_verification': verify_payloads(trial, manifest, raw), 'variants': {}}
        folder = args.output / trial.name; folder.mkdir()
        for name, options in variants.items():
            a.save(folder / (name + '_options.json'), options)
            with (folder / (name + '.log')).open('w') as log:
                started=time.monotonic()
                subprocess.run([str(args.binary), str(trial / 'capture/map'), str(folder / (name + '_options.json')), str(folder / name)], stdout=log, stderr=log, check=True)
            replay = a.load(folder / name / 'result.json')
            metrics = evaluate(trial, raw, replay['keyframes'], causal_rows(raw, replay))
            metrics['loop_factors'] = replay['loop_factors']
            metrics['offline_elapsed_s']=time.monotonic()-started
            logs=a.jsonl(folder/name/'replay/candidates.jsonl')
            metrics['short_path_filtered_count']=sum(x.get('filtered_count',0) for x in logs)
            metrics['registration_attempts']=sum('registration' in x for x in logs)
            events=a.load(trial/'events.json');anchor=a.load(trial/'capture/clock_anchor.json')['anchor']
            pairs=[x for x in logs if x.get('decision')=='accepted_and_optimized']
            frames=replay['keyframes'];prefix=np.r_[0,np.cumsum(np.linalg.norm(np.diff([a.matrix(k['U_local_imu'])[:3,3] for k in frames],axis=0),axis=1))]
            scan_keys={k['scan_id']:k['id'] for k in manifest['keyframes']}
            metrics['submap_members_below_threshold']=int(sum(sum(prefix[x['j']]-prefix[scan_keys[s]]<options.get('min_loop_odom_path_m',0) for s in x.get('target_scan_ids',[])) for x in pairs))
            metrics['return_time_pair_events']=sum(frames[x['j']]['stamp_ns']-anchor[0]+anchor[1]>=events['returned_ns']-20*10**9 and frames[x['i']]['stamp_ns']-anchor[0]+anchor[1]<=events['route_start_ns']+20*10**9 for x in pairs)
            steps=[]
            for previous,current in zip(replay['states'],replay['states'][1:]):
                j=len(current['corrections'])-1;u=a.matrix(frames[j]['U_local_imu'])
                before=a.matrix(previous['corrections'][-1])@u;after=a.matrix(current['corrections'][-1])@u
                steps.append(float(np.linalg.norm(before[:3,3]-after[:3,3])))
            metrics['maximum_same_sample_update_m']=max(steps,default=0)
            row['variants'][name] = metrics
            print('trial', row['trial'], name, metrics['loop_factors'], metrics['history']['settled'], flush=True)
        results.append(row); a.save(args.output / 'results.json', results)


if __name__ == '__main__':
    main()
