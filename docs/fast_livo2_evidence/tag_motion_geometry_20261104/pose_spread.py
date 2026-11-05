#!/usr/bin/env python3
"""Quantify apparent fixed-Tag pose spread; no ground truth or landmark registration."""
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.distance import pdist
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('motion_diagnostic',ROOT/'scripts/fast_livo2/tags/diagnose_motion_geometry.py')
d=importlib.util.module_from_spec(spec);spec.loader.exec_module(d)


def statistics(poses, reference):
    poses=np.asarray(poses)
    distance=np.linalg.norm(poses[:,:3,3]-reference[:3,3],axis=1)
    angles=np.rad2deg(Rotation.from_matrix(reference[:3,:3].T@poses[:,:3,:3]).magnitude())
    flat=poses[:,:3,:3].reshape(-1,9)
    pair_angles=np.rad2deg(np.arccos(np.clip((flat@flat.T-1)*.5,-1,1)))
    return {'count':len(poses),'position_distance_to_reference_m':d.common.quantiles(distance),
            'orientation_distance_to_reference_deg':d.common.quantiles(angles),
            'maximum_pairwise_position_m':float(pdist(poses[:,:3,3]).max()),
            'maximum_pairwise_orientation_deg':float(pair_angles.max())}


def main():
    if sys.argv[1:]!=['--execute']:
        print('DRY-RUN: existing output pose spread only');return 0
    target=HERE/'pose_spread.json'
    if target.exists():raise ValueError('output already exists')
    config_path=Path((HERE/'config_path.txt').read_text().strip())
    config=json.loads(config_path.read_text())
    screen=ROOT/'outputs/fast-livo-tag140-sync-diag02-screen-20261029/observations.jsonl'
    by_image={r['index']:r for r in d.analysis.rows(screen)}
    indices=json.loads((HERE/'cohort.json').read_text());observations=[by_image[i] for i in indices]
    origin=min(r['stamp_ns'] for r in by_image.values())
    static=json.loads((HERE/'static_windows.json').read_text())
    results=[];hashes={str(p):d.common.sha_file(p) for p in [screen,config_path,HERE/'cohort.json']}
    for name in ['livo_0','livo_120000000','lio_110000000']:
        file=ROOT/'outputs/fast-livo-tag140-motion-geometry-20261104/comparison_final'/(name+'.json')
        r=json.loads(file.read_text());case=r['case'];path=Path(case['path'])
        hashes[str(file)]=d.common.sha_file(file);hashes[str(path)]=d.common.sha_file(path)
        records=d.load_native(path) if case['format']=='native_db' else d.analysis.rows(path)
        cameras,support=d.analysis.associate(d.common.PoseHistory(records),observations,d.common.matrix(config['T_imu_camera']),case['offset_ns'])
        assert all(s['available'] for s in support)
        candidates=[[camera@d.common.matrix(h['T_camera_tag']) for h in row['observation']['hypotheses']]
                    for camera,row in zip(cameras,observations)]
        branches=[min(range(len(row['observation']['hypotheses'])),key=lambda i:row['observation']['hypotheses'][i]['pixel_rmse']) for row in observations]
        poses=[p[i] for p,i in zip(candidates,branches)]
        reference=d.common.matrix(r['global']['pose'])
        result={'case':case,'primary_hypothesis_rule':'smallest same-image pixel RMSE; no map-reference branch selection',
                'reference':'single fixed Tag pose fitted on 103 training frames only; not ground truth',
                'reference_pose':r['global']['pose'],'all_cohort':statistics(poses,reference)}
        held=set(r['heldout_images']);held_poses=[p for p,row in zip(poses,observations) if row['index'] in held]
        result['heldout']=statistics(held_poses,reference)
        separation=[np.rad2deg(Rotation.from_matrix(p[0][:3,:3].T@p[1][:3,:3]).magnitude()) for p in candidates if len(p)==2]
        result['two_hypothesis_angular_separation_deg']=d.common.quantiles(separation)
        result['static_windows']=[]
        windows=next(s['static_windows'] for s in static if s['case']['name']==name)
        for window in windows:
            start=window['start_s'];selected=[i for i,row in enumerate(observations) if start*10**9<=row['stamp_ns']-origin<(start+5)*10**9]
            subset=np.asarray([poses[i] for i in selected]);center=reference.copy()
            center[:3,3]=np.median(subset[:,:3,3],axis=0)
            result['static_windows'].append({'start_s':start,'observed_corner_spread_px':window['observed_corner_spread_px'],
                'spread':statistics(subset,center),'position_reference':'componentwise median in this window, diagnostic only; orientation reference remains fixed training fit'})
        result['reprojection_heldout_px']=r['global']['holdout_rmse_px']
        results.append(result)
    assert all(d.common.sha_file(Path(p))==h for p,h in hashes.items())
    summary={'cases':results,'source_sha256':hashes,'source_capture':'fast-livo-tag140_sync_diag_02-35f_m8nd',
        'source_kind':'same-capture current-source offline replay; not an independent truth trajectory',
        'uncertainty':'primary branch spread includes planar-PnP ambiguity and camera/time/trajectory errors; not absolute robot localization error',
        'landmark_created':False,'hardware_started':False,'configuration_changed':False}
    target.write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps([{ 'name':r['case']['name'],'cohort':r['all_cohort'],'heldout':r['heldout'],
                       'two_hypothesis_angular_separation_deg':r['two_hypothesis_angular_separation_deg']} for r in results],indent=2))
    return 0

if __name__=='__main__':raise SystemExit(main())
