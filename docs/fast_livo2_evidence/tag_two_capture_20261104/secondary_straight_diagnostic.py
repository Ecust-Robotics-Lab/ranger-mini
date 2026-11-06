#!/usr/bin/env python3
"""Secondary self-fit control; never revises the independent frozen-Tag result."""
import argparse,importlib.util,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
OUT=ROOT/'outputs/fast-livo-tag140-two-capture-20261104'
spec=importlib.util.spec_from_file_location('backfill_diagnostic',HERE/'backfill_assessment.py')
b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b)
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--execute',action='store_true');p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
if not a.execute:
    print(json.dumps({'action':'DRY-RUN','output':str(a.output)}));raise SystemExit(0)
x=b.collect('straight','c2_new_map_recovery',OUT/'new_local_map/snapshot')
cfg,_,history,_=b.evaluation.load('straight','livo',110000000)
origin=b.d.analysis.rows(ROOT/'outputs'/b.evaluation.SCREENS['straight']/'observations.jsonl')[0]['stamp_ns']
train,held,_=b.d.analysis.split_indices([r['stamp_ns'] for r in x['observations']],origin)
fit,modes,seeds=b.d.fit_one(x['cameras'],x['observations'],cfg['camera'],train,held)
local,support=b.d.analysis.associate(history,x['observations'],b.d.common.matrix(cfg['T_imu_camera']),110000000)
assert all(s['available'] for s in support)
local_fit,_,_=b.d.fit_one(local,x['observations'],cfg['camera'],train,held)
turn_path=OUT/'backfill_new_map_recovery/turn_fit.json';turn=json.loads(turn_path.read_text())
F=b.d.common.matrix(fit['pose']);T=b.d.common.matrix(turn['pose'])
result={'purpose':'secondary diagnostic only; never used to revise the frozen turn landmark or independent result',
        'cross_visit_primary_result_unchanged':True,'time_offset_selection_unchanged':True,
        'physical_time_verified':False,'VALID_landmark_created':False,
        'source_sha256':x['source_sha256']|{str(turn_path):b.d.common.sha_file(turn_path)},
        'train_frames':len(train),'heldout_frames':len(held),'pose_seeds':seeds,'training_modes':modes,
        'straight_self_fit_heldout_rmse_px':fit['holdout_rmse_px'],
        'local_livo_same_cohort_heldout_rmse_px':local_fit['holdout_rmse_px'],
        'map_tag_pose_translation_difference_m':float(np.linalg.norm(F[:3,3]-T[:3,3])),
        'map_tag_pose_rotation_difference_deg':float(Rotation.from_matrix(T[:3,:3].T@F[:3,:3]).magnitude()*180/np.pi),
        'straight_self_fit_pose':fit['pose'],'turn_frozen_fit_pose':turn['pose'],
        'common_image_indices':[r['index'] for r in x['observations']],
        'heldout_image_indices':[x['observations'][i]['index'] for i in held]}
with a.output.open('x') as f:f.write(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:result[k] for k in ['train_frames','heldout_frames','straight_self_fit_heldout_rmse_px','local_livo_same_cohort_heldout_rmse_px','map_tag_pose_translation_difference_m','map_tag_pose_rotation_difference_deg']},indent=2))
