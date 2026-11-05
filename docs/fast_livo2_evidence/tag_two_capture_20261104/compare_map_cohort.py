#!/usr/bin/env python3
"""Same-image local/old-map/new-map Tag consistency, no calibration or map writes."""
import importlib.util,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
OUT=ROOT/'outputs/fast-livo-tag140-two-capture-20261104'
spec=importlib.util.spec_from_file_location('backfill_comparison',HERE/'backfill_assessment.py')
b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b)
if sys.argv[1:]!=['--execute']:
    print('DRY-RUN: compare completed same-cohort trajectories');raise SystemExit(0)
old=b.collect('turn','c2',ROOT/'outputs/fast-livo-backend-validation-20261028-pcl/snapshot')
new=b.collect('turn','c1_new_map_optimized',OUT/'new_local_map/snapshot')
old_index={r['index']:i for i,r in enumerate(old['observations'])}
new_index={r['index']:i for i,r in enumerate(new['observations'])}
indices=sorted(set(old_index)&set(new_index))
observations=[old['observations'][old_index[i]] for i in indices]
assert len(indices)>=20
assert all(old['observations'][old_index[i]]==new['observations'][new_index[i]] for i in indices)
config,_,history,_=b.evaluation.load('turn','livo',110000000)
local,support=b.d.analysis.associate(history,observations,b.d.common.matrix(config['T_imu_camera']),110000000)
assert all(r['available'] for r in support)
origin=b.d.analysis.rows(ROOT/'outputs'/b.evaluation.SCREENS['turn']/'observations.jsonl')[0]['stamp_ns']
train,held,purged=b.d.analysis.split_indices([r['stamp_ns'] for r in observations],origin)
result={'kind':'matched-image internal consistency; not absolute accuracy','cohort_image_indices':indices,
        'train_image_indices':[observations[i]['index'] for i in train],
        'heldout_image_indices':[observations[i]['index'] for i in held],
        'offset_hypothesis_ns':110000000,'physical_time_verified':False,
        'note':'one fixed Tag pose fitted separately in each trajectory coordinate system using identical training frames; validation frames never fitted',
        'source_sha256':old['source_sha256']|new['source_sha256'],'cases':{}}
folder=OUT/'same_cohort_map_comparison';folder.mkdir(exist_ok=False)
for name,cameras in [('local_livo',local),('old_map_c1',[old['cameras'][old_index[i]] for i in indices]),('new_map_c1',[new['cameras'][new_index[i]] for i in indices])]:
    fit,modes,seeds=b.d.fit_one(cameras,observations,config['camera'],train,held)
    (folder/(name+'_fit.json')).write_text(json.dumps(fit,indent=2)+'\n')
    result['cases'][name]={'frames':len(indices),'training_frames':len(train),'heldout_frames':len(held),
            'training_modes':modes,'seeds':seeds,'holdout_rmse_px':fit['holdout_rmse_px'],
            'training_huber_coordinate_cost_mean':fit['training_huber_coordinate_cost_mean']}
assert all(b.d.common.sha_file(Path(p))==s for p,s in result['source_sha256'].items())
(folder/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result['cases'],indent=2))
