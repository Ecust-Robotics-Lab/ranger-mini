#!/usr/bin/env python3
"""Fixed LIO trajectory / fixed camera model: training-only bounded offset selection."""
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
EV=Path(__file__).resolve().parent
OUT=ROOT/'outputs/fast-livo-tag140-motion-geometry-20261104/time_sweep'
spec=importlib.util.spec_from_file_location('consistency', ROOT/'scripts/fast_livo2/tags/analyze_capture_consistency.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)

def main():
    if sys.argv[1:]!=['--execute']:
        print('DRY-RUN: fixed LIO temporal diagnostic, no production changes');return 0
    OUT.mkdir(exist_ok=False)
    screen=ROOT/'outputs/fast-livo-tag140-sync-diag02-screen-20261029'
    obs=a.rows(screen/'observations.jsonl')
    config=json.loads(Path((EV/'config_path.txt').read_text().strip()).read_text())
    trajectory=ROOT/'outputs/fast-livo-tag140-motion-geometry-20261104/lio/states.jsonl'
    history=a.common.PoseHistory(a.rows(trajectory))
    offsets=list(range(-50_000_000,250_000_001,5_000_000))
    datasets={ns:a.associate(history,obs,a.common.matrix(config['T_imu_camera']),ns) for ns in offsets}
    indices=[i for i,r in enumerate(obs) if not r['rejection_reasons'] and all(d[1][i]['available'] for d in datasets.values())]
    cohort=[obs[i] for i in indices]
    train,held,purged=a.split_indices([r['stamp_ns'] for r in cohort],obs[0]['stamp_ns'])
    scores=[]
    for ns in offsets:
        cameras=[datasets[ns][0][i] for i in indices]
        seeds,modes=a.seed_modes(cameras,cohort,train)
        fits=[a.fit_bundle(seed,cameras,cohort,config['camera'],train,held) for seed in seeds]
        best=min(fits,key=lambda f:f['training_huber_coordinate_cost_mean'])
        row={'offset_ns':ns,'training_modes':modes,'seeds':len(fits),'selected_fit':best}
        (OUT/f'fit_{ns}.json').write_text(json.dumps(row,indent=2)+'\n')
        scores.append({'offset_ns':ns,'training_cost':best['training_huber_coordinate_cost_mean'],
                       'training_rmse_px':best['train_rmse_px'],'heldout_rmse_px':best['holdout_rmse_px']})
    selected=min(scores,key=lambda r:r['training_cost'])
    summary={'scores':scores,'training_selected':selected,'common_cohort':len(cohort),
             'train_images':[cohort[i]['index'] for i in train], 'heldout_images':[cohort[i]['index'] for i in held],
             'purged_images':[cohort[i]['index'] for i in purged],
             'source_sha256':{str(p):a.common.sha_file(p) for p in [trajectory,screen/'observations.jsonl']},
             'fixed_extrinsics':True,'fixed_camera_model':True,'fixed_LIO_trajectory':True,
             'physical_time_verified':False,'production_offset_changed':False}
    (OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps({k:v for k,v in summary.items() if k not in ('scores','train_images','heldout_images','purged_images')},indent=2))
    return 0

if __name__=='__main__':raise SystemExit(main())
