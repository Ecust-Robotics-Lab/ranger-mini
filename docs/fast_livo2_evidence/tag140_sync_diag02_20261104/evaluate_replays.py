#!/usr/bin/env python3
"""Compare fixed-Tag reprojection on a common cohort with the same temporal split."""
import argparse
import importlib.util
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('consistency',ROOT/'scripts/fast_livo2/tags/analyze_capture_consistency.py')
analysis=importlib.util.module_from_spec(spec);spec.loader.exec_module(analysis)
common=analysis.common

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--replays',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if not args.execute:
        print(json.dumps({'action':'DRY-RUN','output':str(args.output)}));return 0
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    args.output.mkdir(parents=True,exist_ok=False)
    screen=ROOT/'outputs/fast-livo-tag140-sync-diag02-screen-20261029'
    observations=analysis.rows(screen/'observations.jsonl')
    manifest=json.loads((ROOT/'bags/fast-livo-tag140_sync_diag_02-35f_m8nd/manifest.json').read_text())
    row=next(r for r in manifest['source_files'] if r['source'].endswith('/apriltag_camera.json'))
    path=ROOT/'bags/fast-livo-tag140_sync_diag_02-35f_m8nd'/row['file']
    assert common.sha_file(path)==row['sha256']
    config=json.loads(path.read_text());E=common.matrix(config['T_imu_camera'])
    cases={};source_hashes={}
    for offset in (0,120000000):
        folder=args.replays/str(offset)
        run=json.loads((folder/'manifest.json').read_text());assert run['assumed_img_offset_ns']==offset
        db=next((folder/'result/native_frames').glob('*.db3'))
        connection=common.open_readonly(db)
        topic=connection.execute("SELECT id FROM topics WHERE name='/fast_livo/odometry'").fetchone()[0]
        native=common.pose_records(connection,topic,deserialize_message,Odometry)
        connection.close()
        history=common.PoseHistory(native)
        cases[offset]={'native_count':len(native),'dataset':analysis.associate(history,observations,E,offset)}
        source_hashes[str(db)]=common.sha_file(db)
    indices=[i for i,r in enumerate(observations) if not r['rejection_reasons'] and
             all(case['dataset'][1][i]['available'] for case in cases.values())]
    cohort=[observations[i] for i in indices]
    train,held,purged=analysis.split_indices([r['stamp_ns'] for r in cohort],observations[0]['stamp_ns'])
    results=[]
    for offset,case in cases.items():
        cameras=[case['dataset'][0][i] for i in indices]
        seeds,modes=analysis.seed_modes(cameras,cohort,train)
        fits=[analysis.fit_bundle(seed,cameras,cohort,config['camera'],train,held) for seed in seeds]
        best=min(fits,key=lambda f:f['training_huber_coordinate_cost_mean'])
        result={'offset_ns':offset,'native_count':case['native_count'],'training_modes':modes,
                'seed_budget':4,'evaluated_seeds':len(fits),'selected_fit':best,
                'seed_fit_statistics':[{k:v for k,v in f.items() if k!='frame_errors'} for f in fits]}
        (args.output/f'fit_{offset}.json').write_text(json.dumps(result,indent=2)+'\n')
        results.append({k:v for k,v in result.items() if k not in ('selected_fit','seed_fit_statistics')}|
                       {'selected_fit':{k:v for k,v in best.items() if k!='frame_errors'}})
    split={'cohort_image_indices':[r['index'] for r in cohort],
           'training_image_indices':[cohort[i]['index'] for i in train],
           'heldout_image_indices':[cohort[i]['index'] for i in held],
           'purged_image_indices':[cohort[i]['index'] for i in purged]}
    (args.output/'split.json').write_text(json.dumps(split,indent=2)+'\n')
    result={'source_database_sha256':json.loads((screen/'summary.json').read_text())['source']['sha256'],
        'replay_database_sha256':source_hashes,'replay_root':str(args.replays.resolve()),
        'observations':len(observations),'common_cohort':len(cohort),'training_count':len(train),
        'holdout_count':len(held),'purged_count':len(purged),'cases':results,
        'geometry_convention':'T_O_C(tc)=T_O_I(tc+assumed_offset)*T_I_C; one free fixed Tag per replay',
        'same_temporal_split':True,'hypothesis_offset_selection':'120 ms selected from prior gyro training diagnosis; not from these heldout fits',
        'physical_sync_verified':False,'runtime_configuration_changed':False,
        'limitations':['Same capture temporal holdout, not independent visits',
            'Source trajectory uses image fusion; gyro timing diagnosis is the separate check',
            'Bounded four seed fits do not prove global uniqueness or absolute accuracy']}
    (args.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2));return 0

if __name__=='__main__':raise SystemExit(main())
