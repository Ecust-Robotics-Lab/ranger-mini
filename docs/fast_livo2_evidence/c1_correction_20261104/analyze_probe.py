#!/usr/bin/env python3
"""Compare controlled registrations using the same independent image cohort."""
import argparse,importlib.util,json
from collections import Counter
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
OUT=ROOT/'outputs/fast-livo-c1-correction-20261104'
ORIGINAL=ROOT/'outputs/fast-livo-tag140-two-capture-20261104'
spec=importlib.util.spec_from_file_location('probe_backfill',HERE.parent/'tag_two_capture_20261104/backfill_assessment.py')
b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--capture',choices=['straight','turn'],default='straight')
    p.add_argument('--output',type=Path,required=True);p.add_argument('--execute',action='store_true');a=p.parse_args()
    if not a.execute:print('DRY-RUN: analyze controlled registration results');return 0
    job=json.loads((OUT/(a.capture+'_inputs/job.json')).read_text());probe_path=OUT/(a.capture+'_probe/registrations.jsonl')
    samples=[json.loads(s) for s in probe_path.read_text().splitlines()]
    cases=sorted({r['case'] for r in samples});by={c:{r['stamp_ns']:r for r in samples if r['case']==c} for c in cases}
    assert all(len(by[c])==len(job['queries']) for c in cases)
    which='c2_new_map_recovery' if a.capture=='straight' else 'c1_new_map_optimized'
    data=b.collect(a.capture,which,ORIGINAL/'new_local_map/snapshot')
    cfg,_,history,_=b.evaluation.load(a.capture,'livo',110000000)
    native={r['stamp_ns']:b.d.common.matrix(r['pose']) for r in history.records}
    E=b.d.common.matrix(cfg['T_imu_camera']);cameras={c:[] for c in cases};eligible=[];rejected={c:Counter() for c in cases}
    for i,(obs,prov) in enumerate(zip(data['observations'],data['attachments'])):
        usable=True
        for case in cases:
            for t in prov['segment_scan_stamps_ns']:
                if t not in by[case] or not by[case][t]['same_original_gates_passed']:
                    rejected[case]['one_of_three_registration_supports_failed']+=1;usable=False;break
        if not usable:continue
        eligible.append(i);ts=prov['scan_stamp_ns'];Uc=b.d.common.matrix(prov['native_interpolation']['pose'])
        for case in cases:
            X=b.d.common.matrix(by[case][ts]['registration']['T_target_source'])
            cameras[case].append(X@np.linalg.inv(native[ts])@Uc@E)
    observations=[data['observations'][i] for i in eligible]
    local,support=b.d.analysis.associate(history,observations,E,110000000)
    assert all(s['available'] for s in support)
    cameras['local_native']=local
    cameras['recorded_c1']=[data['cameras'][i] for i in eligible]
    origin=b.d.analysis.rows(ROOT/'outputs'/b.evaluation.SCREENS[a.capture]/'observations.jsonl')[0]['stamp_ns']
    train,held,purged=b.d.analysis.split_indices([r['stamp_ns'] for r in observations],origin)
    assert len(train)>=4 and len(held)>=4
    a.output.mkdir(parents=True,exist_ok=False)
    result={'kind':'single-scan attribution; not runtime localization or accuracy validation','capture':a.capture,
            'original_healthy_image_count':len(data['observations']),'common_healthy_images':len(observations),
            'common_image_indices':[r['index'] for r in observations],'train_image_indices':[observations[i]['index'] for i in train],
            'heldout_image_indices':[observations[i]['index'] for i in held],'cases':{},'source_sha256':data['source_sha256']|{str(probe_path):b.d.common.sha_file(probe_path)},
            'query_job_sha256':b.d.common.sha_file(OUT/(a.capture+'_inputs/job.json')),
            'physical_time_verified':False,'VALID_landmark_created':False,'production_change':False}
    for case,cams in cameras.items():
        fit,modes,seeds=b.d.fit_one(cams,observations,cfg['camera'],train,held)
        (a.output/(case+'_fit.json')).write_text(json.dumps(fit,indent=2)+'\n')
        result['cases'][case]={'training_huber_coordinate_cost_mean':fit['training_huber_coordinate_cost_mean'],
                    'heldout_rmse_px':fit['holdout_rmse_px'],'training_modes':modes,'pose_seeds':seeds}
        if case in by:
            rr=list(by[case].values());keep=[r for r in rr if next(q for q in job['queries'] if q['stamp_ns']==r['stamp_ns'])['recorded_hypotheses']==1]
            result['cases'][case].update(scan_gates_passed=sum(r['same_original_gates_passed'] for r in rr),
                 scan_count=len(rr),image_support_rejections=dict(rejected[case]),
                 elapsed_ms=b.d.common.quantiles(r['diagnostic_elapsed_ms'] for r in rr),
                 point_rmse_m=b.d.common.quantiles(r['registration']['rmse_m'] for r in rr if r['registration']['geometrically_accepted']),
                 difference_from_recorded_translation_m=b.d.common.quantiles(r['difference_from_recorded_translation_m'] for r in keep),
                 difference_from_recorded_rotation_rad=b.d.common.quantiles(r['difference_from_recorded_rotation_rad'] for r in keep))
    assert all(b.d.common.sha_file(Path(f))==h for f,h in result['source_sha256'].items())
    (a.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({c:{k:v for k,v in d.items() if k in ['heldout_rmse_px','training_huber_coordinate_cost_mean','scan_gates_passed','scan_count','elapsed_ms']} for c,d in result['cases'].items()},indent=2));return 0
if __name__=='__main__':raise SystemExit(main())
