#!/usr/bin/env python3
"""Common-image runtime comparison; freeze landmarks on turn, never refit on query."""
import importlib.util,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[3];HERE=Path(__file__).resolve().parent;OUT=ROOT/'outputs/fast-livo-c1-correction-20261104';OLD=ROOT/'outputs/fast-livo-tag140-two-capture-20261104'
spec=importlib.util.spec_from_file_location('runtime_backfill',HERE.parent/'tag_two_capture_20261104/backfill_assessment.py')
b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b)
cspec=importlib.util.spec_from_file_location('runtime_corrections',ROOT/'scripts/fast_livo2/tags/diagnose_c1_corrections.py')
c=importlib.util.module_from_spec(cspec);cspec.loader.exec_module(c)


def main():
    if sys.argv[1:]!=['--execute']:
        print('DRY-RUN: summarize completed runtime trials');return 0
    archive=OLD/'new_local_map/snapshot';folder=OUT/'runtime_comparison';datasets={};fitted={};result={'kind':'runtime common-cohort consistency, not absolute accuracy or blind generalization',
        'cases':{},'trials':{},'physical_time_verified':False,'VALID_landmark_created':False,'production_change':False,'source_sha256':{}}
    for capture in ['turn','straight']:
        native=b.evaluation.load(capture,'livo',110000000)[2]
        data={'pcl':b.collect(capture,'c1_new_map_optimized' if capture=='turn' else 'c2_new_map_recovery',archive),
              'gicp':b.collect(capture,str(OUT/(capture+'_gicp_runtime')),archive)}
        indices=sorted(set(r['index'] for r in data['pcl']['observations'])&set(r['index'] for r in data['gicp']['observations']))
        observation_maps={k:{r['index']:i for i,r in enumerate(v['observations'])} for k,v in data.items()}
        common={}
        for method,d in data.items():
            positions=[observation_maps[method][i] for i in indices]
            common[method]={**d,'observations':[d['observations'][i] for i in positions],'cameras':[d['cameras'][i] for i in positions]}
            result['source_sha256'].update(d['source_sha256'])
        assert common['pcl']['observations']==common['gicp']['observations']
        observations=common['pcl']['observations'];cfg=common['pcl']['config']
        origin=b.d.analysis.rows(ROOT/'outputs'/b.evaluation.SCREENS[capture]/'observations.jsonl')[0]['stamp_ns']
        train,held,_=b.d.analysis.split_indices([r['stamp_ns'] for r in observations],origin)
        for method,d in common.items():
            fit,modes,seeds=b.d.fit_one(d['cameras'],observations,cfg['camera'],train,held)
            fitted[capture,method]=fit
            result['cases'][capture+'_'+method]={'common_images':len(indices),'train':len(train),'heldout':len(held),
                                               'heldout_rmse_px':fit['holdout_rmse_px'],'training_cost':fit['training_huber_coordinate_cost_mean'],
                                               'common_image_indices':indices,'heldout_image_indices':[observations[i]['index'] for i in held]}
        local,support=b.d.analysis.associate(native,observations,b.d.common.matrix(cfg['T_imu_camera']),110000000)
        assert all(r['available'] for r in support)
        fit,_,_=b.d.fit_one(local,observations,cfg['camera'],train,held)
        fitted[capture,'local_native']=fit;result['cases'][capture+'_local_native']={'common_images':len(indices),'train':len(train),'heldout':len(held),'heldout_rmse_px':fit['holdout_rmse_px']}
        datasets[capture]=common
        paths={'pcl':OLD/capture/('c1_new_map_optimized' if capture=='turn' else 'c2_new_map_recovery'),'gicp':OUT/(capture+'_gicp_runtime')}
        samples={k:b.d.analysis.rows(v/'samples.jsonl') for k,v in paths.items()}
        common_stamps=set(r['stamp_ns'] for r in samples['pcl'] if r['pose_valid'])&set(r['stamp_ns'] for r in samples['gicp'] if r['pose_valid'])
        for method,path in paths.items():
            subset=[r for r in samples[method] if r['stamp_ns'] in common_stamps]
            corrections=c.correction_rows(subset,native.records)
            result['cases'][capture+'_'+method]['correction_summary']=c.summarize(corrections)
            r=json.loads((path/'result.json').read_text());valid=[v for v in samples[method] if v['pose_valid']]
            result['trials'][capture+'_'+method]={k:r.get(k) for k in ['passed','processed_pairs','submitted_pairs','cancelled_pairs','valid_samples','states','reasons','wall_seconds','solve_ms_p50_p95_max','node_peak_rss_kib','public_publishers','localizer_exit','global_search','initial_pose_source']}
            result['trials'][capture+'_'+method]['first_valid_query_relative_seconds']=(valid[0]['stamp_ns']-samples[method][0]['stamp_ns'])*1e-9 if valid else None
    folder.mkdir(exist_ok=False)
    for (capture,method),fit in fitted.items():(folder/(capture+'_'+method+'_fit.json')).write_text(json.dumps(fit,indent=2)+'\n')
    result['cross_visit']={}
    for train_method,query_method in [('pcl','pcl'),('pcl','gicp'),('gicp','gicp')]:
        name=train_method+'_turn_landmark_to_'+query_method+'_straight'
        errors=b.errors(b.d.common.matrix(fitted['turn',train_method]['pose']),datasets['straight'][query_method])
        (folder/(name+'.json')).write_text(json.dumps(errors,indent=2)+'\n');result['cross_visit'][name]={k:v for k,v in errors.items() if k!='frame_errors'}
    legacy=OLD/'backfill_new_map_recovery/turn_fit.json';tag=b.d.common.matrix(json.loads(legacy.read_text())['pose'])
    legacy_errors=b.errors(tag,datasets['straight']['gicp']);result['source_sha256'][str(legacy)]=b.d.common.sha_file(legacy)
    (folder/'legacy_frozen_tag_to_gicp.json').write_text(json.dumps(legacy_errors,indent=2)+'\n')
    result['cross_visit']['legacy_frozen_tag_to_gicp']={k:v for k,v in legacy_errors.items() if k!='frame_errors'}
    assert all(b.d.common.sha_file(Path(f))==h for f,h in result['source_sha256'].items())
    (folder/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({'same_cohort':{k:{f:v[f] for f in ['common_images','train','heldout','heldout_rmse_px']} for k,v in result['cases'].items()},'cross_visit':result['cross_visit'],'trials':result['trials']},indent=2));return 0
if __name__=='__main__':raise SystemExit(main())
