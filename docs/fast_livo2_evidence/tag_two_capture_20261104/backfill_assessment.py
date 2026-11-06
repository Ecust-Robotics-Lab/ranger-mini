#!/usr/bin/env python3
"""Conditional C1 backfill assessment; never writes a loadable/VALID Tag map."""
import argparse
from collections import Counter
import importlib.util
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
OUT=ROOT/'outputs/fast-livo-tag140-two-capture-20261104'
spec=importlib.util.spec_from_file_location('independent_evaluation',HERE/'evaluate.py')
evaluation=importlib.util.module_from_spec(spec);spec.loader.exec_module(evaluation)
d=evaluation.d

def collect(name,c2folder,archive):
    config,observations,history,hashes=evaluation.load(name,'livo',110000000)
    path=OUT/name/c2folder
    if not (path/'result.json').exists():raise ValueError('C2 result not complete')
    result=json.loads((path/'result.json').read_text());samples=d.analysis.rows(path/'samples.jsonl')
    native_path=OUT/name/'livo_110000000/result/native_frames/native_frames_0.db3'
    assert d.common.sha_file(native_path)==result['source_database_sha256']
    manifest_path=archive/'manifest.json'
    assert d.common.sha_file(manifest_path)==result['source_manifest_sha256']
    manifest=json.loads(manifest_path.read_text())
    sequence=d.analysis.geometry.C1Sequence(samples,history.records,manifest['map_uuid'],manifest['graph_version'])
    E=d.common.matrix(config['T_imu_camera']);rows=[];cameras=[];attachments=[];reasons=Counter()
    for row in observations:
        if row['rejection_reasons']:reasons['observation_quality']+=1;continue
        try:camera,provenance=sequence.camera(row['stamp_ns']+110000000,E)
        except ValueError as e:reasons[str(e)]+=1;continue
        rows.append(row);cameras.append(camera);attachments.append(provenance)
    hashes.update({str(p):d.common.sha_file(p) for p in [path/'result.json',path/'samples.jsonl',manifest_path]})
    return {'config':config,'observations':rows,'cameras':cameras,'attachments':attachments,'rejections':dict(reasons),
            'C2_passed':result['passed'],'C2_output':str(path),'map_uuid':manifest['map_uuid'],
            'graph_version':manifest['graph_version'],'map_manifest_sha256':result['source_manifest_sha256'],'source_sha256':hashes}

def errors(tag,data):
    delta,positive=d.analysis.projected_errors(tag,np.linalg.inv(data['cameras']),data['observations'],data['config']['camera'])
    lengths=np.linalg.norm(delta,axis=2);rms=np.sqrt(np.mean(lengths**2,axis=1));maximum=np.max(lengths,axis=1)
    return {'frames':len(rms),'rmse_px':d.common.quantiles(rms),'maximum_corner_px':d.common.quantiles(maximum),
            'passing_initial_3px_rms_6px_max':int(np.sum(positive&(rms<=3)&(maximum<=6))),
            'negative_depth':int(np.sum(~positive)),
            'frame_errors':[{'index':r['index'],'stamp_ns':r['stamp_ns'],'rmse_px':float(v),'maximum_px':float(m)} for r,v,m in zip(data['observations'],rms,maximum)]}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--archive',type=Path,default=ROOT/'outputs/fast-livo-backend-validation-20261028-pcl/snapshot')
    parser.add_argument('--turn-c2',default='c2')
    parser.add_argument('--straight-c2',default='c2')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if not args.execute:
        print(json.dumps({'action':'DRY-RUN','output':str(args.output)}));return 0
    assert json.loads((HERE/'setup_declaration.json').read_text())['same_fixed_tag_between_captures']
    turn=collect('turn',args.turn_c2,args.archive);straight=collect('straight',args.straight_c2,args.archive)
    assert turn['map_manifest_sha256']==straight['map_manifest_sha256'] and turn['config']==straight['config']
    args.output.mkdir(exist_ok=False)
    result={'kind':'conditional backfill assessment; not a loadable Tag map','offset_assumption_ns':110000000,
            'map_uuid':turn['map_uuid'],'graph_version':turn['graph_version'],'map_manifest_sha256':turn['map_manifest_sha256'],
            'physical_time_verified':False,'real_VALID_landmark_created':False,'hypothesis':'same fixed physical Tag, explicit user confirmation',
            'source_sha256':turn['source_sha256']|straight['source_sha256']}
    for name,data in [('turn',turn),('straight',straight)]:
        (args.output/(name+'_attachments.jsonl')).write_text(''.join(json.dumps({'image_index':r['index'],'raw_image_stamp_ns':r['stamp_ns'],'effective_image_stamp_ns':r['stamp_ns']+110000000,'source':a})+'\n' for r,a in zip(data['observations'],data['attachments'])))
        result[name]={'healthy_C1_attached_observations':len(data['observations']),'rejections':data['rejections'],'C2_output':data['C2_output'],'C2_passed':data['C2_passed']}
    if len(turn['observations'])<20:
        result['status']='INSUFFICIENT_TURN_C1_SUPPORT'
    else:
        origin=d.analysis.rows(ROOT/'outputs'/evaluation.SCREENS['turn']/'observations.jsonl')[0]['stamp_ns']
        train,held,purged=d.analysis.split_indices([r['stamp_ns'] for r in turn['observations']],origin)
        fit,modes,seeds=d.fit_one(turn['cameras'],turn['observations'],turn['config']['camera'],train,held)
        (args.output/'turn_fit.json').write_text(json.dumps(fit,indent=2)+'\n')
        result['turn_training_count']=len(train);result['turn_heldout_count']=len(held)
        result['turn_heldout_rmse_px']=fit['holdout_rmse_px'];result['turn_training_modes']=modes
        result['candidate_T_map_tag']=fit['pose'];result['candidate_quality']='UNVERIFIED; disabled, not registered'
        result['turn_fit_geometry']=errors(d.common.matrix(fit['pose']),turn)
        # Matched-image control: separate map-registration effects from cohort changes.
        _,_,history,_=evaluation.load('turn','livo',110000000)
        local_cameras,support=d.analysis.associate(history,turn['observations'],
            d.common.matrix(turn['config']['T_imu_camera']),110000000)
        assert all(s['available'] for s in support)
        local_fit,_,_=d.fit_one(local_cameras,turn['observations'],turn['config']['camera'],train,held)
        (args.output/'turn_local_same_cohort_fit.json').write_text(json.dumps(local_fit,indent=2)+'\n')
        result['turn_local_same_cohort_heldout_rmse_px']=local_fit['holdout_rmse_px']
        if len(straight['observations'])<20:result['status']='NO_INDEPENDENT_STRAIGHT_C1_SUPPORT'
        else:
            # Freeze the turn-trained landmark: no refitting it on the straight visit.
            result['straight_frozen_turn_landmark_reprojection']=errors(d.common.matrix(fit['pose']),straight)
            result['status']='ASSESSMENT_ONLY_NOT_VALID_REGISTRATION'
    for key in ['turn_fit_geometry','straight_frozen_turn_landmark_reprojection']:
        if key in result:
            (args.output/(key+'.json')).write_text(json.dumps(result[key],indent=2)+'\n')
            result[key]={k:v for k,v in result[key].items() if k!='frame_errors'}
    assert all(d.common.sha_file(Path(p))==h for p,h in result['source_sha256'].items())
    (args.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    return 0

if __name__=='__main__':raise SystemExit(main())
