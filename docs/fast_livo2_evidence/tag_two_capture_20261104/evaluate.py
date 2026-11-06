#!/usr/bin/env python3
"""Train on turn capture, freeze an offset, then validate without a straight sweep."""
import argparse
import importlib.util
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
OUT=ROOT/'outputs/fast-livo-tag140-two-capture-20261104'
spec=importlib.util.spec_from_file_location('motion_diagnostic',ROOT/'scripts/fast_livo2/tags/diagnose_motion_geometry.py')
d=importlib.util.module_from_spec(spec);spec.loader.exec_module(d)
CAPTURES={'turn':'fast-livo-tag140_turn_03-bk8_pjyy','straight':'fast-livo-tag140_straight_04-ryzykq2l'}
SCREENS={'turn':'fast-livo-tag140-turn03-screen-20261104','straight':'fast-livo-tag140-straight04-screen-20261104'}

def load(name,mode='lio',offset=0):
    capture=ROOT/'bags'/CAPTURES[name];manifest=json.loads((capture/'manifest.json').read_text())
    entry=next(r for r in manifest['source_files'] if r['source'].endswith('/apriltag_camera.json'))
    config_path=capture/entry['file'];assert d.common.sha_file(config_path)==entry['sha256']
    config=json.loads(config_path.read_text());screen=ROOT/'outputs'/SCREENS[name]
    observations=d.analysis.rows(screen/'observations.jsonl')
    folder=OUT/name/(mode+'_'+str(offset))/'result'
    path=folder/'states.jsonl' if mode=='lio' else folder/'native_frames/native_frames_0.db3'
    records=d.analysis.rows(path) if mode=='lio' else d.load_native(path)
    history=d.common.PoseHistory(records)
    assert history.monotonic
    hashes={str(p):d.common.sha_file(p) for p in [path,config_path,screen/'observations.jsonl']}
    return config,observations,history,hashes

def fit_offsets(name,offsets,mode='lio'):
    datasets={};hashes={};config=None;obs=None
    for offset in offsets:
        cfg,rows,history,source=load(name,mode,offset if mode=='livo' else 0)
        if config is not None:assert cfg==config and rows==obs
        config,obs=cfg,rows;hashes.update(source)
        datasets[offset]=d.analysis.associate(history,obs,d.common.matrix(config['T_imu_camera']),offset)
    indices=[i for i,row in enumerate(obs) if not row['rejection_reasons'] and all(p[1][i]['available'] for p in datasets.values())]
    cohort=[obs[i] for i in indices];origin=obs[0]['stamp_ns']
    train,held,purged=d.analysis.split_indices([r['stamp_ns'] for r in cohort],origin)
    assert len(train)>=4 and len(held)>=4
    results=[]
    for offset in offsets:
        cameras=[datasets[offset][0][i] for i in indices]
        best,modes,seeds=d.fit_one(cameras,cohort,config['camera'],train,held)
        results.append({'offset_ns':offset,'modes':modes,'seeds':seeds,'fit':best})
    return {'capture':CAPTURES[name],'mode':mode,'source_sha256':hashes,
            'common_image_indices':[r['index'] for r in cohort],
            'train_image_indices':[cohort[i]['index'] for i in train],
            'heldout_image_indices':[cohort[i]['index'] for i in held],
            'purged_image_indices':[cohort[i]['index'] for i in purged],'scores':results,
            'clock_or_configuration_changed':False,'landmark_created':False}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--phase',choices=['select','validate','livo'],required=True)
    parser.add_argument('--capture',choices=['turn','straight'])
    args=parser.parse_args()
    if not args.execute:
        print(json.dumps({'action':'DRY-RUN','phase':args.phase}));return 0
    if args.phase=='select':
        folder=OUT/'turn/time_sweep';folder.mkdir(exist_ok=False)
        result=fit_offsets('turn',list(range(-50_000_000,250_000_001,5_000_000)))
        # Selection is exclusively training cost; heldout values are never consulted here.
        selected=min(result['scores'],key=lambda row:row['fit']['training_huber_coordinate_cost_mean'])
        for row in result['scores']:(folder/('fit_'+str(row['offset_ns'])+'.json')).write_text(json.dumps(row,indent=2)+'\n')
        result['selected_offset_ns']=selected['offset_ns']
        (folder/'summary.json').write_text(json.dumps({k:v for k,v in result.items() if k!='scores'}|
            {'scores':[{k:v for k,v in r.items() if k!='fit'}|{'fit':{k:v for k,v in r['fit'].items() if k!='frame_errors'}} for r in result['scores']]},indent=2)+'\n')
        frozen={'prior_offset_ns':110000000,'turn_selected_offset_ns':selected['offset_ns'],
                'selection_rule':'turn training Huber coordinate cost only; fixed K/D/E/size/LIO trajectory',
                'selection_result_sha256':d.common.sha_file(folder/'summary.json'),
                'protocol_sha256':d.common.sha_file(HERE/'protocol.json'),
                'selection_capture':CAPTURES['turn'],'validation_capture':CAPTURES['straight'],
                'straight_geometry_not_used_for_selection':True,'physical_time_verified':False}
        with (HERE/'frozen_selection.json').open('x') as stream:stream.write(json.dumps(frozen,indent=2)+'\n')
        print(json.dumps(frozen,indent=2));return 0
    frozen_path=HERE/'frozen_selection.json';frozen=json.loads(frozen_path.read_text())
    assert frozen['protocol_sha256']==d.common.sha_file(HERE/'protocol.json')
    assert frozen['selection_result_sha256']==d.common.sha_file(OUT/'turn/time_sweep/summary.json')
    offsets=sorted({0,frozen['prior_offset_ns'],frozen['turn_selected_offset_ns']})
    names=[args.capture] if args.capture else ['turn','straight']
    for name in names:
        mode='livo' if args.phase=='livo' else 'lio'
        result=fit_offsets(name,offsets,mode)
        result['frozen_selection_sha256']=d.common.sha_file(frozen_path)
        folder=OUT/name/(mode+'_evaluation');folder.mkdir(exist_ok=False)
        for row in result['scores']:(folder/('fit_'+str(row['offset_ns'])+'.json')).write_text(json.dumps(row,indent=2)+'\n')
        summary={k:v for k,v in result.items() if k!='scores'}|{'scores':[{k:v for k,v in r.items() if k!='fit'}|{'fit':{k:v for k,v in r['fit'].items() if k!='frame_errors'}} for r in result['scores']]}
        (folder/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        print(json.dumps({'capture':name,'mode':mode,'cohort':len(result['common_image_indices']),
            'heldout':len(result['heldout_image_indices']),
            'scores':[{'offset_ns':r['offset_ns'],'training_cost':r['fit']['training_huber_coordinate_cost_mean'],
                       'heldout_rmse_px':r['fit']['holdout_rmse_px'],'pass':r['fit']['frames_passing_initial_3px_rms_6px_max']} for r in result['scores']]},indent=2),flush=True)
    return 0

if __name__=='__main__':raise SystemExit(main())
