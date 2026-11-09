#!/usr/bin/env python3
"""Read-only C1 correction diagnostics. Never smooths, registers or publishes poses."""
import argparse
import importlib.util
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

spec=importlib.util.spec_from_file_location('correction_motion',Path(__file__).with_name('diagnose_motion_geometry.py'))
motion=importlib.util.module_from_spec(spec);spec.loader.exec_module(motion)
common=motion.common


def pose(value):
    T=common.matrix(value)
    q=np.asarray(value['xyzw'],float)
    if not np.isfinite(T).all() or abs(np.linalg.norm(q)-1)>1e-6:
        raise ValueError('invalid rigid pose')
    return T


def correction_rows(samples,native):
    stamps=[r['stamp_ns'] for r in samples]
    if not all(isinstance(t,int) and t>0 for t in stamps) or not all(b>a for a,b in zip(stamps,stamps[1:])):
        raise ValueError('nonmonotonic or noninteger sample stamps')
    ns=[r['stamp_ns'] for r in native]
    if not all(b>a for a,b in zip(ns,ns[1:])):raise ValueError('nonmonotonic native stamps')
    by_stamp={r['stamp_ns']:r for r in native}
    output=[];previous=None;reference=None;identity=None
    for row in samples:
        if not row['pose_valid']:continue
        q=row['quality'];stamp=row['stamp_ns']
        if row['state']!=1 or row['last_confirmed_stamp_ns']!=stamp or row['reason']!='map_match_confirmed':
            raise ValueError('valid flag without exact geometric confirmation')
        if not q.get('geometrically_accepted') or not q.get('converged') or not q.get('inputs_ready') or q.get('map_loading'):
            raise ValueError('invalid C1 geometry/inputs')
        key=(row['map_uuid'],q['graph_version'],row['session_id'],row['request_generation'])
        if q['map_uuid']!=row['map_uuid']:raise ValueError('map identity mismatch')
        if identity is not None and key!=identity:raise ValueError('map/session/attempt changed inside diagnostic segment')
        identity=key
        if stamp not in by_stamp:raise ValueError('missing exact native sample')
        U=pose(by_stamp[stamp]['pose']);X=pose(row['pose_map_imu'])
        if not np.allclose(X,pose(q['T_target_source']),atol=1e-8,rtol=0):raise ValueError('stored pose/registration mismatch')
        A=X@np.linalg.inv(U)
        ids=q['native_diagnostics']['target_keyframe_ids']
        if not ids or len(set(ids))!=len(ids):raise ValueError('invalid target IDs')
        if reference is None:reference=A
        frozen=reference@U
        r={'index':row['index'],'stamp_ns':stamp,'map_uuid':key[0],'graph_version':key[1],
           'session_id':key[2],'request_generation':key[3],
           'T_map_local':common.pose_json(A),'T_map_imu':row['pose_map_imu'],'T_local_imu':by_stamp[stamp]['pose'],
           'prediction':q['prediction'],'target_keyframe_ids':ids,
           'rmse_m':q['rmse_m'],'overlap':q['source_overlap'],
           'correspondences':q['correspondences'],'plane_hessian_eigen_ratio':q['normalized_plane_hessian_eigen_ratio'],
           'body_residual_from_first_alignment_m':float(np.linalg.norm(X[:3,3]-frozen[:3,3])),
           'rotation_residual_from_first_alignment_deg':float(Rotation.from_matrix(frozen[:3,:3].T@X[:3,:3]).magnitude()*180/np.pi)}
        if previous is not None:
            prev,prevU,prevX=previous
            dt=(stamp-prev['stamp_ns'])*1e-9
            pred=prevX@np.linalg.inv(prevU)@U
            pa,pb=set(prev['target_keyframe_ids']),set(ids)
            r.update(interval_ns=stamp-prev['stamp_ns'],
                     geometric_step_translation_m=float(np.linalg.norm(X[:3,3]-pred[:3,3])),
                     geometric_step_rotation_deg=float(Rotation.from_matrix(pred[:3,:3].T@X[:3,:3]).magnitude()*180/np.pi),
                     local_speed_mps=float(np.linalg.norm(U[:3,3]-prevU[:3,3])/dt),
                     local_angular_speed_radps=float(Rotation.from_matrix(prevU[:3,:3].T@U[:3,:3]).magnitude()/dt),
                     target_changed=pa!=pb,target_jaccard_distance=1-len(pa&pb)/len(pa|pb),
                     consecutive_native_interval=stamp-prev['stamp_ns']<=150000000)
        output.append(r);previous=(r,U,X)
    if not output:raise ValueError('no geometrically valid samples')
    return output


def summarize(rows):
    result={'frames':len(rows),'target_sets':len({tuple(sorted(r['target_keyframe_ids'])) for r in rows}),
            'absolute_accuracy_validated':False,'physical_time_verified':False}
    for key in ['body_residual_from_first_alignment_m','rotation_residual_from_first_alignment_deg','rmse_m','plane_hessian_eigen_ratio']:
        result[key]=common.quantiles(r[key] for r in rows)
    steps=[r for r in rows if r.get('consecutive_native_interval')]
    result['consecutive_steps']=len(steps);result['target_changes']=sum(r['target_changed'] for r in steps)
    result['steps_by_target_change']={}
    for changed in [False,True]:
        selected=[r for r in steps if r['target_changed']==changed]
        result['steps_by_target_change'][str(changed).lower()]={
            'count':len(selected),'translation_m':common.quantiles(r['geometric_step_translation_m'] for r in selected),
            'rotation_deg':common.quantiles(r['geometric_step_rotation_deg'] for r in selected)}
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--samples',type=Path,required=True);p.add_argument('--native-db',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--execute',action='store_true')
    a=p.parse_args()
    if not a.execute:print(json.dumps({'action':'DRY-RUN','output':str(a.output)}));return 0
    hashes={str(f.resolve()):common.sha_file(f) for f in [a.samples,a.native_db]}
    rows=correction_rows(motion.analysis.rows(a.samples),motion.load_native(a.native_db))
    result=summarize(rows);result['source_sha256']=hashes
    a.output.mkdir(exist_ok=False,parents=True)
    (a.output/'corrections.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    assert all(common.sha_file(Path(f))==h for f,h in hashes.items())
    (a.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2));return 0
if __name__=='__main__':raise SystemExit(main())
