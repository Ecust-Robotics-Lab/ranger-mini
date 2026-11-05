#!/usr/bin/env python3
"""Full-cloud spatial diagnostic, separate from production voxel/normal gating."""
import importlib.util,json,sys
from collections import Counter
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
ROOT=Path(__file__).resolve().parents[3];HERE=Path(__file__).resolve().parent;OUT=ROOT/'outputs/fast-livo-c1-correction-20261104'
spec=importlib.util.spec_from_file_location('pcd_helpers',ROOT/'scripts/fast_livo2/backend/validate_motion.py')
b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b)
if sys.argv[1:]!=['--execute']:
    print('DRY-RUN: full-cloud correspondence attribution');raise SystemExit(0)
job=json.loads((OUT/'straight_inputs/job.json').read_text());queries=job['queries'];by_index={r['index']:r for r in queries}
corrections=[json.loads(s) for s in (OUT/'straight_diagnostics/corrections.jsonl').read_text().splitlines()]
indices={queries[i]['index'] for i in np.linspace(0,len(queries)-1,5).astype(int)}
indices|={r['index'] for r in sorted(corrections[1:],key=lambda r:r['geometric_step_translation_m'],reverse=True)[:5]}
probe=[json.loads(s) for s in (OUT/'straight_probe/registrations.jsonl').read_text().splitlines()]
registrations={(r['index'],r['case']):r for r in probe}
map_root=Path(job['archive']);manifest=json.loads((map_root/'manifest.json').read_text());frames={r['id']:r for r in manifest['keyframes']}
cache={};records=[]
for index in sorted(indices):
    q=by_index[index];clouds=[];labels=[]
    for k in q['target_keyframe_ids']:
        if k not in cache:
            f=frames[k];points=b.read_pcd(map_root/f['directory']/'match.pcd')[:,:3].astype(np.float32)
            T=b.matrix(f['X_map_imu']).astype(np.float32);cache[k]=(points@T[:3,:3].T+T[:3,3]).astype(np.float32)
        clouds.append(cache[k]);labels.extend([k]*len(cache[k]))
    target=np.concatenate(clouds);labels=np.asarray(labels);tree=cKDTree(target)
    source=b.read_pcd(Path(q['source_pcd']))[:,:3];distance=np.linalg.norm(source,axis=1)
    for case in ['pcl_dynamic','gicp_dynamic']:
        reg=registrations[index,case];T=b.matrix(reg['registration']['T_target_source']);world=source@T[:3,:3].T+T[:3,3]
        residual,nearest=tree.query(world);valid=residual<=job['config']['correspondence_distance']
        eigen=np.linalg.eigvalsh(np.cov(world[valid].T))
        hits=Counter(map(int,labels[nearest[valid]]));octants=np.sum((source[valid]>0)*np.array([1,2,4]),axis=1)
        record={'scan_index':index,'stamp_ns':q['stamp_ns'],'case':case,'registration_accepted':reg['same_original_gates_passed'],
                'diagnostic_full_source_points':len(source),'diagnostic_matched_points':int(valid.sum()),
                'full_source_match_fraction':float(valid.mean()),
                'full_source_point_rms_m':float(np.sqrt(np.mean(residual[valid]**2))),
                'full_source_residual_m_p50_p95_max':np.percentile(residual[valid],[50,95,100]).tolist(),
                'nearest_target_frame_counts':dict(hits),'largest_target_frame_fraction':max(hits.values())/sum(hits.values()),
                'inlier_source_octants':dict(Counter(map(int,octants))),
                'inlier_map_xyz_covariance_eigenvalues_m2':eigen.tolist(),
                'mean_nearest_target_minus_transformed_source_m':np.mean(target[nearest[valid]]-world[valid],axis=0).tolist(),
                'range_bins':[]}
        for low,high in [(0,2),(2,5),(5,10),(10,20),(20,1000000)]:
            mask=(distance>=low)&(distance<high);selected=mask&valid
            record['range_bins'].append({'low_m':low,'high_m':high,'source_points':int(mask.sum()),'matched_points':int(selected.sum()),'rms_m':float(np.sqrt(np.mean(residual[selected]**2))) if selected.any() else None})
        records.append(record)
result={'kind':'full-source diagnostic; not production downsampled/normal-filtered quality or absolute accuracy',
        'sampling_rule':'five uniformly spaced recorded scans plus five largest robot-origin correction steps; no image residual selection',
        'selected_scan_indices':sorted(indices),'records':records,'hardware_started':False}
(HERE/'correspondence_inspection.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({'scans':len(indices),'minimum_full_source_match_fraction':min(r['full_source_match_fraction'] for r in records),
                  'largest_single_target_frame_fraction':max(r['largest_target_frame_fraction'] for r in records),
                  'minimum_inlier_xyz_covariance_eigenvalue_m2':min(r['inlier_map_xyz_covariance_eigenvalues_m2'][0] for r in records)},indent=2))
