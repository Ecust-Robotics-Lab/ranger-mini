#!/usr/bin/env python3
"""Summarize the actual C2 baseline, without treating map residuals as truth."""
import importlib.util
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('backend_checks',ROOT/'scripts/fast_livo2/backend/validate_motion.py')
backend=importlib.util.module_from_spec(spec);spec.loader.exec_module(backend)

def main():
    folder=ROOT/'outputs/fast-livo-c2-cross-session-20261104-zero-full'
    result=json.loads((folder/'result.json').read_text())
    rows=[json.loads(l) for l in (folder/'samples.jsonl').read_text().splitlines()]
    origin=rows[0]['stamp_ns'];valid=[r for r in rows if r['pose_valid']]
    elapsed=lambda r:(r['stamp_ns']-origin)*1e-9
    xyz=np.array([r['pose_map_imu']['xyz'] for r in valid])
    summary={'different_capture_used':True,'spatial_overlap_confirmed_by_user':False,
        'source_pairs':len(rows),'valid_samples':len(valid),'first_valid_index':valid[0]['index'],
        'first_valid_sensor_seconds':elapsed(valid[0]),'initial_static_wait_is_included':True,
        'states':result['states'],'loss_transitions':result['loss_transitions'],
        'recovery_transitions':result['recovery_transitions'],'final_state':result['final_state'],
        'absolute_accuracy_validated':False,'manual_initial_pose_provided':False,'tag_candidates_enabled':False,
        'pose_translation_axis_span_m':np.ptp(xyz,axis=0).tolist(),
        'first_pose_map_imu':valid[0]['pose_map_imu'],'last_pose_map_imu':valid[-1]['pose_map_imu'],
        'public_publishers':result['public_publishers'],'wall_seconds':result['wall_seconds'],
        'pacing':'acknowledged offline input; not real-time throughput or live latency'}
    (HERE/'baseline_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    (HERE/'zero_full_result.json').write_bytes((folder/'result.json').read_bytes())
    (HERE/'zero_first300_result.json').write_bytes((ROOT/'outputs/fast-livo-c2-cross-session-20261104-zero-first300/result.json').read_bytes())
    cal=ROOT/'outputs/fast-livo-apriltag-heldout-calibration-20261104'
    (HERE/'heldout_calibration_summary.json').write_bytes((cal/'summary.json').read_bytes())
    import shutil
    shutil.copy2(cal/'hole_boundary_overlay.png',HERE/'hole_boundary_overlay.png')
    poses=backend.load_pairs(Path(result['source_native']))
    selected=valid[0];_,_,cloud=poses[selected['index']]
    dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,
                    'offsets':[next(f.offset for f in cloud.fields if f.name==n) for n in ('x','y','z')],
                    'itemsize':cloud.point_step})
    data=np.ndarray((cloud.height,cloud.width),dtype=dtype,buffer=cloud.data,strides=(cloud.row_step,cloud.point_step))
    points=np.column_stack([data[n].ravel() for n in ('x','y','z')]);points=points[np.isfinite(points).all(axis=1)]
    T=backend.matrix(selected['pose_map_imu']);query=points@T[:3,:3].T+T[:3,3]
    archive=ROOT/'outputs/fast-livo-backend-validation-20261028-pcl/snapshot'
    map_points=backend.read_pcd(archive/'map.pcd')[:,:3];map_points=map_points[::max(1,len(map_points)//60000)]
    fig,axes=plt.subplots(2,2,figsize=(13,8),constrained_layout=True)
    ax=axes[0,0];t=[elapsed(r) for r in rows]
    ax.step(t,[r['state'] for r in rows],where='post');ax.set_yticks([0,1,2,3,4],['INITIALIZING','VALID','DEGRADED','LOST','ERROR'])
    ax.axvline(elapsed(valid[0]),linestyle='--',color='grey')
    ax.set(title='C2 without an initial pose -> C1 tracking',xlabel='Sensor time relative to first query (s)')
    ax=axes[0,1]
    ax.plot([elapsed(r) for r in rows],[r['ack_latency_ms'] for r in rows],'.',markersize=2)
    ax.set(title='Offline acknowledgment delay (includes search)',xlabel='Relative sensor time (s)',ylabel='Wall time per query (ms)')
    for ax,(a,b) in zip(axes[1],[(0,1),(0,2)]):
        ax.scatter(map_points[:,a],map_points[:,b],s=.25,c='grey',alpha=.4,label='Prior map',rasterized=True)
        ax.scatter(query[:,a],query[:,b],s=.5,c='tab:cyan',alpha=.6,label='First confirmed query, transformed once',rasterized=True)
        ax.plot(xyz[:,a],xyz[:,b],color='red',linewidth=1,label='Confirmed trajectory')
        ax.set(title='Local map/query view: '+('XY' if b==1 else 'XZ')+' projection',xlabel='Map '+ 'XYZ'[a]+' (m)',ylabel='Map '+'XYZ'[b]+' (m)')
        center=np.array(selected['pose_map_imu']['xyz'])
        ax.set_xlim(center[a]-8,center[a]+8);half=8 if b==1 else 6
        ax.set_ylim(center[b]-half,center[b]+half)
        ax.set_aspect('equal',adjustable='box');ax.legend(fontsize=7)
    for ax in axes.ravel():ax.grid(alpha=.2)
    fig.suptitle('Independent-capture C2 functional experiment; no Tag, no public TF, no absolute truth')
    fig.savefig(HERE/'global_baseline.png',dpi=160)
    fig.savefig(HERE/'global_baseline.svg')

if __name__=='__main__':main()
