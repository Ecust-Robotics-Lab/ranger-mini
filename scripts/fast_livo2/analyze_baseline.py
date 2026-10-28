#!/usr/bin/env python3
"""Summarize measured frontend behavior; no ground truth or accuracy claim."""
import argparse
import csv
import hashlib
import json
import shutil
from pathlib import Path

import numpy as np


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('data',type=Path);p.add_argument('--out',type=Path,required=True)
    args=p.parse_args()
    if args.out.exists():raise SystemExit('Output must be new')
    args.out.mkdir(parents=True)
    trajectory=np.loadtxt(args.data/'trajectory_imu.tum',ndmin=2)
    with (args.data/'frames.csv').open() as f:frames=list(csv.DictReader(f))
    def values(key):return np.array([float(row[key]) for row in frames])
    x=trajectory[:,1:4]
    delta=np.linalg.norm(x-x[0],axis=1)
    visual=values('visual_solver_updates')
    trans=values('visual_update_translation_m')
    result={
        'source':str(args.data.resolve()),'scope':'nominally static synchronization bag; no ground truth',
        'frontend_summary':json.loads((args.data/'summary.json').read_text()),
        'pose_count':len(trajectory),'sensor_duration_s':float(trajectory[-1,0]-trajectory[0,0]),
        'all_trajectory_values_finite':bool(np.isfinite(trajectory).all()),
        'unique_pose_timestamps':len(np.unique(trajectory[:,0])),
        'frames_with_visual_solver_updates':int(np.count_nonzero(visual)),
        'frames_with_nonzero_visual_translation_update':int(np.count_nonzero(trans>1e-9)),
        'visual_translation_update_m_p50_p95_max':np.percentile(trans,[50,95,100]).tolist(),
        'visual_rotation_update_rad_p50_p95_max':np.percentile(values('visual_update_rotation_rad'),[50,95,100]).tolist(),
        'vio_processing_ms_p50_p95_max':np.percentile(values('processing_ms'),[50,95,100]).tolist(),
        'point_count_min_median_max':np.percentile(values('points'),[0,50,100]).tolist(),
        'observed_displacement_from_first_pose_m_max':float(delta.max()),
        'observed_end_to_start_translation_m':float(np.linalg.norm(x[-1]-x[0])),
        'accuracy_validated':False,'physical_sync_proven':False,
        'note':'Observed dispersion and visual increments are not APE/RPE or calibration accuracy; static data cannot validate motion fusion or loops.'}
    for name in ('trajectory_imu.tum','frames.csv','summary.json'):
        shutil.copyfile(args.data/name,args.out/name)
    (args.out/'metrics.json').write_text(json.dumps(result,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(1,3,figsize=(14,4))
    ax[0].plot(x[:,0],x[:,1],linewidth=0.8)
    ax[0].scatter(x[[0,-1],0],x[[0,-1],1],c=['green','red'],s=25)
    ax[0].set(xlabel='local x (m)',ylabel='local y (m)',title='Observed static-bag trajectory');ax[0].axis('equal')
    t=trajectory[:,0]-trajectory[0,0]
    ax[1].plot(t,trans*1000,linewidth=0.8)
    ax[1].set(xlabel='sensor time (s)',ylabel='visual translation increment (mm)',title='Visual update actually applied')
    ax[2].plot(t,values('processing_ms'),linewidth=0.8)
    ax[2].set(xlabel='sensor time (s)',ylabel='processing (ms)',title='VIO step in accelerated replay')
    fig.suptitle('FAST-LIVO2 frontend baseline | trial calibration | no ground truth')
    fig.tight_layout();fig.savefig(args.out/'overview.png',dpi=160);plt.close(fig)
    hashes={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in args.out.iterdir() if f.is_file()}
    (args.out/'sha256.json').write_text(json.dumps(hashes,indent=2)+'\n')
    print(json.dumps(result,indent=2))
