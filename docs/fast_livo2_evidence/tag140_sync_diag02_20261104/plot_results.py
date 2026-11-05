#!/usr/bin/env python3
"""Reproducible figures from recorded diagnostics; no hardware/ROS graph."""
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
CAPTURE=ROOT/'bags/fast-livo-tag140_sync_diag_02-35f_m8nd'
IMU=ROOT/'outputs/fast-livo-tag140-sync-diag02-imu-bias-20261029'
EVAL=ROOT/'outputs/fast-livo-tag140-sync-diag02-replay-current-evaluation-20261104'
SCREEN=ROOT/'outputs/fast-livo-tag140-sync-diag02-screen-20261029'

def main():
    imu=json.loads((IMU/'summary.json').read_text());origin=imu['image_origin_ns']
    with (CAPTURE/'sensor_assets/timeshare_writer.csv').open() as stream:writer=list(csv.DictReader(stream))
    profiles=json.loads((IMU/'bias_corrected_offset_profile.json').read_text())
    evaluation=json.loads((EVAL/'summary.json').read_text())
    frames=[json.loads(l) for l in (SCREEN/'frames.jsonl').read_text().splitlines()]
    gyro=[json.loads(l) for l in (IMU/'raw_gyro.jsonl').read_text().splitlines()]
    fig,axes=plt.subplots(2,2,figsize=(14,8.5),constrained_layout=True)
    ax=axes[0,0]
    x=[(int(r['scan_start_ns'])-origin)*1e-9 for r in writer]
    y=[(int(r['callback_system_ns'])-int(r['dds_source_ns']))*1e-6 for r in writer]
    ax.plot(x,y,'.',markersize=3)
    ax.set(title='RELIABLE writer: middleware publish to callback',xlabel='Scan stamp relative to first recorded image (s)',
           ylabel='Host software delay (ms)',xlim=(0,120))
    ax.text(.02,.96,'1200 recorded scans all present in writer\nNo publication-sequence gaps; no reported DDS loss',
            transform=ax.transAxes,va='top',fontsize=9)
    ax=axes[0,1]
    x=[r['offset_ns']/1e6 for r in profiles]
    for split,label in [('train','Training (96 intervals)'),('holdout','Held-out (47 intervals)')]:
        ax.plot(x,[r['subsets'][split]['primary_clipped_mean_deg'] for r in profiles],label=label)
    ax.axvline(120,color='grey',linestyle='--',label='+120 ms training hypothesis')
    ax.set(title='Tag / raw-gyro timing sensitivity (static bias removed)',
           xlabel='Assumed effective image-stamp offset (ms)',ylabel='Mean relative-rotation error (deg)')
    ax.legend(fontsize=8)
    ax=axes[1,0]
    for offset,label in [(0,'Replay: zero offset'),(120000000,'Replay: +120 ms hypothesis')]:
        fit=json.loads((EVAL/f'fit_{offset}.json').read_text())['selected_fit']
        ax.plot([(r['stamp_ns']-origin)*1e-9 for r in fit['frame_errors']],
                [r['rmse_px'] for r in fit['frame_errors']],'.',markersize=3,label=label,alpha=.65)
    ax.axhline(3,color='red',linestyle=':',label='Initial 3 px RMS check')
    ax.set(title='Fixed Tag, current-source frontend replays; same cohort',
           xlabel='Original image stamp relative to first image (s)',ylabel='Raw-pixel reprojection RMS (px)',yscale='log')
    ax.legend(fontsize=8)
    ax=axes[1,1]
    bias=np.asarray(imu['image_static_bias_sensitivity']['static_bias_diagnostic']['bias_radps'])
    t=np.array([(r['stamp_ns']-origin)*1e-9 for r in gyro]);w=np.array([r['gyro'] for r in gyro])
    ax.plot(t,np.linalg.norm(w-bias,axis=1),linewidth=.5,label='Bias-subtracted raw gyro norm')
    missing=[(r['stamp_ns']-origin)*1e-9 for r in frames if not r['detections']]
    ax.plot(missing,np.zeros(len(missing)),'|',color='red',markersize=6,label='Tag not detected in frame')
    ax.set(title='Observed motion and visibility; no prescribed schedule',xlabel='Relative sensor time (s)',
           ylabel='Angular rate (rad/s)',xlim=(0,120))
    ax.legend(fontsize=8)
    for ax in axes.ravel():ax.grid(alpha=.25)
    fig.suptitle('120 s Tag 1 diagnostic: delivery improved; physical synchronization remains unverified')
    fig.savefig(HERE/'diagnostic_results.png',dpi=160)
    fig.savefig(HERE/'diagnostic_results.svg')
    summary={'cohort':evaluation['common_cohort'],'training_frames':evaluation['training_count'],
        'heldout_frames':evaluation['holdout_count'],'cases':[],'physical_sync_verified':False}
    for case in evaluation['cases']:
        fit=case['selected_fit']
        summary['cases'].append({'offset_ns':case['offset_ns'],'holdout_rmse_px':fit['holdout_rmse_px'],
             'frames_passing_initial_limits':fit['frames_passing_initial_3px_rms_6px_max']})
    (HERE/'comparison_summary.json').write_text(json.dumps(summary,indent=2)+'\n')

if __name__=='__main__':main()
