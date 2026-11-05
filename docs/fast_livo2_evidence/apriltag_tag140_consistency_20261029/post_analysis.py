#!/usr/bin/env python3
"""Reproduce plots and residual stratification from immutable offline outputs."""
import importlib.util
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SOURCE = ROOT/'outputs/fast-livo-apriltag-tag140-consistency-20261029'
SCREEN = ROOT/'outputs/fast-livo-apriltag-tag140-check-20261029-v2'
spec = importlib.util.spec_from_file_location('tag_geometry', ROOT/'scripts/fast_livo2/tags/assess_landmark.py')
geometry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(geometry)
summary = json.loads((SOURCE/'summary.json').read_text())
observations = [json.loads(line) for line in (SCREEN/'observations.jsonl').read_text().splitlines()]
origin = observations[0]['stamp_ns']
best_offset = summary['diagnostic_training_minimum_offset_ns']
zero = json.loads((SOURCE/'fit_offset_0.json').read_text())['training_selected_fit']
best = json.loads((SOURCE/f'fit_offset_{best_offset}.json').read_text())['training_selected_fit']
split = json.loads((SOURCE/'split.json').read_text())
heldout = set(split['holdout_image_indices'])
timing_rows = [json.loads(line) for line in (SOURCE/'camera_writer_associations.jsonl').read_text().splitlines()]
anomalies = summary['timing']['anomalies']
gap_times = [(row['shared_stamp_ns']-origin)*1e-9 for row in anomalies if row['inside_recorded_lidar_range']]

def stats(values):
    return geometry.common.quantiles(values)

def frame_seconds(row):
    return (row['stamp_ns']-origin)*1e-9

blocks = []
for block in range(12):
    start, end = block*5, (block+1)*5
    z = [row for row in zero['frame_errors'] if start <= frame_seconds(row) < end]
    b = [row for row in best['frame_errors'] if start <= frame_seconds(row) < end]
    blocks.append({'start_s': start, 'end_s': end, 'count': len(z),
                   'heldout_count': sum(row['image_index'] in heldout for row in z),
                   'zero_rmse_px': stats([row['rmse_px'] for row in z]),
                   'diagnostic_offset_rmse_px': stats([row['rmse_px'] for row in b])})
far = [row for row in best['frame_errors'] if min(abs(frame_seconds(row)-t) for t in gap_times) > .5]
derived = {'diagnostic_offset_ns': best_offset, 'blocks': blocks,
           'outside_half_second_of_shared_stamp_anomaly': {
               'frames': len(far), 'rmse_px': stats([row['rmse_px'] for row in far]),
               'frames_over_initial_3px_rms': sum(row['rmse_px'] > 3 for row in far)},
           'note': 'The 0.5 s neighborhood is descriptive stratification, not an exclusion or acceptance gate'}
(HERE/'residual_stratification.json').write_text(json.dumps(derived, indent=2)+'\n')

fig, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
ax = axes[0, 0]
ax.plot([(r['shared_stamp_ns']-origin)*1e-9 for r in timing_rows],
        [r['writer_age_at_camera_arrival_ms'] for r in timing_rows], '.', markersize=3)
ax.set(title='Shared stamp age at camera arrival (software timing)', xlabel='Copied scan stamp relative to first image (s)',
       ylabel='Camera arrival minus consumed writer arrival (ms)')
for t in gap_times:
    ax.axvline(t, color='red', alpha=.25)
ax = axes[0, 1]
for fit, label in [(zero, 'Zero offset'), (best, f'+{best_offset/1e6:.2f} ms diagnostic')]:
    ax.plot([frame_seconds(r) for r in fit['frame_errors']], [r['rmse_px'] for r in fit['frame_errors']], '.',
            markersize=3, label=label)
ax.axhline(3, color='red', linestyle=':', label='Initial 3 px RMS limit')
ax.set(title='One fixed Tag pose, fixed local trajectory', xlabel='Image stamp relative to first image (s)', ylabel='Raw-pixel RMS (px)')
ax.legend(fontsize=8)
ax = axes[1, 0]
scores = json.loads((SOURCE/'offset_sensitivity.json').read_text())
fits = [r['fits'][r['training_selected_seed']] for r in scores]
x = [r['assumed_offset_ns']/1e6 for r in scores]
ax.plot(x, [f['training_huber_coordinate_cost_mean'] for f in fits], 'o-', label='Training Huber coordinate cost')
ax.set(title='Common cohort, training selection only', xlabel='Assumed image-stamp offset (ms)', ylabel='Training cost (initial 1 px Huber scale)')
ax.axvline(best_offset/1e6, color='grey', linestyle='--')
right = ax.twinx()
right.plot(x, [f['holdout_rmse_px']['p95'] for f in fits], 's-', color='orange', label='Held-out frame RMS P95')
right.set_ylabel('Held-out RMS P95 (px)')
handles = ax.get_lines()[:1]+right.get_lines()
ax.legend(handles, [line.get_label() for line in handles], loc='upper right', fontsize=8)

ax = axes[1, 1]
capture = Path(json.loads((SCREEN/'summary.json').read_text())['capture'])
manifest = json.loads((capture/'manifest.json').read_text())
archived = next(row for row in manifest['source_files'] if row['source'].endswith('/apriltag_camera.json'))
E = geometry.common.matrix(json.loads((capture/archived['file']).read_text())['T_imu_camera'])
positions, stamps = [], []
for row in observations:
    if row['local_pose_association']['available']:
        camera = geometry.common.matrix(row['local_pose_association']['pose'])@E
        tag = camera@geometry.common.matrix(row['observation']['hypotheses'][0]['T_camera_tag'])
        positions.append(tag[:3, 3])
        stamps.append(frame_seconds(row))
center = np.median(positions, axis=0)
ax.plot(stamps, np.linalg.norm(np.asarray(positions)-center, axis=1)*1000, '.', markersize=3)
ax.set(title='Conditional local Tag positions before fixed-pose fitting', xlabel='Image stamp relative to first image (s)',
       ylabel='Distance from median position (mm); not absolute error')
for ax in axes.ravel():
    ax.grid(alpha=.25)
fig.suptitle('140 mm Tag 1: offline diagnosis only; no clock or map update')
fig.savefig(HERE/'timing_and_consistency.png', dpi=160)
