#!/usr/bin/env python3
"""Plot the saved offline assessment; does not recompute or calibrate time."""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parents[2]/'outputs/fast-livo-apriltag-landmark-assessment-20261029'
rows = [json.loads(line) for line in (SOURCE/'time_sensitivity.jsonl').read_text().splitlines()]
fig, axes = plt.subplots(2, 2, figsize=(12, 7), constrained_layout=True)
for column, method in enumerate(['pcl', 'gicp']):
    saved = json.loads((SOURCE/f'{method}_zero_offset.json').read_text())
    for i, fit in enumerate(saved['fixed_trajectory_train_holdout_fits']):
        axes[0, column].plot([row['image_index'] for row in fit['frame_errors']],
                            [row['rmse_px'] for row in fit['frame_errors']],
                            'o-', label=f'Training mode {i}')
    axes[0, column].axhline(3, color='red', linestyle=':', label='Current 3 px RMS limit')
    axes[0, column].axvline(738.5, color='grey', linestyle='--', label='Train / held-out split')
    axes[0, column].set(title=f'{method.upper()}: zero-offset fixed Tag fit',
                        xlabel='Image index (only detected frames)', ylabel='Raw-pixel RMS (px)')
    available = [row for row in rows if row['method'] == method and row['available']]
    # Choose each seed using training error only; never choose by held-out score.
    seeds = [min(row['fits'], key=lambda fit: fit['train_rmse_px']['p50']) for row in available]
    for split in ['train', 'holdout']:
        axes[1, column].plot([row['assumed_offset_ns']/1e6 for row in available],
                            [fit[f'{split}_rmse_px']['p50'] for fit in seeds], 'o-', label=split)
    axes[1, column].axhline(3, color='red', linestyle=':')
    axes[1, column].axvline(0, color='grey', linestyle='--')
    axes[1, column].set(title='Time sensitivity only; no calibrated offset',
                        xlabel='Assumed image-stamp offset (ms)', ylabel='Median frame RMS (px)')
    for ax in axes[:, column]:
        ax.grid(alpha=.25)
        ax.legend(fontsize=8)
fig.suptitle('Tag 1, 0.155 m: single capture, 14 observations; no VALID registration')
fig.savefig(HERE/'assessment.png', dpi=160)
