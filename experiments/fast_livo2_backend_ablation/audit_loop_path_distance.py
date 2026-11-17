#!/usr/bin/env python3
"""Audit distances along immutable frontend keyframes, without running devices or PGO."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


THRESHOLDS_M = (0, 3, 5, 10)


def audit(trial):
    archive = trial / 'capture/map'
    manifest_bytes = (archive / 'manifest.json').read_bytes()
    manifest = json.loads(manifest_bytes)
    committed = json.loads((archive / 'COMMITTED').read_text())
    digest = hashlib.sha256(manifest_bytes).hexdigest()
    if committed['manifest_sha256'] != digest:
        raise ValueError('archive commit checksum mismatch')
    frames = manifest['keyframes']
    if [f['id'] for f in frames] != list(range(len(frames))):
        raise ValueError('noncontiguous keyframe IDs')
    positions = np.array([f['U_local_imu']['xyz'] for f in frames], dtype=float)
    if not np.isfinite(positions).all():
        raise ValueError('nonfinite frontend position')
    prefix = np.r_[0., np.cumsum(np.linalg.norm(np.diff(positions, axis=0), axis=1))]
    records = [json.loads(line) for line in (archive / 'candidates.jsonl').read_text().splitlines() if line.strip()]
    pairs = []
    for row in records:
        if 'i' not in row or 'j' not in row:
            continue
        i, j = int(row['i']), int(row['j'])
        if not 0 <= i < j < len(frames):
            raise ValueError('invalid historical candidate IDs')
        pairs.append({'trial': trial.name, 'i': i, 'j': j,
            'decision': row.get('decision', ''), 'candidate_source': row.get('candidate_source', 'unspecified'),
            'odom_keyframe_chain_path_m': float(prefix[j] - prefix[i]),
            'frontend_endpoint_distance_m': float(np.linalg.norm(positions[j] - positions[i]))})
    summary = {'trial': trial.name, 'manifest_sha256': digest,
               'keyframe_chain_total_m': float(prefix[-1]), 'keyframes': len(frames)}
    for name, subset in [('logged_candidates', pairs),
                         ('accepted_events', [p for p in pairs if p['decision'] == 'accepted_and_optimized'])]:
        distances = [p['odom_keyframe_chain_path_m'] for p in subset]
        summary[name] = {'count': len(distances),
            'quantiles_m': dict(zip(('minimum', 'p25', 'median', 'p75', 'maximum'),
                                   np.quantile(distances, (0, .25, .5, .75, 1)).tolist())) if distances else {},
            'below_threshold_counts': {str(t): sum(d < t for d in distances) for t in THRESHOLDS_M}}
    return summary, pairs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--group', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    summaries, pairs = [], []
    for trial in sorted(args.group.glob('trial_*_attempt_01')):
        summary, rows = audit(trial)
        summaries.append(summary)
        pairs.extend(rows)
    if not summaries:
        raise ValueError('no trial archives found')
    args.output.mkdir(parents=True, exist_ok=False)
    result = {'distance_definition': 'sum of 3D translation chords between consecutive raw U_local_imu keyframes; approximate odometry-chain length, not independently measured route length',
        'scope': 'only candidates logged by the old backend after its original search/diversification/budget; counts are not predictions of a new replay',
        'threshold_comparison': 'reject when chain_path_m < threshold_m; equality allowed',
        'thresholds_m': THRESHOLDS_M, 'trials': summaries}
    (args.output / 'distance_audit.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    with (args.output / 'candidate_distances.csv').open('w') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(pairs[0]))
        writer.writeheader()
        writer.writerows(pairs)
    print(json.dumps(summaries, ensure_ascii=False))


if __name__ == '__main__':
    main()
