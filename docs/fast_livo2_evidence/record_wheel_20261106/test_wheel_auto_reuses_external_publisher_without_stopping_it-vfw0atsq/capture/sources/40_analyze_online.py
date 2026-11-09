#!/usr/bin/env python3
"""Summarize recorded private localization; no location/accuracy truth inferred."""
import argparse
from collections import Counter
import json
from pathlib import Path
import numpy as np


def summarize(folder):
    rows = [json.loads(line) for line in (folder/'localization_samples.jsonl').read_text().splitlines()]
    valid = [r for r in rows if r['pose_valid']]
    unique = {(r['request_id'],r['scan_id']):r for r in valid}
    def quantiles(values):
        return np.percentile(values,[50,95,100]).tolist() if values else []
    ages = [(r['clock_ns']-r['stamp_ns'])*1e-9 for r in valid]
    longest = 0.; start = None
    for row in rows:
        if not row['pose_valid']:
            if start is None: start = row['received_steady']
            longest = max(longest, row['received_steady']-start)
        else: start = None
    counters = {k: max((r['metrics'].get(k) or 0) for r in rows)
                for k in ('processed_scans','skipped_scans','stale_results','queued_jobs')}
    return {'folder': str(folder.resolve()), 'workflow': json.loads((folder/'localization_result.json').read_text()),
        'heartbeat_count': len(rows), 'states': dict(Counter(str(r['state']) for r in rows)),
        'reasons': dict(Counter(r['reason'] for r in rows)), 'unique_valid_scans': len(unique),
        'valid_heartbeat_fraction': len(valid)/len(rows) if rows else 0,
        'first_valid_clock_ns': valid[0]['clock_ns'] if valid else None,
        'first_valid_pose': valid[0]['pose'] if valid else None,
        'pose_age_seconds_p50_p95_max': quantiles(ages),
        'valid_age_violations_at_receipt': sum(a<-.5 or a>.5 for a in ages),
        'registration_ms_p50_p95_max': quantiles([r['metrics']['elapsed_ms'] for r in unique.values()
                                               if r['metrics'].get('elapsed_ms') is not None]),
        'longest_invalid_heartbeat_span_seconds': longest, 'counters': counters,
        'location_correctness': 'NOT_EVALUATED', 'absolute_accuracy': 'NOT_EVALUATED',
        'note': 'Heartbeat frequency is not localization frequency; receipt age includes transport delay. '
                'Same-map source replay is a functional check, not independent localization evidence.'}


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path);p.add_argument('output',type=Path)
    args=p.parse_args();result=summarize(args.folder)
    with args.output.open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps(result,indent=2))
