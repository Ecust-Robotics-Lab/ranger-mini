#!/usr/bin/env python3
"""Reference-assisted candidate coverage, not ground-truth recall or accuracy."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def summarize(folder):
    path = folder/'queries.jsonl'
    rows = [json.loads(s) for s in path.read_text().splitlines()]
    result = {'folder': str(folder.resolve()), 'queries_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
              'setup': json.loads((folder/'setup.json').read_text()), 'ground_truth': False, 'splits': {}}
    for split in ('development', 'heldout_same_visit'):
        subset = [r for r in rows if r['split']==split]
        methods = {}
        for method in ('v1','v2','sc20','sc80','std'):
            records = [r['methods'][method] for r in subset]
            matches, disagreements, geometry_queries = [], [], 0
            for row in subset:
                candidates = row['methods'][method]['candidates']
                if any('geometry' in c for c in candidates): geometry_queries += 1
                good = [c for c in candidates if c.get('geometry',{}).get('fine',{}).get('geometrically_accepted')]
                if good: matches.append(row['t'])
                disagreements += [c for c in good if c['geometry'].get('reference_pose_translation_error_m',0)>3.]
            methods[method] = {
                'queries': len(records),
                'reference_region_hits_at_1': sum(bool(r['candidates']) and r['candidates'][0]['reference_distance_m']<3. for r in records),
                'reference_region_hits_at_5': sum(any(c['reference_distance_m']<3. for c in r['candidates']) for r in records),
                'empty_candidate_queries': sum(not r['candidates'] for r in records),
                'feature_count_p50_p95': np.percentile([r['features'] for r in records],[50,95]).tolist() if records else [],
                'retrieval_ms_p50_p95_max': np.percentile([r['elapsed_ms'] for r in records],[50,95,100]).tolist() if records else [],
                'queries_with_evaluated_geometry': geometry_queries,
                'queries_with_any_geometric_match': len(matches),
                'first_geometric_match_seconds': min(matches) if matches else None,
                'accepted_candidates_disagreeing_with_reference_over_3m': len(disagreements)}
        result['splits'][split] = methods
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder', type=Path);p.add_argument('output', type=Path)
    args=p.parse_args();result=summarize(args.folder)
    with args.output.open('x') as f: json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps(result['splits'],indent=2))
