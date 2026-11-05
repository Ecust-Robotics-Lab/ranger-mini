#!/usr/bin/env python3
"""Read-only source/input checks plus current derived evidence summary."""
from pathlib import Path
from collections import defaultdict,Counter
import ast,hashlib,json,re,sys
import numpy as np
ROOT=Path(__file__).resolve().parents[3];HERE=Path(__file__).resolve().parent;OUT=ROOT/'outputs/fast-livo-c1-correction-20261104'
if sys.argv[1:]!=['--execute']:
    print('DRY-RUN: final evidence consistency checks');raise SystemExit(0)
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for block in iter(lambda:f.read(4*1024*1024),b''):h.update(block)
    return h.hexdigest()
baseline=json.loads((HERE/'baseline.json').read_text())
for name in ['scripts/fast_livo2/localizer/validate_motion.py','ws_fast_livo/src/ranger_fast_livo_localizer/src/localizer.cpp','ws_fast_livo/src/ranger_fast_livo_localizer/src/archive_reader.cpp']:
    assert sha(ROOT/name)==baseline['files_sha256'][name]
job=json.loads((OUT/'straight_inputs/job.json').read_text())
assert all(sha(Path(f))==h for f,h in job['source_sha256'].items())
assert all(sha(q['source_pcd'])==q['source_pcd_sha256'] for q in job['queries'])
oldlink=json.loads((HERE.parent/'tag_two_capture_20261104/optimized_live_linkage.json').read_text())
for item in oldlink[0]['libraries']:assert sha(item['path'])==item['sha256']
rows=[json.loads(s) for s in (OUT/'straight_probe/registrations.jsonl').read_text().splitlines()];by=defaultdict(list)
for r in rows:by[r['case']].append(r)
assert len(rows)==2100 and all(len(v)==525 for v in by.values())
tracking={q['index'] for q in job['queries'] if q['recorded_hypotheses']==1}
assert len(tracking)==524
repeat=[r for r in by['pcl_dynamic'] if r['index'] in tracking]
assert all(r['difference_from_recorded_translation_m']==0 and r['difference_from_recorded_rotation_rad']<1e-12 for r in repeat)
score_validity={}
for method,rr in by.items():
    values=[r['registration']['rmse_m'] for r in rr if r['registration']['geometrically_accepted']]
    score_validity[method]={'valid_geometry_score_count':len(values),'point_rmse_m_p50_p95':np.percentile(values,[50,95]).tolist(),
        'rejected_reasons':dict(Counter(r['registration']['reason'] for r in rr if not r['same_original_gates_passed']))}
(HERE/'score_validity.json').write_text(json.dumps({'note':'nonconverged default zero RMSE excluded; frozen comparison remains an immutable selection snapshot','cases':score_validity},indent=2)+'\n')
parsed=[ROOT/'scripts/fast_livo2/tags/diagnose_c1_corrections.py',ROOT/'tests/test_c1_correction_diagnostics.py',*HERE.glob('*.py')]
for f in parsed:ast.parse(f.read_text(),filename=str(f))
docs=[ROOT/'docs'/n for n in ['fast_livo2_c1_correction_report.md','STATUS.md','HANDOFF.md','FAST_LIVO2_APRILTAG_PLAN.md','FAST_LIVO2_BACKEND_PLAN.md','README.md','fast_livo2_localizer_interfaces.md','fast_livo2_tag_two_capture_report.md']]
links=[]
for f in docs:
    for target in re.findall(r'\]\(([^)]+)\)',f.read_text()):
        if '://' in target or target.startswith('#'):continue
        target=target.split('#')[0]
        if target:assert (f.parent/target).exists(),(f,target);links.append(target)
s=json.loads((OUT/'runtime_comparison/summary.json').read_text())
assert s['trials']['straight_gicp']['passed'] and s['trials']['turn_gicp']['passed']
assert s['trials']['straight_gicp']['valid_samples']==520 and s['trials']['straight_gicp']['states']['2']==1
assert all(v==0 for k in ['straight_gicp','turn_gicp'] for v in s['trials'][k]['public_publishers'].values())
assert not s['VALID_landmark_created'] and not s['physical_time_verified']
result={'new_tests':6,'related_geometry_tests':3,'tests_passed':9,'python_files_parsed':len(parsed),'document_links_checked':len(links),
        'derived_clouds_verified':len(job['queries']),'controlled_registrations':len(rows),'PCL_tracking_reproductions':len(tracking),
        'maximum_PCL_reproduction_rotation_difference_rad':max(r['difference_from_recorded_rotation_rad'] for r in repeat),
        'runtime_exit_codes':json.loads((HERE/'runtime_exits.json').read_text()),'original_node_and_library_sources_unchanged':True,
        'default_configuration_and_raw_bag_integrity':'final_integrity.json','physical_time_verified':False,'VALID_landmark_created':False,
        'source_sha256':{str(p.relative_to(ROOT)):sha(p) for p in [*parsed,*docs]},
        'probe_executable_sha256':sha(ROOT/'ws_fast_livo/build_c1_correction_probe_20261104/c1_correction_probe')}
(HERE/'delivery_checks.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k not in ['source_sha256','probe_executable_sha256']},indent=2))
