"""Read-only consistency checks for the current handoff/manuals, not algorithm tests."""
import hashlib
import json
from pathlib import Path
import re
import subprocess
from urllib.parse import unquote

ROOT=Path(__file__).resolve().parents[3]
current=['README.md','AGENTS.md','docs/README.md','docs/HANDOFF.md','docs/STATUS.md',
 'docs/FAST_LIVO2_BACKEND_PLAN.md','docs/PROJECT_LAYOUT.md','docs/localization_backend_contract.md',
 'docs/managed_sessions.md','docs/fast_livo2_interfaces.md','docs/documentation_refresh_report.md','calibration/README.md']
checks={};broken=[];links=0
for rel in current:
    text=(ROOT/rel).read_text()
    for target in re.findall(r'\[[^\]]*\]\(([^\s)]+)\)',text):
        target=target.strip('<>')
        if re.match(r'^[a-zA-Z][a-zA-Z0-9+.-]*:',target) or target.startswith('#'):continue
        path=unquote(target.split('#')[0])
        links+=1
        if not (ROOT/rel).parent.joinpath(path).exists():broken.append({'document':rel,'target':target})
checks['current_document_links']={'documents':len(current),'links':links,'broken':broken}
checks['obsolete_files_deleted']={rel:not (ROOT/rel).exists() for rel in ['ARCHITECTURE_PLAN.md','docs/INITIALIZATION.md']}
checks['old_file_links_removed']=all('](ARCHITECTURE_PLAN.md)' not in (ROOT/r).read_text() and '](../ARCHITECTURE_PLAN.md)' not in (ROOT/r).read_text() and '](INITIALIZATION.md)' not in (ROOT/r).read_text() for r in current)
# Only banners were added to these report bodies. Preserve existing user changes.
history=['docs/00_environment_audit.md','docs/LIDAR_CONNECTION.md','docs/02_base_interface.md',
 'docs/02_ros_motion_test.md','docs/03_mid360_validation.md','docs/04_mapping_report.md',
 'docs/05_localization_interface.md','docs/06_flat_navigation_acceptance.md','docs/architecture_baseline.md',
 'docs/architecture_batch2_report.md','docs/architecture_batch3_report.md',
 'docs/fast_livo2_stage_a_report.md','docs/fast_livo2_online_report.md']
checks['historical_bodies_preserved']={rel:(ROOT/rel).read_text().endswith((ROOT/'.runtime/backups/docs_handoff_20261026'/rel).read_text().split('\n',1)[1]) for rel in history}
versions={}
for repo in json.loads((ROOT/'config/fast_livo2/sources.lock.json').read_text())['repositories']:
    p=subprocess.run(['git','-C',str(ROOT/repo['path']),'rev-parse','HEAD'],capture_output=True,text=True)
    versions[repo['name']]={'expected':repo['commit'],'actual':p.stdout.strip(),'matches':p.returncode==0 and p.stdout.strip()==repo['commit']}
checks['locked_sources']=versions
p=subprocess.run(['git','diff','--check'],cwd=ROOT,capture_output=True,text=True)
checks['git_diff_check']={'exit_code':p.returncode,'output':p.stdout+p.stderr}
checks['no_word_files_in_maintained_tree']=not any(p.suffix.lower() in ('.docx','.doc') for base in [ROOT/'docs',ROOT/'calibration/camera_intrinsics',ROOT/'calibration/lidar_camera_extrinsics/docs'] for p in base.rglob('*') if p.is_file())
checks['scope']='Documentation only; no hardware startup, backend implementation or algorithm retest.'
checks['passed']=(not broken and all(checks['obsolete_files_deleted'].values()) and checks['old_file_links_removed'] and all(checks['historical_bodies_preserved'].values()) and all(v['matches'] for v in versions.values()) and p.returncode==0)
Path(__file__).with_name('validation.json').write_text(json.dumps(checks,indent=2,ensure_ascii=False)+'\n')
print(json.dumps({k:v for k,v in checks.items() if k not in ('locked_sources','historical_bodies_preserved')},indent=2,ensure_ascii=False))
assert checks['passed']
