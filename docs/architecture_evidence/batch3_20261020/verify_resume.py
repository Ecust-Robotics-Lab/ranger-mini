#!/usr/bin/env python3
"""Read-only batch 3 checks; writes verification.json, starts no ROS nodes."""
import ast
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
BASE = json.loads((ROOT/'docs/architecture_evidence/batch1_20261019/baseline.json').read_text())
PREVIOUS = json.loads((ROOT/'docs/architecture_evidence/batch2_20261019/verification.json').read_text())
COMMANDS = []


def run(argv, cwd=ROOT):
    result = subprocess.run(argv, cwd=cwd, capture_output=True, text=True)
    COMMANDS.append({'argv': argv, 'cwd': str(cwd), 'exit_code': result.returncode,
                     'stdout': result.stdout, 'stderr': result.stderr})
    if result.returncode:
        raise RuntimeError(f'{argv}: {result.stderr}')
    return result.stdout.strip()


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    checks = {}
    changed = run(['git', 'diff', '--name-only', 'HEAD']).splitlines()
    untracked = run(['git', 'ls-files', '--others', '--exclude-standard']).splitlines()
    excluded = [p for p in changed+untracked if p == 'docs/03_mid360_validation.md'
                or p.startswith('docs/03_evidence/')]
    ours = sorted(set(changed+untracked)-set(excluded))
    for name in ['field_session', 'field_aux', 'run_navigation', 'start_base_only',
                 'start_lidar_only', 'load_runtime']:
        run(['bash', '-n', 'scripts/'+name+'.sh'])
    checks['shell_syntax'] = True
    for name in ours:
        if name.endswith('.py'):
            ast.parse((ROOT/name).read_text(), filename=name)
    checks['python_ast'] = True
    run(['git', 'diff', '--check', 'HEAD'])
    checks['diff_check'] = True

    pairs = [{'source': p['source'], 'installed': p['installed']}
             for p in PREVIOUS['installed_comparison']]
    pairs.append({'source': 'ws_nav/src/ranger_localization_bridge/ranger_localization_bridge/alignment.py',
                  'installed': 'ws_nav/install/ranger_localization_bridge/lib/python3.10/site-packages/ranger_localization_bridge/alignment.py'})
    for pair in pairs:
        pair['source_sha256'] = sha(ROOT/pair['source'])
        pair['installed_sha256'] = sha(ROOT/pair['installed'])
        pair['equal'] = pair['source_sha256'] == pair['installed_sha256']
    checks['installed_source_matches'] = all(p['equal'] for p in pairs)

    assets = {}
    for name, data in BASE['maps'].items():
        assets['map_'+name] = all(sha(ROOT/p) == item['sha256'] for p, item in data['files'].items())
    for name in PREVIOUS['preserved_assets']:
        if name.startswith('map_'):
            continue
        if name in BASE['configuration_and_patch_files']:
            expected = BASE['configuration_and_patch_files'][name]['sha256']
        else:
            original = subprocess.run(['git', 'show', 'HEAD:'+name], cwd=ROOT,
                                      check=True, capture_output=True).stdout
            expected = hashlib.sha256(original).hexdigest()
        assets[name] = sha(ROOT/name) == expected
    for name, item in BASE['active_bag_verification'].items():
        path = ROOT/'bags/mid360-manual-loop-p42bk0yi/raw'/name
        assets[str(path.relative_to(ROOT))] = sha(path) == item['expected_sha256']
    checks['preserved_assets'] = all(assets.values())

    repos = []
    for original in BASE['repositories'][1:]:
        path = ROOT/original['path']
        head = run(['git', 'rev-parse', 'HEAD'], path)
        branch = run(['git', 'branch', '--show-current'], path) or None
        status = run(['git', 'status', '--short'], path)
        diff = run(['git', 'diff', 'HEAD', '--binary'], path)
        digest = hashlib.sha256(diff.encode()).hexdigest()
        repos.append({'path': original['path'], 'head': head, 'branch': branch,
                      'detached': branch is None, 'status': status, 'tracked_diff_sha256': digest,
                      'unchanged_since_batch1': head == original['head'] and branch == original['branch']
                      and status == original['status'].strip() and digest == original['tracked_diff_sha256']})
    checks['nested_repositories_preserved'] = all(r['unchanged_since_batch1'] for r in repos)

    log_markers = {
        'alignment_verified.log': ['Ran 7 tests', '\nOK\n'],
        'process_verified.log': ['Ran 11 tests', '\nOK\n'],
        'contract_verified.log': ['Ran 11 tests', '\nOK\n'],
        'bridge_verified.log': ['14/14 checks passed'],
        'gate_verified.log': ['12/12 checks passed'],
        'safety_verified.log': ['37/37 checks passed'],
        'managed_complete.log': ['Ran 1 test', '\nOK\n'],
        'build_complete.log': ['Summary: 3 packages finished'],
        'alignment_resume_20261026.log': ['Ran 7 tests', '\nOK\n'],
        'process_resume_20261026.log': ['Ran 11 tests', '\nOK\n'],
    }
    for name, markers in log_markers.items():
        log = (EVIDENCE/name).read_text()
        checks[name] = all(marker in log for marker in markers)
    states = {name: json.loads((EVIDENCE/'managed_complete'/name/'state.json').read_text())
              for name in ['field', 'navigation']}
    checks['managed_cleanup_empty'] = all(not s['owned'] and s['guardian_cleanup_complete']
                                         for s in states.values())
    checks['managed_expected_final_states'] = states['field']['state'] == 'STOPPED' and states['navigation']['state'] == 'FAILED'
    checks['managed_no_physical_components'] = not ({'base', 'lidar', 'localization'} & set(states['field']['components']))
    verification_name = str((EVIDENCE/'verification.json').relative_to(ROOT))
    result = {
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'branch': run(['git', 'branch', '--show-current']),
        'pre_commit_head': run(['git', 'rev-parse', 'HEAD']),
        'scope': '2026-10-26 static audit plus two fresh offline suites; other full-suite logs are from 2026-10-20; no hardware test',
        'checks': checks, 'installed_comparison': pairs, 'preserved_assets': assets,
        'nested_repositories': repos,
        'changed_and_new_files_sha256': {p: sha(ROOT/p) for p in ours if p != verification_name},
        'excluded_existing_work': {p: sha(ROOT/p) for p in excluded},
        'commands': COMMANDS,
    }
    (EVIDENCE/'verification.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(checks, ensure_ascii=False, indent=2))
    return 0 if all(checks.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
