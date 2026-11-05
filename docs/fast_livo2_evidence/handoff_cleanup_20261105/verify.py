#!/usr/bin/env python3
"""Read-only handoff integrity/link checks; no ROS or hardware startup."""
import ast
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess

import yaml

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
DOCS = ['AGENTS.md', 'README.md', 'docs/HANDOFF.md', 'docs/STATUS.md',
        'docs/README.md', 'docs/PROJECT_LAYOUT.md', 'docs/FAST_LIVO2_BACKEND_PLAN.md',
        'docs/FAST_LIVO2_APRILTAG_PLAN.md', 'docs/fast_livo2_recording.md', 'docs/NEXT_AGENT_PROMPT.md',
        'docs/handoff_cleanup_20261105.md']


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8*1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def main():
    before = json.loads((HERE/'before.json').read_text())
    plan = json.loads((HERE/'delete_plan.json').read_text())
    deletion = json.loads((HERE/'deletion_result.json').read_text())
    for name in deletion['deleted']:
        assert not (ROOT/name).exists(), 'delete target remains: '+name
    for name, digest in before['worktree_source_sha256'].items():
        assert sha(ROOT/name) == digest, 'source/config changed: '+name
    for name, digest in before['protected_map_files_sha256'].items():
        assert sha(ROOT/name) == digest, 'map changed: '+name
    deleted_captures = [r['path'] for r in plan['captures']]
    protected_dbs = []
    for name, info in before['raw_db_before'].items():
        if any(name.startswith(folder+'/') for folder in deleted_captures):
            continue
        stat = (ROOT/name).stat()
        assert stat.st_size == info['bytes'] and stat.st_mtime_ns == info['mtime_ns'], 'retained DB changed: '+name
        protected_dbs.append(name)
    assert len(protected_dbs) == 16
    config = yaml.safe_load((ROOT/'config/fast_livo2/ranger_mid360s_hik_trial.yaml').read_text())['/**']['ros__parameters']
    assert config['time_offset']['img_time_offset'] == 0.1
    assert config['uav']['gravity_align_en'] and config['common']['require_livo']
    assert config['publish']['native_tf'] is False
    tag = json.loads((ROOT/'config/fast_livo2/apriltag_camera.json').read_text())
    assert tag['offset_ns'] == 0 and tag['time_verified'] is False
    binaries = {
        'ws_fast_livo/install_c1_confirmation_20261104/ranger_fast_livo_localizer/lib/ranger_fast_livo_localizer/localizer_node':
            '99453aca95043aba1beb78f11fb1a08fb0805140c964e74ec7ad44a32d0bb619',
        'ws_fast_livo/install/fast_livo/lib/fast_livo/fastlivo_mapping':
            'dfcf5d4d54f7b81f573a000589527afa36f4dc71c5fe44df1c94f5bff9ae7b27'}
    for name, digest in binaries.items():
        assert sha(ROOT/name) == digest, 'binary changed: '+name
    links = []
    for name in DOCS:
        path = ROOT/name
        text = path.read_text()
        assert all(line.rstrip() == line for line in text.splitlines()), 'trailing whitespace: '+name
        for match in re.finditer(r'\[[^\]]*\]\(([^)]+)\)', text):
            target = match.group(1).strip().strip('<>')
            if target.startswith(('https://', 'http://', '#', 'mailto:')):
                continue
            target = target.split('#', 1)[0]
            if not target:
                continue
            link = Path(target) if target.startswith('/') else path.parent/target
            assert link.exists(), 'missing link: '+name+' -> '+target
            links.append({'source': name, 'target': target})
    for path in [HERE/'cleanup.py', Path(__file__)]:
        ast.parse(path.read_text())
    sessions = []
    for name in ['fast_livo_live', 'fast_livo_record', 'fast_livo_backend', 'fast_livo_localizer', 'fast_livo_relocalizer']:
        current = ROOT/'.runtime/sessions'/name/'current.json'
        pointer = json.loads(current.read_text())
        state = json.loads((Path(pointer['run_directory'])/'state.json').read_text())
        assert not state.get('owned'), 'owned process remains: '+name
        live = []
        for identity in [state.get('guardian'), state.get('owner')]:
            if not isinstance(identity, dict) or not identity.get('pid'):
                continue
            proc = Path('/proc')/str(identity['pid'])/'stat'
            try:
                tail = proc.read_text().rsplit(')', 1)[-1].split()
                if int(tail[19]) == identity.get('start_ticks'):
                    live.append(identity['pid'])
            except FileNotFoundError:
                pass
        assert not live, 'session remains live: '+name
        sessions.append({'kind': name, 'state': state['state'], 'owned': [], 'matching_live_identities': live})
    head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    assert head == before['head']
    branch = subprocess.check_output(['git', 'branch', '--show-current'], cwd=ROOT, text=True).strip()
    assert branch == before['branch']
    free = shutil.disk_usage(ROOT).free
    result = {'passed': True, 'source_config_files_sha256_unchanged': len(before['worktree_source_sha256']),
              'map_files_sha256_unchanged': len(before['protected_map_files_sha256']),
              'retained_databases_size_mtime_unchanged': len(protected_dbs),
              'retained_databases': protected_dbs, 'binary_sha256_unchanged': binaries,
              'current_document_links_checked': len(links), 'links': links,
              'frontend_trial_img_time_offset_s': 0.1, 'tag_offset_ns': 0, 'tag_time_verified': False,
              'raw_capture_directories_deleted': len(deleted_captures),
              'payload_bytes_removed': deletion['payload_bytes_removed'],
              'free_bytes_before': before['free_bytes_before'], 'free_bytes_after': free,
              'free_bytes_change': free-before['free_bytes_before'], 'sessions': sessions,
              'git_head_unchanged': head, 'git_branch': branch,
              'new_algorithm_tests_run': False, 'hardware_started': False,
              'binary_deployment_performed': False, 'git_commit_or_push': False,
              'current_docs_sha256': {name: sha(ROOT/name) for name in DOCS}}
    (HERE/'verification.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ['links', 'current_docs_sha256', 'retained_databases', 'binary_sha256_unchanged']}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
