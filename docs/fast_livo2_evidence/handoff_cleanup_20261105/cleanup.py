#!/usr/bin/env python3
"""Exact-list cleanup authorized for the 2026-11-05 handoff; default dry-run."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
CAPTURES = ['bags/fast-livo-tag_check_01-xxl8kg83',
            'bags/fast-livo-tag_check_01-pcnds53t',
            'bags/fast-livo-tag_check_01-gbglapyw']
EMPTY = ['bags/fast-livo2-stage-b-chain', 'bags/fast-livo2-stage-b-chain-final',
         'bags/stage-b-acceptance-i5zzwvdx']


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8*1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def active_users(names):
    found = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            for fd in (proc/'fd').iterdir():
                try:
                    target = str(fd.resolve(strict=True))
                    if any(target == str(ROOT/name) or target.startswith(str(ROOT/name)+'/') for name in names):
                        found.append({'pid': int(proc.name), 'target': target})
                except (OSError, RuntimeError):
                    pass
        except OSError:
            pass
    return found


def prepare():
    assert not (HERE/'delete_plan.json').exists()
    assert not active_users(CAPTURES+EMPTY), 'capture still open'
    plan = {'authorization': 'User 2026-11-05 explicitly requested deletion of obsolete files/records/bags during handoff.',
            'classification': 'Three unreferenced early 60s trial captures superseded by the diagnosed 140mm datasets; original recordings, not byte-identical copies. Their raw payloads will not remain recoverable.',
            'captures': [], 'empty_directories': EMPTY, 'cache_directories': [],
            'expected_map_files': len(json.loads((HERE/'before.json').read_text())['protected_map_files_sha256'])}
    for name in CAPTURES:
        path = ROOT/name
        assert path.is_dir() and not path.is_symlink() and path.resolve().parent == ROOT/'bags'
        assert not subprocess.check_output(['git', 'ls-files', '--', name], cwd=ROOT)
        references = subprocess.run(['rg', '-l', '--fixed-strings', Path(name).name,
                                     'docs', 'scripts', 'tests', 'config', 'README.md', 'AGENTS.md'],
                                    cwd=ROOT, capture_output=True, text=True)
        refs = [r for r in references.stdout.splitlines()
                if not r.startswith('docs/fast_livo2_evidence/handoff_cleanup_20261105/')]
        assert not refs, 'unexpected existing capture reference: '+str(refs)
        files = []
        for p in sorted(path.rglob('*')):
            assert not p.is_symlink(), 'unexpected symlink'
            if p.is_file():
                files.append({'path': str(p.relative_to(path)), 'bytes': p.stat().st_size,
                              'mtime_ns': p.stat().st_mtime_ns, 'sha256': sha(p)})
        plan['captures'].append({'path': name, 'bytes': sum(f['bytes'] for f in files),
                                 'existing_references': refs, 'files': files})
    for name in EMPTY:
        path = ROOT/name
        assert path.is_dir() and not any(path.iterdir()), 'not empty: '+name
    for base in ['scripts', 'tests', 'ws_fast_livo/src', 'ws_camera/src/timeshare_writer']:
        for path in (ROOT/base).rglob('__pycache__'):
            if path.is_dir() and not path.is_symlink():
                name = str(path.relative_to(ROOT))
                assert not subprocess.check_output(['git', 'ls-files', '--', name], cwd=ROOT)
                plan['cache_directories'].append(name)
    plan['capture_bytes'] = sum(r['bytes'] for r in plan['captures'])
    (HERE/'delete_plan.json').write_text(json.dumps(plan, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'action': 'PREPARED', 'captures': [r['path'] for r in plan['captures']],
                      'capture_gib': plan['capture_bytes']/2**30, 'empty_directories': len(EMPTY),
                      'cache_directories': len(plan['cache_directories'])}, ensure_ascii=False), flush=True)


def execute():
    assert not (HERE/'deletion_result.json').exists(), 'cleanup already recorded'
    plan = json.loads((HERE/'delete_plan.json').read_text())
    assert [r['path'] for r in plan['captures']] == CAPTURES
    assert plan['empty_directories'] == EMPTY
    assert not active_users(CAPTURES+EMPTY), 'capture still open'
    result = {'deleted': [], 'payload_bytes_removed': 0, 'raw_capture_count': 0,
              'historical_reports_and_unique_failure_evidence_preserved': True}
    for record in plan['captures']:
        path = ROOT/record['path']
        assert path.resolve().parent == ROOT/'bags' and not path.is_symlink()
        current = sorted(str(p.relative_to(path)) for p in path.rglob('*') if p.is_file())
        assert current == sorted(r['path'] for r in record['files'])
        for f in record['files']:
            p = path/f['path']
            assert not p.is_symlink() and p.stat().st_size == f['bytes']
            assert p.stat().st_mtime_ns == f['mtime_ns']
        archive = HERE/'removed_capture_metadata'/path.name
        archive.mkdir(parents=True, exist_ok=False)
        for name in ['manifest.json', 'result.json', 'audit.json', 'recorder_result.json',
                     'record_command.json', 'recording.json', 'raw/metadata.yaml',
                     'sensor_assets/camera_sync.csv', 'sensor_assets/timeshare_writer.csv']:
            p = path/name
            if p.is_file():
                q = archive/name
                q.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(p, q)
        shutil.rmtree(path)
        assert not path.exists()
        result['deleted'].append(record['path'])
        result['payload_bytes_removed'] += record['bytes']
        result['raw_capture_count'] += 1
    for name in EMPTY:
        (ROOT/name).rmdir()
        result['deleted'].append(name)
    for name in plan['cache_directories']:
        p = ROOT/name
        assert p.name == '__pycache__' and not p.is_symlink()
        if p.exists():
            result['payload_bytes_removed'] += sum(f.stat().st_size for f in p.rglob('*') if f.is_file())
            shutil.rmtree(p)
            result['deleted'].append(name)
    (HERE/'deletion_result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--prepare', action='store_true')
    group.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if args.prepare:
        prepare()
    elif args.execute:
        execute()
    else:
        print(json.dumps({'action': 'DRY-RUN', 'captures': CAPTURES, 'empty_directories': EMPTY}))
