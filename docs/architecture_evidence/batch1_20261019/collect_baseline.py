#!/usr/bin/env python3
"""Read-only baseline collector. Prints JSON; never starts ROS or changes config.

Run from any directory; redirect stdout to a NEW snapshot when repeating.
Requires the project's existing Python environment with PyYAML.
"""
import datetime
import hashlib
import json
import math
from pathlib import Path
import subprocess

import yaml

ROOT = Path(__file__).resolve().parents[3]
commands = []


def run(args, cwd=ROOT):
    p = subprocess.run(args, cwd=cwd, text=True, capture_output=True)
    commands.append({'argv': args, 'cwd': str(cwd), 'exit_code': p.returncode,
                     'stdout': p.stdout, 'stderr': p.stderr})
    return p.stdout.strip() if p.returncode == 0 else None


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def inventory(directory):
    return {str(p.relative_to(ROOT)): {'bytes': p.stat().st_size, 'sha256': sha(p)}
            for p in sorted(directory.rglob('*')) if p.is_file()}


repositories = []
for rel in ['.', 'ws_base/src/ranger_ros2', 'ws_base/src/ugv_sdk',
            'ws_livox/src/livox_ros_driver2', 'vendor/livox/Livox-SDK2',
            'ws_slam/src/lightning', 'vendor/livox/lightning-lm-review',
            'ws_camera/src/hik_camera_ros2_driver']:
    path = ROOT / rel
    top = run(['git', 'rev-parse', '--show-toplevel'], path)
    if top is None or Path(top).resolve() != path.resolve():
        repositories.append({'path': rel, 'error': 'not an independent repository'})
        continue
    branch = run(['git', 'branch', '--show-current'], path)
    diff = run(['git', 'diff', 'HEAD', '--binary'], path)
    repositories.append({
        'path': rel, 'head': run(['git', 'rev-parse', 'HEAD'], path),
        'branch': branch or None, 'detached': branch == '',
        'status': run(['git', 'status', '--short'], path),
        'origin': run(['git', 'remote', 'get-url', 'origin'], path),
        'tracked_diff_sha256': hashlib.sha256(diff.encode()).hexdigest()
        if diff is not None else None,
    })

files = {}
for folder in ['config', 'patches']:
    files.update(inventory(ROOT / folder))
for folder in ['scripts', 'ws_nav/src', 'ws_camera/src/hik_camera_ros2_driver/config']:
    for p in sorted((ROOT / folder).rglob('*')):
        if p.is_file() and '__pycache__' not in p.parts and p.suffix in ['.py', '.sh', '.yaml', '.xml']:
            files[str(p.relative_to(ROOT))] = {'bytes': p.stat().st_size, 'sha256': sha(p)}
camera = ROOT / 'ws_camera/src/hik_camera_ros2_driver/src/hik_camera_node.cpp'
files[str(camera.relative_to(ROOT))] = {'bytes': camera.stat().st_size, 'sha256': sha(camera)}

installed = []
for pkg in ['ranger_navigation', 'ranger_localization_bridge']:
    source = ROOT / 'ws_nav/src' / pkg
    for p in sorted(source.rglob('*')):
        if not p.is_file() or p.suffix not in ['.py', '.yaml', '.xml'] or '__pycache__' in p.parts:
            continue
        rel = p.relative_to(source)
        prefix = ROOT / 'ws_nav/install' / pkg
        if rel.parts[0] in ['launch', 'config', 'behavior_trees']:
            candidates = [prefix / 'share' / pkg / rel]
        elif rel.parts[0] == pkg:
            candidates = list((prefix / 'lib').glob('python*/site-packages/' + str(rel)))
        else:
            continue
        for dest in candidates or [None]:
            exists = dest is not None and dest.is_file()
            installed.append({'source': str(p.relative_to(ROOT)),
                              'installed': str(dest.relative_to(ROOT)) if dest else None,
                              'source_sha256': sha(p),
                              'installed_sha256': sha(dest) if exists else None,
                              'equal': exists and sha(p) == sha(dest)})

maps = {}
for suffix in ['2lx5kkho', '8cw1afzl', 'suej7qsz']:
    path = ROOT / ('maps/lightning-offline-' + suffix)
    manifest = json.loads((path / 'manifest.json').read_text())
    maps[suffix] = {
        'path': str(path.relative_to(ROOT)),
        'provenance': {k: manifest.get(k) for k in [
            'created_utc', 'bag', 'current_source_commit', 'current_source_status',
            'online_or_2d_prerequisite', 'executable_sha256']},
        'recorded_bag_files': manifest['bag_inventory']['files'],
        'acceptance': json.loads((path / 'acceptance.json').read_text()),
        'files': inventory(path / 'data/new_map'),
        'config': yaml.safe_load((path / 'config.yaml').read_text()),
    }
    for name in ['manifest.json', 'config.yaml', 'extrinsics.yaml', 'acceptance.json', 'source_versions.json']:
        p = path / name
        maps[suffix]['files'][str(p.relative_to(ROOT))] = {'bytes': p.stat().st_size, 'sha256': sha(p)}

# Confirm the active pair uses the same recorded raw data and compare poses by ID/time.
a, b = maps['2lx5kkho'], maps['8cw1afzl']
pair = {'recorded_bag_files_equal': a['recorded_bag_files'] == b['recorded_bag_files'],
        'same_source_commit': a['provenance']['current_source_commit'] == b['provenance']['current_source_commit']}
rows = []
for entry in [a, b]:
    lines = (ROOT / entry['path'] / 'data/new_map/keyframes.txt').read_text().splitlines()
    rows.append([[float(v) for v in line.split()] for line in lines if line and not line.startswith('#')])
pair['keyframe_counts'] = [len(x) for x in rows]
pair['keyframe_ids_and_times_equal'] = len(rows[0]) == len(rows[1]) and all(
    x[:2] == y[:2] for x, y in zip(*rows))
if pair['keyframe_ids_and_times_equal']:
    distances = [math.dist(x[9:12], y[9:12]) for x, y in zip(*rows)]
    angles = []
    for x, y in zip(*rows):
        q, r = x[12:16], y[12:16]
        dot = abs(sum(v*w for v, w in zip(q, r))) / (math.sqrt(sum(v*v for v in q)) * math.sqrt(sum(v*v for v in r)))
        angles.append(math.degrees(2 * math.acos(min(1.0, dot))))
    pair['max_optimized_translation_difference_m'] = max(distances)
    pair['max_optimized_rotation_difference_deg'] = max(angles)
pair['meaning'] = 'Same-data pose consistency only; not independent geometric or navigation acceptance.'

bag_verification = {}
bag = Path(a['provenance']['bag'])
for rel, expected in a['recorded_bag_files'].items():
    p = bag / rel
    actual = sha(p) if p.is_file() else None
    bag_verification[rel] = {'expected_sha256': expected, 'actual_sha256': actual, 'equal': actual == expected}

run(['git', 'log', '--since=2026-09-28', '--format=%H %ad %s', '--date=iso-strict'])
run(['dpkg-query', '-W', '-f=${Package}\t${Version}\n', 'ros-humble-nav2-*', 'ros-humble-twist-mux'])
run(['bash', 'scripts/field_session.sh'])
run(['bash', 'scripts/run_navigation.sh'])

print(json.dumps({'collected_at': datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  'scope': 'disk/source defaults; no live ROS parameters or hardware validation',
                  'repositories': repositories, 'configuration_and_patch_files': files,
                  'installed_comparison': installed, 'maps': maps,
                  'active_map_pair': pair, 'active_bag_verification': bag_verification,
                  'commands': commands}, ensure_ascii=False, indent=2))
