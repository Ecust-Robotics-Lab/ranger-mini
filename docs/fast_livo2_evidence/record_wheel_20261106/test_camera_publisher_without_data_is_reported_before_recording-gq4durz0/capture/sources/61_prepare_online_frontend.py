#!/usr/bin/env python3
"""Derive a reproducible online I/O build without changing the vendor worktree."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT/'vendor/fast_livo2_review/ros2_frontend'
DESTINATION = ROOT/'ws_fast_livo/online_frontend_source'


def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    lock = json.loads((ROOT/'config/fast_livo2/sources.lock.json').read_text())
    repository = next(r for r in lock['repositories'] if r['name'] == 'ros2_frontend')
    if subprocess.check_output(['git', '-C', str(SOURCE), 'rev-parse', 'HEAD'], text=True).strip() != repository['commit']:
        raise RuntimeError('frontend source commit differs from lock')
    if subprocess.check_output(['git', '-C', str(SOURCE), 'diff', '--binary']) != (ROOT/'patches/fast_livo2_stage_a.patch').read_bytes():
        raise RuntimeError('preserve vendor modifications: tracked frontend diff differs from reviewed patch')
    names = subprocess.check_output(['git', '-C', str(SOURCE), 'ls-files', '--cached', '--others', '--exclude-standard'], text=True).splitlines()
    names = [name for name in names if not name.startswith('Log/') and (SOURCE/name).is_file()]
    contents = {name: (SOURCE/name).read_bytes() for name in names}
    path = 'src/LIVMapper.cpp'
    original = contents[path].decode()
    replacements = {
        '  rclcpp::Rate rate(5000);': '  rclcpp::Rate rate(500);\n  rclcpp::executors::SingleThreadedExecutor executor;\n  executor.add_node(this->node);',
        '    rclcpp::spin_some(this->node);': '    executor.spin_some();',
        '  savePCD();\n}': '  executor.remove_node(this->node);\n  savePCD();\n}',
        '>("/fast_livo/deskewed_imu", 10)': '>("/fast_livo/deskewed_imu", 32)',
        '>("/fast_livo/odometry", 10)': '>("/fast_livo/odometry", 32)'}
    changed = original
    for before, after in replacements.items():
        if changed.count(before) != 1: raise RuntimeError('online I/O source anchor mismatch: '+before)
        changed = changed.replace(before, after)
    contents[path] = changed.encode()
    hashes = {name: hashlib.sha256(data).hexdigest() for name, data in contents.items()}
    if DESTINATION.exists():
        for name, digest in hashes.items():
            if not (DESTINATION/name).is_file() or sha(DESTINATION/name) != digest:
                raise RuntimeError('preserve modified derived source: '+str(DESTINATION/name))
    else:
        DESTINATION.mkdir()
        for name, data in contents.items():
            target = DESTINATION/name; target.parent.mkdir(parents=True, exist_ok=True); target.write_bytes(data)
        for name in ('pcd', 'image', 'result', 'Colmap/sparse/0'):
            (DESTINATION/'Log'/name).mkdir(parents=True, exist_ok=True)
    manifest = {'upstream_commit': repository['commit'], 'stage_a_patch_sha256': sha(ROOT/'patches/fast_livo2_stage_a.patch'),
        'source': str(SOURCE), 'derived_source': str(DESTINATION), 'files_sha256': hashes,
        'changes': 'persistent ROS executor; 500 Hz I/O polling; paired output history 32; fusion mathematics unchanged',
        'io_patch_sha256': hashlib.sha256(changed.encode()).hexdigest()}
    (DESTINATION/'online_source_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    import difflib
    patch_path = ROOT/'patches/fast_livo2_online_io.patch'
    patch = ''.join(difflib.unified_diff(original.splitlines(keepends=True), changed.splitlines(keepends=True),
        fromfile='a/src/LIVMapper.cpp', tofile='b/src/LIVMapper.cpp'))
    if patch_path.exists() and patch_path.read_text() != patch:
        raise RuntimeError('preserve modified online I/O patch: '+str(patch_path))
    if not patch_path.exists(): patch_path.write_text(patch)
    print(DESTINATION)


if __name__ == '__main__': main()
