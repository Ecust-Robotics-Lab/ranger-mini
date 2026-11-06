#!/usr/bin/env python3
"""Verify actual dependency SHAs, compile commands, system Eigen and loaded libraries."""
import hashlib
import json
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def output(*args):
    return subprocess.check_output(args, cwd=ROOT, text=True).strip()


def main():
    lock = json.loads((ROOT/'config/fast_livo2/sources.lock.json').read_text())
    result = {'sources': {}, 'builds': {}, 'runtime': {}, 'passed': True}
    for repo in lock['repositories']:
        sha = output('git', '-C', repo['path'], 'rev-parse', 'HEAD')
        status = output('git', '-C', repo['path'], 'status', '--short')
        result['sources'][repo['name']] = {'sha': sha, 'expected': repo['commit'], 'status': status,
                                         'branch': output('git', '-C', repo['path'], 'rev-parse', '--abbrev-ref', 'HEAD')}
        result['passed'] &= sha == repo['commit']
        if repo['name'] in ['gtsam', 'small_gicp']:
            result['passed'] &= not status
    builds = {'gtsam': 'ws_fast_livo/backend_build/gtsam',
              'small_gicp': 'ws_fast_livo/backend_build/small_gicp',
              'backend': 'ws_fast_livo/backend_build/ros/ranger_fast_livo_backend'}
    for name, folder in builds.items():
        commands = json.loads((ROOT/folder/'compile_commands.json').read_text())
        text = '\n'.join(c.get('command', ' '.join(c.get('arguments', []))) for c in commands)
        native = bool(re.search(r'-march=native|-mavx|-ffast-math', text))
        eigen = '/usr/include/eigen3' in text
        definitions = sorted(set(re.findall(r'-DEIGEN_[^ ]+', text)))
        result['builds'][name] = {'system_eigen_header_path': eigen, 'native_avx_fast_math': native,
                                 'explicit_eigen_definitions': definitions,
                                 'compile_commands_sha256': hashlib.sha256(text.encode()).hexdigest()}
        result['passed'] &= eigen and not native and not definitions
    cache = (ROOT/builds['gtsam']/'CMakeCache.txt').read_text()
    expected = {'GTSAM_USE_SYSTEM_EIGEN': 'ON', 'GTSAM_BUILD_WITH_MARCH_NATIVE': 'OFF',
                'GTSAM_BUILD_UNSTABLE': 'OFF', 'GTSAM_BUILD_PYTHON': 'OFF',
                'GTSAM_INSTALL_MATLAB_TOOLBOX': 'OFF', 'GTSAM_WITH_EIGEN_MKL': 'OFF'}
    result['gtsam_options'] = {}
    for key, value in expected.items():
        found = re.search(r'^'+key+r':[^=]+=(.*)$', cache, re.M)
        actual = found.group(1) if found else None
        result['gtsam_options'][key] = actual
        result['passed'] &= actual == value
    result['gtsam_tag'] = output('git', '-C', 'vendor/fast_livo2_review/gtsam', 'describe', '--tags', '--exact-match', 'HEAD')
    result['gtsam_tags_at_commit'] = output('git', '-C', 'vendor/fast_livo2_review/gtsam', 'tag', '--points-at', 'HEAD').splitlines()
    result['passed'] &= '4.2.0' in result['gtsam_tags_at_commit']
    source = (ROOT/'vendor/fast_livo2_review/gtsam/CMakeLists.txt').read_text()
    result['gtsam_display_prerelease'] = re.search(r'set\s*\(GTSAM_PRERELEASE_VERSION\s+"([^"]+)"', source).group(1)
    executable = ROOT/'ws_fast_livo/backend_install/ranger_fast_livo_backend/lib/ranger_fast_livo_backend/backend_node'
    libraries = output('ldd', str(executable))
    for name in ['gtsam', 'small_gicp']:
        found = re.search(r'lib'+name+r'[^ ]*\s+=>\s+([^ ]+)', libraries)
        path = found.group(1) if found else ''
        result['runtime'][name] = path
        result['passed'] &= path.startswith(str(ROOT/'ws_fast_livo/backend_deps/lib'))
    result['runtime']['ldd'] = libraries
    result['eigen'] = output('dpkg-query', '-W', '-f=${Version}', 'libeigen3-dev')
    result['pcl'] = output('dpkg-query', '-W', '-f=${Version}', 'libpcl-dev')
    result['abi_evidence_scope'] = 'Compile commands/headers/macros and loaded paths checked; Pose3/PCL interoperability exercised by C++ tests. Distro PCL original build flags are not installed, so this is not a universal ABI proof.'
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
