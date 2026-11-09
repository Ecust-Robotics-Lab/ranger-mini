#!/usr/bin/env python3
"""Build optional C2 coarse dependency in a project-local prefix; dry-run first."""
import argparse
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]


def run(*args):
    subprocess.run([str(a) for a in args], check=True, cwd=ROOT)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if not args.execute:
        print('DRY-RUN: locked KISS/ROBIN/PMC/xenium; project-local build/install, no devices')
        return
    entries = json.loads((ROOT/'config/fast_livo2/relocalization_sources.lock.json').read_text())['repositories']
    for entry in entries:
        if entry['name'] == 'sc_a_loam':
            continue
        prefix = ['git', '-C', str(ROOT/entry['path'])]
        if subprocess.check_output(prefix+['rev-parse', 'HEAD'], text=True).strip() != entry['commit']:
            raise RuntimeError('Dependency SHA mismatch: '+entry['name'])
        if subprocess.check_output(prefix+['status', '--porcelain']):
            raise RuntimeError('Preserve modified dependency: '+entry['name'])
    workspace = ROOT/'ws_fast_livo/relocalization_build'
    source = ROOT/'vendor/fast_livo2_review/robin'
    projection = workspace/'robin'
    projection.mkdir(parents=True, exist_ok=True)
    for path in source.iterdir():
        if path.name in {'.git', 'CMakeLists.txt'}:
            continue
        link = projection/path.name
        if not link.is_symlink() and link.exists():
            raise RuntimeError('Unexpected projection entry: '+str(link))
        if not link.is_symlink():
            link.symlink_to(path, target_is_directory=path.is_dir())
        if link.resolve() != path.resolve():
            raise RuntimeError('Unexpected projection link: '+str(link))
    cmake = (source/'CMakeLists.txt').read_text()
    # ROBIN's upstream xenium dependency follows main. Replace download hooks
    # in a generated projection, retaining the pristine locked repositories.
    cmake = cmake.replace('robin_download_pmc()',
        f'set(pmc_SOURCE_DIR "{ROOT}/vendor/fast_livo2_review/pmc")\n'
        'set(pmc_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/pmc")')
    cmake = cmake.replace('robin_download_xenium()',
        'add_library(xenium INTERFACE)\n'
        f'target_include_directories(xenium INTERFACE $<BUILD_INTERFACE:{ROOT}/vendor/fast_livo2_review/xenium> $<INSTALL_INTERFACE:include>)\n'
        f'install(DIRECTORY {ROOT}/vendor/fast_livo2_review/xenium/xenium DESTINATION include)')
    (projection/'CMakeLists.txt').write_text(cmake)
    build = workspace/'kiss'
    run('cmake', '-S', ROOT/'vendor/fast_livo2_review/kiss_matcher/cpp/kiss_matcher', '-B', build,
        '-DFETCHCONTENT_SOURCE_DIR_ROBIN='+str(projection), '-DUSE_SYSTEM_ROBIN=OFF',
        '-DUSE_SYSTEM_EIGEN3=ON', '-DUSE_SYSTEM_TBB=ON', '-DPMC_BUILD_SHARED=OFF',
        '-DBUILD_TESTS=OFF', '-DBUILD_DOCS=OFF', '-DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON',
        '-DCMAKE_POSITION_INDEPENDENT_CODE=ON',
        '-DCMAKE_INSTALL_PREFIX='+str(ROOT/'ws_fast_livo/relocalization_deps'))
    run('cmake', '--build', build, '--target', 'kiss_matcher_core', '-j2')
    run('cmake', '--install', build)


if __name__ == '__main__':
    main()
