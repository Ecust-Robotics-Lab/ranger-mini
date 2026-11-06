#!/usr/bin/env python3
"""AprilTag and directly affected B/C1/C2 regression. Default dry-run, no devices."""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--group', choices=['tag', 'regression', 'all'], default='all')
    args = parser.parse_args()
    evidence = ROOT/'docs/fast_livo2_evidence/apriltag_20261029'
    commands = []
    if args.group in ['tag', 'all']:
        for package, binary in [('ranger_fast_livo_backend', 'tag_tests'), ('ranger_fast_livo_backend', 'tag_detector_tests'),
                                ('ranger_fast_livo_localizer', 'tag_integration_tests')]:
            commands.append((binary, [str(ROOT/f'ws_fast_livo/backend_build/ros/{package}/{binary}')], 'cpp'))
        commands.append(('tag_ros', [sys.executable, 'tests/test_fast_livo_tags_ros.py'], 'python'))
    if args.group in ['regression', 'all']:
        for package, binary in [('ranger_fast_livo_backend', 'backend_tests'), ('ranger_fast_livo_localizer', 'localizer_tests')]:
            commands.append((binary, [str(ROOT/f'ws_fast_livo/backend_build/ros/{package}/{binary}')], 'cpp'))
        for module in ['test_fast_livo_backend_ros', 'test_fast_livo_relocalizer_ros', 'test_fast_livo_localizer_ros']:
            commands.append((module, [], 'ros'))
        for name in ['test_stage5_bridge', 'test_bridge_pgo_guard', 'test_time_alignment', 'test_fast_livo_adapter_contract']:
            commands.append((name, [sys.executable, 'tests/'+name+'.py'], 'python'))
        commands += [('dry_tag_build', ['bash', 'scripts/build_fast_livo2_tags.sh'], 'dry'),
                     ('dry_tag_observer', ['bash', 'scripts/run_fast_livo2_tags.sh'], 'dry'),
                     ('dry_tag_c2', ['bash', 'scripts/run_fast_livo2_relocalizer.sh', '--tag-relocalization', '--tag-monitor'], 'dry')]
    if not args.execute:
        print(json.dumps({'action': 'DRY-RUN', 'hardware_started': False, 'commands': commands}, indent=2))
        return 0
    directory = Path(tempfile.mkdtemp(prefix='acceptance-', dir=evidence)).resolve()
    env = dict(os.environ)
    env.update(PYTHONDONTWRITEBYTECODE='1', ROS_LOCALHOST_ONLY='1', OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1',
               RANGER_TAG_TEST_EVIDENCE=str(directory/'tag_ros'))
    env['LD_LIBRARY_PATH'] = ':'.join(str(ROOT/'ws_fast_livo'/item/'lib') for item in ['apriltag_deps', 'backend_deps', 'relocalization_deps'])+':'+env.get('LD_LIBRARY_PATH', '')
    env['PYTHONPATH'] = str(ROOT/'ws_nav/src/ranger_localization_bridge')+':'+env.get('PYTHONPATH', '')
    result = {'hardware_started': False, 'group': args.group, 'commands': []}
    for name, argv, kind in commands:
        if kind == 'ros':
            code = ("import sys,unittest,pathlib;sys.path.insert(0,'tests');import "+name+" as m;"
                    "p=pathlib.Path("+repr(str(directory/name))+ ");p.mkdir();m.EVIDENCE=p;"
                    +( "import test_fast_livo_localizer_ros as c;c.EVIDENCE=p;" if module_uses_c1(name) else "")+
                    "r=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromModule(m));sys.exit(not r.wasSuccessful())")
            argv = [sys.executable, '-c', code]
        if kind == 'cpp':
            argv += ['--gtest_output=xml:'+str(directory/(name+'.xml'))]
        log = directory/(name+'.log')
        resource = directory/(name+'_resources.json')
        measured = ['/usr/bin/time', '-o', str(resource), '-f', '{"max_rss_kib":%M,"elapsed_seconds":%e,"user_seconds":%U,"sys_seconds":%S}', *argv]
        with log.open('w') as stream:
            process = subprocess.run(measured, cwd=ROOT, env=env, stdout=stream, stderr=subprocess.STDOUT, timeout=240)
        row = {'name': name, 'argv': argv, 'exit_code': process.returncode, 'log': str(log.relative_to(ROOT))}
        raw = resource.read_text().splitlines()
        row['resources'] = json.loads(raw[-1])
        if kind == 'cpp' and (directory/(name+'.xml')).exists():
            tree = ET.parse(directory/(name+'.xml')).getroot()
            row['tests'] = int(tree.attrib['tests'])
            row['failures'] = int(tree.attrib['failures'])+int(tree.attrib.get('errors', 0))
        else:
            found = re.search(r'Ran (\d+) tests?', log.read_text())
            row['tests'] = int(found.group(1)) if found else None
            check_count = re.search(r'(\d+)/(\d+) checks passed', log.read_text())
            if check_count:
                row['checks'] = int(check_count.group(1))
        result['commands'].append(row)
        result['passed'] = all(item['exit_code'] == 0 for item in result['commands'])
        (directory/'results.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(row), flush=True)
    print('Evidence: '+str(directory), flush=True)
    return 0 if result['passed'] else 1


def module_uses_c1(name):
    return name in ['test_fast_livo_relocalizer_ros', 'test_fast_livo_localizer_ros']


if __name__ == '__main__':
    raise SystemExit(main())
