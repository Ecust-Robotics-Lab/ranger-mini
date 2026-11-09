#!/usr/bin/env python3
"""C1 and directly related regression commands. Default dry-run, no devices."""
import argparse
import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[3]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute', action='store_true')
    args = p.parse_args()
    binary = ROOT/'ws_fast_livo/backend_build/ros'
    commands = [
        ('c1_cpp', [str(binary/'ranger_fast_livo_localizer/localizer_tests')]),
        ('c1_ros', [sys.executable, 'tests/test_fast_livo_localizer_ros.py']),
        ('backend_cpp', [str(binary/'ranger_fast_livo_backend/backend_tests')]),
        ('bridge_guard', [sys.executable, 'tests/test_bridge_pgo_guard.py']),
        ('bridge_legacy', [sys.executable, 'tests/test_stage5_bridge.py']),
        ('time_alignment', [sys.executable, 'tests/test_time_alignment.py']),
        ('adapter_contract', [sys.executable, 'tests/test_fast_livo_adapter_contract.py']),
        ('architecture_contract', [sys.executable, 'tests/test_architecture_contract.py']),
        ('project_config', [sys.executable, 'tests/test_project_consolidation.py']),
        ('dry_build', ['bash', 'scripts/build_fast_livo2_localizer.sh']),
        ('dry_private', ['bash', 'scripts/run_fast_livo2_localizer.sh']),
        ('dry_bridge', ['bash', 'scripts/run_fast_livo2_localizer.sh', '--offline-bridge']),
    ]
    if not args.execute:
        print(json.dumps({'action': 'DRY-RUN', 'hardware_started': False, 'commands': commands}, indent=2))
        return 0
    evidence = ROOT/'docs/fast_livo2_evidence/stage_c1_20261028'
    directory = Path(tempfile.mkdtemp(prefix='acceptance-', dir=evidence))
    env = dict(os.environ)
    env.update(PYTHONDONTWRITEBYTECODE='1', OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='196')
    env['PYTHONPATH'] = str(ROOT/'ws_nav/src/ranger_localization_bridge')+':'+env.get('PYTHONPATH', '')
    env['LD_LIBRARY_PATH'] = str(ROOT/'ws_fast_livo/backend_deps/lib')+':'+env.get('LD_LIBRARY_PATH', '')
    results = {'hardware_started': False, 'started_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(), 'commands': []}
    for name, command in commands:
        if name.endswith('_cpp'):
            command = command+['--gtest_output=xml:'+str(directory/(name+'.xml'))]
        with (directory/(name+'.log')).open('w') as log:
            completed = subprocess.run(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
        results['commands'].append({'name': name, 'argv': command, 'exit_code': completed.returncode,
                                    'log': str((directory/(name+'.log')).relative_to(ROOT))})
        results['passed'] = all(r['exit_code'] == 0 for r in results['commands'])
        (directory/'results.json').write_text(json.dumps(results, indent=2)+'\n')
        print(json.dumps(results['commands'][-1]), flush=True)
    print('Evidence: '+str(directory), flush=True)
    return 0 if results['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
