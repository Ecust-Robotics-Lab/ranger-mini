#!/usr/bin/env python3
"""Reproduce stage B offline acceptance. Default dry-run; never starts hardware."""
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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    cases = [
        ('build_audit', [sys.executable, 'scripts/fast_livo2/backend/audit_build.py']),
        ('cpp', [str(ROOT/'ws_fast_livo/backend_build/ros/ranger_fast_livo_backend/backend_tests')]),
        ('bridge_guard', [sys.executable, 'tests/test_bridge_pgo_guard.py']),
        ('adapter_contract', [sys.executable, 'tests/test_fast_livo_adapter_contract.py']),
        ('ros', [sys.executable, 'tests/test_fast_livo_backend_ros.py']),
        ('project', [sys.executable, 'tests/test_project_consolidation.py']),
        ('contract', [sys.executable, 'tests/test_architecture_contract.py']),
        ('time', [sys.executable, 'tests/test_time_alignment.py']),
        ('bridge_legacy', [sys.executable, 'tests/test_stage5_bridge.py']),
        ('gate', [sys.executable, 'tests/test_stage6_gate_unit.py']),
        ('frontend_cpp', [str(ROOT/'ws_fast_livo/install/fast_livo/lib/fast_livo/ranger_livo_checks')]),
        ('frontend_patch', [sys.executable, 'scripts/fast_livo2/check_patch.py']),
        ('dry_private', ['bash', 'scripts/run_fast_livo2_backend.sh']),
        ('dry_bridge', ['bash', 'scripts/run_fast_livo2_backend.sh', '--offline-bridge']),
        ('dry_build', ['bash', 'scripts/build_fast_livo2_backend.sh'])]
    if not args.execute:
        print(json.dumps({'action': 'DRY-RUN', 'commands': cases, 'additional': 'existing header-clock view -> frontend -> private backend; isolated domain 174'}, indent=2))
        return 0
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', ROS_LOCALHOST_ONLY='1', OMP_NUM_THREADS='1')
    env['LD_LIBRARY_PATH'] = ':'.join([str(ROOT/'ws_fast_livo/backend_deps/lib'), str(ROOT/'ws_fast_livo/deps_install/lib'), env.get('LD_LIBRARY_PATH', '')])
    parent = ROOT/'docs/fast_livo2_evidence/stage_b_20261026'
    directory = Path(tempfile.mkdtemp(prefix='acceptance-', dir=parent))
    cases[1][1].append('--gtest_output=xml:'+str(directory/'cpp.xml'))
    chain_data = ROOT/'bags'/('stage-b-'+directory.name)
    cases.append(('frontend_backend_chain', [sys.executable, 'scripts/fast_livo2/dds_smoke.py',
                  'bags/fast-livo2-dds-smoke-final/view', '--existing-view', '--with-backend',
                  '--data', str(chain_data), '--out', str(directory/'chain')]))
    results = {'started_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(), 'hardware_started': False, 'commands': []}
    for name, command in cases:
        case_env = dict(env)
        if name == 'frontend_backend_chain':case_env['ROS_DOMAIN_ID'] = '174'
        with (directory/(name+'.log')).open('w') as log:
            result = subprocess.run(command, cwd=ROOT, env=case_env, stdout=log, stderr=subprocess.STDOUT)
        results['commands'].append({'name': name, 'argv': command, 'exit_code': result.returncode, 'log': str(directory/(name+'.log'))})
        (directory/'results.json').write_text(json.dumps(results, indent=2)+'\n')
        print(name, result.returncode, flush=True)
        if result.returncode:
            print('Failed; evidence:', directory, flush=True)
            return result.returncode
    results['passed'] = True
    (directory/'results.json').write_text(json.dumps(results, indent=2)+'\n')
    print('Evidence:', directory, flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
