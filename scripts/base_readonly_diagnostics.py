#!/usr/bin/env python3
"""Bounded receive-only CAN diagnostics. Does not configure CAN or run ROS."""
import argparse
import datetime
import json
from pathlib import Path
import shutil
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--execute', action='store_true')
    mode.add_argument('--dry-run', action='store_true')
    parser.add_argument('--seconds', type=int, default=10)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 60:
        parser.error('--seconds must be between 1 and 60')
    root = Path(__file__).resolve().parents[1]
    candump = shutil.which('candump') or str(root / 'vendor/base_deps/root/usr/bin/candump')
    command = ['timeout', '--signal=TERM', '--kill-after=2s', f'{args.seconds}s',
               candump, '-e', '-x', 'can0,0:0,#FFFFFFFF']
    if not args.execute:
        print('DRY-RUN: read can0 state before/after; execute', command)
        print('No CAN configuration, transmitted CAN commands, or ROS nodes.')
        return 0
    destination = Path(tempfile.mkdtemp(prefix='can-', dir=root / 'docs/02_evidence'))
    results = []
    def run(label, cmd):
        with (destination / (label + '.log')).open('x') as out:
            result = subprocess.run(cmd, stdout=out, stderr=subprocess.STDOUT, timeout=70)
        results.append({'label': label, 'command': cmd, 'exit': result.returncode})
        return result.returncode
    run('before', ['ip', '-details', '-statistics', 'link', 'show', 'can0'])
    run('before-json', ['ip', '-json', '-details', '-statistics', 'link', 'show', 'can0'])
    code = run('candump', command)
    run('after', ['ip', '-details', '-statistics', 'link', 'show', 'can0'])
    run('after-json', ['ip', '-json', '-details', '-statistics', 'link', 'show', 'can0'])
    (destination / 'commands.json').write_text(json.dumps({
        'time': datetime.datetime.now().astimezone().isoformat(), 'results': results,
        'assessment': 'UNASSESSED: timeout exit 124 is not a communication PASS; '
                      'review received frames, direction, IDs, rate and error-counter deltas.'
    }, indent=2) + '\n')
    print(destination)
    print(f'candump exit={code}; no automatic communication or safety PASS.')
    return 0 if code in (0, 124) and all(r['exit'] == 0 for r in results if r['label'] != 'candump') else 2


if __name__ == '__main__':
    raise SystemExit(main())
