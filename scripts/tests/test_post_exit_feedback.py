#!/usr/bin/env python3
"""Replay real post-exit feedback through the runner's actual assessment block.

No ROS imports, CAN sockets, subprocesses or hardware operations are executed.
"""
import ast
import json
from pathlib import Path
import struct

root = Path(__file__).resolve().parents[2]
tree = ast.parse((root / 'scripts/base_ros_motion_test.py').read_text())
block = None
for node in ast.walk(tree):
    body = getattr(node, 'body', None)
    if not isinstance(body, list):
        continue
    for index, statement in enumerate(body):
        if isinstance(statement, ast.Assign) and any(
            isinstance(target, ast.Subscript)
            and isinstance(target.slice, ast.Constant)
            and target.slice.value == 'after_driver_exit_motion_samples'
            for target in statement.targets
        ):
            block = body[index:index + 4]
assert block is not None
code = compile(ast.Module(body=block, type_ignores=[]), '<actual-runner-assessment>', 'exec')


def assess(samples, initial='initial'):
    context = {'post_exit_motion': samples, 'result': {'assessment': initial}}
    exec(code, context)
    return context['result']


recording = root / 'docs/02_evidence/ros-motion-mp69pbm2/can_frames.jsonl'
samples = []
for line in recording.read_text().splitlines():
    frame = json.loads(line)
    if frame['phase'] == 'after_driver_exit' and frame['id'] == '0x221':
        samples.append({'t': frame['t'], 'values': [x / 1000 for x in
            struct.unpack('>hhhh', bytes.fromhex(frame['data']))]})
assert samples[-1]['values'] == [0, 0, 0, 0]
result = assess(samples)
assert len(result['after_driver_exit_motion_excursions']) == 5
assert 'ABORTED' in result['assessment']
original = 'PARTIAL/ABORTED: Unexpected feedback speed: v=0.101'
combined = assess(samples, initial=original)['assessment']
assert combined.startswith(original) and 'after driver exit' in combined
assert assess([{'t': 0, 'values': [0, 0, 0, 0]}])['assessment'] == 'initial'
for values in ([0.01, 0, 0, 0], [0, -0.01, 0, 0], [0, 0, 0.01, 0]):
    assert 'ABORTED' in assess([{'t': 0, 'values': values}])['assessment']
print('PASS: actual runner rejects recorded post-exit transient even with final zero;')
print('PASS: stationary control and independent forward/yaw/lateral excursions.')
