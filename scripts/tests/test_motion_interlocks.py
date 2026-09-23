#!/usr/bin/env python3
"""Fault injection into the actual runner's CAN interlocks, without ROS/CAN."""
import ast
from pathlib import Path
import time
from types import SimpleNamespace

source = Path(__file__).resolve().parents[1] / 'base_ros_motion_test.py'
tree = ast.parse(source.read_text())
functions = [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
             and n.name in ('fields', 'safe_for_pulse')]
assert len(functions) == 2
limit = next(ast.literal_eval(n.value) for n in tree.body if isinstance(n, ast.Assign)
             and any(isinstance(t, ast.Name) and t.id == 'MAX_FORWARD_FEEDBACK_SPEED_MPS'
                     for t in n.targets))
scope = {'time': time, 'child': SimpleNamespace(poll=lambda: None),
         'forward_feedback_limit': float(limit)}
exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), 'exec'), scope)


def reset():
    now = time.monotonic()
    scope['latest'] = {0x211: (now, bytes.fromhex('000101f400000000')),
                       0x221: (now, bytes(8)), 0x291: (now, bytes(2))}
    scope['child'] = SimpleNamespace(poll=lambda: None)


def must_block(label, change, require_stationary=True):
    reset()
    change()
    try:
        scope['safe_for_pulse'](require_stationary=require_stationary)
    except RuntimeError:
        print('PASS:', label, 'blocked')
        return
    raise AssertionError(label + ' was not blocked')


def replace(identifier, hex_data):
    scope['latest'][identifier] = (time.monotonic(), bytes.fromhex(hex_data))


reset()
assert scope['safe_for_pulse'](require_stationary=True)['v'] == 0
print('PASS: fresh stationary CAN-controlled state accepted')
must_block('RC control', lambda: replace(0x211, '000001f400000000'))
must_block('ESTOP feedback', lambda: replace(0x211, '010101f400000000'))
must_block('fault code', lambda: replace(0x211, '000101f400010000'))
must_block('nonstationary vehicle', lambda: replace(0x221, '000a000000000000'))
must_block('wrong mode', lambda: replace(0x291, '0100'))
must_block('mode changing', lambda: replace(0x291, '0001'))
must_block('short frame', lambda: replace(0x211, '0001'))
must_block('stale feedback', lambda: scope['latest'].__setitem__(0x211, (time.monotonic()-1, bytes(8))))
must_block('missing feedback', lambda: scope['latest'].pop(0x221))
must_block('driver exited', lambda: scope.update(child=SimpleNamespace(poll=lambda: 1)))

# Replay the former 0.101 m/s trigger; retain stationary and upper-bound guards.
reset()
replace(0x221, '0065000000000000')
assert scope['safe_for_pulse']()['v'] == 0.101
must_block('0.101 m/s before motion', lambda: replace(0x221, '0065000000000000'))
for value in (0.12, -0.12):
    reset()
    replace(0x221, (int(round(value * 1000)).to_bytes(2, 'big', signed=True) + bytes(6)).hex())
    assert scope['safe_for_pulse']()['v'] == value
for value in (0.121, -0.121, -0.801):
    payload = (int(round(value * 1000)).to_bytes(2, 'big', signed=True) + bytes(6)).hex()
    must_block(f'forward feedback {value} m/s', lambda: replace(0x221, payload), False)
print('PASS: relaxed forward limit accepts 0.101 and boundary 0.12 m/s; other guards retained')
