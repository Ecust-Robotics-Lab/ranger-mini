#!/usr/bin/env python3
"""Offline analysis of this capture; no sockets or robot commands.

Decode 0x211/0x221 from pinned ugv_sdk agilex_msg_parser_v2.c:64-98,
0x291 from :193-198; byte order from agilex_protocol_v2.h:118-122.
"""
import collections
import json
from pathlib import Path
import re

p = Path(__file__).resolve().parent
frames = []
for line in (p / 'candump.log').read_text().splitlines():
    m = re.fullmatch(r'\s*can0\s+(RX|TX)\s+\S+\s+\S+\s+([0-9A-Fa-f]+)\s+\[(\d+)\]\s*((?:[0-9A-Fa-f]{2}\s*)*)', line)
    if not m:
        raise ValueError('Unparsed line: ' + line)
    direction, identifier, length, raw = m.groups()
    data = bytes.fromhex(raw)
    assert len(data) == int(length)
    frames.append((direction, int(identifier, 16), data))
before = json.loads((p / 'before-json.log').read_text())[0]
after = json.loads((p / 'after-json.log').read_text())[0]
counts = collections.Counter(f'{identifier:03X}' for _, identifier, _ in frames)
system = sorted({(b[0], b[1], round(int.from_bytes(b[2:4], 'big', signed=True) * 0.1, 1),
                 int.from_bytes(b[4:6], 'big')) for _, c, b in frames if c == 0x211 and len(b) == 8})
motion = sorted({tuple(int.from_bytes(b[i:i+2], 'big', signed=True) / 1000.0
                       for i in range(0, 8, 2)) for _, c, b in frames if c == 0x221 and len(b) == 8})
result = {
    'capture_time': json.loads((p / 'commands.json').read_text())['time'],
    'frame_count': len(frames), 'direction_counts': dict(collections.Counter(d for d, _, _ in frames)),
    'id_counts': dict(sorted(counts.items())),
    'approximate_rates_hz_count_divided_by_10_seconds': {k: n / 10.0 for k, n in sorted(counts.items())},
    'rate_limit': 'No per-frame timestamps in capture; averages only, not jitter/latency acceptance.',
    'error_flagged_frames': sum(bool(c & 0x20000000) for _, c, _ in frames),
    'rx_counter_delta': {k: after['stats64']['rx'][k] - v for k, v in before['stats64']['rx'].items()},
    'tx_counter_delta': {k: after['stats64']['tx'][k] - v for k, v in before['stats64']['tx'].items()},
    'can_error_counter_delta': {k: after['linkinfo']['info_xstats'][k] - v
                                for k, v in before['linkinfo']['info_xstats'].items()},
    'system_fields': ['vehicle_state', 'control_mode', 'battery_voltage_V', 'error_code'],
    'system_values': system,
    'motion_fields': ['linear_velocity', 'angular_velocity', 'lateral_velocity', 'steering_angle'],
    'motion_values': motion,
    'motion_mode_and_changing': sorted({tuple(b) for _, c, b in frames if c == 0x291}),
    'motor_angle_271_all_zero': all(not any(b) for _, c, b in frames if c == 0x271),
    'motor_speed_281_all_zero': all(not any(b) for _, c, b in frames if c == 0x281),
    'assessment': 'PASS for passive CAN feedback reception in this window only; '
                  'no CAN transmission, ROS driver acceptance, or motion/stop safety acceptance.',
}
assert frames and result['direction_counts'] == {'RX': len(frames)}
assert {0x211, 0x221, 0x291, 0x271, 0x281} <= {c for _, c, _ in frames}
print(json.dumps(result, ensure_ascii=False, indent=2))
