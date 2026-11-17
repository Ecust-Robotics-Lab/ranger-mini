#!/usr/bin/env python3
"""Read-only observer. Record original stamps plus receipt/availability times."""
import argparse
import json
import signal
import time
from pathlib import Path


def pose(message):
    return {'xyz': [message.position.x, message.position.y, message.position.z],
            'xyzw': [message.orientation.x, message.orientation.y,
                     message.orientation.z, message.orientation.w]}


def run(folder):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from ranger_fast_livo_backend.msg import BackendEstimate
    from session_manager import atomic_json

    stopped = False
    def stop(*_):
        nonlocal stopped
        stopped = True
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, stop)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('ranger_p0_closure_observer')
    anchor = None
    counts = {'frontend': 0, 'online': 0, 'wheel': 0}
    last = {}
    stream = (folder / 'closure_samples.jsonl').open('x', buffering=1)

    def update():
        nonlocal anchor
        try:
            h = json.loads((folder / 'mapping_health.json').read_text())
            anchor = h.get('clock', {}).get('anchor')
        except (OSError, ValueError):
            pass
        atomic_json(folder / 'closure_observer.json', {
            'monotonic_ns': time.monotonic_ns(), 'anchor': anchor,
            'counts': counts, 'last': last, 'read_only': True})

    def receive(message, kind):
        now = time.monotonic_ns()
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        sample_steady = stamp - int(anchor[0]) + int(anchor[1]) if anchor else None
        row = {'kind': kind, 'stamp_ns': stamp, 'received_steady_ns': now,
               'sample_steady_ns': sample_steady, 'frame': message.header.frame_id,
               'pose': pose(message.pose if kind == 'online' else message.pose.pose)}
        if kind == 'online':
            row.update(session_id=message.session_id, graph_version=message.graph_version,
                       scan_id=message.scan_id, reference_keyframe_id=message.reference_keyframe_id,
                       loop_event_id=message.loop_event_id, state=message.state,
                       pose_valid=message.pose_valid, reason=message.reason,
                       sample_age_s=(now - sample_steady) / 1e9 if sample_steady else None)
        else:
            row['child'] = message.child_frame_id
            if kind == 'wheel':
                row['twist_xy'] = [message.twist.twist.linear.x, message.twist.twist.linear.y]
                # Wheel headers use a different epoch; never apply the LiDAR anchor.
                row['sample_steady_ns'] = None
        stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')
        counts[kind] += 1
        last[kind] = now

    qos = QoSProfile(depth=128, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(Odometry, '/fast_livo/odometry', lambda m: receive(m, 'frontend'), qos)
    node.create_subscription(BackendEstimate, '/fast_livo/backend/estimate', lambda m: receive(m, 'online'), qos)
    node.create_subscription(Odometry, '/wheel/odom', lambda m: receive(m, 'wheel'), qos)
    node.create_timer(.2, update)
    try:
        while rclpy.ok() and not stopped:
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        update()
        stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--folder', type=Path, required=True)
    run(p.parse_args().folder)
