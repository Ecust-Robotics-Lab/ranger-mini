#!/usr/bin/env python3
"""Finite recorder worker. Only the managed capture entry invokes this module."""
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

from record_session import input_problems, recorder_command
from session_manager import atomic_json, identity


def run(folder):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.utilities import get_message

    manifest = json.loads((folder/'manifest.json').read_text())
    topics = manifest['topics']
    health_path = Path(os.environ['RANGER_SESSION_HEALTH_FILE'])
    atomic_json(health_path.parent/'capture.json', {'folder': str(folder)})
    stopping = [False]
    def stop(*_):stopping[0] = True
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):signal.signal(sig, stop)
    # rclpy must not replace the worker's cleanup handlers.
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('ranger_fast_livo_record_preflight')
    child = None
    log = None
    forced = False
    outcome = {'exit': None, 'forced': False, 'requested_seconds': manifest['seconds']}
    counts = {topic: 0 for topic in topics}
    last_receipt = {}
    def received(_, topic):
        counts[topic] += 1
        last_receipt[topic] = time.monotonic()
    def callback(topic):
        def on_message(message):received(message, topic)
        return on_message
    for topic, kind in topics.items():
        node.create_subscription(get_message(kind), topic, callback(topic),
                                 QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE), raw=True)
    def graph():
        return {topic: [{'node': e.node_name, 'namespace': e.node_namespace, 'type': e.topic_type,
                         'gid': list(e.endpoint_gid), 'reliability': e.qos_profile.reliability.name,
                         'durability': e.qos_profile.durability.name}
                        for e in node.get_publishers_info_by_topic(topic)] for topic in topics}
    def valid(g):
        return all(len(g[t]) == 1 and g[t][0]['type'] == kind and
                   g[t][0]['reliability'] == 'RELIABLE' for t, kind in topics.items())
    try:
        atomic_json(folder/'recording.json', {'state': 'PREFLIGHT'})
        deadline = time.monotonic()+12
        while time.monotonic() < deadline and not stopping[0]:
            rclpy.spin_once(node, timeout_sec=.05)
            before = graph()
            if valid(before) and all(counts.values()):break
        else:
            raise RuntimeError('输入未就绪：'+'；'.join(input_problems(topics, graph(), counts))
                               if not stopping[0] else '录制启动已中止')
        atomic_json(folder/'graph_before.json', before)
        command = recorder_command(folder, topics)
        atomic_json(folder/'record_command.json', command)
        log = (folder/'record.log').open('xb')
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        atomic_json(folder/'recorder_process.json', identity(child.pid))
        deadline = time.monotonic()+15
        while time.monotonic() < deadline and not stopping[0]:
            rclpy.spin_once(node, timeout_sec=.05)
            if child.poll() is not None:raise RuntimeError('recorder exited before readiness')
            subscribed = all(any(e.node_name == 'rosbag2_recorder' for e in
                                 node.get_subscriptions_info_by_topic(t)) for t in topics)
            if subscribed:break
        else:raise RuntimeError('recorder subscription readiness timeout or interrupted startup')
        started = time.monotonic()
        atomic_json(folder/'recording.json', {'state': 'RECORDING', 'started_unix_ns': time.time_ns(),
                    'started_monotonic': started, 'requested_seconds': manifest['seconds']})
        while not stopping[0] and time.monotonic()-started < manifest['seconds']:
            rclpy.spin_once(node, timeout_sec=.05)
            if child.poll() is not None:raise RuntimeError('recorder exited unexpectedly')
            current = graph()
            if not valid(current) or any(current[t][0]['gid'] != before[t][0]['gid'] for t in topics):
                raise RuntimeError('input publisher disappeared, changed, or became ambiguous')
            stalled = [t for t in topics if time.monotonic()-last_receipt.get(t, 0) > 2]
            if stalled:
                raise RuntimeError('required input stream stalled for over 2 seconds: '+', '.join(stalled))
            if shutil.disk_usage(folder).free < manifest['min_free_bytes']:
                raise RuntimeError('disk free space below configured reserve')
        outcome['elapsed_seconds'] = time.monotonic()-started
        outcome['stopped_by_signal'] = stopping[0]
    except Exception as exc:
        outcome['error'] = str(exc)
        print('CAPTURE ERROR：'+str(exc), flush=True)
    finally:
        atomic_json(folder/'recording.json', {'state': 'FINALIZING'})
        if child and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=25)
            except subprocess.TimeoutExpired:
                forced = True
                child.terminate()
                try:child.wait(timeout=4)
                except subprocess.TimeoutExpired:child.kill();child.wait(timeout=2)
        if log:log.close()
        outcome.update(exit=child.returncode if child else None, forced=forced, observed_messages=counts)
        atomic_json(folder/'recorder_result.json', outcome)
        node.destroy_node()
        rclpy.shutdown()
    if outcome.get('error') or forced or outcome['exit'] != 0:
        return 2
    atomic_json(folder/'recording.json', {'state': 'RECORDED', 'elapsed_seconds': outcome['elapsed_seconds']})
    # Successful finite completion requests normal guardian cleanup. Remain alive
    # until the scheduler signals us, so a normal completion is not a critical exit.
    if not stopping[0]:
        os.kill(os.getppid(), signal.SIGTERM)
        while not stopping[0]:time.sleep(.05)
    return 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder', type=Path, required=True)
    raise SystemExit(run(parser.parse_args().folder))
