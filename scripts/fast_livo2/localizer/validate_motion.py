#!/usr/bin/env python3
"""Recorded frontend -> C1/C2 read-only map function check. Default dry-run.

Given-initial mode uses the same B graph. Global-search mode provides no initial
pose and may declare a distinct capture. Neither establishes absolute accuracy
or a real-time benchmark. No wheel or TF.
"""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'scripts/fast_livo2/backend'))
from validate_motion import load_pairs, sha


def query_declaration(path, manifest_sha, database_sha, global_search):
    if path is None:
        return None
    declaration = json.loads(path.read_text())
    if (declaration['source_manifest_sha256'] != manifest_sha or
            declaration['source_database_sha256'] != database_sha):
        raise ValueError('query provenance does not match map/native source SHA')
    if declaration.get('different_capture') and not global_search:
        raise ValueError('independent capture cannot use same-graph initial pose')
    return declaration


def pair_outcome(message, previous, stamp_ns, allow_recovery):
    """Distinguish an exact scan ack from a timer-cancelled pending scan.

    The node revokes all queued work at its original initialization deadline.
    That invalid heartbeat can legitimately retain the preceding scan stamp.
    Only a newly observed timeout in the same map/session/request may release
    the offline sender; stale, valid, switched or unrecognized messages cannot.
    """
    if message.latest_scan_stamp_ns == stamp_ns:
        return 'acknowledged'
    if (not allow_recovery or message is previous or message.pose_valid or
            message.state != message.INITIALIZING or
            message.reason != 'initial_verification_timeout' or
            previous.reason == 'initial_verification_timeout' or
            message.latest_scan_stamp_ns > stamp_ns):
        return None
    for field in ('map_uuid', 'session_id', 'request_generation', 'initialization_request_id'):
        if getattr(message, field) != getattr(previous, field):
            return None
    return 'cancelled_initial_verification_timeout'


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--archive', type=Path, required=True)
    p.add_argument('--compatibility', type=Path, required=True)
    p.add_argument('--native', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--registration', choices=['pcl_icp', 'small_gicp'], default='pcl_icp')
    p.add_argument('--domain', type=int, default=197)
    p.add_argument('--initial-index', type=int, default=366)
    p.add_argument('--max-pairs', type=int)
    p.add_argument('--skip-pairs', type=int, default=0)
    p.add_argument('--global-search', action='store_true')
    p.add_argument('--query-provenance', type=Path, help='Declaration bound to source map/query SHA; no correctness claim')
    p.add_argument('--allow-recovery', action='store_true', help='C2 evaluation may include LOST or timed-out initialization followed by confirmed recovery')
    p.add_argument('--execute', action='store_true')
    args = p.parse_args()
    if args.allow_recovery and not args.global_search:
        p.error('allow-recovery requires global-search')
    if not 1 <= args.domain <= 232 or args.initial_index < 0 or args.skip_pairs < 0:
        p.error('invalid isolated domain/initial index')
    if not args.execute:
        print(json.dumps({'action': 'DRY-RUN', 'domain': args.domain, 'hardware_started': False,
                          'archive': str(args.archive), 'native': str(args.native), 'initial_index': args.initial_index,
                          'global_search': args.global_search, 'allow_recovery': args.allow_recovery,
                          'query_provenance': str(args.query_provenance) if args.query_provenance else None,
                          'limitations': 'functional experiment; capture relation must be declared; no accuracy/generality/real-time claim'}, indent=2))
        return 0
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1', PYTHONDONTWRITEBYTECODE='1',
                      OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1')
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from rosgraph_msgs.msg import Clock
    from ranger_fast_livo_localizer.msg import LocalizerEstimate
    from ranger_fast_livo_localizer.srv import Initialize
    sys.path.insert(0, str(ROOT/'ws_nav/src/ranger_localization_bridge'))
    from ranger_localization_bridge.geometry import SE3

    archive = args.archive.resolve()
    manifest_sha = sha(archive/'manifest.json')
    manifest = json.loads((archive/'manifest.json').read_text())
    pairs = load_pairs(args.native)[args.skip_pairs:]
    if args.max_pairs:
        pairs = pairs[:args.max_pairs]
    if not pairs or (not args.global_search and args.initial_index >= len(pairs)):
        raise ValueError('initial index exceeds selected input pairs')
    database_sha = sha(next(args.native.glob('*.db3')))
    declaration = query_declaration(args.query_provenance, manifest_sha, database_sha, args.global_search)
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Node('c1_recorded_function_check', parameter_overrides=[Parameter('use_sim_time', value=True)])
    latest = []
    node.create_subscription(LocalizerEstimate, '/fast_livo/localizer/estimate', latest.append, 100)
    start = time.monotonic()
    proc = log = None
    result = {'passed': False, 'hardware_started': False, 'absolute_accuracy_validated': False,
              'independent_session_validated': False, 'physical_sync_validated': False,
              'registration': args.registration, 'domain': args.domain, 'source_pairs': len(pairs),
              'source_native': str(args.native.resolve()), 'source_database_sha256': database_sha,
              'source_manifest_sha256': manifest_sha, 'initial_index': args.initial_index,
              'initial_pose_source': 'none; global search' if args.global_search else 'same B graph; earlier keyframe X * inverse(U) * current U',
              'global_search': args.global_search, 'skip_pairs': args.skip_pairs,
              'pacing': 'acknowledge each input stamp; not a real-time benchmark', 'exclude_identical_stamp': True}
    if declaration is not None:
        result.update(query_provenance=declaration,
                      query_provenance_sha256=sha(args.query_provenance),
                      independent_capture_used=bool(declaration.get('different_capture')))
    result['allow_recovery'] = args.allow_recovery
    try:
        deadline = time.monotonic()+2
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        for topic in ['/fast_livo/odometry', '/fast_livo/deskewed_imu', '/fast_livo/backend/estimate',
                      '/fast_livo/localizer/estimate', '/tf', '/tf_static', '/wheel/odom', '/cmd_vel', '/base/safe_cmd_vel']:
            if node.get_publishers_info_by_topic(topic):
                raise RuntimeError('occupied isolated test topic: '+topic)
        clock = node.create_publisher(Clock, '/clock', 10)
        odom_pub = node.create_publisher(Odometry, '/fast_livo/odometry', 16)
        cloud_pub = node.create_publisher(PointCloud2, '/fast_livo/deskewed_imu', 16)
        command = [str(ROOT/'ws_fast_livo/backend_install/ranger_fast_livo_localizer/lib/ranger_fast_livo_localizer/localizer_node'),
                   '--ros-args', '--params-file', str(ROOT/'config/fast_livo2/localizer.yaml'),
                   '-p', 'archive_path:='+str(archive), '-p', 'compatibility_path:='+str(args.compatibility.resolve()),
                   '-p', 'registration:='+args.registration]
        if args.global_search:
            command += ['--params-file', str(ROOT/'config/fast_livo2/relocalizer.yaml')]
        result['argv'] = command
        log = (args.output/'localizer.log').open('w')
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        current_ns = pairs[0][0]+30_000_000

        def pump(seconds=.02):
            until = time.monotonic()+seconds
            while time.monotonic() < until:
                c = Clock()
                c.clock.sec, c.clock.nanosec = divmod(current_ns, 10**9)
                clock.publish(c)
                rclpy.spin_once(node, timeout_sec=.005)

        def wait(predicate, timeout=15):
            until = time.monotonic()+timeout
            while time.monotonic() < until:
                if proc.poll() is not None:
                    raise RuntimeError('C1 process exited early: '+str(proc.returncode))
                pump()
                if predicate():
                    return
            raise TimeoutError('C1 did not confirm processing/state; last reason '+(latest[-1].reason if latest else 'none'))

        wait(lambda: latest and latest[-1].map_uuid == manifest['map_uuid'] and json.loads(latest[-1].quality_json).get('inputs_ready'))
        client = node.create_client(Initialize, '/fast_livo/localizer/initialize')
        if not client.wait_for_service(timeout_sec=2):
            raise RuntimeError('C1 initial service unavailable')
        counts, reasons = Counter(), Counter()
        loss_transitions = recovery_transitions = 0
        pending_recovery = False
        previous_state = None
        latencies, solve_ms = [], []
        outcomes = Counter()
        valid_stamps, excluded_stamps = set(), {k['stamp_ns'] for k in manifest['keyframes']}
        node_peak_kib = 0
        with (args.output/'samples.jsonl').open('x') as output:
            for index, (ns, odom, cloud) in enumerate(pairs):
                current_ns = ns+30_000_000
                pump(.01)
                if not args.global_search and index == args.initial_index:
                    candidates = [k for k in manifest['keyframes'] if k['stamp_ns'] <= ns]
                    if not candidates:
                        raise ValueError('no earlier archived keyframe for declared initial pose')
                    anchor = candidates[-1]
                    x, u = anchor['X_map_imu'], anchor['U_local_imu']
                    position, rotation = odom.pose.pose.position, odom.pose.pose.orientation
                    initial = (SE3(x['xyz'], x['xyzw'])*SE3(u['xyz'], u['xyzw']).inverse()
                               *SE3((position.x, position.y, position.z), (rotation.x, rotation.y, rotation.z, rotation.w)))
                    request = Initialize.Request()
                    request.request_id = 'same-capture-given-initial'
                    request.map_uuid, request.map_generation, request.session_id = latest[-1].map_uuid, latest[-1].map_generation, latest[-1].session_id
                    request.reference_body, request.initial_pose.header.frame_id = 'fast_livo_imu', 'map'
                    request.initial_pose.pose.pose.position.x, request.initial_pose.pose.pose.position.y, request.initial_pose.pose.pose.position.z = initial.t
                    q = request.initial_pose.pose.pose.orientation
                    q.x, q.y, q.z, q.w = initial.q
                    for k in range(6):
                        request.initial_pose.pose.covariance[7*k] = .01
                    future = client.call_async(request)
                    wait(future.done)
                    if not future.result().accepted:
                        raise AssertionError(future.result().reason)
                    result['initial_pose'] = {'xyz': list(initial.t), 'xyzw': list(initial.q), 'reference_keyframe_id': anchor['id']}
                sent = time.monotonic()
                previous_message = latest[-1]
                odom_pub.publish(odom)
                cloud_pub.publish(cloud)
                wait(lambda: latest and pair_outcome(latest[-1], previous_message, ns, args.allow_recovery))
                message = latest[-1]
                outcome = pair_outcome(message, previous_message, ns, args.allow_recovery)
                outcomes[outcome] += 1
                quality = json.loads(message.quality_json)
                counts[message.state] += 1
                reasons[message.reason] += 1
                if message.state == message.LOST and previous_state != message.LOST:
                    loss_transitions += 1
                    pending_recovery = True
                latency = (time.monotonic()-sent)*1000
                latencies.append(latency)
                if message.pose_valid:
                    if message.state != message.VALID or message.last_confirmed_stamp_ns != ns:
                        raise AssertionError('confirmation did not retain original sample stamp')
                    valid_stamps.add(ns)
                    solve_ms.append(quality['elapsed_ms'])
                    if pending_recovery:
                        recovery_transitions += 1
                        pending_recovery = False
                previous_state = message.state
                pose = message.pose
                output.write(json.dumps({'index': index, 'stamp_ns': ns, 'state': message.state, 'pose_valid': message.pose_valid,
                                         'last_confirmed_stamp_ns': message.last_confirmed_stamp_ns, 'reason': message.reason,
                                         'input_outcome': outcome, 'reported_latest_scan_stamp_ns': message.latest_scan_stamp_ns,
                                         'query_is_archived_keyframe': ns in excluded_stamps, 'ack_latency_ms': latency,
                                         'quality': quality,
                                         'pose_map_imu': {'xyz': [pose.position.x, pose.position.y, pose.position.z],
                                                          'xyzw': [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]},
                                         'map_uuid': message.map_uuid, 'session_id': message.session_id,
                                         'request_generation': message.request_generation})+'\n')
                status = Path('/proc')/str(proc.pid)/'status'
                if status.exists():
                    for line in status.read_text().splitlines():
                        if line.startswith('VmHWM:'):
                            node_peak_kib = max(node_peak_kib, int(line.split()[1]))
                if index % 100 == 0:
                    print(json.dumps({'processed': index+1, 'valid': len(valid_stamps), 'reason': message.reason}), flush=True)
                if len(latest) > 100:
                    del latest[:-50]
        result.update(submitted_pairs=sum(counts.values()), processed_pairs=outcomes['acknowledged'],
                      cancelled_pairs=sum(v for k,v in outcomes.items() if k != 'acknowledged'), input_outcomes=dict(outcomes), valid_samples=len(valid_stamps), states=dict(counts), reasons=dict(reasons),
                      node_peak_rss_kib=node_peak_kib, wall_seconds=time.monotonic()-start,
                      ack_latency_ms_p50_p95_max=np.percentile(latencies, [50, 95, 100]).tolist(),
                      solve_ms_p50_p95_max=np.percentile(solve_ms, [50, 95, 100]).tolist() if solve_ms else [],
                      public_publishers={topic: len(node.get_publishers_info_by_topic(topic)) for topic in ['/tf', '/tf_static', '/cmd_vel', '/base/safe_cmd_vel']})
        result['post_initial_queries'] = len(pairs) if args.global_search else len(pairs)-args.initial_index
        result['tracking_complete_without_loss'] = counts.get(LocalizerEstimate.LOST, 0) == 0 and counts.get(LocalizerEstimate.ERROR, 0) == 0
        result.update(loss_transitions=loss_transitions, recovery_transitions=recovery_transitions,
                      recovery_pending_at_end=pending_recovery, final_state=previous_state)
        state_ok = (counts.get(LocalizerEstimate.ERROR, 0) == 0 and previous_state == LocalizerEstimate.VALID
                    and not pending_recovery) if args.allow_recovery else result['tracking_complete_without_loss']
        if len(valid_stamps) < 10 or not state_ok or any(result['public_publishers'].values()):
            raise AssertionError('insufficient confirmation, tracking loss/errors, or unexpected public outputs')
        if sha(archive/'manifest.json') != manifest_sha:
            raise AssertionError('source map changed')
        result['passed'] = True
    except Exception as error:
        result['error'] = repr(error)
        raise
    finally:
        if proc is not None:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait(timeout=3)
            result['localizer_exit'] = proc.returncode
        if log is not None:
            log.close()
        (args.output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
