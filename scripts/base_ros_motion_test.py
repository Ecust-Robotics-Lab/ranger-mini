#!/usr/bin/env python3
"""One authorized process lifetime: static ROS checks, optional bounded pulse.

Default is dry-run. Requires a present human with a hardware stop. This is a
test runner, not a production safety controller or a firmware watchdog test.
"""
import argparse
import collections
import datetime
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import tempfile
import time

# 人工测试配置：前进指令持续时间，单位秒。仅在此处修改时长。
# 当前 5 秒为用户指定配置，尚未真机验证；停车安全仍未通过。
FORWARD_DURATION_SECONDS = 5.0
# 前进阶段纵向反馈速度绝对值的中止阈值（m/s），不是发送速度。
MAX_FORWARD_FEEDBACK_SPEED_MPS = 0.12


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--motion', action='store_true')
    parser.add_argument('--authorized-this-run', action='store_true')
    args = parser.parse_args()
    try:
        forward_duration = float(FORWARD_DURATION_SECONDS)
    except (TypeError, ValueError, OverflowError):
        parser.error('FORWARD_DURATION_SECONDS must be a finite positive number.')
    if not math.isfinite(forward_duration) or forward_duration <= 0:
        parser.error('FORWARD_DURATION_SECONDS must be a finite positive number.')
    try:
        forward_feedback_limit = float(MAX_FORWARD_FEEDBACK_SPEED_MPS)
    except (TypeError, ValueError, OverflowError):
        parser.error('MAX_FORWARD_FEEDBACK_SPEED_MPS must be a finite positive number.')
    if not math.isfinite(forward_feedback_limit) or forward_feedback_limit <= 0:
        parser.error('MAX_FORWARD_FEEDBACK_SPEED_MPS must be a finite positive number.')
    if not args.execute:
        print('DRY-RUN: no ROS node or CAN access. One Mini V2 driver start; '
              f'static checks; optional vx=0.05 m/s for {forward_duration:g} s; then zeros. '
              f'Forward feedback speed limit: {forward_feedback_limit:g} m/s. '
              'No timeout, RC takeover, ESTOP or CAN disconnect injection.')
        return 0
    if not args.authorized_this_run:
        parser.error('Fresh explicit authorization is required for this invocation.')

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from ranger_msgs.msg import SystemState, MotionState, ActuatorStateArray
    from sensor_msgs.msg import BatteryState
    from tf2_msgs.msg import TFMessage
    from rcl_interfaces.srv import GetParameters
    from rosidl_runtime_py.convert import message_to_ordereddict

    root = Path(__file__).resolve().parents[1]
    folder = Path(tempfile.mkdtemp(prefix='ros-motion-', dir=root / 'docs/02_evidence'))
    print(f'LOG_DIR={folder}', flush=True)
    result = {'start': datetime.datetime.now().astimezone().isoformat(),
              'authorized_scope': 'one driver start; static checks' +
                  (f'; requested vx=0.05 m/s for {forward_duration:g} s and explicit zeros'
                   if args.motion else ''),
              'forward_duration_seconds': forward_duration,
              'max_forward_feedback_speed_mps': forward_feedback_limit,
              'motion_executed': False, 'assessment': 'RUNNING'}
    raw_log = (folder / 'can_frames.jsonl').open('x')
    ros_log = (folder / 'ros_messages.jsonl').open('x')
    command_log = (folder / 'published_commands.jsonl').open('x')
    driver_log = (folder / 'driver.log').open('x')
    latest = {}
    ros_counts = collections.Counter()
    tf_edges = collections.Counter()
    child = None
    node = None
    bus = None
    publisher = None
    subscribers = []
    phase = 'preflight'
    heartbeat_next = 0.0
    post_exit_motion = []

    def snapshot(label):
        data = subprocess.check_output(['ip', '-json', '-details', '-statistics', 'link', 'show', 'can0'], text=True)
        (folder / (label + '.json')).write_text(data)
        return json.loads(data)[0]

    def drain_can():
        for _ in range(3000):
            try:
                packet = bus.recv(16)
            except BlockingIOError:
                break
            if len(packet) != 16:
                raise RuntimeError('Short CAN frame')
            identifier, length, data = struct.unpack('=IB3x8s', packet)
            now = time.monotonic()
            raw_log.write(json.dumps({'t': now, 'phase': phase, 'id': hex(identifier),
                                      'dlc': length, 'data': data[:length].hex()}) + '\n')
            if identifier in (0x211, 0x221, 0x291, 0x271, 0x281):
                latest[identifier] = (now, data[:length])
            if phase == 'after_driver_exit' and identifier == 0x221 and length == 8:
                post_exit_motion.append({'t': now, 'values':
                    [value / 1000.0 for value in struct.unpack('>hhhh', data)]})

    def pump(duration):
        nonlocal heartbeat_next
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            # The first hardware attempt lost commanded mode while static.
            # Maintain only zero commands during static ROS inspection.
            # Forward/explicit-zero/cleanup phases own their own publication.
            if (phase == 'static' and child is not None and child.poll() is None
                    and publisher is not None and time.monotonic() >= heartbeat_next):
                publish(0.0)
                heartbeat_next = time.monotonic() + 0.02
            rclpy.spin_once(node, timeout_sec=0.001)
            drain_can()
            time.sleep(0.002)

    def fields():
        now = time.monotonic()
        for identifier, length in ((0x211, 8), (0x221, 8), (0x291, 2)):
            if identifier not in latest or now - latest[identifier][0] > 0.2:
                raise RuntimeError(f'Missing/stale CAN feedback {identifier:#x}')
            if len(latest[identifier][1]) != length:
                raise RuntimeError(f'Unexpected feedback length {identifier:#x}')
        system = latest[0x211][1]
        motion = latest[0x221][1]
        mode = latest[0x291][1]
        return {'vehicle': system[0], 'control': system[1],
                'fault': int.from_bytes(system[4:6], 'big'),
                'v': int.from_bytes(motion[:2], 'big', signed=True) / 1000.0,
                'w': int.from_bytes(motion[2:4], 'big', signed=True) / 1000.0,
                'steer': int.from_bytes(motion[6:8], 'big', signed=True) / 1000.0,
                'mode': mode[0], 'changing': mode[1]}

    def safe_for_pulse(require_stationary=False):
        if child.poll() is not None:
            raise RuntimeError('Driver exited')
        f = fields()
        if f['vehicle'] != 0 or f['fault'] != 0 or f['control'] != 1:
            raise RuntimeError('Motion interlock: ' + repr(f))
        if f['mode'] != 0 or f['changing'] != 0 or abs(f['steer']) > 0.02:
            raise RuntimeError('Unexpected wheel/motion mode: ' + repr(f))
        if abs(f['v']) > (0.005 if require_stationary else forward_feedback_limit) or abs(f['w']) > 0.02:
            raise RuntimeError('Unexpected feedback speed: ' + repr(f))
        return f

    def publish(v):
        msg = Twist()
        msg.linear.x = v
        publisher.publish(msg)
        command_log.write(json.dumps({'t': time.monotonic(), 'phase': phase, 'vx': v,
                                       'vy': 0.0, 'wz': 0.0}) + '\n')

    def observe(topic):
        def callback(msg):
            ros_counts[topic] += 1
            ros_log.write(json.dumps({'t': time.monotonic(), 'phase': phase,
                                      'topic': topic, 'msg': message_to_ordereddict(msg)}) + '\n')
        return callback

    def tf_callback(msg):
        for tf in msg.transforms:
            tf_edges[f'{tf.header.frame_id}->{tf.child_frame_id}'] += 1

    def interrupted(signum, _):
        raise KeyboardInterrupt(f'Signal {signum}')

    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    try:
        state = snapshot('before')
        can = state['linkinfo']['info_data']
        if 'UP' not in state['flags'] or can['state'] != 'ERROR-ACTIVE' or can['bittiming']['bitrate'] != 500000:
            raise RuntimeError('CAN state changed; no configuration will be applied')
        bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        bus.bind(('can0',))
        bus.setblocking(False)
        rclpy.init()
        node = rclpy.create_node('ranger_stage2_test')
        for topic, cls in [('/system_state', SystemState), ('/motion_state', MotionState),
                           ('/actuator_state', ActuatorStateArray), ('/wheel/odom', Odometry),
                           ('/battery_state', BatteryState)]:
            subscribers.append(node.create_subscription(cls, topic, observe(topic), 100))
        subscribers.append(node.create_subscription(TFMessage, '/tf', tf_callback, 100))
        pump(2.0)
        if 'ranger_base_node' in node.get_node_names():
            raise RuntimeError('An existing driver is present; will not start a second one')
        if node.get_publishers_info_by_topic('/base/safe_cmd_vel'):
            raise RuntimeError('Another command publisher is present')
        publisher = node.create_publisher(Twist, '/base/safe_cmd_vel', 1)
        cmd = [str(root / 'ws_base/install/ranger_base/lib/ranger_base/ranger_base_node'),
               '--ros-args', '--params-file', str(root / 'config/base_mini_v2.yaml'),
               '-r', '/cmd_vel:=/base/safe_cmd_vel']
        result['driver_command'] = cmd
        child = subprocess.Popen(cmd, stdout=driver_log, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        result['driver_pid'] = child.pid
        phase = 'static'
        print('Driver started once; checking static ROS/CAN feedback.', flush=True)
        pump(6.0)
        if child.poll() is not None:
            raise RuntimeError('Driver failed during static checks')
        client = node.create_client(GetParameters, '/ranger_base_node/get_parameters')
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('Parameter service unavailable')
        request = GetParameters.Request()
        request.names = ['robot_model', 'port_name', 'odom_topic_name', 'publish_odom_tf', 'update_rate']
        future = client.call_async(request)
        pump(1.0)
        if not future.done():
            raise RuntimeError('Parameter request timed out')
        values = future.result().values
        result['parameters'] = message_to_ordereddict(future.result())
        if [values[0].string_value, values[1].string_value, values[2].string_value,
                values[3].bool_value, values[4].integer_value] != ['ranger_mini_v2', 'can0', '/wheel/odom', False, 50]:
            raise RuntimeError('Unexpected real driver parameters')
        endpoints = node.get_subscriptions_info_by_topic('/base/safe_cmd_vel')
        result['command_subscriptions'] = [{'node': e.node_name, 'type': e.topic_type} for e in endpoints]
        if len(endpoints) != 1 or endpoints[0].node_name != 'ranger_base_node' or endpoints[0].topic_type != 'geometry_msgs/msg/Twist':
            raise RuntimeError('Unexpected command subscription')
        if any(e.node_name == 'ranger_base_node' for e in node.get_subscriptions_info_by_topic('/cmd_vel')):
            raise RuntimeError('Driver still subscribes to ordinary /cmd_vel')
        for topic in ('/system_state', '/motion_state', '/wheel/odom', '/actuator_state'):
            if ros_counts[topic] < 20:
                raise RuntimeError('Missing ROS stream: ' + topic)
        if tf_edges['odom->base_link']:
            raise RuntimeError('Unexpected odom->base_link TF publisher')
        result['static_ros_checks'] = 'PASS for parameters/remap and four live streams; BMS separately counted'
        result['static_can'] = fields()
        print('Static ROS checks passed. CAN feedback: ' + repr(fields()), flush=True)
        if args.motion:
            safe_for_pulse(require_stationary=True)
            phase = 'forward'
            print(f'Starting authorized pulse: vx=0.05 m/s, target {forward_duration:g} s, then zeros.', flush=True)
            start = time.monotonic()
            # Monotonic deadline; Python scheduling is not a hard real-time bound.
            deadline = start + forward_duration
            next_command = start
            result['motion_executed'] = True
            result['pulse_start_monotonic'] = start
            while time.monotonic() < deadline:
                safe_for_pulse()
                if len(node.get_publishers_info_by_topic('/base/safe_cmd_vel')) != 1:
                    raise RuntimeError('Command publisher conflict')
                now = time.monotonic()
                if now >= next_command and now < deadline:
                    publish(0.05)
                    next_command = now + 0.02
                pump(min(0.005, max(0.0, deadline - time.monotonic())))
            phase = 'explicit_zero'
            result['first_zero_monotonic'] = time.monotonic()
            publish(0.0)
            next_command = time.monotonic() + 0.02
            stop_deadline = time.monotonic() + 2.0
            zero_feedback_since = None
            while time.monotonic() < stop_deadline:
                if time.monotonic() >= next_command:
                    publish(0.0)
                    next_command = time.monotonic() + 0.02
                pump(0.005)
                f = fields()
                if abs(f['v']) <= 0.005 and abs(f['w']) <= 0.005:
                    zero_feedback_since = zero_feedback_since or time.monotonic()
                else:
                    zero_feedback_since = None
            result['post_zero_can'] = fields()
            result['zero_feedback_stable_seconds'] = (time.monotonic() - zero_feedback_since) if zero_feedback_since else 0.0
            if not zero_feedback_since or result['zero_feedback_stable_seconds'] < 0.5:
                raise RuntimeError('Zero command did not produce stable zero feedback; human hardware stop required')
            result['assessment'] = 'Explicit-zero CAN feedback observed; physical motion/stop needs human confirmation. Safety certification incomplete.'
        else:
            result['assessment'] = 'Static ROS checks complete; motion not requested.'
    except BaseException as error:
        result['primary_error'] = repr(error)
        result['abort_phase'] = phase
        result['assessment'] = 'PARTIAL/ABORTED: ' + repr(error)
        print(result['assessment'], flush=True)
    finally:
        phase = 'cleanup_zero'
        # Best effort only; no callback can guarantee stopping after a crash.
        if child is not None and child.poll() is None and publisher is not None:
            for _ in range(50):
                try:
                    publish(0.0)
                    pump(0.02)
                except BaseException:
                    break
            try:
                child.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                child.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=2.0)
                result['forced_driver_kill'] = True
        if child is not None:
            result['driver_exit'] = child.returncode
        phase = 'after_driver_exit'
        if node is not None and bus is not None:
            try:
                pump(2.0)
                result['after_driver_exit_can'] = fields()
                result['after_driver_exit_motion_samples'] = len(post_exit_motion)
                excursions = [sample for sample in post_exit_motion
                              if any(abs(value) > 0.005 for value in sample['values'][:3])]
                result['after_driver_exit_motion_excursions'] = excursions
                if excursions:
                    detail = ('nonzero motion feedback after driver exit; '
                              'physical observation required, stopping safety not passed.')
                    if 'ABORTED' in result['assessment']:
                        result['assessment'] += '; additionally: ' + detail
                    else:
                        result['assessment'] = 'PARTIAL/ABORTED: ' + detail
            except BaseException as error:
                result['post_exit_error'] = repr(error)
                detail = 'post-exit observation failed: ' + repr(error)
                if 'ABORTED' in result['assessment']:
                    result['assessment'] += '; additionally: ' + detail
                else:
                    result['assessment'] = 'PARTIAL/ABORTED: ' + detail
        result['ros_message_counts'] = dict(ros_counts)
        result['tf_edges'] = dict(tf_edges)
        result['end'] = datetime.datetime.now().astimezone().isoformat()
        try:
            snapshot('after')
        except Exception as error:
            result['snapshot_error'] = repr(error)
        (folder / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        for f in (raw_log, ros_log, command_log, driver_log):
            f.close()
        if bus is not None:
            bus.close()
        if node is not None:
            node.destroy_node()
            rclpy.try_shutdown()
        print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    return 2 if 'ABORTED' in result['assessment'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
