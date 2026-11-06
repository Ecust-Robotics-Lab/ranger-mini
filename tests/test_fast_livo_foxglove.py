#!/usr/bin/env python3
"""Installed Foxglove Bridge over real loopback WebSocket, synthetic ROS only."""
import base64
import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]


def exact(sock, size):
    chunks = bytearray()
    while len(chunks)<size:
        data = sock.recv(size-len(chunks))
        if not data:raise EOFError('WebSocket closed')
        chunks.extend(data)
    return bytes(chunks)


def frame(sock):
    first, length = exact(sock, 2)
    if length & 128:raise ValueError('server frames must be unmasked')
    length &= 127
    if length == 126:length = struct.unpack('!H', exact(sock, 2))[0]
    elif length == 127:length = struct.unpack('!Q', exact(sock, 8))[0]
    if length>20_000_000:raise ValueError('unexpected WebSocket frame size')
    return first & 15, exact(sock, length)


def send_json(sock, value):
    data = json.dumps(value).encode()
    mask = os.urandom(4)
    size = len(data)
    head = bytes([129, 128|size]) if size<126 else bytes([129, 254])+struct.pack('!H', size)
    sock.sendall(head+mask+bytes(v ^ mask[i % 4] for i,v in enumerate(data)))


class FoxgloveTest(unittest.TestCase):
    def test_loopback_readonly_channels_and_message_delivery(self):
        import rclpy
        from sensor_msgs.msg import PointCloud2
        from geometry_msgs.msg import Twist
        env = dict(os.environ, ROS_DOMAIN_ID='188', ROS_LOCALHOST_ONLY='1', PYTHONDONTWRITEBYTECODE='1')
        os.environ.update(ROS_DOMAIN_ID='188', ROS_LOCALHOST_ONLY='1')
        rclpy.init()
        node = rclpy.create_node('foxglove_capture_synthetic')
        allowed = node.create_publisher(PointCloud2, '/fast_livo/view/map', 10)
        excluded = node.create_publisher(Twist, '/capture_test_excluded', 10)
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 0));port = probe.getsockname()[1]
        evidence = Path(os.environ.get('EVIDENCE_DIR', tempfile.mkdtemp(prefix='ranger-foxglove-')))
        bridge = None
        sock = None
        try:
            with (evidence/'foxglove_bridge.log').open('wb') as log:
                bridge = subprocess.Popen(['/opt/ros/humble/lib/foxglove_bridge/foxglove_bridge', '--ros-args',
                    '--params-file', str(ROOT/'config/fast_livo2/foxglove.yaml'), '-p', 'port:='+str(port)],
                    stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
                deadline = time.monotonic()+12
                while time.monotonic()<deadline:
                    self.assertIsNone(bridge.poll(), (evidence/'foxglove_bridge.log').read_text())
                    try:sock = socket.create_connection(('127.0.0.1', port), timeout=.5);break
                    except OSError:time.sleep(.1)
                self.assertIsNotNone(sock)
                sock.settimeout(5)
                key = base64.b64encode(os.urandom(16)).decode()
                sock.sendall(('GET / HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
                    'Sec-WebSocket-Version: 13\r\nSec-WebSocket-Key: '+key+'\r\n'
                    'Sec-WebSocket-Protocol: foxglove.sdk.v1\r\n\r\n').encode())
                response = bytearray()
                while not response.endswith(b'\r\n\r\n'):response.extend(exact(sock, 1))
                self.assertIn(b'101 Switching Protocols', response)
                channels = []
                server = None
                deadline = time.monotonic()+12
                while time.monotonic()<deadline:
                    opcode, payload = frame(sock)
                    if opcode != 1:continue
                    message = json.loads(payload)
                    if message['op'] == 'serverInfo':server = message
                    if message['op'] == 'advertise':channels.extend(message['channels'])
                    if server and any(c['topic']=='/fast_livo/view/map' for c in channels):break
                self.assertEqual(server['capabilities'], ['connectionGraph'])
                self.assertNotIn('/capture_test_excluded', [c['topic'] for c in channels])
                channel = next(c for c in channels if c['topic']=='/fast_livo/view/map')
                send_json(sock, {'op':'subscribe','subscriptions':[{'id':1,'channelId':channel['id']}]})
                cloud = PointCloud2()
                cloud.header.frame_id = 'fast_livo_local'
                node.create_timer(.05, lambda:allowed.publish(cloud))
                received = False
                deadline = time.monotonic()+5
                sock.settimeout(.1)
                while time.monotonic()<deadline:
                    rclpy.spin_once(node, timeout_sec=.02)
                    try:
                        opcode, payload = frame(sock)
                        if opcode == 2 and payload[0] == 1:
                            received = True;break
                    except socket.timeout:pass
                self.assertTrue(received)
                evidence.joinpath('foxglove_result.json').write_text(json.dumps({
                    'websocket_upgrade': True, 'capabilities': server['capabilities'],
                    'excluded_topic_hidden': True, 'received_synthetic_cloud': received,
                    'domain':188,'address':'127.0.0.1','physical_sensors_started':False}, indent=2)+'\n')
        finally:
            if sock:sock.close()
            if bridge:
                bridge.send_signal(signal.SIGINT)
                try:bridge.wait(timeout=6)
                except subprocess.TimeoutExpired:bridge.kill();bridge.wait(timeout=2)
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main(verbosity=2)
