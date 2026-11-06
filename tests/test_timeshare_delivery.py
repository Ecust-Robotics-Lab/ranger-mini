#!/usr/bin/env python3
"""Real writer, isolated localhost ROS and synthetic clouds; no physical devices."""
import csv
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time
import unittest

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

ROOT = Path(__file__).resolve().parents[1]
EXE = ROOT/'ws_camera/install/timeshare_writer/lib/timeshare_writer/timeshare_writer_node'
BASE = 1590193436000000000


class Delivery(unittest.TestCase):
    def setUp(self):
        self.folder = Path(tempfile.mkdtemp(prefix=self._testMethodName+'-', dir=os.environ.get('EVIDENCE_DIR')))
        rclpy.init()
        self.node = rclpy.create_node('synthetic_writer_check')
        self.topic = '/synthetic_timeshare/p'+str(os.getpid())+'/'+self._testMethodName
        self.processes = []
        self.logs = []
        self.pub = None

    def tearDown(self):
        for proc in self.processes:
            if proc.poll() is None:
                os.kill(proc.pid, signal.SIGCONT)
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=2)
        for log in self.logs:
            log.close()
        self.node.destroy_node()
        rclpy.shutdown()

    def wait(self, predicate, seconds=5):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            if predicate():
                return
            rclpy.spin_once(self.node, timeout_sec=.01)
        self.fail('timeout; evidence: '+str(self.folder))

    def writer(self, *parameters, name='writer'):
        log = (self.folder/(name+'.log')).open('wb')
        self.logs.append(log)
        args = [str(EXE), '--ros-args', '-p', 'lidar_topic:='+self.topic,
                '-p', 'timeshare_path:='+str(self.folder/'timeshare'),
                '-p', 'csv_path:='+str(self.folder/'writer.csv')]
        for parameter in parameters:
            args += ['-p', parameter]
        proc = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.processes.append(proc)
        return proc

    def publisher(self, reliability=ReliabilityPolicy.RELIABLE):
        self.pub = self.node.create_publisher(PointCloud2, self.topic,
            QoSProfile(depth=100, reliability=reliability))

    def rows(self):
        path = self.folder/'writer.csv'
        if not path.exists():
            return []
        with path.open() as stream:
            return list(csv.DictReader(stream))

    def send(self, index, payload=0):
        msg = PointCloud2()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(BASE+index*100000000, 10**9)
        msg.header.frame_id = 'SYNTHETIC_NOT_A_SENSOR'
        msg.width, msg.height = payload//30, 1
        msg.point_step, msg.row_step = 30, payload
        msg.data = bytes(payload)
        self.pub.publish(msg)

    def ready(self):
        self.wait(lambda:self.pub.get_subscription_count()==1)

    def control(self, index):
        """Best effort may lose an individual probe; retry outside the scored batch."""
        next_send = [0.]
        target = BASE+index*100000000
        def delivered():
            if any(int(r['scan_start_ns'])==target for r in self.rows()):
                return True
            if time.monotonic()>=next_send[0]:
                self.send(index)
                next_send[0] = time.monotonic()+.1
            return False
        self.wait(delivered)

    def load(self, reliability, rate=50, count=80):
        self.publisher()
        self.writer('lidar_reliability:='+reliability)
        self.ready()
        endpoints = self.node.get_subscriptions_info_by_topic(self.topic)
        self.assertEqual(endpoints[0].qos_profile.reliability.name, reliability.upper())
        # Establish that delivery works before testing fragmentation/large payloads.
        self.control(-1)
        payload = 600000
        started = time.monotonic()
        for i in range(count):
            self.send(i, payload)
            deadline = started+(i+1)/rate
            while time.monotonic()<deadline:
                rclpy.spin_once(self.node, timeout_sec=min(.005, max(0., deadline-time.monotonic())))
        elapsed = time.monotonic()-started
        if reliability=='reliable':
            self.wait(lambda:sum(BASE<=int(r['scan_start_ns'])<BASE+count*100000000
                                 for r in self.rows())==count)
        else:
            # Best effort does not promise delivery of the last (or any large) sample.
            deadline = time.monotonic()+1
            while time.monotonic()<deadline:
                rclpy.spin_once(self.node, timeout_sec=.01)
        self.control(count)
        rows = [r for r in self.rows() if BASE<=int(r['scan_start_ns'])<BASE+count*100000000]
        stamps = [int(r['scan_start_ns']) for r in rows]
        expected = [BASE+i*100000000 for i in range(count)]
        self.assertEqual(stamps, sorted(set(stamps)))
        self.assertTrue(set(stamps)<=set(expected))
        if reliability=='reliable':
            self.assertEqual(stamps, expected)
        self.assertTrue(all(int(r['callback_system_ns'])>int(r['arrival_ns']) for r in rows))
        self.assertTrue(all(len(r['publisher_gid_hex'])==48 for r in rows))
        # DDS timestamp availability is implementation-dependent: empty is not zero error.
        self.assertTrue(all(not r['dds_source_ns'] or int(r['dds_source_ns'])>0 for r in rows))
        (self.folder/'result.json').write_text(json.dumps({'synthetic':True,
            'reliability':reliability,'requested_publish_hz':rate,'payload_bytes':payload,
            'actual_publish_loop_seconds':elapsed,'actual_average_publish_hz':count/elapsed,
            'small_control_samples_received':len(self.rows())-len(rows),
            'sent':count,'received':len(rows),'missing_stamps':sorted(set(expected)-set(stamps)),
            'dds_source_available':sum(bool(r['dds_source_ns']) for r in rows),
            'dds_receive_available':sum(bool(r['dds_received_ns']) for r in rows),
            'publication_sequence_available':sum(bool(r['publication_sequence_number']) for r in rows),
            'physical_sync_proven':False}, indent=2)+'\n')

    def test_reliable_large_cloud_delivery_and_metadata(self):
        self.load('reliable')

    def test_best_effort_explicit_comparison(self):
        self.load('best_effort')

    def test_reliable_at_sensor_rate(self):
        self.load('reliable', rate=10, count=30)

    def test_best_effort_at_sensor_rate(self):
        self.load('best_effort', rate=10, count=30)

    def test_invalid_parameters_do_not_touch_existing_shared_file(self):
        shared = self.folder/'timeshare'
        shared.write_bytes(b'original-evidence')
        for parameter in ('lidar_reliability:=typo', 'lidar_queue_depth:=0', 'lidar_queue_depth:=101'):
            proc = self.writer(parameter, name=parameter.split(':=')[0]+parameter.split(':=')[1])
            self.assertEqual(proc.wait(timeout=5), 2)
            self.assertEqual(shared.read_bytes(), b'original-evidence')
            self.assertFalse((self.folder/'writer.csv').exists())

    def test_incompatible_best_effort_publisher_has_no_silent_fallback(self):
        self.publisher(ReliabilityPolicy.BEST_EFFORT)
        self.writer()
        self.wait(lambda:'DDS incompatible QoS' in (self.folder/'writer.log').read_text())
        for i in range(3):
            self.send(i)
            rclpy.spin_once(self.node, timeout_sec=.05)
        self.assertEqual(self.rows(), [])
        self.assertEqual(struct.unpack('qq',(self.folder/'timeshare').read_bytes()), (0,0))

    def test_backlog_is_visible_in_host_metadata_not_rewritten_sensor_time(self):
        self.publisher()
        proc = self.writer()
        self.ready()
        self.send(0)
        self.wait(lambda:len(self.rows())==1)
        os.kill(proc.pid, signal.SIGSTOP)
        self.send(1)
        time.sleep(.25)  # Stop only our synthetic writer, never a device process.
        os.kill(proc.pid, signal.SIGCONT)
        self.wait(lambda:len(self.rows())==2)
        row = self.rows()[1]
        self.assertEqual(int(row['scan_start_ns']),BASE+100000000)
        if row['dds_source_ns']:
            self.assertGreater(int(row['callback_system_ns'])-int(row['dds_source_ns']),200000000)
        else:
            self.assertIn('unsupported timestamps/sequence are empty',(self.folder/'writer.log').read_text())


if __name__=='__main__':
    if os.environ.get('ROS_DOMAIN_ID')!='178' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise SystemExit('Requires ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1')
    unittest.main(verbosity=2)
