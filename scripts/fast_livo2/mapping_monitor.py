#!/usr/bin/env python3
"""Private mapping health and a sensor-relative clock; never rewrites messages."""
import argparse
import json
import os
from pathlib import Path
import struct
import time

import rclpy
from rclpy.clock import Clock as RosClock, ClockType
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from diagnostic_msgs.msg import DiagnosticArray
from ranger_fast_livo_backend.msg import BackendEstimate
from session_manager import atomic_json
from fast_livo2.navigation.live_clock import StartupAnchor


class Monitor(Node):
    def __init__(self, folder, replay, localization=False):
        super().__init__('ranger_online_mapping_monitor')
        self.folder, self.replay = folder, replay
        self.localization = localization
        self.output = Path(os.environ['RANGER_SESSION_HEALTH_FILE'])
        self.started = time.monotonic()
        self.last, self.counts = {}, {}
        self.odometry, self.clouds = {}, {}
        self.pairs, self.sample_ns = 0, None
        self.frames_ok, self.clock_error = True, ''
        self.anchor = None
        self.startup_anchor = StartupAnchor() if localization and not replay else None
        self.backend, self.metrics, self.view = None, {}, {}
        self.backend_receipt = None
        self.receipt_time = None
        # Observers must never backpressure the frontend's reliable writer.
        # The backend and actual bag recorder retain their reliable subscriptions.
        output_qos = QoSProfile(depth=32, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/fast_livo/odometry', self.odom, output_qos)
        self.create_subscription(PointCloud2, '/fast_livo/deskewed_imu', self.cloud, output_qos)
        if localization:
            from ranger_fast_livo_localizer.msg import LocalizerEstimate
            from rclpy.parameter import Parameter
            self.set_parameters([Parameter('use_sim_time', value=True)])
            self.create_subscription(LocalizerEstimate, '/fast_livo/localizer/estimate', self.estimate, 10)
        else:
            self.create_subscription(BackendEstimate, '/fast_livo/backend/estimate', self.estimate, 10)
        self.create_subscription(DiagnosticArray, '/fast_livo/backend/health', self.diagnostics, 10)
        from std_msgs.msg import String
        self.create_subscription(String, '/fast_livo/capture/input_receipts', self.receipts, 10)
        from rclpy.qos import DurabilityPolicy
        self.create_subscription(String, '/fast_livo/backend/view/status', self.view_status,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.steady_clock = RosClock(clock_type=ClockType.STEADY_TIME)
        self.clock_group = MutuallyExclusiveCallbackGroup()
        self.health_group = MutuallyExclusiveCallbackGroup()
        if not replay:
            self.clock_pub = self.create_publisher(Clock, '/clock',
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_timer(.02, self.clock_tick, clock=self.steady_clock, callback_group=self.clock_group)
        self.create_timer(.2, self.health, clock=self.steady_clock, callback_group=self.health_group)

    @staticmethod
    def stamp(message):
        return message.header.stamp.sec*10**9+message.header.stamp.nanosec

    def received(self, name):
        self.last[name] = time.monotonic()
        self.counts[name] = self.counts.get(name, 0)+1

    def receipts(self, message):
        data=json.loads(message.data)
        if data.get('schema_version')!=1 or data.get('context')!=str(self.folder): return
        self.receipt_time=time.monotonic()
        rows=data.get('topics',{})
        for name,topic in [('lidar','/livox/lidar'),('imu','/livox/imu'),('camera','/front/image_sync')]:
            entry=rows.get(topic,{})
            self.counts[name]=entry.get('count',0)
            self.last[name]=entry.get('last_steady_ns',0)/1e9
        lidar=rows.get('/livox/lidar',{})
        if not self.replay and lidar.get('first_header_ns',0)>0:
            if self.startup_anchor:
                acquired=self.startup_anchor.observe(lidar,time.monotonic_ns())
                if self.startup_anchor.error:self.clock_error=self.startup_anchor.error
                if self.anchor is None and acquired:
                    self.anchor=acquired
                    atomic_json(self.folder/'clock_anchor.json', {'context':str(self.folder),
                        'anchor':self.anchor,'source_anchor':self.startup_anchor.source_anchor,
                        'startup_resets':self.startup_anchor.resets,'physical_sync_verified':False})
            elif self.anchor is None:self.anchor=(lidar['first_header_ns'],lidar['first_steady_ns'])
            if self.anchor:
                expected=self.anchor[0]+lidar.get('last_steady_ns',0)-self.anchor[1]
                if abs(lidar.get('last_header_ns',0)-expected)>2_000_000_000:
                    self.clock_error='sensor timestamp diverged from fixed steady-clock anchor by >2s'

    def clock_tick(self):
        if self.anchor:
            ns = self.anchor[0]+time.monotonic_ns()-self.anchor[1]
            message = Clock()
            message.clock.sec, message.clock.nanosec = divmod(ns, 10**9)
            self.clock_pub.publish(message)

    def odom(self, message):
        self.odometry[self.stamp(message)] = message
        self.match(self.stamp(message))

    def cloud(self, message):
        self.clouds[self.stamp(message)] = message
        self.match(self.stamp(message))

    def match(self, stamp):
        if stamp in self.odometry and stamp in self.clouds:
            odom, cloud = self.odometry.pop(stamp), self.clouds.pop(stamp)
            if (odom.header.frame_id != 'fast_livo_local' or odom.child_frame_id != 'fast_livo_imu'
                    or cloud.header.frame_id != 'fast_livo_imu'):
                self.frames_ok = False
            self.pairs += 1
            self.sample_ns = stamp
            self.received('paired')
        for cache in (self.odometry, self.clouds):
            while len(cache) > 32: cache.pop(next(iter(cache)))

    def estimate(self, message):
        self.backend_receipt = time.monotonic()
        self.backend = {'session_id': message.session_id, 'state': message.state,
            'reason': message.reason, 'pose_valid': message.pose_valid,
            'stamp_ns': self.stamp(message), 'graph_version': message.graph_version,
            'scan_id': message.scan_id, 'loop_event_id': getattr(message, 'loop_event_id', 0)}
        if self.localization:
            self.metrics = json.loads(message.quality_json or '{}')
            self.backend['map_uuid'] = message.map_uuid
            self.backend['request_id'] = message.initialization_request_id
            self.backend['pose'] = {'xyz': [message.pose.position.x, message.pose.position.y, message.pose.position.z],
                'xyzw': [message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w]}
            with (self.folder/'localization_samples.jsonl').open('a') as handle:
                handle.write(json.dumps({**self.backend, 'received_steady': self.backend_receipt,
                    'clock_ns': self.get_clock().now().nanoseconds,
                    'metrics': {k: self.metrics.get(k) for k in ('queued_jobs', 'skipped_scans',
                        'processed_scans', 'stale_results', 'sample_age_seconds', 'elapsed_ms')}})+'\n')

    def diagnostics(self, message):
        for status in message.status:
            for value in status.values:
                if value.key == 'metrics': self.metrics = json.loads(value.value)

    def view_status(self, message):
        self.view = json.loads(message.data)

    def health(self):
        now = time.monotonic()
        alive = lambda name: now-self.last.get(name, -1e9) < 1.5
        backend_alive = self.backend_receipt is not None and now-self.backend_receipt < 1.5
        backend_ok = backend_alive and self.backend['state'] != BackendEstimate.ERROR
        backend_ready = backend_ok and self.backend['pose_valid'] and self.backend['state'] == BackendEstimate.VALID
        counters = self.metrics.get('pair_counters', {})
        input_loss = self.metrics.get('invalid_messages', 0) or any(counters.get(k, 0) for k in
            ('full_complete', 'cloud_byte_budget_rejected', 'capacity_evicted_unpaired', 'unpaired_timeout'))
        backend_ready = backend_ready and not input_loss
        checks = {'probe_live': now-self.started > 2 and not self.clock_error, 'lidar': alive('lidar') and alive('imu'),
            'camera': alive('camera'), 'frontend': alive('paired') and self.pairs >= 3 and self.frames_ok,
            'backend_ready': backend_ready, 'clock_ok': not self.clock_error,
            'backend_listening': any(e.node_name == ('ranger_fast_livo_localizer' if self.localization else 'ranger_fast_livo_backend') for e in
                self.get_subscriptions_info_by_topic('/fast_livo/deskewed_imu'))}
        if self.localization:
            checks['localizer_alive'] = backend_ok and bool(self.backend.get('map_uuid'))
        checks['receipt_probe_live']=self.receipt_time is not None and now-self.receipt_time<1.
        for key, topic in [('lidar_unclaimed', '/livox/lidar'), ('camera_unclaimed', '/front/image_sync'),
                           ('frontend_unclaimed', '/fast_livo/odometry'),
                           ('receipts_unclaimed', '/fast_livo/capture/input_receipts'),
                           ('backend_unclaimed', '/fast_livo/localizer/estimate' if self.localization else '/fast_livo/backend/estimate')]:
            checks[key] = not self.get_publishers_info_by_topic(topic)
        clocks = self.get_publishers_info_by_topic('/clock')
        checks['clock_unclaimed'] = not clocks if self.replay else len(clocks) == 1 and clocks[0].node_name == self.get_name()
        checks['frontend_subscribed'] = all(any(e.node_name == 'laserMapping' or 'mapping' in e.node_name.lower()
            or 'fastlivo' in e.node_name.lower() for e in self.get_subscriptions_info_by_topic(topic))
            for topic in ('/livox/lidar', '/livox/imu', '/front/image_sync'))
        if not self.replay:
            try:
                high, low = struct.unpack('qq', Path(os.environ['RANGER_LIVO_TIMESHARE']).read_bytes())
                checks['writer'] = high > 0 and high % 2 == 0 and low > 0
            except (OSError, ValueError, struct.error): checks['writer'] = False
        try:
            recording = json.loads((self.folder/'recording.json').read_text())
        except (OSError, ValueError): recording = {}
        checks['capture_recording'] = recording.get('state') == 'RECORDING'
        state = {'monotonic': now, 'checks': checks, 'counts': dict(self.counts),
            'paired_frames': self.pairs, 'sample_ns': self.sample_ns, 'backend': self.backend,
            'backend_alive': backend_alive, 'backend_metrics': self.metrics, 'view': self.view,
            'frontend_backend_lag_seconds': ((self.sample_ns-self.backend['stamp_ns'])/1e9
                if self.sample_ns is not None and self.backend and self.backend['pose_valid'] else None),
            'clock': {'mode': 'replay_clock' if self.replay else 'fixed_lidar_header_steady_anchor',
                'anchor': self.anchor, 'error': self.clock_error, 'physical_host_sync_proven': False}}
        atomic_json(self.output, state)
        atomic_json(self.folder/('localization_health.json' if self.localization else 'mapping_health.json'), state)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder', required=True, type=Path)
    parser.add_argument('--replay', action='store_true')
    parser.add_argument('--localization', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = Monitor(args.folder, args.replay, args.localization)
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try: executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
