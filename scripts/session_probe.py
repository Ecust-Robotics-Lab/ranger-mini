#!/usr/bin/env python3
"""Read-only ROS health observer for managed sessions; publishes no control."""
import argparse
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu, PointCloud2
from livox_ros_driver2.msg import CustomMsg
import tf2_ros

from session_manager import atomic_json


class Probe(Node):
    def __init__(self, output):
        super().__init__('ranger_session_probe_'+str(os.getpid()))
        self.output = output
        self.seen = {}
        self.ready = False
        self.wheel_valid = False
        self.odom_max_age = float(os.environ.get('ODOM_MAX_AGE_S', '.5'))
        self.pose_max_age = float(os.environ.get('TF_MAX_AGE_S', '.5'))
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        # Raw serialized clouds avoid converting every point to Python objects.
        self.create_subscription(CustomMsg, '/livox/lidar', lambda _: self.mark('cloud'),
                                 qos_profile_sensor_data, raw=True)
        self.create_subscription(Imu, '/livox/imu', lambda _: self.mark('imu'), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/wheel/odom', self.odom, 10)
        self.create_subscription(Bool, '/navigation_ready', self.on_ready, 10)
        self.create_subscription(Bool, '/ranger/motion_permitted', lambda _: self.mark('gate'), 10)
        self.create_subscription(PointCloud2, '/livox/obstacles', lambda _: self.mark('obstacles'), 10, raw=True)
        names = ['collision_monitor', 'map_server', 'planner_server', 'controller_server',
                 'bt_navigator', 'velocity_smoother']
        self.lifecycle_clients = {name: self.create_client(GetState, '/'+name+'/get_state') for name in names}
        self.pending, self.states = {}, {}
        self.create_timer(.2, self.tick)

    def mark(self, name):
        self.seen[name] = time.monotonic()

    def odom(self, msg):
        age = self.get_clock().now().nanoseconds*1e-9 - (msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
        self.wheel_valid = (msg.header.frame_id == 'odom' and msg.child_frame_id == 'base_link'
                            and 0 <= age <= self.odom_max_age)
        self.mark('wheel')

    def on_ready(self, msg):
        self.ready = msg.data
        self.mark('ready')

    def fresh(self, name, limit=.7):
        return name in self.seen and time.monotonic()-self.seen[name] <= limit

    def tick(self):
        now = time.monotonic()
        for name, client in self.lifecycle_clients.items():
            if name in self.pending:
                future, started = self.pending[name]
                if future.done():
                    try:
                        result = future.result()
                        self.states[name] = (result.current_state.id, now)
                    except Exception:
                        self.states.pop(name, None)
                    self.pending.pop(name)
                elif now-started > 1.:
                    future.cancel(); self.pending.pop(name)
            if name not in self.pending and client.service_is_ready():
                self.pending[name] = (client.call_async(GetState.Request()), now)
        topics = ['/livox/lidar', '/livox/imu', '/wheel/odom', '/localization/estimate',
                  '/navigation_ready', '/lightning/loc_status', '/lightning/loc_confidence',
                  '/base/safe_cmd_vel', '/cmd_vel_after_gate', '/cmd_vel_gated_in',
                  '/cmd_vel_mux', '/cmd_vel_nav', '/map', '/livox/obstacles', '/ranger/motion_permitted']
        publishers = {topic: self.count_publishers(topic) for topic in topics}
        native, sensor_tf = False, False
        try:
            tr = self.buffer.lookup_transform('map', os.environ.get('LOC_CHILD_FRAME', 'lightning_imu_pose'), rclpy.time.Time())
            age = self.get_clock().now().nanoseconds*1e-9 - (tr.header.stamp.sec+tr.header.stamp.nanosec*1e-9)
            native = 0 <= age <= self.pose_max_age
        except tf2_ros.TransformException:
            pass
        try:
            self.buffer.lookup_transform('base_link', 'livox_frame', rclpy.time.Time())
            sensor_tf = True
        except tf2_ros.TransformException:
            pass
        checks = {
            'probe_live': True,
            'lidar': self.fresh('cloud') and self.fresh('imu') and
                     publishers['/livox/lidar'] == publishers['/livox/imu'] == 1,
            'odom': self.fresh('wheel', self.odom_max_age) and self.wheel_valid and publishers['/wheel/odom'] == 1,
            'localization': self.fresh('ready', .5) and self.ready and
                            publishers['/navigation_ready'] == publishers['/localization/estimate'] == 1,
            'native_localization': native,
            'sensor_tf': sensor_tf,
            'lidar_unclaimed': publishers['/livox/lidar'] == publishers['/livox/imu'] == 0,
            'odom_unclaimed': publishers['/wheel/odom'] == 0,
            'native_unclaimed': publishers['/lightning/loc_status'] == publishers['/lightning/loc_confidence'] == 0,
            'bridge_unclaimed': publishers['/navigation_ready'] == 0 and
                (os.environ.get('LOCALIZATION_BACKEND') == 'standard' or publishers['/localization/estimate'] == 0),
            'nav_unclaimed': all(publishers[t] == 0 for t in ['/base/safe_cmd_vel',
                '/cmd_vel_after_gate', '/cmd_vel_gated_in', '/cmd_vel_mux', '/cmd_vel_nav', '/map', '/livox/obstacles', '/ranger/motion_permitted']),
            'map_unclaimed': publishers['/map'] == 0,
            'navigation': all(n in self.states and self.states[n][0] == 3 and
                              now-self.states[n][1] < 2. for n in self.lifecycle_clients) and
                              publishers['/base/safe_cmd_vel'] == publishers['/map'] == 1 and
                              self.fresh('gate', .5) and self.fresh('obstacles') and
                              all(publishers[t] == 1 for t in ['/cmd_vel_after_gate',
                                  '/cmd_vel_gated_in', '/cmd_vel_mux', '/livox/obstacles']),
        }
        atomic_json(self.output, {'monotonic': now, 'checks': checks, 'publishers': publishers,
                                  'lifecycle_states': self.states})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=os.environ.get('RANGER_SESSION_HEALTH_FILE'))
    args = parser.parse_args()
    if args.output is None:
        parser.error('--output or RANGER_SESSION_HEALTH_FILE is required')
    rclpy.init()
    node = Probe(args.output)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
