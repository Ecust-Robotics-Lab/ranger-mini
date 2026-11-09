#!/usr/bin/env python3
"""Convert wheel sampling stamps to the managed sensor-clock; no driver or TF."""
import argparse
from collections import deque
import copy
import json
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from live_clock import LiveClock


class WheelClockNode(Node):
    def __init__(self, folder):
        super().__init__('fast_livo_wheel_clock')
        self.folder=folder;self.mapping=LiveClock(str(folder))
        self.forwarded=0;self.dropped=0;self.last_stamp=None
        self.pending=deque();self.clock_ns=None;self.clock_floor_ns=None;self.clock_received=None
        self.pub=self.create_publisher(Odometry,'/fast_livo/navigation/wheel/odom',100)
        self.health=self.create_publisher(String,'/fast_livo/navigation/clock_status',10)
        self.create_subscription(String,'/fast_livo/capture/input_receipts',self.receipts,10)
        self.create_subscription(Odometry,'/wheel/odom',self.wheel,qos_profile_sensor_data)
        self.create_subscription(Clock,'/clock',self.on_clock,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(.1,self.tick)

    def receipts(self,msg):
        try:
            if self.mapping.anchor is None:
                path=self.folder/'clock_anchor.json'
                if not path.exists():return
                anchor=json.loads(path.read_text())
                if anchor.get('context')!=str(self.folder):return
                self.mapping.set_anchor(anchor['anchor'],anchor['source_anchor'],time.time_ns(),time.monotonic_ns())
            self.mapping.receipts(json.loads(msg.data),time.time_ns(),time.monotonic_ns())
        except (ValueError,TypeError,KeyError):self.mapping.fail('invalid receipt record')

    def wheel(self,msg):
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if self.last_stamp is not None and stamp<=self.last_stamp:
            self.mapping.fail('wheel clock regressed; reopen session')
        self.last_stamp=stamp
        ns=self.mapping.convert(stamp,time.time_ns(),time.monotonic_ns())
        unique=(len(self.get_publishers_info_by_topic('/wheel/odom'))==1 and
                len(self.get_publishers_info_by_topic('/fast_livo/capture/input_receipts'))==1)
        if ns is None or not unique or msg.header.frame_id!='odom' or msg.child_frame_id!='base_link':
            self.dropped+=1;return
        out=copy.deepcopy(msg);out.header.stamp.sec,out.header.stamp.nanosec=divmod(ns,10**9)
        if len(self.pending)>=100:self.pending.popleft();self.dropped+=1
        self.pending.append((ns,time.monotonic_ns(),out));self.flush()

    def on_clock(self,msg):
        ns=msg.clock.sec*10**9+msg.clock.nanosec
        if self.clock_ns is not None and ns<self.clock_ns:
            self.mapping.fail('sensor ROS clock regressed; reopen session');self.pending.clear()
        self.clock_floor_ns=self.clock_ns;self.clock_ns=ns;self.clock_received=time.monotonic_ns()
        self.flush()

    def flush(self):
        now=time.monotonic_ns()
        if (self.clock_floor_ns is None or self.clock_received is None or now-self.clock_received>500_000_000
                or not self.mapping.ready(time.time_ns(),now)):return
        # Wait for one following clock tick, giving downstream clock callbacks
        # time to run. Sampling stamps are retained; future data is never forced in.
        while self.pending and self.pending[0][0]<=self.clock_floor_ns:
            ns,received,msg=self.pending.popleft()
            if self.clock_ns-ns>500_000_000 or now-received>500_000_000:
                self.dropped+=1;continue
            self.pub.publish(msg);self.forwarded+=1

    def tick(self):
        status=self.mapping.status(time.time_ns(),time.monotonic_ns())
        status.update(forwarded=self.forwarded,dropped=self.dropped,pending=len(self.pending),
                      clock_ns=self.clock_ns,original_wheel_modified=False)
        msg=String();msg.data=json.dumps(status);self.health.publish(msg)
        path=self.folder/'wheel_clock.json';tmp=path.with_suffix('.tmp')
        tmp.write_text(json.dumps(status,indent=2)+'\n');tmp.replace(path)


def main():
    p=argparse.ArgumentParser();p.add_argument('--folder',type=Path,required=True);args=p.parse_args()
    rclpy.init();node=WheelClockNode(args.folder.resolve())
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
