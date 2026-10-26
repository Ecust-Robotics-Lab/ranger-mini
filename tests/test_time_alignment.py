#!/usr/bin/env python3
"""Numerical interpolation and delayed 6DoF localization tests, no actuators."""
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ws_nav/src/ranger_localization_bridge'))
os.environ['ROS_DOMAIN_ID']='183'
os.environ['ROS_LOCALHOST_ONLY']='1'
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray
from ranger_localization_msgs.msg import LocalizationEstimate
import tf2_ros
from ranger_localization_bridge.alignment import PoseHistory, interpolate, stamp_ns
from ranger_localization_bridge.geometry import SE3, rpy_to_quat, se3_from_tf_msg


class HistoryTests(unittest.TestCase):
    def test_midpoint_translation_and_full_rotation(self):
        a=SE3((1,2,3)); b=SE3((3,4,5),rpy_to_quat(0,math.pi/2,0))
        m=interpolate(a,b,.5)
        self.assertLess(math.dist(m.t,(2,3,4)),1e-12)
        self.assertLess(math.dist(m.q,rpy_to_quat(0,math.pi/4,0)),1e-12)

    def test_antipodal_shortest_arc(self):
        a=SE3(q=rpy_to_quat(0,0,math.radians(179)))
        b=SE3(q=rpy_to_quat(0,0,math.radians(-179)))
        self.assertAlmostEqual(abs(interpolate(a,b,.5).yaw()),math.pi,places=8)
        anti=SE3(q=tuple(-v for v in a.q))
        self.assertAlmostEqual(abs(sum(x*y for x,y in zip(interpolate(a,anti,.5).q,a.q))),1.)

    def test_no_extrapolation_and_gap(self):
        h=PoseHistory(max_gap_s=.1)
        h.add(1_000_000_000,SE3());h.add(1_200_000_000,SE3((1,0,0)))
        for stamp in [900_000_000,1_100_000_000,1_300_000_000]:
            self.assertIsNone(h.lookup(stamp)[0])
        self.assertIsNotNone(h.lookup(1_000_000_000)[0])

    def test_duplicate_and_regression(self):
        h=PoseHistory();h.add(1_000_000_000,SE3())
        self.assertEqual(h.add(1_000_000_000,SE3((99,0,0))),'duplicate')
        self.assertEqual(h.lookup(1_000_000_000)[0].t,(0.,0.,0.))
        self.assertEqual(h.add(900_000_000,SE3()),'regressed')
        self.assertIsNone(h.lookup(1_000_000_000)[0])

    def test_integer_epoch_and_capacity(self):
        h=PoseHistory(duration_s=.2,max_gap_s=.1,max_samples=3)
        epoch=1_792_000_000_000_000_000
        for i in range(10):h.add(epoch+i*20_000_000,SE3((float(i),0,0)))
        self.assertEqual(len(h.samples),3)
        self.assertAlmostEqual(h.lookup(epoch+170_000_000)[0].t[0],8.5)
        self.assertIsNone(h.lookup(epoch)[0])


class DelayedWorld(Node):
    def __init__(self):
        super().__init__('delayed_world')
        self.start_ns=self.get_clock().now().nanoseconds
        self.delay=.18;self.freeze=False;self.frozen=None;self.odom_on=True;self.odom_back_once=False;self.loc_back_once=False
        self.ready=False;self.true_count=0;self.health={}
        self.odom_pub=self.create_publisher(Odometry,'/wheel/odom',10)
        self.loc_pub=self.create_publisher(LocalizationEstimate,'/localization/estimate',10)
        self.create_subscription(Bool,'/navigation_ready',self.on_ready,10)
        self.create_subscription(DiagnosticArray,'/localization_health',self.on_health,10)
        self.buf=tf2_ros.Buffer();self.listener=tf2_ros.TransformListener(self.buf,self)
        self.correction=SE3((2.,-1.,.3),rpy_to_quat(.1,-.1,.7))
        self.create_timer(.02,self.odom)
        self.create_timer(.05,self.loc)

    def on_ready(self,m):
        self.ready=m.data
        if m.data:self.true_count+=1

    def on_health(self,m):
        if m.status:self.health={v.key:v.value for v in m.status[0].values}

    def trajectory(self,ns):
        t=(ns-self.start_ns)*1e-9
        return SE3((.3*t*t,.2*t,.1*math.sin(.4*t)),rpy_to_quat(.03*t,.02*t,.4*t))

    def odom(self):
        if not self.odom_on:return
        ns=self.get_clock().now().nanoseconds
        if self.odom_back_once:
            ns-=60_000_000;self.odom_back_once=False
        pose=self.trajectory(ns);m=Odometry()
        m.header.stamp.sec=ns//1_000_000_000;m.header.stamp.nanosec=ns%1_000_000_000
        m.header.frame_id='odom';m.child_frame_id='base_link'
        m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=pose.t
        m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w=pose.q
        self.odom_pub.publish(m)

    def loc(self):
        ns=self.get_clock().now().nanoseconds-round(self.delay*1e9)
        if self.loc_back_once:
            ns-=200_000_000;self.loc_back_once=False
        if self.freeze:
            if self.frozen is None:self.frozen=ns
            ns=self.frozen
        pose=self.correction*self.trajectory(ns);m=LocalizationEstimate()
        m.header.stamp.sec=ns//1_000_000_000;m.header.stamp.nanosec=ns%1_000_000_000
        m.header.frame_id='map';m.base_frame='base_link';m.state=m.VALID;m.reason='known delayed trajectory'
        m.pose.position.x,m.pose.position.y,m.pose.position.z=pose.t
        m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w=pose.q
        self.loc_pub.publish(m)


def wait_for(node,predicate,timeout=6):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.01)
        if predicate():return True
    return False


class AlignedRosTests(unittest.TestCase):
    def test_delayed_correction_values_and_staleness(self):
        rclpy.init();node=DelayedWorld()
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'bridge.log';log=path.open('w')
            proc=subprocess.Popen([sys.executable,'-m','ranger_localization_bridge.bridge_node',
                '--ros-args','-p','adapter_backend:=external'],start_new_session=True,stdout=log,stderr=subprocess.STDOUT)
            try:
                self.assertTrue(wait_for(node,lambda:node.true_count>=5,12))
                errors=[]
                for delay in [.18,.32,.06]:
                    node.delay=delay
                    before=node.true_count
                    self.assertTrue(wait_for(node,lambda:node.true_count>=before+5,5))
                    end=time.monotonic()+.6
                    while time.monotonic()<end:
                        rclpy.spin_once(node,timeout_sec=.01)
                        if not node.ready:continue
                        tf=node.buf.lookup_transform('map','odom',rclpy.time.Time())
                        correction=se3_from_tf_msg(tf)
                        delta=node.correction.inverse()*correction
                        errors.append((math.dist(delta.t,(0,0,0)),abs(delta.yaw())))
                self.assertGreater(len(errors),30)
                max_translation=max(x[0] for x in errors);max_yaw=max(x[1] for x in errors)
                print('delayed correction max translation error=',max_translation,'m; yaw=',max_yaw,'rad')
                self.assertLess(max_translation,.001)
                self.assertLess(max_yaw,.0001)
                for attr in ['odom_back_once','loc_back_once']:
                    setattr(node,attr,True)
                    self.assertTrue(wait_for(node,lambda:not node.ready,3),attr)
                    before=node.true_count
                    self.assertTrue(wait_for(node,lambda:node.true_count>=before+3,5),attr+' recovery')
                node.freeze=True
                self.assertTrue(wait_for(node,lambda:not node.ready and node.health.get('time_aligned')=='0',3))
                node.freeze=False;before=node.true_count
                self.assertTrue(wait_for(node,lambda:node.true_count>=before+3))
                node.odom_on=False
                self.assertTrue(wait_for(node,lambda:not node.ready and node.health.get('odom_present')=='0',3))
                self.assertIsNone(proc.poll())
            finally:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                node.destroy_node();rclpy.shutdown();log.close();print(path.read_text())


class ClockResetTests(unittest.TestCase):
    def test_ros_clock_regression_clears_all_cached_transforms(self):
        from types import SimpleNamespace
        from ranger_localization_bridge.bridge_node import LocalizationBridge
        rclpy.init();node=LocalizationBridge()
        try:
            clock=[1_000_000_000_000]
            node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=clock[0]))
            node.now_s()
            node.odom_history.add(clock[0],SE3())
            node._last_odom_stamp=clock[0]*1e-9
            node._last_aligned_ns=clock[0];node._map_to_odom=SE3()
            node.sample=LocalizationEstimate();node.received=clock[0]*1e-9
            clock[0]-=1_000_000_000
            node.now_s()
            self.assertEqual(node.odom_history.samples,[])
            self.assertIsNone(node._map_to_odom);self.assertIsNone(node._last_aligned_ns)
            self.assertIsNone(node.sample);self.assertIsNone(node._last_odom_stamp)
        finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main(verbosity=2)
