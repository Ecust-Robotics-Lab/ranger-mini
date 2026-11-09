"""Isolated ROS transport check; no devices, TF or controller."""
import json
import sys
import tempfile
import time
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/fast_livo2/navigation'))
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from wheel_clock_node import WheelClockNode

class TransportTest(unittest.TestCase):
    def test_sample_conversion_and_loss(self):
        rclpy.init()
        with tempfile.TemporaryDirectory() as d:
            converter=WheelClockNode(Path(d));driver=Node('clock_test_inputs')
            wheel=driver.create_publisher(Odometry,'/wheel/odom',10)
            receipts=driver.create_publisher(String,'/fast_livo/capture/input_receipts',10)
            clock=driver.create_publisher(Clock,'/clock',10)
            output=[];driver.create_subscription(Odometry,'/fast_livo/navigation/wheel/odom',output.append,100)
            def pump(seconds):
                end=time.monotonic()+seconds
                while time.monotonic()<end:
                    rclpy.spin_once(converter,timeout_sec=.005);rclpy.spin_once(driver,timeout_sec=.005)
            try:
                pump(1)
                steady=time.monotonic_ns();sensor=1_590_000_000_000_000_001
                (Path(d)/'clock_anchor.json').write_text(json.dumps({'context':d,
                    'anchor':[sensor,steady-1_100_000_000],'source_anchor':[sensor,steady-1_100_000_000]}))
                msg=String();msg.data=json.dumps({'schema_version':1,'context':d,'topics':{'/livox/lidar':{
                    'count':12,'first_header_ns':sensor,'first_steady_ns':steady-1_100_000_000,
                    'last_header_ns':sensor+1_100_000_000,'last_steady_ns':steady}}})
                receipts.publish(msg);pump(.1)
                sample=time.time_ns()-10_000_000
                m=Odometry();m.header.frame_id='odom';m.child_frame_id='base_link'
                m.header.stamp.sec,m.header.stamp.nanosec=divmod(sample,10**9);m.pose.pose.position.x=2.3;m.pose.pose.orientation.w=1.
                wheel.publish(m);pump(.1)
                self.assertEqual(len(output),0,'future wheel must wait for clock')
                expected=sample-converter.mapping.host_minus_steady+sensor-(steady-1_100_000_000)
                for ns in [expected-1,expected+1,expected+20_000_000]:
                    tick=Clock();tick.clock.sec,tick.clock.nanosec=divmod(ns,10**9);clock.publish(tick);pump(.03)
                self.assertEqual(len(output),1)
                actual=output[0].header.stamp.sec*10**9+output[0].header.stamp.nanosec
                self.assertEqual(actual,sample-converter.mapping.host_minus_steady+sensor-(steady-1_100_000_000))
                self.assertEqual(output[0].pose,m.pose)
                self.assertEqual(m.header.stamp.sec*10**9+m.header.stamp.nanosec,sample)
                pump(.6)
                sample2=time.time_ns();m.header.stamp.sec,m.header.stamp.nanosec=divmod(sample2,10**9)
                wheel.publish(m);pump(.1);self.assertEqual(len(output),1)
                self.assertEqual(len(driver.get_publishers_info_by_topic('/tf')),0)
                self.assertEqual(len(driver.get_publishers_info_by_topic('/cmd_vel')),0)
            finally:converter.destroy_node();driver.destroy_node();rclpy.shutdown()
if __name__=='__main__':unittest.main(verbosity=2)
