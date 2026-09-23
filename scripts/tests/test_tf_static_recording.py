#!/usr/bin/env python3
"""Isolated localhost/domain test: publish once BEFORE rosbag recorder starts."""
import os
os.environ['ROS_DOMAIN_ID']='178';os.environ['ROS_LOCALHOST_ONLY']='1'
import pathlib,tempfile,subprocess,signal,time,json,sqlite3
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
root=pathlib.Path(__file__).resolve().parents[2];folder=pathlib.Path(tempfile.mkdtemp(prefix='mid360-static-qos-synthetic-'))
rclpy.init();node=rclpy.create_node('synthetic_static_test')
pub=node.create_publisher(TFMessage,'/tf_static',QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
t=TransformStamped();t.header.frame_id='synthetic_parent';t.child_frame_id='synthetic_child';t.transform.rotation.w=1.;pub.publish(TFMessage(transforms=[t]))
time.sleep(.2)
env=os.environ.copy();env['ROS_LOG_DIR']=str(folder/'logs')
with (folder/'recorder.log').open('w') as log:
 child=subprocess.Popen(['ros2','bag','record','-s','sqlite3','-o',str(folder/'bag'),'--qos-profile-overrides-path',str(root/'config/mid360/bag_qos.yaml'),'/tf_static'],stdout=log,stderr=subprocess.STDOUT,env=env)
 try:
  end=time.monotonic()+4
  while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
 finally:child.send_signal(signal.SIGINT);child.wait(timeout=10);node.destroy_node();rclpy.shutdown()
assert child.returncode==0
with sqlite3.connect((folder/'bag/bag_0.db3').as_uri()+'?mode=ro',uri=True) as db:count=db.execute('select count(*) from messages').fetchone()[0]
assert count==1,count
print(json.dumps(dict(result='PASS',scope='synthetic localhost ROS_DOMAIN_ID=178 only',bag=str(folder/'bag'),messages=count,meaning='One retained /tf_static published before recorder startup was captured; no robot TF published'),indent=2))
