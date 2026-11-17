import json,time
from pathlib import Path
import rclpy
from rclpy.parameter import Parameter
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict

rclpy.init()
node=rclpy.create_node('changed_scene_field_probe',parameter_overrides=[Parameter('use_sim_time',Parameter.Type.BOOL,True)])
types={'/localization/estimate':'ranger_localization_msgs/msg/LocalizationEstimate',
 '/navigation_ready':'std_msgs/msg/Bool','/navigation/state':'std_msgs/msg/String',
 '/motion_state':'ranger_msgs/msg/MotionState','/system_state':'ranger_msgs/msg/SystemState',
 '/fast_livo/navigation/wheel/odom':'nav_msgs/msg/Odometry',
 '/base/safe_cmd_vel':'geometry_msgs/msg/Twist','/navigation/safety_ready':'std_msgs/msg/Bool',
 '/ranger/motion_permitted':'std_msgs/msg/Bool','/navigation/control_owner':'std_msgs/msg/String',
 '/diagnostics':'diagnostic_msgs/msg/DiagnosticArray'}
latest={};counts={};start=time.monotonic()
for topic,kind in types.items():
 def receive(msg,topic=topic):
  latest[topic]={'received_steady':time.monotonic(),'message':message_to_ordereddict(msg)}
  counts[topic]=counts.get(topic,0)+1
 node.create_subscription(get_message(kind),topic,receive,10)
try:
 while time.monotonic()-start<15:
  rclpy.spin_once(node,timeout_sec=.05)
  if latest:
   data={'steady':time.monotonic(),'clock_ns':node.get_clock().now().nanoseconds,'counts':counts,'topics':latest}
   path=Path('/home/lab307/ranger_nav/.runtime/live_field_probe_state.json')
   path.write_text(json.dumps(data))
finally:
 print(json.dumps({'counts':counts,'topics':latest},indent=2))
 node.destroy_node();rclpy.shutdown()
