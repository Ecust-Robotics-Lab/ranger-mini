import json,time,uuid,math,argparse
from pathlib import Path
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Bool,String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ranger_navigation_msgs.msg import ControlCommand
from ranger_localization_msgs.msg import LocalizationEstimate

p=argparse.ArgumentParser();p.add_argument('--name',required=True);p.add_argument('--mode',type=int,default=0);p.add_argument('--vx',type=float,default=0.);p.add_argument('--vy',type=float,default=0.);p.add_argument('--wz',type=float,default=0.);p.add_argument('--duration',type=float,default=1.);a=p.parse_args()
assert math.hypot(a.vx,a.vy)<=.05 and abs(a.wz)<=.1 and 0<a.duration<=2 and a.mode in (0,1,2)
rclpy.init();n=rclpy.create_node('field_micro_'+a.name,parameter_overrides=[Parameter('use_sim_time',Parameter.Type.BOOL,True)])
enable=n.create_publisher(Bool,'/ranger/motion_enable',1);pub=n.create_publisher(ControlCommand,'/teleop/control',1)
wheel=None;wheel_at=None;healthy=False;health_at=None;pose=None;pose_at=None;mode=None;actual=[];trace=[];diag='unknown'
def on_wheel(m):
 global wheel,wheel_at
 wheel,wheel_at=m,time.monotonic()
def health(m):
 global healthy,health_at
 healthy,health_at=m.data,time.monotonic()
def loc(m):
 global pose,pose_at
 pose,pose_at=m,time.monotonic()
def on_actual(m):actual.append([time.monotonic(),m.linear.x,m.linear.y,m.angular.z])
def on_mode(m):
 global mode
 mode=m.data
from std_msgs.msg import UInt8
n.create_subscription(Odometry,'/fast_livo/navigation/wheel/odom',on_wheel,10)
n.create_subscription(Bool,'/navigation/safety_ready',health,10)
n.create_subscription(LocalizationEstimate,'/localization/estimate',loc,10)
n.create_subscription(Twist,'/base/safe_cmd_vel',on_actual,10)
n.create_subscription(UInt8,'/navigation/actual_mode',on_mode,10)
session=uuid.uuid4().hex;began=time.monotonic();start_xy=None;requested=False;abort=None;phase='prepare';pulse_start=None;last_pub=0.;stop_start=None
start_map=None
def map_pose():
 if pose is None:return None
 q=pose.pose.orientation;p=pose.pose.position
 return [p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
def fresh():
 t=time.monotonic();return healthy and wheel is not None and pose is not None and pose.state==pose.VALID and all(x is not None and 0<=t-x<.25 for x in (wheel_at,health_at,pose_at))
def emit(moving,armed):
 enable.publish(Bool(data=armed));m=ControlCommand();m.header.stamp=wheel.header.stamp if wheel is not None else n.get_clock().now().to_msg();m.header.frame_id='base_link';m.session_id=session;m.generation=0;m.mode=a.mode;m.valid=wheel is not None;m.reason='field_micro_'+a.name
 if moving:m.twist.linear.x=a.vx;m.twist.linear.y=a.vy;m.twist.angular.z=a.wz
 pub.publish(m)
try:
 while time.monotonic()-began<10:
  rclpy.spin_once(n,timeout_sec=.005);now=time.monotonic()
  if wheel is not None:
   xy=[wheel.pose.pose.position.x,wheel.pose.pose.position.y];v=wheel.twist.twist;trace.append({'time':now,'phase':phase,'xy':xy,'map_pose':map_pose(),'v':[v.linear.x,v.linear.y,v.angular.z],'mode':mode,'health':healthy})
  if phase=='prepare' and now-began>1.5 and fresh():
   phase='arm';stop_start=now;start_xy=xy;start_map=map_pose()
  if phase=='arm' and now-stop_start>.8 and mode==a.mode and fresh():phase='pulse';pulse_start=now;requested=True
  if phase=='pulse':
   if not fresh():abort='input_health_withdrawn';phase='stop';stop_start=now
   elif math.dist(xy,start_xy)>.10:abort='displacement_limit';phase='stop';stop_start=now
   elif now-pulse_start>=a.duration:phase='stop';stop_start=now
  if now-last_pub>=.04:
   emit(phase=='pulse',phase in ('arm','pulse'));last_pub=now
  if phase=='stop' and now-stop_start>1.5:break
  if phase in ('prepare','arm') and now-began>6:abort='not_ready_or_mode_not_confirmed';break
finally:
 for _ in range(12):
  emit(False,False);rclpy.spin_once(n,timeout_sec=.02)
 end_xy=[wheel.pose.pose.position.x,wheel.pose.pose.position.y] if wheel is not None else None
 result={'name':a.name,'mode':a.mode,'command':[a.vx,a.vy,a.wz],'pulse_requested':requested,'abort':abort,'start_xy':start_xy,'end_xy':end_xy,'wheel_displacement_m':math.dist(start_xy,end_xy) if start_xy and end_xy else None,'final_wheel_velocity':[wheel.twist.twist.linear.x,wheel.twist.twist.linear.y,wheel.twist.twist.angular.z] if wheel is not None else None,'nonzero_final_outputs':sum(max(abs(x) for x in row[1:])>1e-6 for row in actual),'actual_output':actual,'trace':trace,'note':'Real finite test through teleop/control + independent gate; wheel distance is estimated, not ground truth brake distance.'}
 result.update(start_map_pose=start_map,end_map_pose=map_pose())
 if start_map and map_pose():
  end=map_pose();dx=end[0]-start_map[0];dy=end[1]-start_map[1];yaw=start_map[2]
  result['localization_body_delta']=[math.cos(yaw)*dx+math.sin(yaw)*dy,-math.sin(yaw)*dx+math.cos(yaw)*dy,math.remainder(end[2]-yaw,2*math.pi)]
 Path('/home/lab307/ranger_nav/docs/exact_mppi_evidence/live_changed_scene_20261110/'+a.name+'.json').write_text(json.dumps(result,indent=2))
 print(json.dumps({k:v for k,v in result.items() if k not in ('trace','actual_output')},indent=2));n.destroy_node();rclpy.shutdown()
