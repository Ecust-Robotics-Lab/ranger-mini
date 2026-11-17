"""Finite authorized live goal, cancellation and stop evidence. No device startup."""
import argparse,json,math,time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from std_msgs.msg import Bool,String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from ranger_localization_msgs.msg import LocalizationEstimate

p=argparse.ArgumentParser();p.add_argument('--name',required=True)
p.add_argument('--dx',type=float,default=0);p.add_argument('--dy',type=float,default=0)
p.add_argument('--x',type=float);p.add_argument('--y',type=float);p.add_argument('--yaw',type=float)
p.add_argument('--ready-timeout',type=float,default=20);p.add_argument('--timeout',type=float,default=60);p.add_argument('--cancel-after',type=float)
a=p.parse_args();assert 0<a.timeout<=600
out=Path('/home/lab307/ranger_nav/docs/exact_mppi_evidence/live_changed_scene_20261110')/(a.name+'.json')
assert not out.exists()
rclpy.init();n=rclpy.create_node('field_goal_'+a.name,parameter_overrides=[Parameter('use_sim_time',Parameter.Type.BOOL,True)])
permit=n.create_publisher(Bool,'/ranger/motion_enable',1)
client=ActionClient(n,NavigateToPose,'/navigate_to_pose')
loc=None;loc_at=0;health=False;health_at=0;state={};events=[];outputs=[];wheel=[];poses=[]
def estimate(m):
 global loc,loc_at
 loc=m;loc_at=time.monotonic();q=m.pose.orientation
 poses.append([loc_at,m.pose.position.x,m.pose.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),m.state])
def ready(m):
 global health,health_at
 health=m.data;health_at=time.monotonic()
def status(m):
 global state
 value=json.loads(m.data)
 if value!=state:events.append([time.monotonic(),value])
 state=value
n.create_subscription(LocalizationEstimate,'/localization/estimate',estimate,10)
n.create_subscription(Bool,'/navigation/safety_ready',ready,10)
n.create_subscription(String,'/navigation/state',status,10)
n.create_subscription(Twist,'/base/safe_cmd_vel',lambda m:outputs.append([time.monotonic(),m.linear.x,m.linear.y,m.angular.z]),10)
n.create_subscription(Odometry,'/fast_livo/navigation/wheel/odom',lambda m:wheel.append([time.monotonic(),m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z]),10)
began=time.monotonic();handle=None;result=None;goal=None;error=None;cancel=None
def spin(seconds,enabled=False):
 deadline=time.monotonic()+seconds;last=0
 while time.monotonic()<deadline:
  rclpy.spin_once(n,timeout_sec=.01)
  if time.monotonic()-last>.04:permit.publish(Bool(data=enabled));last=time.monotonic()
def until(future,seconds,enabled=False):
 deadline=time.monotonic()+seconds
 while not future.done() and time.monotonic()<deadline:spin(.03,enabled)
 return future.done()
try:
 deadline=time.monotonic()+min(a.ready_timeout,120)
 while time.monotonic()<deadline:
  spin(.1)
  now=time.monotonic()
  if loc is not None and loc.state==loc.VALID and health and now-loc_at<.25 and now-health_at<.25 and state.get('inputs_valid') and client.server_is_ready():break
 else:raise RuntimeError('fresh navigation inputs/action unavailable')
 start=poses[-1][1:4];yaw=start[2] if a.yaw is None else a.yaw
 goal=[a.x,a.y,yaw] if a.x is not None and a.y is not None else [start[0]+math.cos(start[2])*a.dx-math.sin(start[2])*a.dy,start[1]+math.sin(start[2])*a.dx+math.cos(start[2])*a.dy,yaw]
 request=NavigateToPose.Goal();request.pose.header.frame_id='map';request.pose.header.stamp=n.get_clock().now().to_msg()
 request.pose.pose.position.x,request.pose.pose.position.y=goal[:2]
 request.pose.pose.orientation.z=math.sin(yaw/2);request.pose.pose.orientation.w=math.cos(yaw/2)
 future=client.send_goal_async(request)
 if not until(future,5):raise RuntimeError('goal acceptance timeout')
 handle=future.result()
 if not handle.accepted:raise RuntimeError('goal rejected')
 goal_id=bytes(handle.goal_id.uuid).hex();print(json.dumps({'goal':goal,'goal_id':goal_id}),flush=True)
 future=handle.get_result_async();sent=time.monotonic();lastprint=sent
 while not future.done() and time.monotonic()-sent<a.timeout:
  spin(.04,True)
  if a.cancel_after and time.monotonic()-sent>a.cancel_after:break
  if time.monotonic()-lastprint>5:
   print(json.dumps({k:state.get(k) for k in ('state','reason','inputs_valid','input_reason','metrics')}),flush=True);lastprint=time.monotonic()
 if future.done():result=future.result().status
 else:
  cancel_future=handle.cancel_goal_async()
  if until(cancel_future,3):cancel={'return_code':cancel_future.result().return_code,'count':len(cancel_future.result().goals_canceling)}
  if until(future,3):result=future.result().status
finally:
 if handle is not None and result is None:
  future=handle.cancel_goal_async();until(future,2)
 spin(2,False)
 data={'name':a.name,'goal':goal,'result_status':result,'cancel':cancel,'elapsed_s':time.monotonic()-began,'events':events,'poses':poses,'outputs':outputs,'wheel':wheel,'final_state':state,'note':'Real action test; localization/wheel are estimates, no external truth.'}
 out.write_text(json.dumps(data,indent=2));print(json.dumps({k:v for k,v in data.items() if k not in ('events','poses','outputs','wheel')},indent=2))
 n.destroy_node();rclpy.shutdown()
