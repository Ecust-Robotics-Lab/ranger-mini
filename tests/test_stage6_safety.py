#!/usr/bin/env python3
"""Isolated ROS full-chain/action fault test; mock feedback only, no CAN/driver.

Run after sourcing ws_base/install_exact and ws_nav/install_exact. Only processes
created by this test are stopped. No global process-name kill is used.
"""
import argparse,json,math,os,signal,subprocess,sys,tempfile,time
import yaml
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
os.environ.update(ROS_DOMAIN_ID='231',ROS_LOCALHOST_ONLY='1')
sys.path.insert(0,str(ROOT/'ws_nav/src/ranger_navigation'))
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped,PoseStamped,Twist
from nav_msgs.msg import Odometry,OccupancyGrid
from nav2_msgs.action import NavigateToPose
from ranger_localization_msgs.msg import LocalizationEstimate
from ranger_msgs.msg import SystemState,MotionState
from std_msgs.msg import Bool,String,UInt8
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from diagnostic_msgs.msg import DiagnosticArray
from tf2_ros import TransformBroadcaster
from ranger_navigation.geometry import load_geometry,integrate
from ranger_navigation.ros_utils import twist_array,twist_msg


class World(Node):
 def __init__(self):
  super().__init__('exact_navigation_test_world')
  self.pose=np.zeros(3);self.velocity=np.zeros(3);self.actual_mode=0;self.request=None;self.request_at=None
  self.stamps={};self.commands=[];self.upstream=[];self.state={};self.ready=True
  self.cloud_on=self.wheel_on=self.mode_on=self.system_on=self.tf_on=True
  self.can=True;self.soft=False;self.enabled=False;self.teleop=False
  self.reject_switch=False;self.obstacles_world=np.empty((0,2));self.metrics=[];self.metrics_seen=set()
  self.collided=False
  self.gate_reason='startup';self.trace=[]
  self.clock=0;self.step=0
  self.body,self.r,self.t=load_geometry(ROOT/'config/fast_livo2/navigation_prep.yaml')
  self.wheel=self.create_publisher(Odometry,'/fast_livo/navigation/wheel/odom',10)
  self.fast=self.create_publisher(Odometry,'/fast_livo/odometry',10)
  self.cloud=self.create_publisher(__import__('sensor_msgs.msg',fromlist=['PointCloud2']).PointCloud2,'/fast_livo/deskewed_imu',2)
  self.system=self.create_publisher(SystemState,'/system_state',10)
  self.mode=self.create_publisher(MotionState,'/motion_state',10)
  self.est=self.create_publisher(LocalizationEstimate,'/localization/estimate',10)
  self.readypub=self.create_publisher(Bool,'/navigation_ready',10)
  self.permission=self.create_publisher(Bool,'/ranger/motion_enable',1)
  self.estop=self.create_publisher(Bool,'/ranger/soft_estop',1)
  self.manual=self.create_publisher(Twist,'/teleop/cmd_vel',1)
  self.manual_mode=self.create_publisher(UInt8,'/teleop/motion_mode',1)
  self.tf=TransformBroadcaster(self)
  self.create_subscription(Twist,'/base/safe_cmd_vel',self.on_cmd,10)
  self.create_subscription(Twist,'/cmd_vel_nav',lambda m:self.upstream.append((time.monotonic(),twist_array(m))),10)
  self.create_subscription(UInt8,'/base/motion_mode_request',self.on_mode,10)
  self.create_subscription(String,'/navigation/state',self.on_state,10)
  self.create_subscription(DiagnosticArray,'/diagnostics',self.on_diagnostic,10)
  qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
  self.map=self.create_publisher(OccupancyGrid,'/map',qos)
  m=OccupancyGrid();m.header.frame_id='map';m.info.width=m.info.height=200;m.info.resolution=.05
  m.info.origin.position.x=m.info.origin.position.y=-5.;m.info.origin.orientation.w=1.;m.data=[0]*40000
  self.map.publish(m)
  self.client=ActionClient(self,NavigateToPose,'navigate_to_pose')
  self.create_timer(.02,self.tick)
 def on_cmd(self,m):
  self.velocity=twist_array(m)
  self.commands.append((time.monotonic(),self.velocity.copy()))
 def on_state(self,m):
  self.state=json.loads(m.data)
  self.trace.append({'time':time.monotonic(),'state':self.state,'pose':self.pose.tolist(),'gate':self.gate_reason,'velocity':self.velocity.tolist()})
  metric=self.state.get('metrics',{})
  if metric.get('valid') and 'end_to_end_s' in metric and metric.get('request_id') not in self.metrics_seen:
   self.metrics.append(metric['end_to_end_s']*1000);self.metrics_seen.add(metric.get('request_id'))
 def on_diagnostic(self,m):
  for row in m.status:
   if row.name=='ranger_motion_gate':self.gate_reason=row.message
 def on_mode(self,m):
  if self.request!=m.data:self.request,self.request_at=m.data,time.monotonic()
 def tick(self):
  self.step+=1;stamp=self.get_clock().now().to_msg()
  if self.request is not None and not self.reject_switch and time.monotonic()-self.request_at>.15 and np.linalg.norm(self.velocity)<.02:self.actual_mode=self.request
  self.pose=integrate(self.pose,self.velocity,.02)
  if len(self.obstacles_world):
   c,s=math.cos(self.pose[2]),math.sin(self.pose[2])
   local=(self.obstacles_world-self.pose[:2])@np.array([[c,-s],[s,c]])
   self.collided |= bool(np.any((np.abs(local[:,0])<self.body.length/2)&(np.abs(local[:,1])<self.body.width/2)))
  wheel=Odometry();wheel.header.frame_id='odom';wheel.child_frame_id='base_link';wheel.header.stamp=stamp
  wheel.pose.pose.position.x,wheel.pose.pose.position.y=map(float,self.pose[:2])
  wheel.pose.pose.orientation.z,wheel.pose.pose.orientation.w=math.sin(self.pose[2]/2),math.cos(self.pose[2]/2)
  wheel.twist.twist=twist_msg(self.velocity)
  fast=Odometry();fast.header.frame_id='fast_livo_local';fast.child_frame_id='fast_livo_imu';fast.header.stamp=stamp;fast.pose.pose.orientation.w=1.
  self.fast.publish(fast)
  if self.wheel_on:self.wheel.publish(wheel)
  if self.tf_on:
   tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='odom';tf.header.stamp=stamp;tf.transform.rotation.w=1.;self.tf.sendTransform(tf)
  if self.cloud_on and self.step%5==0:
   a=np.linspace(-np.pi,np.pi,1800,endpoint=False);base=np.column_stack((3*np.cos(a),3*np.sin(a),np.full(len(a),.4)))
   if len(self.obstacles_world):
    c,s=math.cos(self.pose[2]),math.sin(self.pose[2]);xy=(self.obstacles_world-self.pose[:2])@np.array([[c,-s],[s,c]])
    base=np.vstack((base,np.column_stack((xy,np.full(len(xy),.4)))))
   imu=(base-self.t)@self.r
   cloud=point_cloud2.create_cloud_xyz32(Header(stamp=stamp,frame_id='fast_livo_imu'),imu.tolist());self.cloud.publish(cloud)
  if self.system_on:
   m=SystemState();m.header.stamp=stamp;m.control_mode=1 if self.can else 0;m.vehicle_state=0;self.system.publish(m)
  if self.mode_on:
   m=MotionState();m.header.stamp=stamp;m.motion_mode=self.actual_mode;self.mode.publish(m)
  estimate=LocalizationEstimate();estimate.header.frame_id='map';estimate.base_frame='base_link';estimate.header.stamp=stamp
  estimate.pose=wheel.pose.pose;estimate.state=estimate.VALID;estimate.reason='session=synthetic; map=synthetic'
  self.est.publish(estimate);self.readypub.publish(Bool(data=self.ready))
  if self.teleop:
   self.manual_mode.publish(UInt8(data=1));self.manual.publish(Twist())
 def spin(self,seconds):
  end=time.monotonic()+seconds
  while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.01)
 def wait(self,predicate,timeout=15):
  end=time.monotonic()+timeout
  while time.monotonic()<end:
   rclpy.spin_once(self,timeout_sec=.01)
   if predicate():return
  raise AssertionError('timeout; state='+json.dumps(self.state))
 def arm(self,value):
  self.permission.publish(Bool(data=value));self.spin(.12)
 def goal(self,x,y,yaw=0.):
  g=NavigateToPose.Goal();g.pose.header.frame_id='map';g.pose.pose.position.x=float(x);g.pose.pose.position.y=float(y)
  g.pose.pose.orientation.z,g.pose.pose.orientation.w=math.sin(yaw/2),math.cos(yaw/2)
  f=self.client.send_goal_async(g);self.wait(f.done);h=f.result();assert h.accepted;return h
 def stopped_after(self,started,delay=.30):
  rows=[v for t,v in self.commands if t>=started+delay]
  assert len(rows)>=3 and max(np.max(np.abs(v)) for v in rows)<1e-8


def main():
 parser=argparse.ArgumentParser(description=__doc__)
 parser.add_argument('--evidence-directory',type=Path)
 parser.add_argument('--clearance-profile',choices=('standard','narrow'),default='standard')
 options=parser.parse_args()
 evidence=options.evidence_directory or Path(tempfile.mkdtemp(prefix='exact-ros-evidence-',dir=ROOT/'.runtime'))
 evidence.mkdir(parents=True,exist_ok=True)
 if (evidence/'ros_safety.json').exists():raise RuntimeError('preserve existing evidence; choose a new directory')
 profile=yaml.safe_load((ROOT/'ws_nav/src/ranger_navigation/config/exact_navigation.yaml').read_text())['clearance_profiles'][options.clearance_profile]
 folder=Path(tempfile.mkdtemp(prefix='exact-ros-',dir=ROOT/'.runtime'))
 args=['--ros-args','-p','geometry_config:='+str(ROOT/'config/fast_livo2/navigation_prep.yaml'),
       '-p','clearance:='+str(profile['clearance'])]
 prefix=ROOT/'ws_nav/install_exact/ranger_navigation/lib/ranger_navigation'
 processes=[];logs=[];results=[]
 def check(name,function):
  function();results.append({'name':name,'pass':True});print('PASS',name,flush=True)
 try:
  parameters=[part for key in ('static_map_reserve','visibility_reserve','segment_yaw_tolerance','prefer_parallel_in_narrow')
              for part in ('-p',key+':='+str(profile[key]).lower())]
  for name,extra in [('motion_gate',[]),('exact_navigation',['-p','optimizer_python:='+str(ROOT/'.runtime/exact_mppi_venv/bin/python3'),'-p','session_root:='+str(folder),*parameters])]:
   log=(folder/(name+'.log')).open('w');logs.append(log)
   processes.append(subprocess.Popen([str(prefix/name),*args,*extra],stdout=log,stderr=subprocess.STDOUT))
  rclpy.init();w=World();w.wait(lambda:w.state.get('worker',{}).get('ready',False) and w.state.get('inputs_valid'),30.)
  w.wait(lambda:w.client.server_is_ready())
  h=w.goal(1.,0);w.spin(1.)
  check('startup_permission_off',lambda:w.stopped_after(w.commands[0][0]))
  w.arm(True);w.wait(lambda:any(np.linalg.norm(v)>.003 for t,v in w.commands[-5:]),10.)
  check('valid_trajectory_reaches_final_output',lambda:None)
  f=h.cancel_goal_async();w.wait(f.done);t=time.monotonic();w.spin(.6)
  check('action_cancel_stops',lambda:w.stopped_after(t,.15))
  old=w.goal(w.pose[0]+1.,w.pose[1]);old_result=old.get_result_async()
  new=w.goal(w.pose[0],w.pose[1]+.5);w.wait(old_result.done)
  check('goal_replacement_terminates_old_goal',lambda:None)
  new_result=new.get_result_async();w.wait(new_result.done,25.)
  assert new_result.result().status==4
  check('lateral_goal_stops_and_succeeds',lambda:None)
  h=w.goal(w.pose[0]+2.,w.pose[1]);w.spin(1.)
  x,y=w.pose[0]+.8,w.pose[1]
  a=np.linspace(-.15,.15,20);w.obstacles_world=np.vstack((np.column_stack((np.full(len(a),x),a+y)),np.column_stack((np.full(len(a),x+.12),a+y))))
  # Full stops and replanning at steering-axis changes are part of the plant.
  # Allow their wall time; production's continuous blocked timeout stays 15s.
  f=h.get_result_async();w.wait(f.done,180.)
  assert f.result().status==4,json.dumps({'state':w.state,'pose':w.pose.tolist(),'action_status':f.result().status})
  assert not w.collided,'simulated body intersected the inserted obstacle'
  check('dynamic_blockage_stops_replans_and_goes_around',lambda:None)
  w.obstacles_world=np.empty((0,2));w.spin(.3)
  w.reject_switch=True;h=w.goal(w.pose[0],w.pose[1],w.pose[2]+1.57);f=h.get_result_async();w.wait(f.done,8.)
  assert f.result().status==6
  check('fresh_but_unconfirmed_mode_times_out',lambda:None);w.reject_switch=False;w.spin(.2)
  h=w.goal(10.,0.);f=h.get_result_async();w.wait(f.done,18.)
  assert f.result().status==6
  check('unreachable_goal_reports_blocked_failure',lambda:None)
  for field,name in [('cloud_on','cloud_loss'),('wheel_on','wheel_loss'),('mode_on','mode_loss'),('system_on','system_loss'),('tf_on','tf_loss')]:
   h=w.goal(w.pose[0]+1.,w.pose[1]);w.spin(.8);setattr(w,field,False);t=time.monotonic();w.spin(.7)
   check(name,lambda:w.stopped_after(t))
   setattr(w,field,True);f=h.cancel_goal_async();w.wait(f.done);w.spin(.3)
  h=w.goal(w.pose[0]+1.,w.pose[1]);w.spin(.8);w.ready=False;t=time.monotonic();w.spin(.5)
  check('localization_withdrawal',lambda:w.stopped_after(t,.15));w.ready=True
  w.soft=True;w.estop.publish(Bool(data=True));t=time.monotonic();w.spin(.5)
  check('soft_estop',lambda:w.stopped_after(t,.1));w.estop.publish(Bool(data=False));w.spin(.3)
  w.teleop=True;t=time.monotonic();w.spin(.5)
  check('teleop_owns_velocity_and_mode',lambda:w.stopped_after(t,.15));w.teleop=False;w.spin(.4)
  w.can=False;t=time.monotonic();w.spin(.5)
  check('rc_takeover',lambda:w.stopped_after(t,.15));w.can=True;w.spin(.5)
  w.wait(lambda:w.state.get('worker',{}).get('pid') is not None)
  pid=w.state['worker']['pid'];cmd=Path('/proc')/str(pid)/'cmdline'
  assert str(folder).encode() in cmd.read_bytes()
  os.kill(pid,signal.SIGSTOP);t=time.monotonic();w.spin(.6)
  check('blocked_worker_independent_stop',lambda:w.stopped_after(t,.15));os.kill(pid,signal.SIGCONT)
  os.kill(pid,signal.SIGKILL);t=time.monotonic();w.spin(.6)
  check('worker_exit_stop',lambda:w.stopped_after(t,.15))
  timing={k:float(np.percentile(w.metrics,p)) for k,p in [('p50',50),('p95',95),('p99',99)]} if w.metrics else {}
  (evidence/'ros_safety.json').write_text(json.dumps({'pass':True,'clearance_profile':options.clearance_profile,'profile':profile,'results':results,'control_end_to_end_ms':timing,'logs':str(folder),'hardware_started':False},indent=2)+'\n')
  return 0
 except Exception as e:
  if 'w' in locals():(folder/'trace.json').write_text(json.dumps(w.trace))
  (evidence/'ros_safety.json').write_text(json.dumps({'pass':False,'results':results,'error':str(e),'logs':str(folder)},indent=2)+'\n')
  raise
 finally:
  if 'w' in locals():w.destroy_node()
  if rclpy.ok():rclpy.shutdown()
  for process in processes:
   if process.poll() is None:process.terminate()
  for process in processes:
   try:process.wait(timeout=5)
   except subprocess.TimeoutExpired:process.kill();process.wait(timeout=5)
  for log in logs:log.close()
if __name__=='__main__':raise SystemExit(main())
