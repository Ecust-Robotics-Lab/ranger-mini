"""Actual coordinator + GPU reset service, isolated ROS; no device nodes."""
import json,os,signal,subprocess,time,sys
from pathlib import Path
if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or os.environ.get('ROS_DOMAIN_ID')!='227':
 raise RuntimeError('Requires ROS_DOMAIN_ID=227 ROS_LOCALHOST_ONLY=1')
import rclpy
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from ranger_navigation_msgs.msg import LeasedControlCommand
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from navigation_health import navigation_compute_available
from session_manager import read_session,atomic_json
managed='--managed' in sys.argv or '--managed-final' in sys.argv
out=ROOT/'docs/workbench_evidence/20261117/navigation_reset'/('managed_ros_final' if '--managed-final' in sys.argv else 'managed_ros' if managed else 'actual_ros');out.mkdir(exist_ok=True)
rclpy.init();node=rclpy.create_node('synthetic_navigation_reset_check')
clock=node.create_publisher(Clock,'/clock',10);states=[];controls=[];velocities=[];reset_health=[]
def state_message(message):
 s=json.loads(message.data);states.append(s)
 if s['state']=='RESETTING':reset_health.append(navigation_compute_available(s,time.monotonic()))
 if managed:
  run,_=read_session(out/'session')
  if run:atomic_json(run/'health.json',{'monotonic':time.monotonic(),'checks':{'navigation':navigation_compute_available(s,time.monotonic())}})
node.create_subscription(String,'/navigation/state',state_message,20)
node.create_subscription(LeasedControlCommand,'/navigation/control',controls.append,20)
node.create_subscription(Twist,'/cmd_vel_nav',velocities.append,20)
client=node.create_client(Trigger,'/navigation/reset');action=ActionClient(node,NavigateToPose,'/navigate_to_pose')
log=(out/'coordinator.log').open('w')
command=[str(ROOT/'ws_nav/install_exact/ranger_navigation/lib/ranger_navigation/exact_navigation'),
 '--ros-args','-p','use_sim_time:=true','-p','geometry_config:='+str(ROOT/'config/fast_livo2/navigation_prep.yaml'),
 '-p','optimizer_python:='+str(ROOT/'.runtime/exact_mppi_venv/bin/python'),'-p','samples:=32',
 '-p','static_map_clearance:=0.01','-p','session_root:='+str(out/'compute')]
if managed:
 (out/'command.json').write_text(json.dumps(command)+'\n')
 command=[sys.executable,str(ROOT/'tests/managed_navigation_reset_fixture.py'),str(out/'session'),str(out/'command.json')]
process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
def pump():
 c=Clock();c.clock.sec=100;clock.publish(c);rclpy.spin_once(node,timeout_sec=.01)
def wait(predicate,timeout=35):
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  pump()
  if predicate():return
  assert process.poll() is None,'coordinator exited: '+str(out/'coordinator.log')
 raise AssertionError('timeout: '+str(states[-1:] or 'no states'))
def goal():
 g=NavigateToPose.Goal();g.pose.header.frame_id='map';g.pose.pose.position.x=.3;g.pose.pose.orientation.w=1.
 f=action.send_goal_async(g);wait(f.done);return f.result()
try:
 wait(lambda:states and states[-1]['worker'].get('ready') and client.service_is_ready() and action.server_is_ready())
 if managed:wait(lambda:(read_session(out/'session')[1] or {}).get('state')=='RUNNING')
 coordinator_pid=read_session(out/'session')[1]['components']['navigation']['process']['pid'] if managed else process.pid
 first=states[-1]['worker'];f=client.call_async(Trigger.Request());wait(f.done);assert f.result().success
 assert not goal().accepted,'new goal accepted during reset'
 wait(lambda:states[-1]['state']=='IDLE' and states[-1]['worker'].get('ready') and states[-1]['worker']['nonce']!=first['nonce'])
 second=states[-1]['worker'];assert second['pid']!=first['pid'];assert any(s['state']=='RESETTING' for s in states)
 assert reset_health and all(reset_health),'managed probe would withdraw health during bounded reset'
 if managed:
  _,managed_state=read_session(out/'session');assert managed_state['state']=='RUNNING',managed_state
  assert managed_state['components']['navigation']['process']['pid']==coordinator_pid
 # New goals work after reset, but lack of synthetic sensor inputs still holds zero.
 handle=goal();assert handle.accepted
 f=client.call_async(Trigger.Request());wait(f.done);assert not f.result().success
 f=handle.cancel_goal_async();wait(f.done);result=handle.get_result_async();wait(result.done);assert result.result().status==5
 assert controls and all(not c.valid for c in controls)
 assert velocities and all(abs(v.linear.x)+abs(v.linear.y)+abs(v.angular.z)==0 for v in velocities)
 for topic in ['/wheel/odom','/livox/lidar','/system_state']:
  assert not node.get_publishers_info_by_topic(topic),topic
 (out/'result.json').write_text(json.dumps({'pass':True,'hardware_started':False,'coordinator_pid_unchanged':coordinator_pid,
  'old_worker':first,'new_worker':second,'commands_all_invalid_and_zero':True,'bounded_managed_reset_health':True,
  'actual_guardian_with_one_second_loss_grace':managed,
  'checks':['actual_gpu_restart','reset_phase_visible','reject_goal_during_reset','accept_goal_after_reset',
            'refuse_reset_with_active_goal','cancel_while_inputs_invalid','no_device_publishers']},indent=2)+'\n')
 print('Actual coordinator/GPU/ROS reset PASS; no device nodes')
finally:
 if process.poll() is None:
  process.send_signal(signal.SIGINT)
  try:process.wait(timeout=8)
  except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=3)
 log.close();node.destroy_node();rclpy.shutdown()
