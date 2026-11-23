"""Real DDS/action transport in an isolated localhost domain; synthetic publishers only."""
import asyncio
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or os.environ.get('ROS_DOMAIN_ID') not in ('227','228'):
 raise RuntimeError('Use isolated ROS_DOMAIN_ID=228 ROS_LOCALHOST_ONLY=1')

class TransportTests(unittest.IsolatedAsyncioTestCase):
 async def asyncSetUp(self):
  import rclpy
  from workbench.core import Assets
  from workbench.daemon import Owner
  from workbench.ros_bridge import RosBridge
  from rclpy.action import ActionServer,GoalResponse,CancelResponse
  from rclpy.executors import MultiThreadedExecutor
  from rclpy.callback_groups import ReentrantCallbackGroup
  from nav2_msgs.action import NavigateToPose
  from std_msgs.msg import Bool,String
  from ranger_localization_msgs.msg import LocalizationEstimate
  from rosgraph_msgs.msg import Clock
  from ranger_msgs.msg import SystemState
  from nav_msgs.msg import Odometry
  from std_srvs.srv import Trigger
  self.tmp=tempfile.TemporaryDirectory();self.owner=Owner(Assets(),self.tmp.name)
  self.owner.bridge=RosBridge(self.owner.consume,self.owner.result)
  self.fake=rclpy.create_node('workbench_synthetic_robot');self.enable=[];self.goals=[];self.stopthread=False
  self.fake.create_subscription(Bool,'/ranger/motion_enable',lambda m:self.enable.append(m.data),10)
  pubs={topic:self.fake.create_publisher(kind,topic,10) for topic,kind in [('/clock',Clock),('/localization/estimate',LocalizationEstimate),('/navigation_ready',Bool),('/system_state',SystemState),('/base/execution_state',String),('/fast_livo/navigation/wheel/odom',Odometry),('/navigation/state',String)]}
  self.navstate={'state':'IDLE','generation':0,'worker':{'ready':True,'nonce':'initial'}};self.reset_calls=0
  def reset(request,response):
   self.reset_calls+=1;old=self.navstate['worker']['nonce'];self.navstate={'state':'IDLE','generation':self.reset_calls,'worker':{'ready':True,'nonce':'reset-'+str(self.reset_calls)}}
   response.success=True;response.message=json.dumps({'generation':self.reset_calls,'previous_worker_nonce':old});return response
  self.reset_service=self.fake.create_service(Trigger,'/navigation/reset',reset)
  def publish():
   ns=time.time_ns();sec,nano=divmod(ns,10**9);clock=Clock();clock.clock.sec,clock.clock.nanosec=sec,nano;pubs['/clock'].publish(clock)
   loc=LocalizationEstimate();loc.header.frame_id='map';loc.header.stamp.sec,loc.header.stamp.nanosec=sec,nano;loc.base_frame='base_link';loc.state=1;loc.pose.orientation.w=1.;loc.reason=f'confirmed; map={self.owner.state.map_uuid}; session=synthetic'
   pubs['/localization/estimate'].publish(loc);pubs['/navigation_ready'].publish(Bool(data=True));system=SystemState();system.header.stamp=loc.header.stamp;system.control_mode=1;system.vehicle_state=0;system.error_code=0;pubs['/system_state'].publish(system);pubs['/base/execution_state'].publish(String(data='{}'))
   wheel=Odometry();wheel.header.stamp=loc.header.stamp;pubs['/fast_livo/navigation/wheel/odom'].publish(wheel)
   pubs['/navigation/state'].publish(String(data=json.dumps(self.navstate)))
  self.fake.create_timer(.02,publish)
  def execute(h):
   self.goals.append(h.request);end=time.monotonic()+2
   while time.monotonic()<end and not self.stopthread:
    if h.is_cancel_requested:h.canceled();return NavigateToPose.Result()
    time.sleep(.01)
   h.succeed();return NavigateToPose.Result()
  self.action=ActionServer(self.fake,NavigateToPose,'/navigate_to_pose',execute_callback=execute,
      goal_callback=lambda _:GoalResponse.ACCEPT,cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=ReentrantCallbackGroup())
  self.executor=MultiThreadedExecutor(num_threads=3);self.executor.add_node(self.fake)
  self.thread=threading.Thread(target=self.executor.spin,daemon=True);self.thread.start()
  self.owner.session={'id':'synthetic','state':'READY'};self.tick=asyncio.create_task(self.owner.tick())
  end=time.monotonic()+5
  while not self.owner.state.snapshot().get('localization') or not self.owner.state.snapshot()['localization']['valid'] or not self.owner.bridge.action.server_is_ready() or any(self.owner.state.snapshot()['topic_receipt_age_ms'].get(t,1e9)>200 for t in ('/navigation_ready','/system_state','/base/execution_state')):
   if time.monotonic()>end:self.fail('synthetic transport not ready')
   await asyncio.sleep(.02)
 async def asyncTearDown(self):
  self.owner.alive=False;await self.tick;self.stopthread=True
  self.owner.bridge.close();self.executor.shutdown();self.thread.join(3);self.action.destroy();self.fake.destroy_service(self.reset_service);self.fake.destroy_node();self.tmp.cleanup()
 def req(self,**kw):return {'view':'live','session_id':'synthetic','map_id':self.owner.map_id,'pose':[1.,0.,0.],'request_id':'synthetic-request',**kw}
 async def test_action_lifecycle_and_idempotence(self):
  task=await self.owner.request('goal',self.req());await self.owner.request('goal',self.req())
  self.assertEqual(task['state'],'RUNNING');self.assertEqual(len(self.goals),1)
  while self.owner.task['state'] not in ('SUCCEEDED','FAILED'):await asyncio.sleep(.02)
  self.assertEqual(self.owner.task['state'],'SUCCEEDED');await asyncio.sleep(.1);self.assertFalse(self.enable[-1])
 async def test_bounded_callback_drain_keeps_observer_fresh_under_topic_load(self):
  from workbench.ros_bridge import TOPICS
  from rosidl_runtime_py.utilities import get_message
  existing={'/clock','/localization/estimate','/navigation_ready','/system_state','/base/execution_state','/fast_livo/navigation/wheel/odom','/navigation/state'}
  pubs=[]
  for topic,typ in TOPICS.items():
   if topic in existing:continue
   kind=get_message(typ);pubs.append((topic,self.fake.create_publisher(kind,topic,1),kind))
  def publish_extra():
   for topic,pub,kind in pubs:
    msg=kind()
    if kind.__name__=='String':msg.data='{"state":"IDLE"}' if topic=='/navigation/state' else '{}'
    pub.publish(msg)
  timer=self.fake.create_timer(.02,publish_extra)
  bridge=self.owner.bridge;batch=bridge.spin
  async def sample(seconds):
   values=[];end=time.monotonic()+seconds
   while time.monotonic()<end:
    snap=self.owner.state.snapshot();values.append(max(snap['topic_receipt_age_ms'].get(t,1000) for t in ('/localization/estimate','/system_state','/navigation_ready')))
    await asyncio.sleep(.01)
   return values
  try:
   bridge.spin=lambda:bridge.executor.spin_once(timeout_sec=0.)
   await asyncio.sleep(.5);old=await sample(1.)
   bridge.spin=batch;await asyncio.sleep(.4);new=await sample(1.)
   import numpy as np
   a,b=float(np.percentile(old,95)),float(np.percentile(new,95))
   print('observer_callback_load_p95_ms',{'one_per_tick':a,'bounded_batch':b,'topics':len(TOPICS)})
   self.assertLess(b,100.);self.assertLess(b,a*.65)
   self.assertEqual(self.goals,[]);self.assertNotIn(True,self.enable)
  finally:
   bridge.spin=batch;self.fake.destroy_timer(timer)
 async def test_cancel_uses_real_action_protocol(self):
  task=await self.owner.request('goal',self.req())
  await self.owner.request('cancel',self.req(task_id=task['id'],request_id='synthetic-cancel'))
  end=time.monotonic()+3
  while self.owner.task['state']=='CANCELING' and time.monotonic()<end:await asyncio.sleep(.02)
  self.assertEqual(self.owner.task['state'],'CANCELED');await asyncio.sleep(.1);self.assertFalse(self.enable[-1])
 async def test_failed_goal_reset_then_new_goal_without_restarting_device_session(self):
  self.owner.task={'id':'old-terminal','state':'FAILED'}
  response=await self.owner.request('navigation.reset',self.req(request_id='reset-after-failure'))
  self.assertEqual(response['state'],'RESETTING');await self.owner.reset_job
  self.assertEqual(self.owner.navigation_reset['state'],'READY');self.assertIsNone(self.owner.task)
  self.assertEqual(self.reset_calls,1);self.assertEqual(self.goals,[]);self.assertNotIn(True,self.enable)
  task=await self.owner.request('goal',self.req(request_id='new-goal-after-reset'))
  self.assertEqual(task['state'],'RUNNING')
  await self.owner.request('cancel',self.req(task_id=task['id'],request_id='end-test-goal'))
 async def test_active_goal_is_canceled_before_ros_reset_service(self):
  await self.owner.request('goal',self.req(request_id='goal-before-reset'))
  await self.owner.request('navigation.reset',self.req(request_id='reset-active-goal'));await self.owner.reset_job
  self.assertEqual(self.owner.navigation_reset['state'],'READY');self.assertIsNone(self.owner.task)
  self.assertEqual(self.reset_calls,1);self.assertEqual(len(self.goals),1);self.assertFalse(self.enable[-1])
 async def test_independent_watchdog_revokes_dead_owner(self):
  from workbench.core import atomic
  path=Path(self.tmp.name)/'dead.json';atomic(path,{'active':True,'identity':None,'steady':time.monotonic()})
  p=subprocess.Popen([sys.executable,'-m','workbench.watchdog','--heartbeat',str(path)],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
  try:
   start=len(self.enable);end=time.monotonic()+4
   while len(self.enable)==start and time.monotonic()<end:await asyncio.sleep(.03)
   self.assertGreater(len(self.enable),start);self.assertFalse(self.enable[-1]);self.assertNotIn(True,self.enable[start:])
  finally:p.send_signal(2);await asyncio.to_thread(p.wait,5)

 async def test_http_disconnect_keeps_single_robot_goal(self):
  from aiohttp.test_utils import TestClient,TestServer
  from workbench.server import application
  socket=Path(self.tmp.name)/'test.sock'
  async def handle(reader,writer):
   req=json.loads(await reader.readline())
   try:result={'ok':True,'result':await self.owner.request(req['method'],req.get('data',{}))}
   except Exception as exc:result={'ok':False,'error':str(exc)}
   writer.write((json.dumps(result)+'\n').encode());await writer.drain();writer.close();await writer.wait_closed()
  ipc=await asyncio.start_unix_server(handle,path=socket)
  app=application(socket,Path(self.tmp.name)/'web');client=TestClient(TestServer(app));await client.start_server()
  origin=str(client.make_url('/')).rstrip('/')
  try:
   await client.post('/api/v1/login',json={'key':app['api'].key},headers={'Origin':origin})
   request=self.req(request_id='http-disconnect-goal')
   # This test embeds HTTP and ROS in one loop; production uses separate processes.
   # Asset/login setup can briefly age the synthetic inputs. Wait for a fresh set.
   end=time.monotonic()+3
   while not self.owner.state.snapshot()['localization']['valid'] or any(self.owner.state.snapshot()['topic_receipt_age_ms'].get(t,1e9)>100 for t in ('/system_state','/navigation_ready','/base/execution_state')):
    if time.monotonic()>end:self.fail('fresh synthetic inputs did not recover after HTTP setup')
    await asyncio.sleep(.01)
   response=await client.post('/api/v1/goals',json=request,headers={'Origin':origin});self.assertEqual(response.status,200,await response.text())
   again=await client.post('/api/v1/goals',json=request,headers={'Origin':origin});self.assertEqual(again.status,200)
   ws=await client.ws_connect('/api/v1/events',headers={'Origin':origin});await ws.receive_json();await ws.close()
   await client.close()
   end=time.monotonic()+4
   while self.owner.task['state']=='RUNNING' and time.monotonic()<end:await asyncio.sleep(.02)
   self.assertEqual(self.owner.task['state'],'SUCCEEDED');self.assertEqual(len(self.goals),1)
  finally:
   await client.close();ipc.close();await ipc.wait_closed()

if __name__=='__main__':unittest.main(verbosity=2)
