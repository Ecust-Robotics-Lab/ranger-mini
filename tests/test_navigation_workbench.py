"""Offline contract/security/lifecycle checks: no CAN, sensors, or real goals."""
import asyncio
import copy
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch
import sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from workbench.core import Assets,Journal,Presets,Replay,State,pose
from workbench.daemon import Owner,localization_progress

UUID='map-a'
def loc(n=1_000_000_000,state=1,session='a',map_uuid=UUID):
 return {'header':{'frame_id':'map','stamp':{'sec':n//10**9,'nanosec':n%10**9}},'base_frame':'base_link',
         'state':state,'reason':f'confirmed; map={map_uuid}; session={session}; graph=1',
         'pose':{'position':{'x':1.,'y':2.,'z':0.},'orientation':{'x':0.,'y':0.,'z':0.,'w':1.}}}

def clock(n):return {'clock':{'sec':n//10**9,'nanosec':n%10**9}}
class CoreTests(unittest.TestCase):
 def test_localization_phase_reports_matching_and_confirmation_requirements(self):
  text=localization_progress('searching_geometric_registration',{'query_window_frames':8,'query_window_required_frames':8})
  self.assertIn('正在检索和几何匹配',text);self.assertIn('8/8',text);self.assertNotIn('7/8',text)
  text=localization_progress('verifying_stationary_support_restarted',{'support_queries':1,'support_elapsed_s':0.,'support_anchor_strong':True})
  self.assertIn('丢弃较弱初始候选',text);self.assertIn('确认 1 组',text);self.assertIn('需至少 3 组',text)
  text=localization_progress('verifying_wait_strong_stationary_geometry',{'best_rmse_m':.13,'stationary_max_rmse_m':.12})
  self.assertIn('0.130m',text);self.assertIn('0.120m',text)
 def test_stale_sample_heartbeat_does_not_refresh(self):
  s=State('a',UUID);s.consume('/clock',clock(1_000_000_000),10**9);s.consume('/localization/estimate',loc(),10**9)
  self.assertTrue(s.snapshot(10**9)['localization']['valid'])
  s.consume('/clock',clock(1_401_000_000),1_401_000_000);s.consume('/localization/estimate',loc(),1_401_000_000)
  self.assertFalse(s.snapshot(1_401_000_000)['localization']['valid']);self.assertEqual(len(s.trace),1)
 def test_400ms_pose_bound_keeps_receipt_future_and_backend_gates(self):
  for age,expected in [(250_000_000,True),(350_000_000,True),(400_000_000,True),(400_000_001,False),(-1,False)]:
   s=State('a',UUID);s.consume('/clock',clock(10**9+age),2*10**9);s.consume('/localization/estimate',loc(),2*10**9)
   self.assertEqual(s.snapshot(2*10**9)['localization']['valid'],expected)
  s=State('a',UUID);s.consume('/clock',clock(1_300_000_000),2*10**9);s.consume('/localization/estimate',loc(),2*10**9)
  self.assertFalse(s.snapshot(2_251_000_000)['localization']['valid'])
  s.consume('/localization/estimate',loc(state=2),2_251_000_000)
  self.assertFalse(s.snapshot(2_251_000_000)['localization']['valid'])
 def test_receipt_expiry(self):
  s=State('a',UUID);s.consume('/clock',clock(10**9),10**9);s.consume('/localization/estimate',loc(),10**9)
  self.assertFalse(s.snapshot(1_251_000_000)['localization']['valid'])
 def test_identity_and_validity_break_path(self):
  s=State('a',UUID)
  for i,(state,session) in enumerate([(1,'a'),(2,'a'),(1,'a'),(1,'b')]):
   s.consume('/localization/estimate',loc(10**9+i*10**8,state,session))
  self.assertEqual(s.trace.count(None),2)
 def test_wrong_map_and_wrong_base_never_valid(self):
  for changed in [dict(base_frame='imu_link'),dict(reason='map=other; session=a')]:
   s=State('a',UUID);s.consume('/clock',clock(10**9));s.consume('/localization/estimate',{**loc(),**changed})
   self.assertFalse(s.snapshot()['localization']['valid']);self.assertEqual(s.trace,[])
 def test_time_rollback_breaks_path(self):
  s=State('a',UUID);s.consume('/localization/estimate',loc(2*10**9));s.consume('/clock',clock(2*10**9));s.consume('/clock',clock(10**9));s.consume('/localization/estimate',loc())
  self.assertIn(None,s.trace)
 def test_finite_goal(self):
  for p in [[1,2],[1,float('nan'),0],[1,2,float('inf')]]:
   with self.assertRaises(ValueError):pose(p)
 def test_journal_seek_order_and_finalization(self):
  with tempfile.TemporaryDirectory() as tmp:
   j=Journal(Path(tmp)/'record',{'map_uuid':UUID},min_free_bytes=0)
   j.add('snapshot',{'value':1},10**9);j.add('event',{'kind':'goal'},1_500_000_000);j.add('snapshot',{'value':2},2*10**9);j.close()
   replay=Replay(j.folder)
   self.assertEqual(replay.seek(.4)['snapshot']['value'],1)
   self.assertEqual(replay.seek(.6)['events'][0]['kind'],'goal')
   self.assertEqual(replay.seek(100)['snapshot']['value'],2)
   self.assertTrue(json.loads((j.folder/'result.json').read_text())['finalized'])
 def test_recording_fault_does_not_block(self):
  with tempfile.TemporaryDirectory() as tmp:
   j=Journal(Path(tmp)/'record',{},min_free_bytes=10**30);j.add('snapshot',{},10**9);j.thread.join(2)
   self.assertIn('disk_low',j.error);j.add('snapshot',{});j.close()
 def test_exact_map_and_single_preset(self):
  assets=Assets();m=next(iter(assets.maps.values()))
  with tempfile.TemporaryDirectory() as tmp:
   p=Presets(Path(tmp)/'presets.json',assets)
   r=p.import_data({'frame':'map','map_uuid':m['map_uuid'],'source_manifest_sha256':m['source_manifest_sha256'],'presets':[{'name':'one','pose':[0,0,0]}]})
   self.assertEqual(len(r),1)
   with self.assertRaises(ValueError):p.import_data({'frame':'map','map_uuid':'other','presets':[{'name':'one','pose':[0,0,0]}]})

class FakeBridge:
 def __init__(self):self.calls=[];self.owner=None
 def arm(self,v):self.calls.append(('arm',v))
 async def goal(self,id,target):self.calls.append(('goal',id,target))
 async def cancel(self,id):self.calls.append(('cancel',id))
class OwnerTests(unittest.IsolatedAsyncioTestCase):
 async def test_failed_task_keeps_planner_cause_instead_of_only_stall(self):
  with tempfile.TemporaryDirectory() as tmp:
   owner=Owner(Assets(),tmp,bridge=FakeBridge());owner.task={'id':'current','state':'RUNNING'}
   owner.state.latest['/navigation/state']={'state':'FAILED','reason':'no_progress_timeout','failure_cause':'goal_live_obstacle_collision','planning_failure':{'reason':'goal_live_obstacle_collision','details':{'goal':{'dynamic_free':False}}}}
   owner.result('current','FAILED')
   self.assertEqual(owner.task['reason'],'goal_live_obstacle_collision');self.assertEqual(owner.task['terminal_reason'],'no_progress_timeout')
   self.assertEqual(owner.bridge.calls,[('arm',False)])
   owner.result('retired','FAILED');self.assertEqual(owner.task['reason'],'goal_live_obstacle_collision')
 async def asyncSetUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.assets=Assets();self.bridge=FakeBridge();self.owner=Owner(self.assets,self.tmp.name,bridge=self.bridge)
  o=self.owner;o.session={'id':'session','state':'READY'}
  n=time.time_ns();o.state.consume('/clock',clock(10**9),n);o.state.consume('/localization/estimate',loc(map_uuid=o.state.map_uuid),n)
  for t in ('/navigation_ready','/base/execution_state'):o.state.consume(t,{'data':True},n)
  o.state.consume('/system_state',{'header':{'stamp':{'sec':n//10**9,'nanosec':n%10**9}},'control_mode':1,'vehicle_state':0,'error_code':0},n)
 async def asyncTearDown(self):self.tmp.cleanup()
 def req(self,**kw):return {'view':'live','session_id':'session','map_id':self.owner.map_id,'pose':[1,2,0],'request_id':'abcdefgh123',**kw}
 async def test_idempotent_goal_no_repeat(self):
  a=await self.owner.request('goal',self.req());b=await self.owner.request('goal',self.req())
  self.assertEqual(a,b);self.assertEqual(sum(c[0]=='goal' for c in self.bridge.calls),1)
 async def test_conflicting_idempotency_key_rejected(self):
  await self.owner.request('goal',self.req())
  with self.assertRaises(ValueError):await self.owner.request('goal',self.req(pose=[2,3,0]))
 async def test_replay_is_readonly(self):
  with self.assertRaises(ValueError):await self.owner.request('goal',self.req(view='replay'))
  self.assertEqual(self.bridge.calls,[])
 async def test_old_session_cannot_submit(self):
  with self.assertRaises(ValueError):await self.owner.request('goal',self.req(session_id='old'))
 async def test_cancel_then_feedback_not_assumed(self):
  task=await self.owner.request('goal',self.req())
  out=await self.owner.request('cancel',self.req(task_id=task['id'],request_id='cancel-key'))
  self.assertEqual(out['state'],'CANCELING');self.assertIn(('arm',False),self.bridge.calls)
 async def test_replacement_late_old_result_cannot_disable_new(self):
  a=await self.owner.request('goal',self.req());b=await self.owner.request('goal',self.req(request_id='new-goal-key',replace=True))
  count=len(self.bridge.calls);self.owner.result(a['id'],'CANCELED');self.assertEqual(len(self.bridge.calls),count)
  self.assertEqual(self.owner.task['id'],b['id'])
 async def test_result_disables_and_browser_reads_have_no_effect(self):
  a=await self.owner.request('goal',self.req());count=len(self.bridge.calls)
  for _ in range(3):await self.owner.request('snapshot',{})
  self.assertEqual(len(self.bridge.calls),count);self.owner.result(a['id'],'SUCCEEDED');self.assertEqual(self.bridge.calls[-1],('arm',False))
 async def test_stale_pose_cannot_execute(self):
  self.owner.state.clock_ns=2*10**9
  with self.assertRaises(ValueError):await self.owner.request('goal',self.req())
 async def test_demo_cannot_start_hardware(self):
  self.owner.demo=True
  with self.assertRaises(ValueError):await self.owner.request('session.start',self.req())
 async def test_start_does_not_take_foreign_session(self):
  o=self.owner;o.session={'id':'session','state':'STARTING'}
  with patch('workbench.daemon.session_live',return_value=True):await o.start()
  self.assertEqual(o.session['state'],'FAILED');self.assertEqual(o.children,{})

class HTTPTests(unittest.IsolatedAsyncioTestCase):
 async def asyncSetUp(self):
  from aiohttp.test_utils import TestClient,TestServer
  from workbench.server import application
  self.tmp=tempfile.TemporaryDirectory();self.app=application('/no-socket',self.tmp.name)
  self.client=TestClient(TestServer(self.app));await self.client.start_server();self.origin=str(self.client.make_url('/')).rstrip('/')
 async def asyncTearDown(self):await self.client.close();self.tmp.cleanup()
 async def login(self):
  r=await self.client.post('/api/v1/login',json={'key':self.app['api'].key},headers={'Origin':self.origin});self.assertEqual(r.status,200)
 async def test_auth_and_origin(self):
  self.assertEqual((await self.client.get('/api/v1/maps')).status,401)
  self.assertEqual((await self.client.post('/api/v1/login',json={'key':self.app['api'].key},headers={'Origin':'http://evil'})).status,403)
  await self.login();self.assertEqual((await self.client.get('/api/v1/maps')).status,200)
  self.assertEqual((await self.client.post('/api/v1/goals',json={})).status,403)
 async def test_no_arbitrary_command_endpoint(self):
  await self.login();self.assertEqual((await self.client.post('/api/v1/shell',json={'cmd':'touch /tmp/bad'},headers={'Origin':self.origin})).status,404)
 async def test_path_escape_and_unknown_replay(self):
  await self.login();r=await self.client.get('/api/v1/replays/..%2F..%2Fetc%2Fpasswd');self.assertIn(r.status,(400,404))
 async def test_server_restart_cookie_stays_valid(self):
  await self.login();token=self.client.session.cookie_jar.filter_cookies(self.client.make_url('/'))['ranger_session'].value
  from workbench.server import API
  self.assertTrue(API('/no-socket',self.tmp.name).authenticated(token))
 async def test_static_map_and_headers(self):
  await self.login();maps=await (await self.client.get('/api/v1/maps')).json()
  r=await self.client.get('/api/v1/maps/'+maps[0]['id']+'/image');self.assertEqual(r.content_type,'image/png')
  self.assertIn("frame-ancestors 'none'",r.headers['Content-Security-Policy'])
 async def test_empty_replay_returns_error(self):
  await self.login();r=await self.client.get('/api/v1/replays/missing');self.assertEqual(r.status,400)


class AdditionalContracts(unittest.TestCase):
 def test_nonfinite_telemetry_can_be_json_encoded(self):
  s=State('a',UUID);s.consume('/navigation/minimum_distance',{'data':float('inf')})
  s.consume('/navigation/state',{'data':'{"minimum":NaN}'})
  json.dumps(s.snapshot(),allow_nan=False)
 def test_unconfirmed_initial_pose_is_hidden(self):
  s=State('a',UUID);s.consume('/localization/estimate',loc(state=0))
  self.assertIsNone(s.snapshot()['localization']['body_pose'])
 def test_invalid_pose_keeps_last_confirmed_body(self):
  s=State('a',UUID);s.consume('/localization/estimate',loc());m=loc(state=3);m['pose']['position']['x']=999
  s.consume('/localization/estimate',m);self.assertEqual(s.snapshot()['localization']['body_pose'][0],1)
 def test_segment_rollover_and_cross_segment_seek(self):
  with tempfile.TemporaryDirectory() as tmp:
   j=Journal(Path(tmp)/'record',{},min_free_bytes=0);j.add('snapshot',{'v':1},10**9);j.add('event',{'kind':'goal'},2*10**9);j.add('snapshot',{'v':2},602*10**9);j.close()
   r=Replay(j.folder);self.assertEqual(len(r.files),2);self.assertEqual(r.seek(602)['snapshot']['v'],2);self.assertEqual(r.seek(602)['events'][0]['kind'],'goal')
 def test_watchdog_only_revokes_owned_active_task(self):
  from workbench.watchdog import expired
  with patch('workbench.watchdog.same_process',return_value=False):
   self.assertFalse(expired({'active':False},1));self.assertTrue(expired({'active':True,'steady':1},1))
  with patch('workbench.watchdog.same_process',return_value=True):
   self.assertTrue(expired({'active':True,'steady':1},1.6));self.assertFalse(expired({'active':True,'steady':1},1.4))

class LifecycleContracts(unittest.IsolatedAsyncioTestCase):
 async def test_observe_only_shutdown_does_not_publish_stop(self):
  with tempfile.TemporaryDirectory() as tmp:
   bridge=FakeBridge();owner=Owner(Assets(),tmp,bridge=bridge);await owner.stop();self.assertEqual(bridge.calls,[])
 async def test_stale_cancel_cannot_affect_new_task(self):
  with tempfile.TemporaryDirectory() as tmp:
   owner=Owner(Assets(),tmp,bridge=FakeBridge());owner.session={'id':'new','state':'READY'};owner.task={'id':'new-task','state':'RUNNING'}
   with self.assertRaises(ValueError):await owner.request('cancel',{'view':'live','session_id':'old','task_id':'new-task','request_id':'old-cancel'})
   self.assertEqual(owner.bridge.calls,[])
 async def test_owned_process_stops_without_touching_foreign(self):
  import subprocess
  with tempfile.TemporaryDirectory() as tmp:
   owner=Owner(Assets(),tmp,bridge=FakeBridge());owner.session={'id':'test','state':'READY'}
   foreign=subprocess.Popen([sys.executable,'-c','import time;time.sleep(30)'])
   try:
    owner.spawn('base',[sys.executable,'-c','import time;time.sleep(30)'],Path(tmp)/'mock-session')
    child=owner.children['base'][0];await owner.stop();self.assertIsNotNone(child.poll());self.assertEqual(owner.children,{});self.assertIsNone(foreign.poll())
   finally:foreign.terminate();foreign.wait()



class FeedbackContracts(unittest.TestCase):
 def test_stationarity_requires_original_samples_and_dwell(self):
  s=State('a',UUID)
  def wheel(n):return {'header':{'stamp':{'sec':n//10**9,'nanosec':n%10**9}},'twist':{'twist':{'linear':{'x':0.,'y':0.},'angular':{'z':0.}}}}
  for i in range(6):
   n=10**9+i*100_000_000;s.consume('/clock',clock(n),n);s.consume('/fast_livo/navigation/wheel/odom',wheel(n),n)
   self.assertEqual(s.snapshot(n)['feedback_stopped'],i==5)
  self.assertFalse(s.snapshot(n+300_000_000)['feedback_stopped'])
 def test_zero_quaternion_cannot_show_confirmed_pose(self):
  s=State('a',UUID);m=loc();m['pose']['orientation']['w']=0;s.consume('/localization/estimate',m)
  self.assertIsNone(s.snapshot()['localization']['body_pose'])
 def test_environment_expiry_does_not_refresh_observed_free(self):
  s=State('a',UUID);s.consume('/clock',clock(10**9));s.consume('/workbench/environment',{'valid':True,'stamp_ns':10**9,'coverage':[[0,0]]})
  s.consume('/clock',clock(1_300_000_000));self.assertFalse(s.snapshot()['topics']['/workbench/environment']['valid'])
 def test_optional_raw_worker_confirms_writes_and_finalizes(self):
  import signal,sqlite3
  from workbench.raw_record import run
  with tempfile.TemporaryDirectory() as tmp:
   root=Path(tmp);folder=root/'record';folder.mkdir();source=root/'runtime';source.mkdir()
   (folder/'workbench_manifest.json').write_text(json.dumps({'localization_folder':str(source)}))
   class Child:
    returncode=None
    def __init__(self,*args,**kw):
     bag=folder/'bag';bag.mkdir()
     with sqlite3.connect(bag/'bag_0.db3') as db:
      db.execute('create table topics(id integer primary key,name text)');db.execute('create table messages(id integer primary key,topic_id integer)')
      for i,t in enumerate(('/livox/lidar','/livox/imu','/front/image_sync','/fast_livo/odometry','/fast_livo/deskewed_imu','/wheel/odom')):
       db.execute('insert into topics values(?,?)',(i,t));db.execute('insert into messages(topic_id) values(?)',(i,))
    def poll(self):return self.returncode
    def send_signal(self,_):self.returncode=0
    def wait(self,timeout=None):return self.returncode
   previous={s:signal.getsignal(s) for s in (signal.SIGINT,signal.SIGTERM)}
   try:
    with patch('workbench.raw_record.subprocess.Popen',Child):self.assertEqual(run(folder,.01),0)
   finally:
    for sig,handler in previous.items():signal.signal(sig,handler)
   result=json.loads((folder/'recorder_result.json').read_text())
   self.assertEqual(len(result['observed_messages']),6);self.assertGreater(result['recorded_seconds'],0)



class StartupFeedbackTests(unittest.IsolatedAsyncioTestCase):
 async def test_can_setup_precedes_chassis_and_records_original_state(self):
  from unittest.mock import AsyncMock
  with tempfile.TemporaryDirectory() as tmp:
   owner=Owner(Assets(),tmp,bridge=FakeBridge());owner.session={'id':'startup','state':'STARTING','profile':'standard'}
   order=[]
   def prepare():order.append('can');return {'enabled_by_capture':True,'before':{'flags':[]},'after':{'flags':['UP']},'rollback_command':['sudo','-n','/usr/sbin/ip','link','set','can0','down']}
   def spawn(name,*args,**kwargs):order.append(name)
   try:
    with patch('workbench.daemon.session_live',return_value=False),patch('workbench.daemon.prepare_chassis_can',side_effect=prepare),patch.object(owner,'spawn',side_effect=spawn),patch.object(owner,'wait_ready',new=AsyncMock()),patch.object(owner,'wait_chassis_control',new=AsyncMock()),patch.object(owner,'wait_localization_ready',new=AsyncMock()):await owner.start()
    self.assertEqual(order,['can','base','localization','navigation']);self.assertEqual(owner.session['state'],'READY')
    saved=json.loads((Path(tmp)/'logs/startup/can_preparation.json').read_text());self.assertEqual(saved['before']['flags'],[]);self.assertTrue(saved['enabled_by_capture'])
   finally:
    if owner.journal:owner.journal.close()
 async def test_can_failure_never_starts_chassis_and_keeps_reason(self):
  with tempfile.TemporaryDirectory() as tmp:
   owner=Owner(Assets(),tmp,bridge=FakeBridge());owner.session={'id':'startup','state':'STARTING','profile':'standard'}
   with patch('workbench.daemon.session_live',return_value=False),patch('workbench.daemon.prepare_chassis_can',side_effect=RuntimeError('未发现can0，请检查USB-CAN连接')),patch.object(owner,'spawn') as spawn:
    await owner.start();spawn.assert_not_called()
   self.assertEqual(owner.session['state'],'FAILED');self.assertIn('未发现can0',owner.session['reason']);self.assertEqual(owner.bridge.calls,[])
 async def test_nested_launch_error_is_exposed(self):
  from types import SimpleNamespace
  from workbench.daemon import startup_failure
  with tempfile.TemporaryDirectory() as tmp:
   folder=Path(tmp);(folder/'base.log').write_text('[INFO] starting\n[ERROR] [launch]: can0 must already be UP/ERROR-ACTIVE with known bitrate.\n')
   record={'pid':42,'start_ticks':123,'boot_id':'boot'}
   state={'guardian':record,'reason':'base exited 1','components':{'base':{'state':'EXITED'}}}
   with patch('workbench.daemon.read_session',return_value=(folder,state)):
    message=startup_failure('base',folder,record);self.assertIn('can0 must already be UP',message)
    self.assertIsNone(startup_failure('base',folder,{**record,'start_ticks':999}))
    owner=Owner(Assets(),folder/'web',bridge=FakeBridge());owner.session={'id':'startup','state':'STARTING'}
    owner.children['base']=(SimpleNamespace(poll=lambda:2,returncode=2),record,folder)
    with self.assertRaisesRegex(RuntimeError,'can0 must already be UP'):await owner.wait_ready('base',1)
 async def test_foreign_session_prevents_can_side_effect(self):
  with tempfile.TemporaryDirectory() as tmp:
   owner=Owner(Assets(),tmp,bridge=FakeBridge());owner.session={'id':'startup','state':'STARTING'}
   with patch('workbench.daemon.session_live',return_value=True),patch('workbench.daemon.prepare_chassis_can') as prepare:
    await owner.start();prepare.assert_not_called()
 async def test_camera_timeout_is_shown_while_waiting_and_on_failure(self):
  from types import SimpleNamespace
  from workbench.daemon import startup_failure
  with tempfile.TemporaryDirectory() as tmp:
   folder=Path(tmp)
   (folder/'camera.log').write_text(
       'Camera connected and grabbing started\nGetImageBuffer timeout: no frame for 1500 ms\n'
       'Camera stream recovered successfully\n')
   (folder/'lidar.log').write_text('STAGE3_TIME handle=1 data_type=0 time_type=0 raw_timestamp=123\n')
   (folder/'health.json').write_text(json.dumps(
       {'checks':{'camera':False,'lidar':True,'writer':True},'counts':{'camera':0}}))
   record={'pid':42,'start_ticks':123,'boot_id':'boot'}
   state={'guardian':record,'state':'STARTING','components':{'camera':{'state':'STARTING'}}}
   owner=Owner(Assets(),folder/'web',bridge=FakeBridge())
   owner.session={'id':'startup','state':'STARTING'}
   owner.children['localization']=(SimpleNamespace(poll=lambda:None),record,folder)

   async def next_tick(_):
    self.assertIn('MV_E_NODATA',owner.session['reason'])
    self.assertIn('time_type=0',owner.session['reason'])
    state['state']='RUNNING'

   with patch('workbench.daemon.read_session',return_value=(folder,state)), \
        patch('workbench.daemon.asyncio.sleep',side_effect=next_tick):
    await owner.wait_ready('localization',1)
    state.update(state='FAILED',reason='startup readiness timeout; camera(STARTING, health=camera)')
    self.assertIn('MV_E_NODATA',startup_failure('localization',folder,record))
    with (folder/'camera.log').open('a') as handle:
     handle.write('Publishing image: 1280x1024 RGB8, first frame=1\n')
    reason=startup_failure('localization',folder,record)
    self.assertIn('已有出图',reason)
    self.assertNotIn('MV_E_NODATA',reason)
 async def test_canceled_can_preparation_never_starts_chassis(self):
  with tempfile.TemporaryDirectory() as tmp:
   owner=Owner(Assets(),tmp,bridge=FakeBridge());owner.session={'id':'startup','state':'STARTING','profile':'standard'}
   def prepare():owner.session['state']='STOPPING';return {'enabled_by_capture':False,'before':{'flags':['UP']}}
   with patch('workbench.daemon.session_live',return_value=False),patch('workbench.daemon.prepare_chassis_can',side_effect=prepare),patch.object(owner,'spawn') as spawn:
    await owner.start();spawn.assert_not_called()
 def test_existing_driver_prevents_can_preparation(self):
  from workbench.daemon import prepare_chassis_can
  with patch('scripts.fast_livo2.wheel_capture.check_other_drivers',side_effect=RuntimeError('已有底盘进程')),patch('scripts.fast_livo2.wheel_capture.can_state') as read:
   with self.assertRaisesRegex(RuntimeError,'已有底盘进程'):prepare_chassis_can()
   read.assert_not_called()



class ComputerControlTests(unittest.IsolatedAsyncioTestCase):
 def sample(self,owner,mode=1,age_ns=0,vehicle=0,error=0):
  n=time.time_ns();t=n-age_ns
  owner.state.consume('/system_state',{'header':{'stamp':{'sec':t//10**9,'nanosec':t%10**9}},'control_mode':mode,'vehicle_state':vehicle,'error_code':error},n)
 async def test_computer_control_needs_no_remote_topic_or_switch_flag(self):
  from workbench.daemon import chassis_control_feedback
  with tempfile.TemporaryDirectory() as tmp:
   owner=Owner(Assets(),tmp,bridge=FakeBridge());owner.session={'id':'test','state':'STARTING'};self.sample(owner)
   self.assertTrue(chassis_control_feedback(owner.state)[0]);await owner.wait_chassis_control(.1)
   self.assertEqual(owner.bridge.calls,[]);self.assertEqual(owner.session['stage'],'chassis_control')
 async def test_rc_priority_is_preserved_without_mode_retries(self):
  with tempfile.TemporaryDirectory() as tmp:
   owner=Owner(Assets(),tmp,bridge=FakeBridge());owner.session={'id':'test','state':'STARTING'};self.sample(owner,mode=3)
   with self.assertRaisesRegex(RuntimeError,'遥控器当前占用控制权'):await owner.wait_chassis_control(.01)
   self.assertEqual(owner.bridge.calls,[])
 async def test_old_original_feedback_cannot_confirm_mode(self):
  from workbench.daemon import chassis_control_feedback
  with tempfile.TemporaryDirectory() as tmp:
   owner=Owner(Assets(),tmp,bridge=FakeBridge());self.sample(owner,age_ns=300_000_000)
   ready,reason=chassis_control_feedback(owner.state);self.assertFalse(ready);self.assertIn('原采样时间已过期',reason)
 async def test_fault_and_standby_are_not_computer_permission(self):
  from workbench.daemon import chassis_control_feedback
  with tempfile.TemporaryDirectory() as tmp:
   owner=Owner(Assets(),tmp,bridge=FakeBridge())
   for mode,vehicle,error in [(0,0,0),(1,2,0),(1,0,16)]:
    self.sample(owner,mode=mode,vehicle=vehicle,error=error);self.assertFalse(chassis_control_feedback(owner.state)[0])
 async def test_goal_does_not_override_rc_or_standby(self):
  with tempfile.TemporaryDirectory() as tmp:
   owner=Owner(Assets(),tmp,bridge=FakeBridge());owner.session={'id':'test','state':'READY'}
   n=time.time_ns();owner.state.consume('/clock',clock(10**9),n);owner.state.consume('/localization/estimate',loc(map_uuid=owner.state.map_uuid),n)
   owner.state.consume('/navigation_ready',{'data':True},n);owner.state.consume('/base/execution_state',{},n)
   for mode in (0,3):
    self.sample(owner,mode=mode)
    with self.assertRaises(ValueError):await owner.request('goal',{'view':'live','session_id':'test','map_id':owner.map_id,'pose':[1,0,0],'request_id':'mode-'+str(mode)+'-goal'})
   self.assertEqual(owner.bridge.calls,[])



class RestartAndStopTests(unittest.IsolatedAsyncioTestCase):
 def wheel(self,n,v):
  return {'header':{'stamp':{'sec':n//10**9,'nanosec':n%10**9}},'twist':{'twist':{'linear':{'x':v,'y':0.},'angular':{'z':0.}}}}
 async def test_raw_wheel_stop_is_independent_of_sensor_clock(self):
  state=State('a',UUID)
  for i in range(6):
   n=10**9+i*100_000_000;state.consume('/wheel/odom',self.wheel(n,.023),n)
   snapshot=state.snapshot(n)
   self.assertEqual(snapshot['feedback_stopped'],i==5)
  self.assertEqual(snapshot['feedback_source'],'/wheel/odom');self.assertEqual(snapshot['motion_limits']['stop_linear'],.05)
  self.assertFalse(state.snapshot(n+251_000_000)['feedback_stopped'])
 async def test_stop_boundaries_and_raw_wheel_staleness(self):
  for velocity,expected in [(.05,True),(.05001,False),(-.05,True)]:
   state=State('a',UUID)
   for i in range(6):
    n=10**9+i*100_000_000;state.consume('/wheel/odom',self.wheel(n,velocity),n)
   self.assertEqual(state.snapshot(n)['feedback_stopped'],expected)
   # A fresh clock-mapped relay cannot renew an expired native sample.
   state.consume('/clock',clock(n+300_000_000),n+300_000_000)
   state.consume('/fast_livo/navigation/wheel/odom',self.wheel(n+300_000_000,0),n+300_000_000)
   self.assertFalse(state.snapshot(n+300_000_000)['feedback_stopped'])
 async def test_linear_threshold_does_not_relax_rotation_or_dwell(self):
  state=State('a',UUID)
  for i in range(6):
   n=10**9+i*100_000_000;m=self.wheel(n,.02);m['twist']['twist']['angular']['z']=.026;state.consume('/wheel/odom',m,n)
  self.assertFalse(state.snapshot(n)['feedback_stopped'])
 async def test_localization_wait_reports_true_cause_and_never_launches_navigation(self):
  with tempfile.TemporaryDirectory() as tmp:
   owner=Owner(Assets(),tmp,bridge=FakeBridge());owner.session={'id':'second','state':'STARTING'};owner.localization_folder=tmp
   Path(tmp,'localization_health.json').write_text(json.dumps({'backend':{'reason':'searching_waiting_query_window'},'backend_metrics':{'query_window_frames':4,'query_window_required_frames':8}}))
   with self.assertRaisesRegex(RuntimeError,'重定位未就绪'):await owner.wait_localization_ready(.01)
   self.assertIn('4/8',owner.session['reason']);self.assertEqual(owner.session['stage'],'relocalization');self.assertEqual(owner.bridge.calls,[])
 async def test_repeated_lifecycle_has_new_identity_and_no_old_task(self):
  from unittest.mock import AsyncMock
  with tempfile.TemporaryDirectory() as tmp:
   owner=Owner(Assets(),tmp,bridge=FakeBridge());ids=[];order=[]
   def prepare():return {'enabled_by_capture':False,'before':{'flags':['UP']}}
   async def wait_localization(*args,**kw):order.append('localization_confirmed')
   def spawn(name,*args,**kwargs):order.append(name)
   with patch('workbench.daemon.session_live',return_value=False),patch('workbench.daemon.prepare_chassis_can',side_effect=prepare),patch.object(owner,'spawn',side_effect=spawn),patch.object(owner,'wait_ready',new=AsyncMock()),patch.object(owner,'wait_chassis_control',new=AsyncMock()),patch.object(owner,'wait_localization_ready',side_effect=wait_localization):
    for index in range(2):
     result=await owner.request('session.start',{'view':'live','request_id':f'lifecycle-start-{index}','map_id':owner.map_id})
     await owner.start_job;ids.append(owner.session['id']);self.assertEqual(owner.session['state'],'READY');self.assertIsNone(owner.task)
     owner.task={'id':'old-terminal-task','state':'SUCCEEDED'}
     await owner.request('session.stop',{'view':'live','request_id':f'lifecycle-stop-{index}','session_id':owner.session['id']})
     await owner.stop_job;self.assertEqual(owner.session['state'],'STOPPED');self.assertEqual(owner.children,{})
   self.assertNotEqual(*ids);self.assertEqual(order,['base','localization','localization_confirmed','navigation']*2)

if __name__=='__main__':unittest.main()
