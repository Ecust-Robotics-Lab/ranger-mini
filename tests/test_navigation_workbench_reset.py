"""Owned reset works after failure and without valid localization; no devices."""
import asyncio,json,tempfile,time,unittest
from pathlib import Path
from workbench.core import Assets
from workbench.daemon import Owner
from tests.test_navigation_workbench import FakeBridge

class ResetBridge(FakeBridge):
 def __init__(self,owner):super().__init__();self.owner=owner;self.wait=None;self.error=None
 async def reset(self,task_id):
  self.calls.append(('reset',task_id))
  if self.wait:await self.wait.wait()
  if self.error:raise RuntimeError(self.error)
  self.owner.consume('/navigation/state',{'data':json.dumps({'state':'IDLE','generation':8,'worker':{'ready':True,'nonce':'new'}})})
  return {'generation':8,'previous_worker_nonce':'old'}

class ResetTests(unittest.IsolatedAsyncioTestCase):
 async def asyncSetUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.owner=Owner(Assets(),self.tmp.name);self.owner.bridge=ResetBridge(self.owner)
  self.owner.session={'id':'owned','state':'READY'};self.owner.task={'id':'failed-task','state':'FAILED'}
  self.owner.state.latest['/navigation/state']={'state':'FAILED','worker':{'ready':True,'nonce':'old'}}
 def req(self,**kw):return {'view':'live','session_id':'owned','request_id':'reset-request-1',**kw}
 async def asyncTearDown(self):
  if self.owner.reset_job and not self.owner.reset_job.done():self.owner.reset_job.cancel();await self.owner.reset_job
  self.tmp.cleanup()
 async def test_failed_task_can_reset_without_pose_and_keeps_device_children(self):
  marker=object();self.owner.children={'base':marker,'localization':marker,'navigation':marker}
  self.owner.state.latest['/plan']={'points':[[1,2,0]]}
  result=await self.owner.request('navigation.reset',self.req());self.assertEqual(result['state'],'RESETTING')
  await self.owner.reset_job
  self.assertEqual(self.owner.navigation_reset['state'],'READY');self.assertIsNone(self.owner.task)
  self.assertEqual(self.owner.children,{'base':marker,'localization':marker,'navigation':marker})
  self.assertNotIn('/plan',self.owner.state.latest);self.assertFalse(any(c==('arm',True) for c in self.owner.bridge.calls))
  self.assertEqual(self.owner.bridge.calls,[('arm',False),('reset','failed-task'),('arm',False)])
 async def test_reset_is_idempotent_and_rejects_new_goal_while_busy(self):
  self.owner.bridge.wait=asyncio.Event()
  await self.owner.request('navigation.reset',self.req());job=self.owner.reset_job
  await self.owner.request('navigation.reset',self.req());self.assertIs(self.owner.reset_job,job)
  with self.assertRaisesRegex(ValueError,'正在重置'):await self.owner.request('goal',self.req(request_id='goal-during-reset'))
  self.owner.bridge.wait.set();await job
  self.assertEqual(len([c for c in self.owner.bridge.calls if c[0]=='reset']),1)
 async def test_stop_interrupts_reset_instead_of_waiting_for_warmup(self):
  self.owner.bridge.wait=asyncio.Event();await self.owner.request('navigation.reset',self.req());await asyncio.sleep(0)
  await asyncio.wait_for(self.owner.stop(),.5)
  self.assertEqual(self.owner.session['state'],'STOPPED');self.assertEqual(self.owner.navigation_reset['state'],'CANCELED')
 async def test_failed_reset_stays_disabled_and_can_retry_own_reset(self):
  self.owner.bridge.error='warmup failed';await self.owner.request('navigation.reset',self.req());await self.owner.reset_job
  self.assertEqual(self.owner.navigation_reset['state'],'FAILED');self.assertEqual(self.owner.bridge.calls[-1],('arm',False))
  self.owner.state.latest['/navigation/state']['state']='RESETTING';self.owner.bridge.error=None
  await self.owner.request('navigation.reset',self.req(request_id='reset-request-2'));await self.owner.reset_job
  self.assertEqual(self.owner.navigation_reset['state'],'READY')
 async def test_external_task_replay_demo_and_wrong_session_cannot_reset(self):
  for changed in [{'view':'replay'},{'session_id':'retired'}]:
   with self.assertRaises(ValueError):await self.owner.request('navigation.reset',self.req(**changed))
  self.owner.state.latest['/navigation/state']['state']='EXECUTING'
  with self.assertRaisesRegex(ValueError,'外部'):await self.owner.request('navigation.reset',self.req())
  self.owner.demo=True
  with self.assertRaises(ValueError):await self.owner.request('navigation.reset',self.req())
  self.assertEqual(self.owner.bridge.calls,[])

if __name__=='__main__':unittest.main()
