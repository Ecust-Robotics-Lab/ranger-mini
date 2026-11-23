"""Restart only an owned stopped compute child; old mailbox results cannot resume."""
import json,os,signal,sys,time
from pathlib import Path
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'ws_nav/src/ranger_navigation'))
from ranger_navigation.worker import WorkerClient
from ranger_navigation.task import Task

def wait_ready(worker,timeout=35):
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  value=worker.status()
  if value.get('ready'):return value
  assert not value.get('error'),value
  time.sleep(.01)
 raise AssertionError('compute child did not become ready')

def test_reset_clears_terminal_goal_route_and_both_generations():
 task=Task();task.begin([1,0,0],0);task.block('goal_live_obstacle_collision',1);task.invalidate('no_progress_timeout','FAILED')
 old=task.generation;history=task.history_generation;task.reset()
 assert task.goal is None and not task.segments and task.state=='RESETTING'
 assert task.generation>old and task.history_generation>history and task.failure_cause is None
 assert not task.plan([],old,2.)

def test_owned_gpu_worker_recovers_from_sigstop_without_accepting_old_nonce(tmp_path):
 worker=WorkerClient(tmp_path,ROOT/'.runtime/exact_mppi_venv/bin/python',ROOT/'config/fast_livo2/navigation_prep.yaml',samples=32,static_clearance=.01)
 try:
  first=wait_ready(worker);old_nonce=worker.nonce;old_process=worker.process
  os.kill(old_process.pid,signal.SIGSTOP)
  worker.restart();assert worker.pending is None
  status=worker.status();assert not status.get('ready') and status.get('resetting')
  second=wait_ready(worker)
  assert worker.nonce!=old_nonce and second['pid']!=first['pid'] and old_process.poll() is not None
  request=worker.submit('plan',42,{'cells':np.zeros((100,100)),'pose':np.zeros(3),'goal':np.array([.3,0,0]),'points':np.empty((0,2))},
      {'resolution':.05,'origin':[-2.,-2.,0.],'actual_mode':0},budget=3.)
  assert request['nonce']==worker.nonce
  end=time.monotonic()+4;result=None
  while result is None and time.monotonic()<end:result=worker.poll();time.sleep(.005)
  assert result and not result[0].get('error'),result
  assert result[0]['nonce']==second['nonce'] and result[0]['generation']==42 and result[0]['segments']
 finally:worker.close()
