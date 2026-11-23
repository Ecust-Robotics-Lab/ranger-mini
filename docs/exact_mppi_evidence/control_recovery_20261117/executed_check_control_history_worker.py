import sys,time,json,numpy as np
from pathlib import Path
R=Path('/home/lab307/ranger_nav');sys.path.insert(0,str(R/'ws_nav/src/ranger_navigation'))
from ranger_navigation.worker import WorkerClient
from ranger_navigation.geometry import validate_sweep,Body
from ranger_navigation.models import Limits,Mode,feasible
E=R/'docs/exact_mppi_evidence/control_recovery_20261117';folder=E/'worker_mailbox'
worker=WorkerClient(folder,R/'.runtime/exact_mppi_venv/bin/python',R/'config/fast_livo2/navigation_prep.yaml',samples=128)
rows=[]
try:
 end=time.monotonic()+40
 while not worker.status().get('ready'):
  if time.monotonic()>end:raise TimeoutError(worker.status())
  time.sleep(.03)
 arrays=dict(pose=np.zeros(3),speed=np.zeros(3),path=np.array([[0.,0.,0.],[.6,0.,0.]]),points=np.empty((0,2)),visibility_cells=np.zeros((80,80),np.int16),observer_pose=np.zeros(3))
 ack=None
 for label,gen,route,stamp,ready,waiting in [('first',1,1,10.,True,False),('executed',1,1,10.05,True,False),('soft_pause',2,1,10.1,True,True),('replacement',3,2,10.15,True,False),('feedback_missing',4,2,10.2,False,False)]:
  if label=='executed':arrays['pose']=previous['trajectory'][1];arrays['speed']=previous['command']
  if label=='soft_pause':arrays['pose']=np.zeros(3);arrays['speed']=np.zeros(3)
  meta=worker.submit('control',gen,arrays,dict(mode=1,segment=0,sample_stamp=stamp,sample_stamp_ns=round(stamp*1e9),history_generation=route,accepted_request_id=ack,execution_ready=ready,steering_wait=waiting),budget=.1)
  assert meta
  result=None
  while result is None:result=worker.poll();time.sleep(.001)
  meta,output=result
  assert not meta.get('error'),meta
  assert validate_sweep(output['trajectory'],[],Body(),coverage=lambda *args:True)[0]
  assert all(feasible(v,Mode.PARALLEL) for v in output['controls'])
  delta=np.diff(np.vstack((arrays['speed'],output['controls'])),axis=0)
  assert np.max(np.linalg.norm(delta[:,:2],axis=1))<=.01501
  if label=='executed':assert meta['history']['reused'] and meta['history']['route_resets']==1
  if label=='soft_pause':assert meta['generation']==2 and meta['history']['reused'] and meta['history']['shift_steps']==0 and meta['history']['route_resets']==1
  if label=='replacement':assert not meta['history']['reused'] and meta['history']['route_resets']==2
  if label=='feedback_missing':assert not meta['history']['reused'] and meta['history']['reason']=='execution_feedback_unavailable'
  rows.append(dict(name=label,request_id=meta['id'],generation=gen,history_generation=route,history=meta['history'],command=output['command'].tolist(),compute_ms=meta['compute_s']*1000))
  # An unvalidated result is deliberately left unacknowledged in soft_pause.
  if label!='executed':ack=meta['id']
  previous=output
 print(json.dumps(rows,indent=2));(E/'worker_history_results.json').write_text(json.dumps(rows,indent=2))
finally:worker.close()
