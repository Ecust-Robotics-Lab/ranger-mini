"""ROS/GPU A/B with a synthetic finite-acceleration base, never CAN/devices."""
import os,sys,json,time,subprocess,tempfile,importlib.util,tarfile,io
from pathlib import Path
import numpy as np
R=Path('/home/lab307/ranger_nav');E=R/'docs/exact_mppi_evidence/control_recovery_20261117';baseline=R/'.runtime/backups/control_recovery_20261117/baseline_python'
if not baseline.exists():
 target=baseline/'ranger_navigation';target.mkdir(parents=True)
 data=subprocess.check_output(['git','archive','HEAD:ws_nav/src/ranger_navigation/ranger_navigation'])
 with tarfile.open(fileobj=io.BytesIO(data)) as archive:archive.extractall(target)
spec=importlib.util.spec_from_file_location('chain_test',R/'tests/test_stage6_safety.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
from ranger_navigation.ros_utils import twist_array
from std_msgs.msg import Bool
import rclpy

class Plant(module.World):
 def __init__(self):
  self.requested_velocity=np.zeros(3);self.inject_since=None;self.measurements=[]
  super().__init__()
 def on_cmd(self,m):
  self.requested_velocity=twist_array(m);self.commands.append((time.monotonic(),self.requested_velocity.copy()))
 def tick(self):
  delta=self.requested_velocity-self.velocity;length=np.linalg.norm(delta[:2]);self.velocity[:2]+=delta[:2]*min(1.,.30*.02/max(length,1e-12));self.velocity[2]+=np.clip(delta[2],-.50*.02,.50*.02)
  now=time.monotonic()
  self.ready=(self.inject_since is None or now-self.inject_since<1.5 or (now-self.inject_since-1.5)%.8>=.06)
  super().tick()
  if self.inject_since is not None:self.measurements.append([now-self.inject_since,*self.pose,*self.velocity,self.ready])

results=[]
for variant,domain in [('baseline',223),('optimized',224)]:
 os.environ['ROS_DOMAIN_ID']=str(domain);os.environ['ROS_LOCALHOST_ONLY']='1'
 env=os.environ.copy();env['XLA_PYTHON_CLIENT_PREALLOCATE']='false';env['OPENBLAS_NUM_THREADS']='1'
 if variant=='baseline':env['PYTHONPATH']=str(baseline)+os.pathsep+env.get('PYTHONPATH','')
 folder=E/('continuity_'+variant);folder.mkdir();procs=[];files=[]
 prefix=R/'ws_nav/install_exact/ranger_navigation/lib/ranger_navigation'
 args=['--ros-args','-p','geometry_config:='+str(R/'config/fast_livo2/navigation_prep.yaml'),'-p','clearance:=0.03']
 try:
  for name,extra in [('motion_gate',[]),('exact_navigation',['-p','optimizer_python:='+str(R/'.runtime/exact_mppi_venv/bin/python'),'-p','session_root:='+str(folder),'--log-level','error'])]:
   f=(folder/(name+'.log')).open('w');files.append(f);procs.append(subprocess.Popen([str(prefix/name),*args,*extra],env=env,stdout=f,stderr=subprocess.STDOUT))
  rclpy.init();w=Plant();w.wait(lambda:w.state.get('worker',{}).get('ready') and w.state.get('inputs_valid'),40.)
  w.wait(lambda:w.client.server_is_ready());goal=w.goal(2.,0.);future=goal.get_result_async();w.inject_since=time.monotonic();w.arm(True)
  end=w.inject_since+30
  while time.monotonic()<end and not future.done():rclpy.spin_once(w,timeout_sec=.005)
  finished=time.monotonic();elapsed=finished-w.inject_since;status=future.result().status if future.done() else None
  if not future.done():f=goal.cancel_goal_async();w.wait(f.done)
  start=w.inject_since;w.arm(False);w.spin(.2)
  commands=np.array([[t-start,*v] for t,v in w.commands if start<=t<=finished]);zero=np.max(abs(commands[:,1:]),axis=1)<1e-6
  times=np.r_[commands[:,0],elapsed];duration=np.diff(times);zero_time=float(np.sum(duration[zero]));runs=[];run=0.
  for z,dt in zip(zero,duration):
   if z:run+=dt
   elif run:runs.append(run);run=0.
  if run:runs.append(run)
  history=[v['state'].get('metrics',{}).get('history',{}) for v in w.trace]
  row=dict(variant=variant,elapsed_s=elapsed,status=status,position=w.pose.tolist(),zero_command_time_s=zero_time,zero_command_fraction=zero_time/elapsed,longest_zero_interval_s=max(runs,default=0.),last_generation=w.state.get('generation'),history_reuse_observed=any(x.get('reused') for x in history),hardware_started=False)
  results.append(row);print(json.dumps(row),flush=True);(folder/'result.json').write_text(json.dumps(row,indent=2));(folder/'trace.json').write_text(json.dumps(w.trace));(folder/'plant.json').write_text(json.dumps(w.measurements))
 finally:
  if 'w' in locals():w.destroy_node();del w
  if rclpy.ok():rclpy.shutdown()
  for p in procs:
   if p.poll() is None:p.terminate()
  for p in procs:
   try:p.wait(timeout=4.)
   except subprocess.TimeoutExpired:p.kill();p.wait()
  for f in files:f.close()
(E/'continuity_comparison.json').write_text(json.dumps(dict(results=results,plant='Ideal .30m/s2/.50rad/s2 base, 20ms feedback, 60ms readiness withdrawal every .8s after 1.5s; command zero includes real injected withdrawals. This is a synthetic software comparison, not field performance.'),indent=2))
