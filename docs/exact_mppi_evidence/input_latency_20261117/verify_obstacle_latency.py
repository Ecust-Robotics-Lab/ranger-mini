import sys,importlib.util,time,json,numpy as np
from pathlib import Path
R=Path('/home/lab307/ranger_nav');sys.path.insert(0,str(R/'ws_nav/src/ranger_navigation'))
from ranger_navigation.obstacles import ObstacleMemory
from ranger_navigation.geometry import load_geometry
spec=importlib.util.spec_from_file_location('ranger_navigation.obstacles_baseline',R/'.runtime/backups/input_latency_20261117/obstacles.py');base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
E=R/'docs/exact_mppi_evidence/input_latency_20261117';z=np.load(E/'replay_frames.npz');frames=[(int(ns)*1e-9,z['cloud_'+str(i)],z['poses'][i]) for i,ns in enumerate(z['stamps'])];b,r,t=load_geometry(R/'config/fast_livo2/navigation_prep.yaml',.03)
old=base.ObstacleMemory(b,sectors=180,angular_guard=0,control_memory=2.);new=ObstacleMemory(b,sectors=180,angular_guard=0,control_memory=2.)
for stamp,xyz,pose in frames:
 assert old.update(xyz,r,t,pose,stamp)==new.update(xyz,r,t,pose,stamp)
 np.testing.assert_array_equal(old.points(),new.points());np.testing.assert_array_equal(old.control_grid.cells,new.control_grid.cells);np.testing.assert_array_equal(old._visible_grid.cells,new._visible_grid.cells)
times={'baseline':[],'optimized':[]}
for repeat in range(4):
 for name,cls in ([('baseline',base.ObstacleMemory),('optimized',ObstacleMemory)] if repeat%2==0 else [('optimized',ObstacleMemory),('baseline',base.ObstacleMemory)]):
  m=cls(b,sectors=180,angular_guard=0,control_memory=2.)
  for stamp,xyz,pose in frames:
   started=time.perf_counter();m.update(xyz,r,t,pose,stamp);times[name].append((time.perf_counter()-started)*1000)
out=dict(frames=len(frames),equivalence='PASS: all full obstacle points, control cells and safety visibility cells identical at every frame',method='4 interleaved replay passes, no profiler, raw clouds and same-stamp interpolated wheel; no ROS or devices',timing_ms={k:dict(zip(['p50','p95','p99','max'],map(float,np.percentile(v,[50,95,99,100])))) for k,v in times.items()})
(E/'obstacle_equivalence_and_timing.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
