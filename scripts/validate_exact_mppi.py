#!/usr/bin/env python3
"""No hardware/ROS graph: JAX benchmark and deterministic closed-loop plant."""
import argparse,json,time,resource,sys
import yaml
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ws_nav/src/ranger_navigation'))
from ranger_navigation.geometry import Body,wrap,integrate,validate_sweep,load_geometry
from ranger_navigation.models import Mode,Limits,feasible,braking_commands,rollout
from ranger_navigation.planner import Planner,Grid
from ranger_navigation.task import Task
from ranger_navigation.exact_controller import ExactController


def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--benchmark-only',action='store_true')
 p.add_argument('--trace-output',type=Path,help='Optional visualization sidecar; does not change the plant/controller')
 p.add_argument('--clearance-profile',choices=('standard','narrow'),default='standard')
 p.add_argument('--scenario',action='append',choices=('forward','reverse','lateral','diagonal','spin','left_turn','right_turn','static_obstacle','door','narrow_corridor'))
 args=p.parse_args()
 traces=[]
 cfg=yaml.safe_load((ROOT/'ws_nav/src/ranger_navigation/config/exact_navigation.yaml').read_text())
 profile=cfg['clearance_profiles'][args.clearance_profile]
 body,_,_=load_geometry(ROOT/'config/fast_livo2/navigation_prep.yaml',profile['clearance'])
 import jax
 from exact_mppi.mppi_jax.logger import logger
 logger.setLevel(40)
 c=ExactController(body,samples=256,visibility_reserve=profile['visibility_reserve']);warm=c.prewarm()
 path=np.column_stack((np.linspace(0,.8,60),np.zeros((60,2))))
 rng=np.random.default_rng(4);a=np.linspace(-np.pi,np.pi,512,endpoint=False)
 points=np.column_stack((2*np.cos(a),2*np.sin(a)))
 measurements=[]
 for i in range(110):
  began=time.perf_counter();c.compute(np.zeros(3),np.zeros(3),path,Mode.DRIVE,points)
  measurements.append((time.perf_counter()-began)*1000)
 report={'clearance_profile':args.clearance_profile,'profile':profile,
         'devices':[str(d) for d in jax.devices()],'warmup_seconds':warm,
         'benchmark_ms':{'p50':float(np.percentile(measurements,50)),'p95':float(np.percentile(measurements,95)),'p99':float(np.percentile(measurements,99))},
         'rss_kib':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,'scenarios':[]}
 args.output.parent.mkdir(parents=True,exist_ok=True)
 if not args.benchmark_only:
  cases=[('forward',[1.,0,0],None),('reverse',[-.7,0,0],None),('lateral',[0,.7,0],None),
         ('diagonal',[.6,.6,0],None),('spin',[0,0,1.57],None),('left_turn',[1.,.6,1.57],None),
         ('right_turn',[1.,-.6,-1.57],None),('static_obstacle',[1.5,0,0],(.65,0,.12,.16)),
         ('door',[1.5,0,0],None)]
  if args.clearance_profile=='narrow' or (args.scenario and 'narrow_corridor' in args.scenario):
   cases.append(('narrow_corridor',[1.5,0,0],None))
  for name,goal,obstacle in cases:
   if args.scenario and name not in args.scenario:continue
   cells=np.zeros((140,160),dtype=int);origin=(-3.,-3.,0.);resolution=.05
   if name=='narrow_corridor':
    # 65cm physical opening between two full-width one-cell walls. Known
    # static area and real boundary voxel centers are validated independently.
    origin=(-3.,-3.025,0.);cells[53,:]=100;cells[67,:]=100
   if obstacle:
    x,y,hx,hy=obstacle
    ix=int((x-origin[0])/resolution);iy=int((y-origin[1])/resolution)
    cells[iy-int(hy/resolution):iy+int(hy/resolution)+1,ix-int(hx/resolution):ix+int(hx/resolution)+1]=100
   if name=='door':
    ix=int((.7-origin[0])/resolution);cells[:,ix]=100
    y0=int((-.5-origin[1])/resolution);y1=int((.5-origin[1])/resolution);cells[y0:y1,ix]=0
   grid=Grid(cells,resolution,origin)
   blocked=np.column_stack(np.nonzero(grid.blocked)[::-1])
   obs=(blocked+.5)*resolution+np.array(origin[:2]) if len(blocked) else np.empty((0,2))
   # Match production: search the static map plus the observed voxel layer,
   # including the same point uncertainty used by local/final validation.
   planning_grid=grid.with_obstacles(obs)
   pose,velocity=np.zeros(3),np.zeros(3)
   c.reset();task=Task(segment_yaw_tolerance=profile['segment_yaw_tolerance']);task.begin(goal,0.)
   started=time.perf_counter()
   planner_args=dict(static_reserve=profile['static_map_reserve'],prefer_parallel_in_narrow=profile['prefer_parallel_in_narrow'])
   try:segments=Planner(body,timeout=10,**planner_args).plan(planning_grid,pose,goal)
   except Exception as e:
    report['scenarios'].append({'name':name,'pass':False,'reason':str(e)});continue
   task.plan(segments,task.generation,0.)
   route=lambda ss:[{'mode':s.mode.name,'poses':s.poses.tolist()} for s in ss]
   visual={'name':name,'goal':goal,'obstacles':obs.tolist(),'resolution':resolution,
           'route_history':[{'time':0.,'segments':route(segments)}],'frames':[]}
   latest_prediction=[]
   actual_mode=Mode.DRIVE;switch_request=None;switch_at=0.;collided=False;previous_index=-1;max_error=0.
   for step in range(2400):
    now=step*.05
    if switch_request is not None and now>=switch_at:
     actual_mode=switch_request;switch_request=None
    segment=task.tick(pose,velocity,actual_mode,True,'navigation',now)
    if task.state in Task.TERMINAL:break
    command=np.zeros(3)
    if task.index<len(task.segments) and actual_mode!=task.segments[task.index].mode and np.linalg.norm(velocity)<.02 and switch_request is None:
     switch_request=task.segments[task.index].mode;switch_at=now+.15
    if segment is not None:
     if task.index!=previous_index:c.reset();previous_index=task.index
     path=segment.poses
     nearest=int(np.argmin(np.linalg.norm(path[:,:2]-pose[:2],axis=1)+.15*np.abs(wrap(path[:,2]-pose[2]))))
     if len(obs)>512:
      # Test map contains few obstacle boundary cells; retain complete validation.
      sampled=obs[np.argsort(np.linalg.norm(obs-pose[:2],axis=1))[:512]]
     else:sampled=obs
     try:
      result=c.compute(pose,velocity,path,segment.mode,sampled)
      okay,_,reason=validate_sweep(result['trajectory'],obs,body,grid=grid,margin=body.clearance+.04,
                                  map_margin=body.clearance+profile['static_map_reserve'])
      if okay:
       command=result['command']
       latest_prediction=result['trajectory'][::2].tolist()
      else:task.block(reason,now)
     except RuntimeError as e:task.block(str(e),now)
    if task.state in ('PLANNING','BLOCKED'):
     try:
      replanned=Planner(body,timeout=2,**planner_args).plan(planning_grid,pose,goal,actual_mode)
      task.plan(replanned,task.generation,now)
      if args.trace_output:visual['route_history'].append({'time':now,'segments':route(replanned)})
     except Exception:task.block('no_route',now)
    # Plant holds the explicit mode and integrates body velocity; steering/mode
    # latency is separate. Emergency zero bypasses normal acceleration.
    assert feasible(command,actual_mode)
    if args.trace_output:
     visual['frames'].append({'time':round(now,3),'pose':pose.tolist(),'command':command.tolist(),
                             'mode':actual_mode.name,'state':task.state,'reason':task.reason,
                             'prediction':latest_prediction if step%4==0 else []})
    velocity=command
    pose=integrate(pose,velocity,.05)
    if not grid.free(pose,body,0.):collided=True;break
   row={'name':name,'pass':task.state=='SUCCEEDED' and not collided,'state':task.state,'reason':task.reason,
        'pose':pose.tolist(),'goal':goal,'simulated_seconds':now,'wall_seconds':time.perf_counter()-started,
        'collided':collided,'segments':[{'mode':s.mode.name,'reverse':s.reverse} for s in segments]}
   report['scenarios'].append(row);print(json.dumps(row),flush=True)
   if args.trace_output:
    visual['result']=row
    visual['frames'].append({'time':round(now,3),'pose':pose.tolist(),'command':velocity.tolist(),
                            'mode':actual_mode.name,'state':task.state,'reason':task.reason,'prediction':[]})
    traces.append(visual)
   args.output.write_text(json.dumps(report,indent=2)+'\n')
 report['pass']=all(r['pass'] for r in report['scenarios'])
 args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report['benchmark_ms']),flush=True)
 if args.trace_output:
  args.trace_output.parent.mkdir(parents=True,exist_ok=True)
  args.trace_output.write_text(json.dumps({'kind':'new_offline_simulation_trace','hardware_started':False,
      'model_dt':.05,'body':[body.length,body.width],'scenarios':traces},separators=(',',':'))+'\n')
 return 0 if report['pass'] else 1
if __name__=='__main__':raise SystemExit(main())
