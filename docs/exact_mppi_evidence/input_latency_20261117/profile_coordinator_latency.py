import sys,time,json,cProfile,pstats,math
from pathlib import Path
from types import SimpleNamespace
import numpy as np
R=Path('/home/lab307/ranger_nav');sys.path[:0]=[str(R/'ws_nav/src/ranger_navigation'),str(R/'ws_nav/src/ranger_localization_bridge')]
from ranger_navigation.coordinator import Coordinator
from ranger_navigation.geometry import load_geometry
from ranger_navigation.planner import Grid,Segment
from ranger_navigation.models import Limits,Mode
from ranger_navigation.task import Task
from ranger_navigation.obstacles import ObstacleMemory
from ranger_navigation.ros_utils import transform2
from nav_msgs.msg import Odometry
E=R/'docs/exact_mppi_evidence/input_latency_20261117';z=np.load(E/'replay_frames.npz');b,r,t=load_geometry(R/'config/fast_livo2/navigation_prep.yaml',.03)
mem=ObstacleMemory(b,sectors=180,angular_guard=0,control_memory=2.)
for i,ns in enumerate(z['stamps']):mem.update(z['cloud_'+str(i)],r,t,z['poses'][i],int(ns)*1e-9)
request=np.load(R/'docs/exact_mppi_evidence/ramp_corridor_attempt_20261116/final_stall_request.npz');pose=request['pose'];od=z['poses'][-1];yaw=pose[2]-od[2];correction=np.r_[pose[:2]-transform2(od[None,:2],[0,0,yaw])[0],yaw]
seg=Segment(Mode.PARALLEL,request['path']);task=Task();task.begin(seg.poses[-1],time.monotonic());task.plan([seg],task.generation,time.monotonic());task.state='EXECUTING';task.tick=lambda *args:seg
wheel=Odometry();wheel.header.stamp.sec=10
pub=SimpleNamespace(publish=lambda x:None)
class Harness:
 tick=Coordinator.tick
 now=lambda self:10.
 process_cloud=lambda self:None
 sample=lambda self,now:(pose,np.zeros(3),correction)
h=Harness();h.last_clock=None;h.worker=SimpleNamespace(pending={'kind':'control'},status=lambda:{'ready':True},poll=lambda:None,submit=lambda *a,**kw:False)
h.task=task;h.last_path_check=(mem.last_stamp,task.generation,0);h.memory=mem;h.body=b;h.grid=Grid.from_yaml(R/'outputs/fast-livo-demo-changed-scene-20261110/navigation_map_05/map.yaml');h.limits=Limits(speed=.1,yaw_rate=.2);h.command=None;h.last_plan=h.last_control=h.last_state_publish=-1e9;h.actual_mode=Mode.PARALLEL;h.owner='navigation';h.reference_wheel=wheel;h.session='offline-benchmark';h.metrics={};h.goal_handle=None;h.input_reason='valid';h.static_reserve=.01
for name in ['command_pub','velocity_pub','state_pub','path_pub','local_pub','minimum_pub']:setattr(h,name,pub)
prof=cProfile.Profile();times=[];prof.enable()
for i in range(300):
 start=time.perf_counter();h.tick();times.append((time.perf_counter()-start)*1000)
prof.disable();pstats.Stats(prof).sort_stats('cumtime').print_stats(20)
name=sys.argv[1];prof.dump_stats(str(E/(name+'_tick.prof')));out=dict(iterations=len(times),scenario='Exact production tick with frozen real memory/path and busy asynchronous worker; no devices, publishers discarded',tick_ms=dict(zip(['p50','p95','p99','max'],map(float,np.percentile(times,[50,95,99,100])))))
(E/(name+'_tick_timing.json')).write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
