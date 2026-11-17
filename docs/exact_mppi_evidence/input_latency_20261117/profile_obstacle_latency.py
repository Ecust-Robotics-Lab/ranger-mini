import sys,sqlite3,json,cProfile,pstats,time,numpy as np
from pathlib import Path
R=Path('/home/lab307/ranger_nav');sys.path[:0]=[str(R/'ws_nav/src/ranger_navigation'),str(R/'ws_nav/src/ranger_localization_bridge')]
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from ranger_navigation.geometry import load_geometry
from ranger_navigation.coordinator import cloud_xyz
from ranger_navigation.obstacles import ObstacleMemory
from scipy.spatial.transform import Rotation
E=R/'docs/exact_mppi_evidence/input_latency_20261117';bundle=E/'replay_frames.npz'
if not bundle.exists():
 con=sqlite3.connect((R/'bags/fast-livo-localization-z7vdr1vj/raw/raw_0.db3').as_uri()+'?mode=ro',uri=True);T={n:(i,t) for i,n,t in con.execute('select id,name,type from topics')}
 w=json.loads((R/'docs/exact_mppi_evidence/ramp_corridor_attempt_20261116/stall_timing.json').read_text());end=round(w['end']*1e9);start=end-8_000_000_000
 def rows(n):
  i,t=T[n];C=get_message(t)
  return [(ts,deserialize_message(d,C)) for ts,d in con.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp',(i,start-1_000_000_000,end))]
 wheels={}
 for _,m in rows('/fast_livo/navigation/wheel/odom'):
  p=m.pose.pose.position;q=m.pose.pose.orientation;wheels[m.header.stamp.sec*10**9+m.header.stamp.nanosec]=[p.x,p.y,Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz')[2]]
 keys=np.array(sorted(wheels),dtype=np.int64);wp=np.array([wheels[k] for k in keys]);wp[:,2]=np.unwrap(wp[:,2]);frames=[]
 for ts,m in rows('/fast_livo/deskewed_imu'):
  ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec
  if not keys[0]<=ns<=keys[-1] or ts<start:continue
  pose=np.array([np.interp((ns-keys[0])*1e-9,(keys-keys[0])*1e-9,wp[:,i]) for i in range(3)])
  frames.append((ns,cloud_xyz(m),pose))
 con.close();np.savez_compressed(bundle,stamps=np.array([f[0] for f in frames],dtype=np.int64),poses=np.array([f[2] for f in frames]),**{'cloud_'+str(i):f[1] for i,f in enumerate(frames)})
z=np.load(bundle);b,r,t=load_geometry(R/'config/fast_livo2/navigation_prep.yaml',.03)
mem=ObstacleMemory(b,sectors=180,angular_guard=0,control_memory=2.)
profile=cProfile.Profile();times=[];profile.enable()
for i,ns in enumerate(z['stamps']):
 a=time.perf_counter();mem.update(z['cloud_'+str(i)],r,t,z['poses'][i],int(ns)*1e-9);times.append((time.perf_counter()-a)*1000)
profile.disable();pstats.Stats(profile).sort_stats('cumtime').print_stats(25)
print('frames',len(times),'update_ms',np.percentile(times,[50,95,99,100]))
out=sys.argv[1] if len(sys.argv)>1 else 'baseline'
profile.dump_stats(str(E/(out+'.prof')));(E/(out+'_timing.json')).write_text(json.dumps(dict(frames=len(times),update_ms=dict(zip(['p50','p95','p99','max'],map(float,np.percentile(times,[50,95,99,100]))))),indent=2))
