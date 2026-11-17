import json,sqlite3,sys,numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
R=Path('/home/lab307/ranger_nav');sys.path[:0]=[str(R/'ws_nav/src/ranger_navigation'),str(R/'ws_nav/src/ranger_localization_bridge')]
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from ranger_navigation.geometry import load_geometry,validate_sweep
from ranger_navigation.planner import Grid
from ranger_navigation.obstacles import ObstacleMemory
from ranger_navigation.coordinator import cloud_xyz
E=R/'docs/exact_mppi_evidence/ramp_corridor_attempt_20261116';B=R/'bags/fast-livo-localization-z7vdr1vj';assert json.loads((B/'recorder_result.json').read_text())['exit']==0
con=sqlite3.connect('file:'+str(B/'raw/raw_0.db3')+'?mode=ro',uri=True);T={n:(i,t) for i,n,t in con.execute('select id,name,type from topics')};i,t=T['/localization/estimate'];C=get_message(t);poses={}
for raw, in con.execute('select data from messages where topic_id=?',(i,)):
 m=deserialize_message(raw,C)
 if m.state!=m.VALID:continue
 q=m.pose.orientation;position=m.pose.position;poses[m.header.stamp.sec*1000000000+m.header.stamp.nanosec]=[position.x,position.y,float(Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz')[2])]
i,t=T['/fast_livo/deskewed_imu'];C=get_message(t);frames=[]
for raw, in con.execute('select data from messages where topic_id=? order by timestamp desc limit 70',(i,)):
 m=deserialize_message(raw,C);stamp=m.header.stamp.sec*1000000000+m.header.stamp.nanosec
 if stamp in poses:frames.append((stamp,cloud_xyz(m),poses[stamp]))
 if len(frames)>=25:break
frames=frames[::-1];con.close();b,rbi,tbi=load_geometry(R/'config/fast_livo2/navigation_prep.yaml',.03);g=Grid.from_yaml(R/'outputs/fast-livo-demo-changed-scene-20261110/navigation_map_05/map.yaml');rows=[]
for height in [.10,.08,.06,.05,.03]:
 mem=ObstacleMemory(b,min_height=height,sectors=180,angular_guard=0,control_memory=2.)
 for ns,xyz,pose in frames:mem.update(xyz,rbi,tbi,pose,ns*1e-9)
 pose=np.array(frames[-1][2]);goal=np.array([2.15,.6,pose[2]]);v=validate_sweep([pose,goal],mem.points(),b,grid=g,coverage=mem.covers,margin=.07,map_margin=.04)
 delta=goal-pose;length=np.linalg.norm(delta[:2]);maximum=0.
 for d in np.arange(0.,length+.001,.01):
  q=pose+delta*min(1.,d/length);valid,_,_=validate_sweep([pose,q],mem.points(),b,grid=g,coverage=mem.covers,margin=.07,map_margin=.04)
  if not valid:break
  maximum=float(d)
 row={'height':height,'frames':len(frames),'pose':pose.tolist(),'points':len(mem.points()),'goal':goal.tolist(),'valid':v[0],'minimum':v[1],'reason':v[2],'verified_prefix_m':maximum};rows.append(row);print(row,flush=True)
(E/'ground_band_comparison.json').write_text(json.dumps(rows,indent=2))
