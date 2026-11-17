"""Read finalized bags only; same-stamp IMU cloud + full 6DoF localization."""
import json,sqlite3,sys,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path('/home/lab307/ranger_nav');sys.path[:0]=[str(ROOT/'ws_nav/src/ranger_navigation'),str(ROOT/'ws_nav/src/ranger_localization_bridge')]
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from ranger_navigation.coordinator import cloud_xyz
from ranger_navigation.geometry import load_geometry
E=ROOT/'docs/exact_mppi_evidence/door_ramp_diagnosis_20261116';body,rbi,tbi=load_geometry(ROOT/'config/fast_livo2/navigation_prep.yaml',.03)
rows=[]
for name in ['snwbprb6','d_pha29a']:
 bag=ROOT/('bags/fast-livo-localization-'+name);assert json.loads((bag/'recorder_result.json').read_text())['exit']==0
 db=bag/'raw/raw_0.db3';con=sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True);topics={n:(i,t) for i,n,t in con.execute('select id,name,type from topics')}
 idx,kind=topics['/localization/estimate'];cls=get_message(kind);poses={};ordered=[]
 for ts,raw in con.execute('select timestamp,data from messages where topic_id=? order by timestamp',(idx,)):
  m=deserialize_message(raw,cls)
  if m.state!=m.VALID:continue
  stamp=m.header.stamp.sec*1000000000+m.header.stamp.nanosec;q=m.pose.orientation;p=m.pose.position
  rot=Rotation.from_quat([q.x,q.y,q.z,q.w]);position=np.array([p.x,p.y,p.z]);poses[stamp]=(position,rot)
  ordered.append([stamp,p.x,p.y,p.z,*rot.as_euler('xyz')])
 idx,kind=topics['/fast_livo/deskewed_imu'];cls=get_message(kind);world=[];base=[];paired=[]
 for ts,raw in con.execute('select timestamp,data from messages where topic_id=? order by timestamp desc limit 80',(idx,)):
  m=deserialize_message(raw,cls);stamp=m.header.stamp.sec*1000000000+m.header.stamp.nanosec
  if stamp not in poses:continue
  pos,rot=poses[stamp];xyz=cloud_xyz(m);xyz=xyz[np.isfinite(xyz).all(1)];xyz=xyz@rbi.T+tbi
  world.append(rot.apply(xyz)+pos);base.append(xyz);paired.append(stamp)
  if len(paired)>=30:break
 con.close();world=np.vstack(world);base=np.vstack(base);a=np.array(ordered);floor_ref=float(np.median(a[-100:,3]));profile=[]
 for low in np.arange(0.,2.6,.1):
  select=(world[:,0]>=1.8)&(world[:,0]<=2.4)&(world[:,1]>=low)&(world[:,1]<low+.1)&(world[:,2]>floor_ref-.3)&(world[:,2]<floor_ref+.3)
  z=world[select,2]
  if len(z)>=10:profile.append({'map_y_center_m':float(low+.05),'points':len(z),'z_q10_q50_q90_m':np.percentile(z,[10,50,90]).tolist(),'relative_q10_m':float(np.percentile(z,10)-floor_ref)})
 row={'bag':str(bag),'paired_original_stamps':paired,'floor_reference_from_base_z_m':floor_ref,'pose_first_last':a[[0,-1]].tolist(),'roll_pitch_deg_p5_p50_p95':np.percentile(np.degrees(a[:,4:6]),[5,50,95],axis=0).tolist(),'map_height_p5_p50_p95_m':np.percentile(a[:,3],[5,50,95]).tolist(),'door_strip_x_m':[1.8,2.4],'raw_height_profile':profile,'notes':['Low height quantiles are not a certified ground segmentation; walls/objects/occlusion may contaminate bins.','Full 6DoF pose and exact original stamp matching used; no latest TF or repeated LiDAR extrinsic.','No production parameters changed and no devices started.']}
 rows.append(row);np.savez_compressed(E/(name+'_clouds.npz'),world=world[(world[:,0]>1)&(world[:,0]<4)&(world[:,1]>-.5)&(world[:,1]<3.5)],base=base,poses=a)
 print(name,'matched',len(paired),'pitchroll',row['roll_pitch_deg_p5_p50_p95'],'zrange',row['map_height_p5_p50_p95_m'],flush=True)
 print('profile',[(round(x['map_y_center_m'],2),x['points'],round(x['relative_q10_m'],3)) for x in profile],flush=True)
(E/'terrain_diagnosis.json').write_text(json.dumps({'user_reported_drop_m':.06,'user_reported_length_m':.50,'approx_slope_degrees':float(np.degrees(np.arctan(.06/.5))),'geometry_sha256':hashlib.sha256((ROOT/'config/fast_livo2/navigation_prep.yaml').read_bytes()).hexdigest(),'datasets':rows},indent=2))
