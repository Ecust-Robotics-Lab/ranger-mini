"""Read-only raw-cloud comparison; no ROS publishers or hardware startup."""
import sys,json,sqlite3,time,hashlib
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(root/'ws_nav/src/ranger_navigation'))
from ranger_navigation.obstacles import ObstacleMemory
from ranger_navigation.geometry import load_geometry
from ranger_navigation.coordinator import cloud_xyz
from ranger_navigation.ros_utils import stamp_ns
from ranger_localization_bridge.alignment import PoseHistory
from ranger_localization_bridge.geometry import SE3
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
bag=root/'bags/fast-livo-localization-95eecsy_/raw/raw_0.db3'
c=sqlite3.connect('file:'+str(bag)+'?mode=ro',uri=True)
ids=dict(c.execute('select name,id from topics'))
h=PoseHistory(duration_s=2000,max_samples=100000)
for blob, in c.execute('select data from messages where topic_id=? order by timestamp',(ids['/fast_livo/navigation/wheel/odom'],)):
 m=deserialize_message(blob,Odometry);p,q=m.pose.pose.position,m.pose.pose.orientation
 h.add(stamp_ns(m.header.stamp),SE3((p.x,p.y,p.z),(q.x,q.y,q.z,q.w)))
body,r,t=load_geometry(root/'config/fast_livo2/navigation_prep.yaml',.03)
memories=[ObstacleMemory(body,angular_guard=0,control_memory=v) for v in [0.,.5]]
rows=[]
for blob, in c.execute('select data from messages where topic_id=? order by timestamp limit 500',(ids['/fast_livo/deskewed_imu'],)):
 m=deserialize_message(blob,PointCloud2);ns=stamp_ns(m.header.stamp);p,reason=h.lookup(ns)
 if p is None:continue
 pose=np.array([*p.t[:2],p.yaw()]);xyz=cloud_xyz(m);entry=[]
 for memory in memories:
  began=time.perf_counter();okay=memory.update(xyz,r,t,pose,ns*1e-9);elapsed=time.perf_counter()-began
  coverage=[]
  for distance in [0.,.05,.15,.30]:
   target=pose.copy();target[:2]+=distance*np.array([np.cos(pose[2]),np.sin(pose[2])]);target[:2]-=memory.visibility_pose[:2]
   coverage.append(bool(okay and memory.control_grid.free(target,body,.045)))
  entry.append({'coverage':coverage,'filter_ms':elapsed*1000})
 rows.append(entry)
summary={'source':str(bag),'frames':len(rows),'distances_m':[0,.05,.15,.30], 'comparisons':[]}
for i in range(2):
 summary['comparisons'].append({'memory_s':[0,.5][i],'covered_frames':np.sum([row[i]['coverage'] for row in rows],axis=0).tolist(),'filter_p50_p95_ms':np.percentile([row[i]['filter_ms'] for row in rows],[50,95]).tolist()})
print(json.dumps(summary,indent=2))
(root/'docs/exact_mppi_evidence/live_changed_scene_20261110/continuity_recorded_comparison.json').write_text(json.dumps(summary,indent=2)+'\n')
