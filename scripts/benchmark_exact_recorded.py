#!/usr/bin/env python3
"""Read-only recorded FAST clouds/wheel alignment, plus real frontend contention.

No ROS publishers or devices are started. The optional frontend process is the
existing direct-callback *offline* three-fusion replay executable.
"""
import argparse,hashlib,json,os,sqlite3,subprocess,sys,time,tempfile
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'ws_nav/src/ranger_navigation'),str(ROOT/'scripts/fast_livo2/navigation')]
from ranger_navigation.geometry import load_geometry,validate_sweep
from ranger_navigation.obstacles import ObstacleMemory
from ranger_navigation.models import Mode
from ranger_navigation.ros_utils import pose2,stamp_ns
from ranger_navigation.coordinator import cloud_xyz
from ranger_localization_bridge.alignment import PoseHistory
from ranger_localization_bridge.geometry import SE3
from capture_clock import CaptureClock
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


def sha(path):
 h=hashlib.sha256()
 with Path(path).open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()


def extract(output):
 capture=ROOT/'bags/fast-livo-wheel_alignment_visit01-pwr5lx7u'
 analysis=ROOT/'outputs/fast-livo-demo-wheel-alignment-20261106/analysis_20261109_split'
 clock=CaptureClock(analysis,capture)
 db=capture/'raw/raw_0.db3'
 conn=sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True)
 ids={name:id for id,name in conn.execute('select id,name from topics')}
 wheel=[];history=PoseHistory(duration_s=200.,max_samples=11000)
 for blob, in conn.execute('select data from messages where topic_id=? order by timestamp',(ids['/wheel/odom'],)):
  m=deserialize_message(blob,Odometry);ns=clock.host_to_sensor(stamp_ns(m.header.stamp));p,q=m.pose.pose.position,m.pose.pose.orientation
  pose=SE3((p.x,p.y,p.z),(q.x,q.y,q.z,q.w));history.add(ns,pose);wheel.append(ns)
 paired=set()
 for blob, in conn.execute('select data from messages where topic_id=? order by timestamp',(ids['/fast_livo/odometry'],)):
  paired.add(stamp_ns(deserialize_message(blob,Odometry).header.stamp))
 body,r,t=load_geometry(ROOT/'config/fast_livo2/navigation_prep.yaml');memory=ObstacleMemory(body)
 arrays={};rows=[];filter_ms=[];failures={};processed=0
 for blob, in conn.execute('select data from messages where topic_id=? order by timestamp',(ids['/fast_livo/deskewed_imu'],)):
  m=deserialize_message(blob,PointCloud2);ns=stamp_ns(m.header.stamp)
  if ns not in paired:raise RuntimeError('recorded FAST pair missing')
  pose,reason=history.lookup(ns)
  if pose is None:failures[reason]=failures.get(reason,0)+1;continue
  p=np.array([*pose.t[:2],pose.yaw()]);start=time.perf_counter()
  okay=memory.update(cloud_xyz(m),r,t,p,ns*1e-9)
  filter_ms.append((time.perf_counter()-start)*1000)
  processed+=1
  if not okay:failures[memory.error]=failures.get(memory.error,0)+1;continue
  if processed%15:continue
  i=len(rows);arrays['pose_'+str(i)]=p;arrays['points_'+str(i)]=memory.sampled(512)
  arrays['full_'+str(i)]=memory.points();arrays['ranges_'+str(i)]=memory.ranges;arrays['sensor_'+str(i)]=memory.visibility_pose
  arrays['visibility_'+str(i)]=memory.control_grid.cells
  rows.append({'stamp_ns':ns,'full_points':len(memory.points()),'sampled_points':len(arrays['points_'+str(i)])})
 conn.close();np.savez(output,**arrays)
 report={'source_bag':str(capture/'raw'),'db_sha256':sha(db),'metadata_sha256':sha(capture/'raw/metadata.yaml'),
         'clock_mapping':clock.to_dict(),'strict_fast_pairs':len(paired),'processed_clouds':processed,
         'selected_queries':rows,'rejections':failures,'filter_ms':stats(filter_ms),'original_assets_modified':False}
 return report


def stats(values):
 if not len(values):return {}
 return {k:float(np.percentile(values,p)) for k,p in [('p50',50),('p95',95),('p99',99),('max',100)]}


def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--contention',action='store_true')
 p.add_argument('--extract-only',action='store_true',help='Prepare recorded inputs without running JAX')
 p.add_argument('--reuse-extraction',type=Path,help='Verified extraction.json from this source revision')
 args=p.parse_args()
 args.output.mkdir(parents=True,exist_ok=True)
 inputs=ROOT/'.runtime/exact_recorded_inputs.npz'
 if args.reuse_extraction:
  provenance=json.loads(args.reuse_extraction.read_text())
  if provenance.get('inputs_sha256')!=sha(inputs) or provenance.get('extractor_sha256')!=sha(__file__):
   raise RuntimeError('derived input or extraction source identity changed')
 else:
  provenance=extract(inputs)
  provenance.update(inputs_sha256=sha(inputs),extractor_sha256=sha(__file__))
 (args.output/'extraction.json').write_text(json.dumps(provenance,indent=2)+'\n')
 if args.extract_only:
  print(json.dumps({'processed_clouds':provenance['processed_clouds'],'selected_queries':len(provenance['selected_queries'])}))
  return 0
 from ranger_navigation.exact_controller import ExactController
 from exact_mppi.mppi_jax.logger import logger
 logger.setLevel(40)
 body,_,_=load_geometry(ROOT/'config/fast_livo2/navigation_prep.yaml')
 controller=ExactController(body);warm=controller.prewarm()
 with np.load(inputs) as data:arrays={k:data[k] for k in data.files}
 frontend=None;log=None
 if args.contention:
  folder=Path(tempfile.mkdtemp(prefix='exact-frontend-contention-',dir=ROOT/'.runtime'))
  env=dict(os.environ,ROS_LOCALHOST_ONLY='1',ROS_DOMAIN_ID='230',OMP_NUM_THREADS='4')
  env['LD_LIBRARY_PATH']=':'.join([str(ROOT/'ws_fast_livo/deps_install/lib'),
      str(ROOT/'ws_fast_livo/online_frontend_install/vikit_common/lib'),
      str(ROOT/'ws_fast_livo/install/vikit_common/lib'),
      str(ROOT/'ws_livox/install/livox_ros_driver2/lib'),env.get('LD_LIBRARY_PATH','')])
  command=[str(ROOT/'ws_fast_livo/online_frontend_install/fast_livo/lib/fast_livo/ranger_livo_replay'),
           '--ros-args','--params-file',str(ROOT/'config/fast_livo2/ranger_mid360s_hik_trial.yaml'),
           '-p','offline.bag:='+provenance['source_bag'],'-p','offline.output:='+str(folder/'result'),
           '-p','offline.max_frames:=600','--log-level','warn']
  log=(args.output/'frontend.log').open('w')
  frontend=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT)
  provenance['contention_command']=command
 compute_times=[];end_to_end=[];accepted=invalid=concurrent=0;reasons={}
 try:
  for repeat in range(3 if args.contention else 1):
   for i,row in enumerate(provenance['selected_queries']):
    pose=arrays['pose_'+str(i)];mode=Mode(i%3)
    path=np.tile(pose,(60,1))
    if mode==Mode.SPIN:path[:,2]+=np.linspace(0,.2,60)
    else:path[:,:2]+=np.linspace(0,.3,60)[:,None]*np.array([np.cos(pose[2]),np.sin(pose[2])])
    memory=ObstacleMemory(body);memory.set_visibility(arrays['sensor_'+str(i)],arrays['ranges_'+str(i)],arrays['visibility_'+str(i)])
    start=time.perf_counter()
    if frontend is not None and frontend.poll() is None:concurrent+=1
    try:
     result=controller.compute(pose,np.zeros(3),path,mode,arrays['points_'+str(i)],
                               (arrays['visibility_'+str(i)],arrays['sensor_'+str(i)]))
     computed=time.perf_counter();compute_times.append((computed-start)*1000)
     valid,_,reason=validate_sweep(result['trajectory'],arrays['full_'+str(i)],body,coverage=memory.covers,margin=body.clearance+.04)
     accepted+=int(valid);invalid+=int(not valid)
     if not valid:reasons[reason]=reasons.get(reason,0)+1
    except RuntimeError as error:
     compute_times.append((time.perf_counter()-start)*1000);invalid+=1
     reasons[str(error)]=reasons.get(str(error),0)+1
    end_to_end.append((time.perf_counter()-start)*1000)
  if frontend is not None:
   # Finish the authorized finite offline replay; do not terminate it by name.
   while frontend.poll() is None:
    time.sleep(.2)
   provenance['contention_exit']=frontend.returncode
   summary=folder/'result/summary.json'
   if summary.is_file():provenance['three_fusion_summary']=json.loads(summary.read_text())
 finally:
  if frontend is not None and frontend.poll() is None:
   frontend.terminate();frontend.wait(timeout=10)
  if log is not None:log.close()
 report={'input_provenance':provenance,'query_contract':'recorded clouds and same-capture interpolated wheel poses; explicitly stationary optimizer queries, no commands published',
         'warmup_s':warm,'compute_ms':stats(compute_times),'optimizer_plus_independent_validation_ms':stats(end_to_end),
         'accepted_stationary_queries':accepted,'rejected_stationary_queries':invalid,'queries_while_real_three_fusion_frontend_alive':concurrent,
         'rejection_reasons':reasons,
         'timing_target_met':bool(np.percentile(end_to_end,95)<=40 and np.percentile(end_to_end,99)<=50),
         'performance_target_pass':bool(accepted>0 and np.percentile(end_to_end,95)<=40 and np.percentile(end_to_end,99)<=50),
         'hardware_started':False,'physical_sync_verified':False}
 (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='input_provenance'},indent=2))
 return 0
if __name__=='__main__':raise SystemExit(main())
