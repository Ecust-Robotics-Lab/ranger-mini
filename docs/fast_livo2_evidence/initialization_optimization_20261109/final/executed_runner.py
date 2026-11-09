#!/usr/bin/env python3
"""Replay recorded FAST pairs at header-time speed in an isolated localhost domain."""
import argparse,json,os,sqlite3,subprocess,time,signal,hashlib
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--baseline',action='store_true');p.add_argument('--static',default='true',choices=['true','false']);p.add_argument('--seconds',type=float,default=45);p.add_argument('--capture',default='bags/fast-livo-localization-co968xa3');p.add_argument('--output',required=True);p.add_argument('--domain',default='223');a=p.parse_args()
root=Path.cwd();out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
(out/'executed_runner.py').write_bytes(Path(__file__).read_bytes())
os.environ.update(ROS_DOMAIN_ID=a.domain,ROS_LOCALHOST_ONLY='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
import rclpy
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock
from ranger_fast_livo_localizer.msg import LocalizerEstimate
capture=root/a.capture;db=sqlite3.connect(next((capture/'raw').glob('*.db3')).resolve().as_uri()+'?mode=ro',uri=True)
topics=dict((name,i) for i,name in db.execute('select id,name from topics'))
poses={};clouds={}
for topic,typ,dest in [('/fast_livo/odometry',Odometry,poses),('/fast_livo/deskewed_imu',PointCloud2,clouds)]:
 for data, in db.execute('select data from messages where topic_id=? order by timestamp,id',(topics[topic],)):
  m=deserialize_message(data,typ);ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec;dest[ns]=m
stamps=sorted(poses.keys()&clouds.keys());first=stamps[0];stamps=[ns for ns in stamps if (ns-first)*1e-9<=a.seconds]
rclpy.init();node=rclpy.create_node('initialization_replay');latest=[];records=[];begin=None
f=(out/'samples.jsonl').open('w')
def observe(m):
 latest.append(m)
 q=json.loads(m.quality_json);r={'wall':time.monotonic()-begin if begin else None,'state':m.state,'valid':m.pose_valid,'reason':m.reason,'stamp':m.last_confirmed_stamp_ns,'latest_stamp':m.latest_scan_stamp_ns,'quality':q};records.append(r);f.write(json.dumps(r)+'\n')
node.create_subscription(LocalizerEstimate,'/fast_livo/localizer/estimate',observe,100)
clock=node.create_publisher(Clock,'/clock',10);op=node.create_publisher(Odometry,'/fast_livo/odometry',16);cp=node.create_publisher(PointCloud2,'/fast_livo/deskewed_imu',16)
binary=root/(' .runtime/backups/initialization_20261109/localizer_node'.strip() if a.baseline else 'ws_fast_livo/online_localization_install/ranger_fast_livo_localizer/lib/ranger_fast_livo_localizer/localizer_node')
archive=root/'outputs/fast-livo-demo-online-mapping-20261105/full_replay_12/map'
identity=json.loads((archive.parent/'source_identity.json').read_text())
cmd=[str(binary),'--ros-args','--params-file','config/fast_livo2/localizer.yaml','--params-file','config/fast_livo2/relocalizer.yaml','-p','archive_path:='+str(archive),'-p','online_scheduling:=true','-p','global_descriptor:=radial_pair_multiscale_v2','-p','registration:=small_gicp']
for k in ['sensor_configuration_sha256','time_contract_sha256','frontend_sources_sha256']:cmd+=['-p',k+':='+identity[k]]
if not a.baseline:cmd+=['-p','global_stationary_initialization:='+a.static]
env=os.environ.copy()
if a.baseline:env['LD_LIBRARY_PATH']=str(root/'.runtime/backups/initialization_20261109')+':'+env['LD_LIBRARY_PATH']
proc=None
try:
 for _ in range(100):rclpy.spin_once(node,timeout_sec=.01)
 if node.get_publishers_info_by_topic('/fast_livo/localizer/estimate'):raise RuntimeError('domain occupied')
 proc=subprocess.Popen(cmd,stdout=(out/'node.log').open('w'),stderr=subprocess.STDOUT,env=env,start_new_session=True)
 deadline=time.monotonic()+35
 while not(latest and json.loads(latest[-1].quality_json).get('inputs_ready')):
  c=Clock();c.clock.sec,c.clock.nanosec=divmod(first+30000000,10**9);clock.publish(c);rclpy.spin_once(node,timeout_sec=.01)
  if proc.poll() is not None:raise RuntimeError('node exited')
  if time.monotonic()>deadline:raise TimeoutError('map readiness')
 begin=time.monotonic();index=0
 while time.monotonic()-begin<a.seconds+.5:
  elapsed=time.monotonic()-begin;ns=first+int((elapsed+.03)*1e9);c=Clock();c.clock.sec,c.clock.nanosec=divmod(ns,10**9);clock.publish(c)
  while index<len(stamps) and (stamps[index]-first)*1e-9<=elapsed:
   ns=stamps[index];op.publish(poses[ns]);cp.publish(clouds[ns]);index+=1
  rclpy.spin_once(node,timeout_sec=.003)
 valid=[r for r in records if r['valid']];solves=[r['quality'].get('elapsed_ms') for r in records if r['quality'].get('native_diagnostics',{}).get('initial_ambiguity_probe')]
 result={'hardware_started':False,'pacing':'original FAST header intervals, wall-clock 1x','capture':a.capture,'first_valid_wall':valid[0]['wall'] if valid else None,'first_valid_source':(valid[0]['stamp']-first)*1e-9 if valid else None,'valid_stamps':len({r['stamp'] for r in valid}),'initial_full_solve_ms':solves,'states':{str(s):sum(r['state']==s for r in records) for s in range(5)},'argv':cmd,'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest()}
 (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
finally:
 if proc and proc.poll() is None:
  os.killpg(proc.pid,signal.SIGINT)
  try:proc.wait(timeout=8)
  except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
 f.close();node.destroy_node();rclpy.shutdown()
