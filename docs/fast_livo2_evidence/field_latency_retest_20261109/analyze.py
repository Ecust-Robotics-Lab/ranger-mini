"""Read-only analysis of the authorized field05 capture."""
from pathlib import Path
import collections,csv,json,sqlite3,math
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
root=Path(__file__).resolve().parents[3];capture=root/'bags/fast-livo-localization-co968xa3'
def stats(v):
 v=sorted(v)
 return {'count':len(v),'median':v[len(v)//2],'p95':v[min(len(v)-1,int(len(v)*.95))],'maximum':max(v)} if v else {}
cam=list(csv.DictReader((capture/'sensor_assets/camera_sync.csv').open()))
camera={'rows':len(cam),'statuses':dict(collections.Counter(x['status'] for x in cam))}
for key,a,b in [('sdk_wait_ms','arrival_ns','get_begin_ns'),('convert_ms','convert_end_ns','arrival_ns'),('publish_queue_ms','raw_begin_ns','convert_end_ns'),('raw_publish_ms','raw_end_ns','raw_begin_ns'),('sync_after_raw_ms','publish_ns','raw_end_ns')]:
 vals=[(int(x[a])-int(x[b]))/1e6 for x in cam if x.get(a) and int(x[a])>0];camera[key]=stats(vals)
camera['get_return_interval_ms']=stats([(int(b['arrival_ns'])-int(a['arrival_ns']))/1e6 for a,b in zip(cam,cam[1:])])
db=sqlite3.connect(f'file:{capture/"raw/raw_0.db3"}?mode=ro',uri=True)
names=['/fast_livo/odometry','/fast_livo/localizer/estimate','/navigation_ready','/localization_health','/clock','/fast_livo/navigation/wheel/odom','/wheel/odom']
types={i:(n,get_message(k)) for i,n,k in db.execute('select id,name,type from topics') if n in names}
counts=dict(db.execute('select t.name,count(*) from messages m join topics t on m.topic_id=t.id group by t.id'))
first_fast_stamp=None
series=collections.defaultdict(list);transitions=[];lastready=None;health={};lastclock=None;future=0;states=collections.Counter();confirmed=set();first_valid=None;estimate_changes=[];last_estimate=None;motion=[];private_valid=False;false_samples=[]
for tid,t,data in db.execute('select topic_id,timestamp,data from messages order by timestamp,id'):
 if tid not in types:continue
 n,k=types[tid];m=deserialize_message(data,k);series[n].append(t)
 if n=='/fast_livo/odometry' and first_fast_stamp is None:first_fast_stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
 if n=='/clock':lastclock=m.clock.sec*10**9+m.clock.nanosec
 elif n=='/fast_livo/navigation/wheel/odom':future+=int(lastclock is not None and m.header.stamp.sec*10**9+m.header.stamp.nanosec>lastclock)
 elif n=='/wheel/odom':motion.append((m.pose.pose.position.x,m.pose.pose.position.y,math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y)))
 elif n=='/localization_health':
  if m.status:health={x.key:x.value for x in m.status[0].values}
  if transitions and not transitions[-1]['ready'] and 'health_after' not in transitions[-1]:transitions[-1]['health_after']=health
  if private_valid and health.get('odom_present')=='0':false_samples.append({'t':t,'health':health})
 elif n=='/fast_livo/localizer/estimate':
  states[m.state]+=1;private_valid=m.pose_valid
  if m.pose_valid:
   confirmed.add(m.last_confirmed_stamp_ns)
   if first_valid is None:first_valid=(t,m.last_confirmed_stamp_ns)
  key=(m.state,m.reason,m.request_generation)
  if key!=last_estimate:
   q=json.loads(m.quality_json or '{}');estimate_changes.append({'t':t,'state':m.state,'reason':m.reason,'request_generation':m.request_generation,'local_recovery':q.get('local_recovery',False)})
  last_estimate=key
 elif n=='/navigation_ready' and m.data!=lastready:
  transitions.append({'t':t,'ready':m.data,'health_before':health});lastready=m.data
post_false=[]
for i,x in enumerate(transitions):
 if not x['ready'] and any(a['ready'] for a in transitions[:i]):
  following=next((a for a in transitions[i+1:] if a['ready']),None)
  post_false.append(dict(start=x['t'],duration_seconds=(following['t']-x['t'])/1e9 if following else None,health_before=x['health_before'],health_after=x.get('health_after'),private_changes_nearby=[a for a in estimate_changes if abs(a['t']-x['t'])<250_000_000]))
observers=[json.loads(s) for s in (Path(__file__).parent/'live_observer.jsonl').read_text().splitlines()]
active=[x for x in observers if x['graph']['/wheel/odom'] and x['graph']['/tf']]
output={'capture':str(capture),'read_only':True,'counts':counts,'camera':camera,'states':dict(states),'distinct_confirmed':len(confirmed),'first_valid_from_first_fast_arrival_seconds':(first_valid[0]-series['/fast_livo/odometry'][0])/1e9 if first_valid else None,'first_valid_from_first_fast_sample_seconds':(first_valid[1]-first_fast_stamp)/1e9 if first_valid else None,'first_valid_stamp':first_valid[1] if first_valid else None,'max_arrival_gap_seconds':{n:max((b-a)/1e9 for a,b in zip(v,v[1:])) for n,v in series.items() if len(v)>1},'ready_transitions':transitions,'post_ready_outages':post_false,'estimate_changes':estimate_changes,'future_wheel_against_recorded_clock':future,'private_valid_odom_absent':len(false_samples),'observer_active_samples':len(active),'tf_endpoint_count_unique_all_observed':all(len(x['graph']['/tf'])==1 for x in active),'tf_identified_samples':sum(x['graph']['/tf']==['ranger_localization_bridge'] for x in active),'tf_unknown_samples':sum(x['graph']['/tf']==['_NODE_NAME_UNKNOWN_'] for x in active),'tf_unique_all_observed':all(x['graph']['/tf']==['ranger_localization_bridge'] for x in active),'velocity_publishers_max':max([len(x['graph']['/cmd_vel'])+len(x['graph']['/base/safe_cmd_vel']) for x in observers],default=0),'wheel_integrated_path_m':sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(motion,motion[1:])),'wheel_max_speed_mps':max([x[2] for x in motion],default=0)}
print(json.dumps(output,indent=2,ensure_ascii=False));db.close()
