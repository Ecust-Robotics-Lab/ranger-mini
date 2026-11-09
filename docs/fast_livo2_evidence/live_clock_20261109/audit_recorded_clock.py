"""Read-only recorded receipt proxy audit; bag timestamps are not live monotonic receipts."""
import json,sqlite3,struct,sys
from pathlib import Path
root=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(root/'scripts/fast_livo2/navigation'))
from live_clock import LiveClock
bag=root/'bags/fast-livo-wheel_alignment_visit01-pwr5lx7u/raw/raw_0.db3'
db=sqlite3.connect(f'file:{bag}?mode=ro',uri=True)
ids={name:id for id,name in db.execute('select id,name from topics')}
x=LiveClock('recorded-proxy');first=None;count=0;ready=0;dropped=0;residuals=[];first_fault=None
for host,tid,data in db.execute('select timestamp,topic_id,substr(data,1,12) from messages where topic_id in (?,?) order by timestamp,id',(ids['/livox/lidar'],ids['/wheel/odom'])):
    sec,ns=struct.unpack_from('<iI',data,4);stamp=sec*10**9+ns
    if first is None:first=host
    steady=host-first+10_000_000_000
    if tid==ids['/livox/lidar']:
        count+=1
        if count==1:anchor=[stamp,steady]
        x.receipts({'schema_version':1,'context':'recorded-proxy','topics':{'/livox/lidar':dict(count=count,first_header_ns=anchor[0],first_steady_ns=anchor[1],last_header_ns=stamp,last_steady_ns=steady)}},host,steady)
        if x.residual_ns is not None:residuals.append(x.residual_ns)
        if x.error and first_fault is None:first_fault=dict(elapsed=(host-first)*1e-9,error=x.error)
    else:
        converted=x.convert(stamp,host,steady)
        ready+=converted is not None;dropped+=converted is None
result=dict(kind='bag receipt proxy; not actual online/physical sync validation',lidar_samples=count,wheel_forwardable=ready,wheel_withheld=dropped,first_fault=first_fault,residual_min_max_ns=[min(residuals),max(residuals)],final_status=x.status(host,steady),source_read_only=True)
print(json.dumps(result,indent=2))
