#!/usr/bin/env python3
"""Read finalized SQLite rosbag receipts; never launches ROS or devices.

First VALID message per original sample is used, excluding heartbeat ageing.
Inter-topic lags are recorder-receipt estimates, not CPU or physical latency.
"""
import argparse
import json
from pathlib import Path
import sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def percentiles_ms(values):
    a=np.asarray(values,dtype=float)*1e-6
    return dict(zip(('p50','p95','p99','max'),map(float,np.percentile(a,[50,95,99,100])))) if len(a) else None


def audit(bag,start_ns,end_ns):
    result=json.loads((bag/'recorder_result.json').read_text())
    if result.get('exit')!=0:
        raise ValueError('finalized successful recorder required')
    databases=sorted((bag/'raw').glob('*.db3'))
    if len(databases)!=1:
        raise ValueError('audit currently requires a single SQLite segment')
    con=sqlite3.connect(databases[0].resolve().as_uri()+'?mode=ro',uri=True)
    topics={name:(i,t) for i,name,t in con.execute('select id,name,type from topics')}
    names=['/fast_livo/odometry','/fast_livo/deskewed_imu','/fast_livo/localizer/estimate',
           '/localization/estimate','/navigation/observation']
    first={};quality=[];clocks=[]
    for name in [*names,'/clock']:
        if name not in topics:continue
        i,kind=topics[name];cls=get_message(kind);samples={}
        for receipt,raw in con.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp',
                                      (i,start_ns-1_000_000_000,end_ns)):
            msg=deserialize_message(raw,cls)
            if name=='/clock':
                clocks.append((receipt,msg.clock.sec*10**9+msg.clock.nanosec));continue
            if receipt<start_ns:continue
            if hasattr(msg,'state') and msg.state!=msg.VALID:continue
            if hasattr(msg,'valid') and not msg.valid:continue
            ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            # All samples belong to this one explicitly bounded recorded session.
            if ns in samples:continue
            samples[ns]=receipt
            if name=='/fast_livo/localizer/estimate':
                quality.append(json.loads(msg.quality_json or '{}'))
        if name!='/clock':first[name]=samples
    con.close()
    clock_t=np.array([t for t,s in clocks],dtype=np.int64);clock_s=np.array([s for t,s in clocks],dtype=np.int64)
    stages={}
    for name,samples in first.items():
        receipts=np.array(sorted(samples.values()),dtype=np.int64)
        ages=[]
        for ns,t in samples.items():
            j=np.searchsorted(clock_t,t,side='right')-1
            if j>=0:ages.append(int(clock_s[j])-ns)
        stages[name]=dict(new_samples=len(samples),first_receipt_recorded_clock_age_ms=percentiles_ms(ages),
                          new_sample_receipt_interval_ms=percentiles_ms(np.diff(receipts)))
    links={}
    for a,b in [(names[0],names[1]),(names[0],names[2]),(names[2],names[3]),(names[1],names[4])]:
        aa,bb=first.get(a,{}),first.get(b,{})
        common=sorted(aa.keys()&bb.keys())
        links[a+' -> '+b]=dict(matched_samples=len(common),receipt_lag_ms=percentiles_ms([bb[n]-aa[n] for n in common]))
    timing={}
    for key in ('elapsed_ms','queued_jobs','skipped_scans','processed_scans','sample_age_seconds'):
        values=[q[key] for q in quality if isinstance(q.get(key),(float,int))]
        if values:timing[key]=dict(zip(('p50','p95','max'),map(float,np.percentile(values,[50,95,100]))))
    return dict(bag=str(bag),start_ns=start_ns,end_ns=end_ns,stages=stages,links=links,private_quality=timing,
                limitations='Recorder receipt deltas include DDS/scheduling; recorded clock ages are quantized by clock messages. First VALID samples only; no physical synchronization or CPU timing claim.')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bag',type=Path)
    p.add_argument('--window-json',type=Path,required=True,help='JSON containing receipt start/end in host epoch seconds')
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();w=json.loads(a.window_json.read_text())
    out=audit(a.bag,round(w['start']*1e9),round(w['end']*1e9))
    a.output.write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))

if __name__=='__main__':main()
