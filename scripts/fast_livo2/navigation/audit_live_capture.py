#!/usr/bin/env python3
"""Read-only functional summary of a public localization capture; no accuracy verdict."""
import argparse
import collections
import json
from pathlib import Path
import sqlite3

TOPICS={'/wheel/odom','/fast_livo/navigation/wheel/odom','/fast_livo/navigation/clock_status',
        '/fast_livo/localizer/estimate','/navigation_ready','/tf','/localization_health'}

def summarize(records):
    counts=collections.Counter();states=collections.Counter();confirmed=set();errors=set();edges=collections.Counter()
    ready_count=0;transitions=[];last_ready=None;clock_ready=0;locked=False;last_stamp=None;stamp_regressions=0
    for name,msg in records:
        counts[name]+=1
        if name=='/fast_livo/localizer/estimate':
            states[str(msg.state)]+=1
            if msg.pose_valid:confirmed.add(msg.last_confirmed_stamp_ns)
        elif name=='/navigation_ready':
            ready_count+=bool(msg.data)
            if msg.data!=last_ready:transitions.append(bool(msg.data));last_ready=bool(msg.data)
        elif name=='/fast_livo/navigation/clock_status':
            status=json.loads(msg.data);clock_ready+=bool(status.get('ready'))
            if status.get('error'):errors.add(status['error'])
        elif name=='/localization_health':
            for row in msg.status:
                values={v.key:v.value for v in row.values}
                locked|=values.get('innovation_unlocked')=='0'
        elif name=='/fast_livo/navigation/wheel/odom':
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            if last_stamp is not None and stamp<=last_stamp:stamp_regressions+=1
            last_stamp=stamp
        elif name=='/tf':
            for transform in msg.transforms:edges[transform.header.frame_id+'->'+transform.child_frame_id]+=1
    missing=sorted(TOPICS-set(counts))
    observed=bool(not missing and ready_count and confirmed and clock_ready and
                  edges['map->odom'] and edges['odom->base_link'] and not errors and not locked and not stamp_regressions)
    return dict(functional_result='OBSERVED' if observed else 'INCOMPLETE_OR_REJECTED',
                physical_sync='NOT_VALIDATED',absolute_accuracy='NOT_VALIDATED',counts=dict(counts),
                missing_topics=missing,private_states=dict(states),distinct_confirmed_samples=len(confirmed),
                ready_true_messages=ready_count,ready_transitions=transitions,clock_ready_messages=clock_ready,
                clock_errors=sorted(errors),innovation_latched=locked,converted_wheel_regressions=stamp_regressions,
                tf_edges=dict(edges),note='Transitions alone do not establish fault injection or recovery acceptance; TF publisher uniqueness needs live graph evidence.')


def audit(folder):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    databases=sorted((Path(folder)/'raw').glob('*.db3'))
    if not databases:raise ValueError('no finalized raw sqlite3 bag found')
    def records():
        for path in databases:
            db=sqlite3.connect(f'file:{path.resolve()}?mode=ro',uri=True)
            try:
                types={id:(name,get_message(kind)) for id,name,kind in db.execute('select id,name,type from topics') if name in TOPICS}
                if not types:continue
                placeholders=','.join('?' for _ in types)
                for id,data in db.execute(f'select topic_id,data from messages where topic_id in ({placeholders}) order by timestamp,id',tuple(types)):
                    name,kind=types[id];yield name,deserialize_message(data,kind)
            finally:db.close()
    result=summarize(records());result['source_capture']=str(Path(folder).resolve());result['read_only']=True
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--capture',type=Path,required=True);args=p.parse_args()
    result=audit(args.capture);print(json.dumps(result,indent=2));return 0 if result['functional_result']=='OBSERVED' else 2
if __name__=='__main__':raise SystemExit(main())
