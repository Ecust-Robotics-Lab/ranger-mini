#!/usr/bin/env python3
"""Create a NEW header-clock playback view; preserve every serialized payload."""
import argparse
import hashlib
import json
import sqlite3
from pathlib import Path

import rosbag2_py
from audit_bag import header


def create_view(source, output, seconds):
    if output.exists():
        raise ValueError('Output must be new')
    dbs=list(source.glob('*.db3'))
    if len(dbs)!=1:
        raise ValueError('Expected single SQLite bag')
    selected={'/livox/lidar','/livox/imu','/front/image_sync'}
    with sqlite3.connect(dbs[0].resolve().as_uri()+'?mode=ro',uri=True) as conn:
        topics={r[0]:r[1:] for r in conn.execute('SELECT id,name,type,serialization_format,offered_qos_profiles FROM topics') if r[1] in selected}
        if {t[0] for t in topics.values()} != selected:
            raise ValueError('All three sensor topics required')
        rows=[]
        for topic_id in topics:
            for mid,prefix in conn.execute('SELECT id,substr(data,1,256) FROM messages WHERE topic_id=? ORDER BY timestamp,id',(topic_id,)):
                t,_=header(prefix);rows.append((t,mid,topic_id))
        rows.sort()
        start=min(t for t,_,i in rows if topics[i][0]=='/livox/lidar')
        end=start+int(seconds*1e9)
        writer=rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(output),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
        for name,kind,fmt,qos in topics.values():
            writer.create_topic(rosbag2_py.TopicMetadata(name=name,type=kind,serialization_format=fmt,offered_qos_profiles=qos))
        counts={name:0 for name in selected};digests={name:hashlib.sha256() for name in selected}
        for t,mid,topic_id in rows:
            if not start<=t<=end: continue
            data=conn.execute('SELECT data FROM messages WHERE id=?',(mid,)).fetchone()[0]
            name=topics[topic_id][0];writer.write(name,data,t)
            counts[name]+=1;digests[name].update(data)
        del writer
    # Independently read the new file and verify no serialized byte changed.
    check={name:hashlib.sha256() for name in selected}
    with sqlite3.connect(next(output.glob('*.db3')).resolve().as_uri()+'?mode=ro',uri=True) as conn:
        for name,data,t in conn.execute('SELECT t.name,m.data,m.timestamp FROM messages m JOIN topics t ON m.topic_id=t.id ORDER BY m.timestamp,m.id'):
            stamp,_=header(data)
            if stamp != t: raise ValueError('Playback view record/header mismatch')
            check[name].update(data)
    original={k:v.hexdigest() for k,v in digests.items()}
    copied={k:v.hexdigest() for k,v in check.items()}
    if copied != original: raise ValueError('Serialized payload changed')
    manifest={'source':str(source.resolve()),'output':str(output.resolve()),'start_ns':start,'end_ns':end,
              'counts':counts,'serialized_payload_sha256':original,'payloads_identical':True,
              'record_time':'original message header, only in this new bag',
              'header_or_point_timestamp_modified':False,'physical_sync_proven':False}
    (output.parent/'view_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('source',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--seconds',type=float,default=6)
    args=p.parse_args()
    if args.seconds <= 0: p.error('seconds must be positive')
    print(json.dumps(create_view(args.source,args.output,args.seconds),indent=2))
