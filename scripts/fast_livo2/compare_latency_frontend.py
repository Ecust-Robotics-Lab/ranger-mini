#!/usr/bin/env python3
"""Compare release opportunities in recorded receipt order, plus exact native output bytes."""
import argparse,csv,json,sqlite3
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


def native_equal(a,b,name):
    if name=='/fast_livo/odometry':
        return deserialize_message(a,Odometry)==deserialize_message(b,Odometry)
    x,y=deserialize_message(a,PointCloud2),deserialize_message(b,PointCloud2)
    if any(getattr(x,k)!=getattr(y,k) for k in x.get_fields_and_field_types() if k!='data'):
        return False
    types={1:'i1',2:'u1',3:'i2',4:'u2',5:'i4',6:'u4',7:'f4',8:'f8'}
    endian='>' if x.is_bigendian else '<'
    for f in x.fields:
        dtype=np.dtype(endian+types[f.datatype])
        shape=(x.height,x.width,f.count);strides=(x.row_step,x.point_step,dtype.itemsize)
        aa=np.ndarray(shape,dtype=dtype,buffer=x.data,offset=f.offset,strides=strides)
        bb=np.ndarray(shape,dtype=dtype,buffer=y.data,offset=f.offset,strides=strides)
        if not np.allclose(aa,bb,rtol=0.,atol=1e-7,equal_nan=True):return False
    return True


def load_native(folder):
    files=list((folder/'native_frames').glob('*.db3'))
    if len(files)!=1:raise ValueError('single segment required')
    c=sqlite3.connect(files[0].resolve().as_uri()+'?mode=ro',uri=True)
    names=dict(c.execute('select id,name from topics'));result={}
    for ns,i,data in c.execute('select timestamp,topic_id,data from messages order by timestamp,id'):
        result[(ns,names[i])]=data
    c.close();return result


def compare(baseline,candidate):
    def release(folder):
        with (folder/'frame_release.csv').open() as f:
            return {int(r['sample_ns']):int(r['input_receipt_ns']) for r in csv.DictReader(f)}
    a,b=release(baseline),release(candidate);common=sorted(a.keys()&b.keys())
    earlier=np.array([(a[k]-b[k])*1e-6 for k in common])
    intervals={}
    for name,series in [('baseline',a),('candidate',b)]:
        delta=np.diff([series[k] for k in common])*1e-6
        intervals[name]=dict(zip(['p50','p95','p99','max'],map(float,np.percentile(delta,[50,95,99,100]))))
    aa,bb=load_native(baseline),load_native(candidate);keys=aa.keys()&bb.keys()
    changed_bytes=[(ns,name) for ns,name in keys if aa[(ns,name)]!=bb[(ns,name)]]
    # CDR/PCL padding is not message data. Compare every declared point field;
    # tolerate only sub-micrometre floating roundoff, not geometric changes.
    changed=[k for k in changed_bytes if not native_equal(aa[k],bb[k],k[1])]
    return dict(baseline=str(baseline),candidate=str(candidate),matched_frames=len(common),
        frames_released_earlier=int(sum(earlier>0)),frames_released_later=int(sum(earlier<0)),
        release_advance_ms=dict(zip(['p50','p95','p99','max'],map(float,np.percentile(earlier,[50,95,99,100])))),
        input_receipt_release_interval_ms=intervals,
        baseline_frames=len(a),candidate_frames=len(b),matched_native_messages=len(keys),
        changed_native_messages=len(changed),changed_native_examples=changed[:5],
        serialized_bytes_differ=len(changed_bytes),
        point_field_atol=1e-7,
        note='Release advance is the difference in recorded-input receipt times when each frame becomes executable in direct callback replay. Not measured live wall latency. Odometry messages compare exactly; all declared cloud fields compare at absolute tolerance 1e-7, excluding CDR/PCL padding.')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('baseline',type=Path);p.add_argument('candidate',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=compare(a.baseline,a.candidate);a.output.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
