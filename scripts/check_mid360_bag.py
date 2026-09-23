#!/usr/bin/env python3
"""Offline full-deserialization audit; input bag is opened read-only, never replayed."""
import argparse,collections,json,math,pathlib,sqlite3
import numpy as np
import yaml

def span_stats(values):
    a=np.asarray(values,dtype=float)
    if not len(a):return None
    return dict(min=float(a.min()),max=float(a.max()),mean=float(a.mean()),p50=float(np.median(a)),p99=float(np.quantile(a,.99)))

def audit(folder):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rosbag2_py import SequentialReader,StorageOptions,ConverterOptions
    metadata=yaml.safe_load((folder/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    if metadata['storage_identifier']!='sqlite3':raise ValueError('Expected sqlite3')
    integrity={}
    for name in metadata['relative_file_paths']:
        path=(folder/name).resolve()
        if not path.is_relative_to(folder.resolve()):raise ValueError('Bag path escapes directory')
        with sqlite3.connect(path.as_uri()+'?mode=ro',uri=True) as db:
            integrity[name]=db.execute('PRAGMA integrity_check').fetchall()
    reader=SequentialReader();reader.open(StorageOptions(uri=str(folder),storage_id='sqlite3'),ConverterOptions('',''))
    types={t.name:t.type for t in reader.get_all_topics_and_types()}
    classes={t:get_message(k) for t,k in types.items()}
    counts=collections.Counter();stamps=collections.defaultdict(list);arrivals=collections.defaultdict(list);frames=collections.defaultdict(set)
    point_counts=[];offset_min=[];offset_max=[];acc=[];gyro=[];orientations=collections.Counter();cov0=collections.Counter();tf=[]
    invalid=0;point_total=0;near=0;nonmono_offsets=0;timebase_mismatch=0;length_mismatch=0;bins=np.zeros(36,dtype=np.int64);bounds=None;nonfinite_imu=0
    while reader.has_next():
        topic,data,t=reader.read_next();m=deserialize_message(data,classes[topic]);counts[topic]+=1;arrivals[topic].append(t)
        if hasattr(m,'header'):
            stamps[topic].append(m.header.stamp.sec*10**9+m.header.stamp.nanosec);frames[topic].add(m.header.frame_id)
        if topic=='/livox/lidar':
            if types[topic]!='livox_ros_driver2/msg/CustomMsg':raise ValueError('Wrong raw cloud type')
            point_counts.append(m.point_num);point_total+=len(m.points)
            length_mismatch+=int(m.point_num!=len(m.points));timebase_mismatch+=int(m.timebase!=stamps[topic][-1])
            if not m.points:continue
            offsets=np.array([p.offset_time for p in m.points],dtype=np.int64)
            offset_min.append(int(offsets.min()));offset_max.append(int(offsets.max()));nonmono_offsets+=int(np.any(np.diff(offsets)<0))
            xyz=np.array([(p.x,p.y,p.z) for p in m.points]);valid=np.isfinite(xyz).all(axis=1);invalid+=int((~valid).sum());xyz=xyz[valid]
            r=np.linalg.norm(xyz,axis=1);near+=int((r<.1).sum());xyz=xyz[r>=.1]
            if len(xyz):
                b=np.floor((np.arctan2(xyz[:,1],xyz[:,0])+np.pi)/(2*np.pi)*36).astype(int)%36
                bins+=np.bincount(b,minlength=36)
                lo=xyz.min(axis=0);hi=xyz.max(axis=0)
                bounds=[lo,hi] if bounds is None else [np.minimum(bounds[0],lo),np.maximum(bounds[1],hi)]
        elif topic=='/livox/imu':
            av=[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z];gv=[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z]
            if not all(math.isfinite(x) for x in av+gv):nonfinite_imu+=1
            acc.append(av);gyro.append(gv)
            orientations[str([m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w])]+=1
            cov0[str(m.orientation_covariance[0])]+=1
        elif topic in ('/tf','/tf_static'):
            for x in m.transforms:tf.append(dict(topic=topic,parent=x.header.frame_id,child=x.child_frame_id))
    topic_stats={}
    anomalies=[]
    for topic,n in counts.items():
        ts=np.array(stamps[topic],dtype=np.int64);rx=np.array(arrivals[topic],dtype=np.int64);dt=np.diff(ts)/1e9
        item=dict(type=types[topic],count=n,frame_ids=sorted(frames[topic]),bag_arrival_hz=(n-1)*1e9/(int(rx[-1])-int(rx[0])) if n>1 and rx[-1]>rx[0] else None)
        if len(ts):
            item.update(header_first_ns=int(ts[0]),header_last_ns=int(ts[-1]),header_dt_seconds=span_stats(dt),header_backwards=int((dt<0).sum()),header_duplicates=int((dt==0).sum()),header_hz=(len(ts)-1)*1e9/(int(ts[-1])-int(ts[0])) if len(ts)>1 and ts[-1]>ts[0] else None,arrival_minus_header_seconds=span_stats((rx-ts)/1e9))
            if topic in ('/livox/lidar','/livox/imu'):
                limit=.5 if topic.endswith('lidar') else .1
                if (dt<=0).any() or (dt>limit).any():anomalies.append(topic+': nonmonotonic or large header gap')
                if np.any(np.abs((rx-ts)/1e9)>1):anomalies.append(topic+': clock-domain offset >1 s')
        topic_stats[topic]=item
    meta_counts={x['topic_metadata']['name']:x['message_count'] for x in metadata['topics_with_message_count']}
    metadata_match=all(counts[t]==n for t,n in meta_counts.items()) and sum(counts.values())==metadata['message_count']
    for t in ('/livox/lidar','/livox/imu'):
        if counts[t]<2:anomalies.append('Missing/insufficient '+t)
    if invalid or nonfinite_imu or length_mismatch or timebase_mismatch:anomalies.append('Invalid numeric fields or cloud length/timebase mismatch')
    if offset_max and max(offset_max)>.5e9:anomalies.append('Point offset range >0.5 s')
    if nonmono_offsets:anomalies.append('Point offset order reversals')
    if not metadata_match or any(v!=[('ok',)] for v in integrity.values()):anomalies.append('Bag consistency/integrity failure')
    imu={}
    if acc:
        acc=np.array(acc);gyro=np.array(gyro)
        imu=dict(acceleration_units='g, unmodified upstream raw output; NOT SI m/s^2',angular_velocity_units='rad/s',acc_mean_xyz=acc.mean(axis=0).tolist(),acc_std_xyz=acc.std(axis=0).tolist(),acc_norm=span_stats(np.linalg.norm(acc,axis=1)),gyro_mean_xyz=gyro.mean(axis=0).tolist(),gyro_std_xyz=gyro.std(axis=0).tolist(),gyro_norm=span_stats(np.linalg.norm(gyro,axis=1)),orientation_samples=dict(orientations),orientation_covariance_0=dict(cov0),orientation_valid=False)
        if not .8<float(np.linalg.norm(acc.mean(axis=0)))<1.2 or float(np.linalg.norm(gyro.mean(axis=0)))>.1:anomalies.append('Static IMU plausibility check failed')
    return dict(scope='Static raw data and bag validation; not calibration, hardware synchronization, SLAM or motion safety',data_check='PASS' if not anomalies else 'PARTIAL',anomalies=anomalies,sqlite_integrity=integrity,metadata_count_match=metadata_match,total_deserialized=sum(counts.values()),topics=topic_stats,point_cloud=dict(total_points=point_total,points_per_frame=span_stats(point_counts),offset_unit='ns',offset_min_ns=span_stats(offset_min),offset_max_ns=span_stats(offset_max),nonmonotonic_offset_frames=nonmono_offsets,header_timebase_mismatches=timebase_mismatch,invalid_points=invalid,below_0p1m_points=near,azimuth_10degree_bins_from_minus_pi=bins.tolist(),xyz_bounds_m=None if bounds is None else [b.tolist() for b in bounds],occlusion_conclusion='Sector statistics alone cannot distinguish hardware obstruction from scene; human confirmation required'),imu=imu,tf_edges=[dict(t) for t in {tuple(sorted(x.items())) for x in tf}],limitations=['Raw IMU orientation is not a pose','No base_link installation TF is fabricated','Header proximity to wall clock does not prove PTP/GPS synchronization'])

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bag',type=pathlib.Path);p.add_argument('--output',type=pathlib.Path);a=p.parse_args()
    result=audit(a.bag)
    if a.output:
        with a.output.open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
    print(json.dumps(result,indent=2,allow_nan=False))
    return 0 if result['data_check']=='PASS' else 2
if __name__=='__main__':raise SystemExit(main())
