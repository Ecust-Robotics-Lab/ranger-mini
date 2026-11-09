#!/usr/bin/env python3
"""Read-only bag clock/installation audit. Receipt time never attests physical sync."""
import argparse
import hashlib
import json
from pathlib import Path
import sqlite3
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[3]


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def bag_audit(folder, maximum_samples=120):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    info=yaml.safe_load((folder/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    rows={}
    for db in info['relative_file_paths']:
        with sqlite3.connect('file:'+str((folder/db).resolve())+'?mode=ro',uri=True) as con:
            for topic,kind,tid in con.execute('select name,type,id from topics'):
                if topic not in ['/livox/lidar','/livox/imu','/front/image_sync','/fast_livo/odometry','/wheel/odom']:continue
                ids=con.execute('select id,timestamp from messages where topic_id=? order by timestamp,id',(tid,)).fetchall()
                if not ids:continue
                row=rows.setdefault(topic,{'type':kind,'total_messages':0,'sampled_headers_ns':[],
                                           'sampled_record_ns':[],'errors':[]})
                row['total_messages']+=len(ids)
                try:
                    cls=get_message(kind)
                    for index in np.unique(np.linspace(0,len(ids)-1,min(maximum_samples,len(ids)),dtype=int)):
                        key,record=ids[index]
                        message=deserialize_message(con.execute('select data from messages where id=?',(key,)).fetchone()[0],cls)
                        stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
                        row['sampled_headers_ns'].append(stamp);row['sampled_record_ns'].append(record)
                except Exception as exc:row['errors'].append(str(exc))
    for row in rows.values():
        stamps=np.array(row.pop('sampled_headers_ns'),dtype=np.int64)
        records=np.array(row.pop('sampled_record_ns'),dtype=np.int64)
        row['sample_count']=len(stamps)
        if not len(stamps):continue
        row['header_range_ns']=[int(stamps.min()),int(stamps.max())]
        row['sampled_header_monotonic']=bool(np.all(np.diff(stamps)>0))
        offsets=(records-stamps)*1e-9
        row['record_minus_header_seconds_p1_p50_p99']=np.percentile(offsets,[1,50,99]).tolist()
        row['header_span_seconds']=int(stamps[-1]-stamps[0])*1e-9
        row['record_span_seconds']=int(records[-1]-records[0])*1e-9
    wheel=rows.get('/wheel/odom');sensor=rows.get('/livox/lidar')
    clock='MISSING_WHEEL' if not wheel else 'NO_COMPARABLE_HEADERS'
    if wheel and sensor and 'header_range_ns' in wheel and 'header_range_ns' in sensor:
        a,b=wheel['header_range_ns'],sensor['header_range_ns']
        clock='OVERLAPPING_HEADERS_NOT_SYNC_CERTIFICATE' if max(a[0],b[0])<=min(a[1],b[1]) else 'DISJOINT_HEADER_EPOCHS'
    return {'path':str(folder.resolve()),'metadata_sha256':sha(folder/'metadata.yaml'),'topics':rows,
            'wheel_sensor_clock_observation':clock,'physical_time_mapping_verified':False,
            'storage_time_is_not_sample_time':True}


def geometry_audit(root=ROOT):
    config=root/'ws_nav/src/ranger_navigation/config'
    runtime=yaml.safe_load((root/'config/runtime.yaml').read_text())
    params=yaml.safe_load((config/'nav2_params.yaml').read_text())
    collision=yaml.safe_load((config/'collision_monitor.yaml').read_text())['collision_monitor']['ros__parameters']
    vertices=np.array(collision['PolygonStop']['points']).reshape(-1,2)
    output={'t_base_imu_xyz':runtime['localization']['t_base_imu_xyz'],
            't_base_imu_rpy':runtime['localization']['t_base_imu_rpy'],
            'installation_evidence':'config/mid360/extrinsics_pending.yaml',
            'base_imu_field_calibrated':False,'base_link_origin_verified':False,
            'reference_body_length_width_m':[.74,.52],
            'stop_polygon_xy':vertices.tolist(),'costmaps':{}}
    for name in ['global_costmap','local_costmap']:
        p=params[name][name]['ros__parameters'];radius=float(p['robot_radius'])
        output['costmaps'][name]={'robot_radius':radius,'reference_body_half_width_m':.26,
                                  'circle_can_contain_reference_body':bool(radius>=np.hypot(.37,.26))}
    output['geometry_consistent_for_navigation']=all(v['circle_can_contain_reference_body'] for v in output['costmaps'].values())
    output['source_files_sha256']={str(p.relative_to(root)):sha(p) for p in [root/'config/runtime.yaml',config/'nav2_params.yaml',config/'collision_monitor.yaml',root/'config/mid360/extrinsics_pending.yaml']}
    return output


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--bag',type=Path,action='append',required=True)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    result={'schema_version':1,'kind':'navigation_input_readiness_audit','bags':[bag_audit(b) for b in args.bag],
            'geometry':geometry_audit(),'hardware_started':False,'planner_controller_changed':False,
            'field_navigation_ready':False,
            'limitations':['Header/storage comparisons observe epoch/transport behavior only, not physical synchronization.',
                           'Current FAST capture and measured base extrinsics still require their own evidence.']}
    with args.output.open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
