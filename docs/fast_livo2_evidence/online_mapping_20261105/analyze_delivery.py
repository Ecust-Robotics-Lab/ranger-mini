"""Read-only checks of one archived mapping run and its re-recorded inputs."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sqlite3
import struct
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'scripts/fast_livo2/backend'))
from validate_motion import check_snapshot,matrix


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('run',type=Path)
    parser.add_argument('output',type=Path)
    args=parser.parse_args();folder=args.run.resolve()
    manifest,records,geometry=check_snapshot(folder/'map')
    result=json.loads((folder/'mapping_result.json').read_text())
    source=json.loads((Path(json.loads((folder/'manifest.json').read_text())['replay_view'])/'view_manifest.json').read_text())
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry
    from ranger_fast_livo_backend.msg import BackendEstimate
    with sqlite3.connect(next((folder/'raw').glob('*.db3')).as_uri()+'?mode=ro',uri=True) as conn:
        topics=dict(conn.execute('SELECT name,id FROM topics'))
        arrivals={};payloads={}
        for name in ('/livox/lidar','/livox/imu','/front/image_sync'):
            digest=hashlib.sha256();times=[]
            for stamp,data in conn.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id',(topics[name],)):
                digest.update(data);times.append(stamp)
            payloads[name]={'count':len(times),'sha256':digest.hexdigest(),
                'matches_source':len(times)==source['counts'][name] and digest.hexdigest()==source['serialized_payload_sha256'][name]}
            delta=np.diff(np.asarray(times,dtype=np.int64))/1e9
            arrivals[name]={'maximum_recorded_arrival_gap_seconds':float(delta.max()),
                'arrival_gap_seconds_p50_p95_p99':np.percentile(delta,[50,95,99]).tolist()}
        lookup={}
        for name in ('/fast_livo/odometry','/fast_livo/deskewed_imu'):
            lookup[name]={}
            for row,prefix in conn.execute('SELECT id,substr(data,1,12) FROM messages WHERE topic_id=?',(topics[name],)):
                sec,ns=struct.unpack_from('<iI',prefix,4);stamp=sec*10**9+ns
                if stamp in lookup[name]:raise AssertionError('duplicate frontend stamp')
                lookup[name][stamp]=row
        if set(lookup['/fast_livo/odometry'])!=set(lookup['/fast_livo/deskewed_imu']):
            raise AssertionError('recorded frontend outputs are not strictly paired')
        for key in manifest['keyframes']:
            stamp=key['stamp_ns'];cloud_row=lookup['/fast_livo/deskewed_imu'][stamp]
            raw=conn.execute('SELECT data FROM messages WHERE id=?',(cloud_row,)).fetchone()[0]
            recorded=deserialize_message(raw,PointCloud2)
            archived=deserialize_message((folder/'map'/key['directory']/'scan.cdr').read_bytes(),PointCloud2)
            for name in ('header','height','width','fields','is_bigendian','point_step','row_step','is_dense','data'):
                if getattr(recorded,name)!=getattr(archived,name):raise AssertionError('archived input differs: '+name)
            raw=conn.execute('SELECT data FROM messages WHERE id=?',(lookup['/fast_livo/odometry'][stamp],)).fetchone()[0]
            odom=deserialize_message(raw,Odometry)
            if not np.allclose(matrix(key['U_local_imu']),matrix(odom.pose.pose),rtol=0,atol=1e-9):
                raise AssertionError('archived raw pose differs')
        states=Counter();versions=set();scans=set()
        for (data,) in conn.execute('SELECT data FROM messages WHERE topic_id=?',(topics['/fast_livo/backend/estimate'],)):
            message=deserialize_message(data,BackendEstimate);states[str(message.state)]+=1
            if message.pose_valid:scans.add((message.session_id,message.scan_id))
            versions.add((message.session_id,message.graph_version))
    summary={'run':str(folder),'workflow_success':result['success'],
        'mapping_complete_marker':(folder/'MAPPING_COMPLETE').is_file(),
        'geometry_checks':geometry,'re_recorded_sensor_payloads':payloads,'arrival_gaps':arrivals,
        'recorded_paired_frontend_frames':len(lookup['/fast_livo/odometry']),
        'archived_inputs_checked':len(manifest['keyframes']),
        'backend_heartbeat_states':dict(states),'unique_valid_scan_heartbeats':len(scans),
        'accepted_objective_increases':sum(
            r['graph_error_after_same_factors']>r['graph_error_before_same_factors']+1e-6
            for r in records if r['decision']=='accepted_and_optimized'),
        'physical_sync':'PARTIAL','absolute_accuracy':'NOT_EVALUATED','hardware_started':False}
    if not all(p['matches_source'] for p in payloads.values()):raise AssertionError('sensor payload coverage differs from source')
    args.output.write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
