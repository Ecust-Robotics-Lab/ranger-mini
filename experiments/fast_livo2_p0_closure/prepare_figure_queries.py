#!/usr/bin/env python3
"""Export unmodified real frontend pairs at 2 Hz for an offline C2 figure."""
import argparse,json,sqlite3,sys
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts/fast_livo2'))
import render_demo as rd

def main():
 p=argparse.ArgumentParser();p.add_argument('--trial',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
 args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
 trial=args.trial.resolve();capture=trial/'capture';m=json.loads((capture/'map/manifest.json').read_text())
 anchor=json.loads((capture/'clock_anchor.json').read_text())['anchor'];events=json.loads((trial/'events.json').read_text())
 begin=events['route_start_ns']-anchor[1]+anchor[0]+30_000_000_000
 end=begin+35_000_000_000
 c=rd.db_open(capture/'raw');topics={name:tid for tid,name in c.execute('SELECT id,name FROM topics')}
 odom={}
 for blob, in c.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp,id',(topics['/fast_livo/odometry'],)):
  msg=deserialize_message(blob,Odometry);p0=msg.pose.pose;stamp=rd.stamp(msg)
  odom[stamp]={'xyz':[p0.position.x,p0.position.y,p0.position.z],'xyzw':[p0.orientation.x,p0.orientation.y,p0.orientation.z,p0.orientation.w]}
 rows=[];previous=0
 for scan,(blob,) in enumerate(c.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp,id',(topics['/fast_livo/deskewed_imu'],))):
  msg=deserialize_message(blob,PointCloud2);stamp=rd.stamp(msg)
  if not begin<=stamp<=end or stamp-previous<490_000_000:continue
  assert stamp in odom and msg.header.frame_id=='fast_livo_imu'
  points=rd.cloud_xyz(msg);name=f'query_{len(rows):03d}.pcd'
  field=next(f for f in msg.fields if f.name=='intensity')
  intensity=np.ndarray((msg.height,msg.width),dtype='<f4',buffer=msg.data,offset=field.offset,
                       strides=(msg.row_step,msg.point_step)).reshape(-1)
  assert len(points)==len(intensity),'Unexpected nonfinite XYZ in recorded query'
  with (args.output/name).open('wb') as f:
   f.write((f'VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA binary\n').encode())
   f.write(np.column_stack((points,intensity)).astype('<f4').tobytes())
  rows.append({'path':name,'stamp_ns':stamp,'scan_id':scan,'U':odom[stamp]});previous=stamp
 d={'source_trial':str(trial),'selection':'fixed interval departure+30 to +65 seconds, original pairs selected at 2Hz; no query poses from map',
    'identity':{k:m['provenance'][k] for k in ('sensor_configuration_sha256','time_contract_sha256','frontend_sources_sha256')},'frames':rows}
 (args.output/'queries.json').write_text(json.dumps(d,indent=2)+'\n');print('Exported',len(rows),'real pairs')
if __name__=='__main__':main()
