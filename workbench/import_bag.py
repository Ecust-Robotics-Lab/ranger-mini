"""Read existing SQLite rosbag CDR offline; never initializes ROS or publishes."""
import argparse
import json
from pathlib import Path
import sqlite3
import time
from workbench.core import Assets,Journal,State
from workbench.ros_bridge import TOPICS

def import_bag(folder,output,map_folders=None):
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rclpy.serialization import deserialize_message
    folder=Path(folder); assets=Assets(map_folders)
    provenance=json.loads((folder/'provenance.json').read_text()) if (folder/'provenance.json').exists() else {}
    if (folder/'workbench_manifest.json').exists():
        context=json.loads((folder/'workbench_manifest.json').read_text())
        uuid,sha=context['map_uuid'],context['source_manifest_sha256']
    else:
        archive=Path(provenance.get('archive','/does-not-exist'))
        if not (archive/'manifest.json').is_file():raise ValueError('缺少地图来源身份')
        uuid=json.loads((archive/'manifest.json').read_text())['map_uuid']
        sha=provenance.get('archive_manifest_sha256')
    selected=[m for m in assets.maps.values() if m['map_uuid']==uuid and m['source_manifest_sha256']==sha]
    if len(selected)!=1:raise ValueError('历史记录与当前地图身份不匹配')
    m=selected[0];state=State(m['id'],m['map_uuid'])
    files=sorted(folder.glob('**/*.db3'))
    if not files:raise ValueError('没有 SQLite rosbag 数据')
    journal=Journal(output,{**assets.public(m['id']),'source_bag':str(folder.resolve()),'data_kind':'recorded_rosbag'},min_free_bytes=200_000_000)
    from tf2_ros import Buffer
    from rclpy.duration import Duration
    from rclpy.time import Time
    from workbench.transforms import observation as project
    buffer=Buffer(cache_time=Duration(seconds=30))
    extra={'/navigation/observation','/tf','/tf_static'}
    count=0;last=None
    try:
        # Chunk ordering is determined by original recording time, not filename.
        streams=[]
        for f in files:
            db=sqlite3.connect(f'file:{f}?mode=ro',uri=True)
            types={row[0]:(row[1],get_message(row[2])) for row in db.execute('select id,name,type from topics') if row[1] in TOPICS or row[1] in extra}
            if not types:db.close();continue
            cursor=db.execute('select timestamp,topic_id,data from messages where topic_id in ('+','.join('?'*len(types))+') order by timestamp,id',list(types))
            streams.append((db,types,cursor))
        import heapq
        heap=[]
        for i,(_,_,c) in enumerate(streams):
            row=c.fetchone()
            if row:heapq.heappush(heap,(row[0],i,row))
        while heap:
            t,i,row=heapq.heappop(heap);db,types,cursor=streams[i];topic,kind=types[row[1]]
            msg=deserialize_message(row[2],kind);data=message_to_ordereddict(msg)
            if topic in ('/tf','/tf_static'):
                for transform in msg.transforms:
                    if topic=='/tf_static':buffer.set_transform_static(transform,'recorded')
                    else:buffer.set_transform(transform,'recorded')
            elif topic=='/navigation/observation':
                try:
                    transform=buffer.lookup_transform('map',msg.header.frame_id,Time.from_msg(msg.header.stamp))
                    state.consume('/workbench/environment',project(data,message_to_ordereddict(transform)),t)
                except Exception:
                    state.consume('/workbench/environment',{'valid':False,'reason':'same_stamp_tf_missing'},t)
            else:state.consume(topic,data,t)
            count+=1
            if last is None or t-last>=100_000_000:
                # Offline writes may run faster than the bounded writer. Wait off the robot thread.
                while journal.q.qsize()>24 and not journal.error:time.sleep(.005)
                journal.add('snapshot',state.snapshot(t),t);last=t
            following=cursor.fetchone()
            if following:heapq.heappush(heap,(following[0],i,following))
        if last is not None:journal.add('snapshot',state.snapshot(t),t)
        for db,_,_ in streams:db.close()
    finally:journal.close()
    if journal.error:raise RuntimeError(journal.error)
    return {'messages':count,'output':str(output),'dropped':journal.dropped}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--folder',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--map',dest='maps',action='append');a=p.parse_args()
    print(json.dumps(import_bag(a.folder,a.output,a.maps)))
