#!/usr/bin/env python3
"""Receipt-time health and bounded RViz-only accumulation of paired LIVO frames."""
import json
import math
import os
import struct
import time
from collections import OrderedDict
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField, Image, Imu
from nav_msgs.msg import Odometry, Path as RosPath
from geometry_msgs.msg import PoseStamped


class Monitor(Node):
    def __init__(self):
        super().__init__('ranger_fast_livo_view')
        self.output=Path(os.environ['RANGER_SESSION_HEALTH_FILE'])
        self.start=time.monotonic();self.last={};self.counts={};self.odom={};self.cloud={}
        self.voxels=OrderedDict();self.path=RosPath();self.pairs=0;self.sample_ns=None
        self.latest_stamp=None;self.last_map=0.;self.frames_ok=True;self.first_pose=None
        self.qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE)
        self.cloud_pub=self.create_publisher(PointCloud2,'/fast_livo/view/scan',self.qos)
        self.map_pub=self.create_publisher(PointCloud2,'/fast_livo/view/map',self.qos)
        self.path_pub=self.create_publisher(RosPath,'/fast_livo/view/path',self.qos)
        self.pose_pub=self.create_publisher(PoseStamped,'/fast_livo/view/pose',self.qos)
        for name,kind in [('/livox/lidar',PointCloud2),('/livox/imu',Imu),('/front/image_sync',Image)]:
            self.create_subscription(kind,name,lambda m,n=name:self.received(n),self.qos,raw=True)
        self.create_subscription(Odometry,'/fast_livo/odometry',self.on_odom,self.qos)
        self.create_subscription(PointCloud2,'/fast_livo/deskewed_imu',self.on_cloud,self.qos)
        self.create_timer(.2,self.health)
        self.create_timer(.5,self.publish_map)

    def received(self,name):
        self.last[name]=time.monotonic();self.counts[name]=self.counts.get(name,0)+1

    @staticmethod
    def key(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec

    def on_odom(self,m):
        self.odom[self.key(m)]=m;self.match(self.key(m))
        while len(self.odom)>10:self.odom.pop(next(iter(self.odom)))

    def on_cloud(self,m):
        self.cloud[self.key(m)]=m;self.match(self.key(m))
        while len(self.cloud)>10:self.cloud.pop(next(iter(self.cloud)))

    def match(self,k):
        if k not in self.odom or k not in self.cloud:return
        o=self.odom.pop(k);c=self.cloud.pop(k)
        if o.header.frame_id!='fast_livo_local' or o.child_frame_id!='fast_livo_imu' or c.header.frame_id!='fast_livo_imu':
            self.frames_ok=False;return
        q=o.pose.pose.orientation;t=o.pose.pose.position
        values=[q.x,q.y,q.z,q.w,t.x,t.y,t.z]
        if not all(math.isfinite(v) for v in values):self.frames_ok=False;return
        x,y,z,w=np.array(values[:4])/np.linalg.norm(values[:4])
        R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                    [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                    [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
        fields={f.name:f for f in c.fields}
        coords=[]
        for name in ('x','y','z','intensity'):
            f=fields[name]
            if f.datatype!=PointField.FLOAT32 or c.is_bigendian:
                self.frames_ok=False;return
            coords.append(np.ndarray((c.height,c.width),dtype='<f4',buffer=c.data,offset=f.offset,
                                     strides=(c.row_step,c.point_step)).reshape(-1))
        cloud=np.column_stack(coords)
        cloud[:,:3]=cloud[:,:3]@R.T+np.array([t.x,t.y,t.z])
        cloud=cloud[np.isfinite(cloud).all(axis=1)]
        self.latest_stamp=o.header.stamp;self.sample_ns=k;self.pairs+=1;self.received('paired')
        self.cloud_pub.publish(self.message(cloud))
        pose=PoseStamped();pose.header=o.header;pose.pose=o.pose.pose
        self.pose_pub.publish(pose)
        self.path.header=o.header;self.path.poses.append(pose)
        self.path.poses=self.path.poses[-5000:];self.path_pub.publish(self.path)
        if self.first_pose is None:self.first_pose=[t.x,t.y,t.z]
        if self.pairs%5==0:
            keys=np.floor(cloud[:,:3]/.08).astype(np.int32)
            for key,point in zip(keys,cloud):self.voxels[tuple(key)]=tuple(float(v) for v in point)
            while len(self.voxels)>300000:self.voxels.popitem(last=False)

    def message(self,points):
        m=PointCloud2();m.header.frame_id='fast_livo_local';m.header.stamp=self.latest_stamp
        m.height=1;m.width=len(points);m.is_dense=True;m.point_step=16;m.row_step=16*len(points)
        for i,name in enumerate(('x','y','z','intensity')):
            f=PointField();f.name=name;f.offset=4*i;f.datatype=PointField.FLOAT32;f.count=1;m.fields.append(f)
        m.data=np.asarray(points,dtype='<f4').reshape((-1,4)).tobytes();return m

    def publish_map(self):
        if self.voxels and self.latest_stamp is not None:
            self.map_pub.publish(self.message(list(self.voxels.values())))

    def health(self):
        now=time.monotonic()
        alive=lambda n:now-self.last.get(n,-1e9)<1.5
        checks={'probe_live':now-self.start>2,
                'lidar':alive('/livox/lidar') and alive('/livox/imu'),
                'camera':alive('/front/image_sync'),
                'frontend':alive('paired') and self.pairs>=3 and self.frames_ok}
        for key,topic in [('lidar_unclaimed','/livox/lidar'),('camera_unclaimed','/front/image_sync'),('frontend_unclaimed','/fast_livo/odometry')]:
            checks[key]=not self.get_publishers_info_by_topic(topic)
        share=Path(os.environ['RANGER_LIVO_TIMESHARE'])
        try:
            data=share.read_bytes();high,low=struct.unpack('qq',data)
            checks['writer']=high>0 and high%2==0 and low>0
        except (OSError,ValueError,struct.error):checks['writer']=False
        state={'monotonic':now,'checks':checks,'counts':self.counts,'paired_frames':self.pairs,
               'map_points':len(self.voxels),'sample_ns':self.sample_ns,'first_pose':self.first_pose,
               'frame_contract_ok':self.frames_ok,'physical_sync_proven':False,'map_optimized':False}
        tmp=self.output.with_suffix('.tmp');tmp.write_text(json.dumps(state,indent=2)+'\n');tmp.replace(self.output)
        if self.pairs and self.pairs%50<3:
            with (self.output.parent/'metrics.jsonl').open('a') as f:f.write(json.dumps(state)+'\n')


if __name__=='__main__':
    rclpy.init();n=Monitor()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.shutdown()
