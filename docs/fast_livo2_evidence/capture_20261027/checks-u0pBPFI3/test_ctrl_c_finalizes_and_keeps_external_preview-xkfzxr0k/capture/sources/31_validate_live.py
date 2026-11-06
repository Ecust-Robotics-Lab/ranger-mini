#!/usr/bin/env python3
"""Read-only short observation of live input/output rates in domain 175."""
import argparse
import collections
import json
import os
import struct
import time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import PointCloud2,Imu,Image
from nav_msgs.msg import Odometry

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True)
    args=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='175' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':p.error('Requires domain 175 localhost')
    rclpy.init();n=rclpy.create_node('fast_livo_live_validation')
    topics=[('/livox/lidar',PointCloud2),('/livox/imu',Imu),('/front/image_sync',Image),('/fast_livo/odometry',Odometry)]
    data=collections.defaultdict(list)
    def callback_for(name):
        def cb(raw):
            sec,ns=struct.unpack_from('<iI',raw,4);data[name].append((time.monotonic(),sec*10**9+ns))
        return cb
    for name,kind in topics:
        n.create_subscription(kind,name,callback_for(name),QoSProfile(depth=2000,reliability=ReliabilityPolicy.RELIABLE),raw=True)
    try:
        end=time.monotonic()+2
        while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
        data.clear();start=time.monotonic();end=start+5
        while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.005)
        result={'seconds':time.monotonic()-start,'topics':{}}
        for name,_ in topics:
            arr=data[name];stamps=[x[1] for x in arr]
            result['topics'][name]={'count':len(arr),'sensor_rate_hz':(len(arr)-1)/((stamps[-1]-stamps[0])*1e-9) if len(arr)>1 else None,
                'nonincreasing':sum(b<=a for a,b in zip(stamps,stamps[1:])),
                'publishers':[i.node_name for i in n.get_publishers_info_by_topic(name)]}
        result['image_lidar_equal_headers']=len(set(x[1] for x in data['/front/image_sync']) & set(x[1] for x in data['/livox/lidar']))
        result['tf_publishers']=[i.node_name for i in n.get_publishers_info_by_topic('/tf')]
        result['base_cmd_publishers']=[i.node_name for t in ['/cmd_vel','/base/safe_cmd_vel'] for i in n.get_publishers_info_by_topic(t)]
        result['physical_sync_proven']=False
        with args.out.open('x') as f:json.dump(result,f,indent=2);f.write('\n')
        print(json.dumps(result,indent=2))
    finally:n.destroy_node();rclpy.shutdown()
