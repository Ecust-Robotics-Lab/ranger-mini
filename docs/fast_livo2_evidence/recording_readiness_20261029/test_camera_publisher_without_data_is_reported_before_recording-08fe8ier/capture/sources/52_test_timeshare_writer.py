#!/usr/bin/env python3
"""Exercise the real writer without a sensor: ownership and committed snapshots."""
import json
import mmap
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


if __name__=='__main__':
    if os.environ.get('ROS_DOMAIN_ID')!='176' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise SystemExit('Requires ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1')
    rclpy.init();node=rclpy.create_node('timeshare_offline_check')
    pub=node.create_publisher(PointCloud2,'/livox/lidar',10)
    exe=str(Path(__file__).resolve().parents[2]/'ws_camera/install/timeshare_writer/lib/timeshare_writer/timeshare_writer_node')
    result={};first=second=None
    with tempfile.TemporaryDirectory(prefix='ranger-timeshare-test-') as temp:
        share=Path(temp)/'timeshare'
        args=[exe,'--ros-args','-p','timeshare_path:='+str(share)]
        log=open(Path(temp)/'writer.log','w')
        first=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            end=time.monotonic()+8
            while time.monotonic()<end:
                rclpy.spin_once(node,timeout_sec=.05)
                if share.exists() and share.stat().st_size==16 and pub.get_subscription_count()==1:break
            else:raise RuntimeError('Writer discovery timeout')
            base=1590193436000000000
            def publish(i):
                msg=PointCloud2();t=base+i*100000000
                msg.header.stamp.sec=t//10**9;msg.header.stamp.nanosec=t%10**9;pub.publish(msg)
            publish(0)
            end=time.monotonic()+3
            while time.monotonic()<end:
                rclpy.spin_once(node,timeout_sec=.01)
                high,low=struct.unpack('qq',share.read_bytes())
                if high==2 and low==base:break
            else:raise RuntimeError('Committed first snapshot not received')
            before=share.read_bytes()
            second=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            second.wait(timeout=5)
            result['duplicate_writer_rejected']=second.returncode!=0
            result['duplicate_did_not_truncate']=share.read_bytes()==before and first.poll() is None
            accepted=0;last=0
            with share.open('rb') as f:
                mem=mmap.mmap(f.fileno(),16,access=mmap.ACCESS_READ)
                for i in range(1,101):
                    publish(i)
                    end=time.monotonic()+.01
                    while time.monotonic()<end:
                        h1=struct.unpack_from('q',mem,0)[0]
                        value=struct.unpack_from('q',mem,8)[0]
                        h2=struct.unpack_from('q',mem,0)[0]
                        if h1==h2 and h1>0 and h1%2==0:
                            if value!=base+(h1//2-1)*100000000:raise RuntimeError('Mixed committed snapshot')
                            accepted+=1;last=max(last,h1)
                        rclpy.spin_once(node,timeout_sec=.0001)
                mem.close()
            result['consistent_snapshots_checked']=accepted;result['last_sequence']=last
            result['passed']=all([result['duplicate_writer_rejected'],result['duplicate_did_not_truncate'],accepted>100,last>100])
        finally:
            for proc in (second,first):
                if proc and proc.poll() is None:
                    os.killpg(proc.pid,signal.SIGINT)
                    try:proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
            log.close();node.destroy_node();rclpy.shutdown()
        print(json.dumps(result,indent=2))
        if not result.get('passed'):raise SystemExit(1)
