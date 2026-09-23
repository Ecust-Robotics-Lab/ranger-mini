#!/usr/bin/env python3
"""Synthetic SQLite3 fixtures only; no ROS nodes, sensor connections, or replay."""
import importlib.util,pathlib,tempfile,json
from rosbag2_py import SequentialWriter,StorageOptions,ConverterOptions,TopicMetadata
from rclpy.serialization import serialize_message
from livox_ros_driver2.msg import CustomMsg,CustomPoint
from sensor_msgs.msg import Imu
ROOT=pathlib.Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('checker',ROOT/'scripts/check_mid360_bag.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
base=pathlib.Path(tempfile.mkdtemp(prefix='mid360-synthetic-'))
for bad in (False,True):
    folder=base/('bad' if bad else 'good');w=SequentialWriter();w.open(StorageOptions(uri=str(folder),storage_id='sqlite3'),ConverterOptions('',''))
    for topic,typ in [('/livox/lidar','livox_ros_driver2/msg/CustomMsg'),('/livox/imu','sensor_msgs/msg/Imu')]:w.create_topic(TopicMetadata(name=topic,type=typ,serialization_format='cdr'))
    for i in range(30):
        t=1789470000000000000+i*5000000;m=Imu();m.header.frame_id='livox_frame';stamp=t if not bad or i!=29 else t-100000000
        m.header.stamp.sec=stamp//10**9;m.header.stamp.nanosec=stamp%10**9;m.linear_acceleration.z=1.;w.write('/livox/imu',serialize_message(m),t+1000000)
        if i%20==0:
            c=CustomMsg();c.header=m.header;c.timebase=t;c.points=[CustomPoint(offset_time=0,x=1.),CustomPoint(offset_time=50000000,x=2.)];c.point_num=2;w.write('/livox/lidar',serialize_message(c),t+2000000)
    del w
    result=mod.audit(folder)
    assert result['total_deserialized']==32
    assert result['metadata_count_match']
    assert result['data_check']==('PARTIAL' if bad else 'PASS'),result
    print(json.dumps(dict(fixture=str(folder),expected='PARTIAL' if bad else 'PASS',actual=result['data_check'],anomalies=result['anomalies'])))
print('PASS: full deserialization/counts, IMU plausibility, and timestamp reversal rejection. Synthetic evidence only.')
