"""Full cloud comparison ignores serialization padding, but catches changed points."""
from pathlib import Path
import importlib.util
import sys
import struct
import pytest
pytest.importorskip('rclpy')
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2,PointField
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('compare_latency',ROOT/'scripts/fast_livo2/compare_latency_frontend.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def cloud(padding=0,x=1.):
    m=PointCloud2();m.header.frame_id='fast_livo_imu';m.header.stamp.sec=10
    m.width=m.height=1;m.point_step=m.row_step=16
    m.fields=[PointField(name=n,offset=o,datatype=PointField.FLOAT32,count=1) for n,o in [('x',0),('y',4),('z',8)]]
    m.data=list(struct.pack('<fffI',x,2.,3.,padding))
    return serialize_message(m)


def test_cloud_padding_is_not_a_geometric_change():
    assert module.native_equal(cloud(),cloud(padding=123),'/fast_livo/deskewed_imu')


def test_changed_coordinate_or_original_stamp_is_rejected():
    assert not module.native_equal(cloud(),cloud(x=1.001),'/fast_livo/deskewed_imu')
    from rclpy.serialization import deserialize_message
    m=deserialize_message(cloud(),PointCloud2);m.header.stamp.sec+=1
    assert not module.native_equal(cloud(),serialize_message(m),'/fast_livo/deskewed_imu')
