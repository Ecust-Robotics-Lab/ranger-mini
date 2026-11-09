#!/usr/bin/env python3
"""Read-only static map and confirmed path display; no TF or control."""
import argparse
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Path as RosPath
from geometry_msgs.msg import PoseStamped
from ranger_fast_livo_localizer.msg import LocalizerEstimate


class View(Node):
    def __init__(self, archive):
        super().__init__('ranger_localization_view')
        with (archive/'map.pcd').open('rb') as f:
            while True:
                line = f.readline()
                if not line: raise ValueError('missing binary PCD data')
                if line.strip() == b'DATA binary': break
            points = np.frombuffer(f.read(), dtype='<f4').reshape(-1, 4)
        points = points[::max(1, (len(points)+199999)//200000)].copy()
        cloud = PointCloud2(); cloud.header.frame_id = 'map'; cloud.height = 1; cloud.width = len(points)
        cloud.fields = [PointField(name=n, offset=k*4, datatype=PointField.FLOAT32, count=1)
                        for k,n in enumerate(('x','y','z','intensity'))]
        cloud.point_step = 16; cloud.row_step = cloud.width*16; cloud.data = points.tobytes(); cloud.is_dense = True
        self.map = self.create_publisher(PointCloud2, '/fast_livo/localizer/view/map',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.map.publish(cloud)
        self.path = RosPath(); self.path.header.frame_id = 'map'
        self.publisher = self.create_publisher(RosPath, '/fast_livo/localizer/view/path', 1)
        self.identity = None; self.stamp = None
        self.create_subscription(LocalizerEstimate, '/fast_livo/localizer/estimate', self.estimate, 10)

    def estimate(self, message):
        identity = (message.map_uuid, message.session_id, message.request_generation)
        if identity != self.identity:
            self.path.poses.clear(); self.identity = identity; self.stamp = None
        if not message.pose_valid or message.state != message.VALID or message.header.stamp == self.stamp: return
        self.stamp = message.header.stamp
        pose = PoseStamped(); pose.header = message.header; pose.pose = message.pose
        self.path.header = message.header; self.path.poses.append(pose); self.path.poses = self.path.poses[-2000:]
        self.publisher.publish(self.path)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('--archive', type=Path, required=True)
    args = p.parse_args(); rclpy.init(); node = View(args.archive)
    try: rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
