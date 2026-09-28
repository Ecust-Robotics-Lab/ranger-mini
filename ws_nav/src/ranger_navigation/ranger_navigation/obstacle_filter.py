#!/usr/bin/env python3
"""ranger_obstacle_filter: prepare the live Mid-360S cloud for anti-collision use.

Input  /livox/points_rviz (PointCloud2, frame livox_frame, ~10 Hz, derived copy
        of the CustomMsg; ground points are NOT removed here).
Output /livox/obstacles (same frame/fields).

Only frame-independent filtering is done here:
  - range gate: drop points closer than min_range or farther than max_range;
  - self-points: drop points whose horizontal distance to the sensor origin is
    below self_radius_xy (mount/body returns; they would sit inside the
    collision polygons and cause a permanent stop).

Ground/ceiling selection is deliberately left downstream where it can use the
base_link frame with correct height bands: collision_monitor min/max_height
(base frame, 0.10-0.80 m band, same band as the g2p5 mapping) and costmap
obstacle layer min/max_obstacle_height. Dynamic obstacles (people etc.) are
never filtered out here — this node only removes what cannot be a real
obstacle (too near = body, too far = beyond sensor trust range).
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
]


class ObstacleFilterNode(Node):

    def __init__(self):
        super().__init__('ranger_obstacle_filter')
        d = self.declare_parameter
        self.min_range = float(d('min_range', 0.30).value)
        self.max_range = float(d('max_range', 12.0).value)
        self.self_radius_xy = float(d('self_radius_xy', 0.35).value)

        self.pub = self.create_publisher(PointCloud2, 'obstacles_out', 10)
        self.sub = self.create_subscription(PointCloud2, 'cloud_in', self.cb, 10)
        self.dropped_pub = self.create_publisher(PointCloud2, 'dropped_debug', 10)
        self.get_logger().info(
            f'obstacle filter up: range [{self.min_range}, {self.max_range}] m, '
            f'self_radius_xy {self.self_radius_xy} m')

    def cb(self, msg):
        # 字段防御：异常/残缺点云（缺 x/y/z）丢弃并告警，不让节点崩溃——
        # 过滤器一死，CM 与局部代价地图就失去全部实时障碍输入（fail-open）。
        names = {f.name for f in msg.fields}
        if not {'x', 'y', 'z'} <= names:
            self.get_logger().warn(f'cloud without xyz fields {sorted(names)}; dropped')
            return
        fields = ('x', 'y', 'z', 'intensity') if 'intensity' in names else ('x', 'y', 'z')
        import numpy as np
        pts = np.asarray(point_cloud2.read_points(msg, field_names=fields, skip_nans=True))
        if pts.size == 0:
            # 空云也要发布（保留 header 时间戳）：下游 motion_gate 以本话题的
            # 新鲜度判定"防撞输入在流"——空障碍集是有效信息，不是断流。
            self.pub.publish(point_cloud2.create_cloud(msg.header, FIELDS[:3], []))
            return
        x, y, z = pts['x'], pts['y'], pts['z']
        inten = pts['intensity'] if 'intensity' in pts.dtype.names else np.zeros_like(x)
        r3 = np.sqrt(x * x + y * y + z * z)
        keep = (np.hypot(x, y) >= self.self_radius_xy) & (r3 >= self.min_range) & (r3 <= self.max_range)
        rows = np.column_stack((x[keep], y[keep], z[keep], inten[keep]))
        out_fields = FIELDS if 'intensity' in names else FIELDS[:3]
        self.pub.publish(point_cloud2.create_cloud(msg.header, out_fields, rows))


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for fini in (node.destroy_node, rclpy.shutdown):
            try:
                fini()
            except Exception:
                pass


if __name__ == '__main__':
    main()
