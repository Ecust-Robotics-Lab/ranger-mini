#!/usr/bin/env python3
"""ranger_obstacle_filter: prepare the live Mid-360S cloud for anti-collision use.

2026-09-30 治本改版：直接订阅 /livox/lidar 的 livox CustomMsg（不再依赖
mid360_derived 的 PointCloud2 中转——该可视化节点曾以 73% CPU 卡在安全链
上游，是定位 TF 滞后 1.7s 的主要负载源），并按 every_n 抽帧降频（默认每 3
帧取 1 ≈ 3.3 Hz；0.10 m/s 时帧间位移 ~3 cm，防撞足够）。空云也发布（新鲜度
心跳）。兼容模式 input_type=pointcloud2 供集成测试注入合成云。

Input  /livox/lidar (livox_ros_driver2/CustomMsg, ~10 Hz, frame livox_frame)
Output /livox/obstacles (PointCloud2, 同 frame)

过滤（只删不可能是障碍的点，行人等动态障碍永远保留）：
  - range: 3D 距离在 [min_range, max_range] 外的丢弃；
  - self: 传感器原点水平 self_radius_xy 内的自身/支架回波丢弃；
  - 地面/天花板由下游在 base 系高度带处理（CM 0.10-0.80 m + 局部代价地图）。
"""

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
        self.every_n = int(d('every_n', 3).value)          # 抽帧：每 N 帧处理 1 帧
        self.input_type = str(d('input_type', 'custom').value)  # custom|pointcloud2
        self._frame_count = 0

        self.pub = self.create_publisher(PointCloud2, 'obstacles_out', 10)

        if self.input_type == 'custom':
            from livox_ros_driver2.msg import CustomMsg
            self.get_logger().info(
                f'obstacle filter up: CustomMsg 直订, every_n={self.every_n} '
                f'(≈{10.0/max(self.every_n,1):.1f}Hz), range '
                f'[{self.min_range},{self.max_range}], self_r={self.self_radius_xy}')
            self.sub = self.create_subscription(CustomMsg, 'cloud_in', self.cb_custom, 10)
        else:
            self.get_logger().info(f'obstacle filter up: pointcloud2 兼容模式')
            self.sub = self.create_subscription(PointCloud2, 'cloud_in', self.cb_pc2, 10)

    def _publish_filtered(self, header, xs, ys, zs, inten):
        import numpy as np
        x = np.asarray(xs); y = np.asarray(ys); z = np.asarray(zs)
        r3 = np.sqrt(x * x + y * y + z * z)
        keep = ((np.hypot(x, y) >= self.self_radius_xy) &
                (r3 >= self.min_range) & (r3 <= self.max_range))
        rows = np.column_stack((x[keep], y[keep], z[keep],
                                np.asarray(inten)[keep] if len(inten) == len(x)
                                else np.zeros(int(keep.sum()))))
        self.pub.publish(point_cloud2.create_cloud(header, FIELDS, rows))

    def cb_custom(self, msg):
        # 抽帧 + 空云心跳：未选中帧也要发布空云（保持下游新鲜度节奏一致）
        self._frame_count += 1
        if self.every_n > 1 and (self._frame_count % self.every_n) != 1:
            empty = point_cloud2.create_cloud(msg.header, FIELDS[:3], [])
            self.pub.publish(empty)
            return
        xs, ys, zs, inten = [], [], [], []
        for p in msg.points:
            xs.append(p.x); ys.append(p.y); zs.append(p.z)
            inten.append(float(p.reflectivity))
        self._publish_filtered(msg.header, xs, ys, zs, inten)

    def cb_pc2(self, msg):
        import numpy as np
        pts = np.asarray(point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True))
        if pts.size == 0:
            self.pub.publish(point_cloud2.create_cloud(msg.header, FIELDS[:3], []))
            return
        names = pts.dtype.names or ()
        inten = pts['intensity'] if 'intensity' in names else []
        self._publish_filtered(msg.header, pts['x'], pts['y'], pts['z'], inten)


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
