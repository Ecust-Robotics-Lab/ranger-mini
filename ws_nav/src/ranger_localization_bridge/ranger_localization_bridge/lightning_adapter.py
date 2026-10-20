"""Lightning-only status interpretation and sensor-to-body pose conversion.

Publishes the normalized contract; never publishes public navigation TF.
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import tf2_ros
from ranger_localization_msgs.msg import LocalizationEstimate

from .geometry import SE3, rpy_to_quat, se3_from_tf_msg
from .health import fresh


class LightningAdapter(Node):
    def __init__(self):
        super().__init__('ranger_lightning_adapter')
        d = self.declare_parameter
        self.map_frame = d('map_frame', 'map').value
        self.base_frame = d('base_frame', 'base_link').value
        self.loc_child = d('loc_child_frame', 'imu_link').value
        self.required_status = int(d('required_status', 2).value)
        self.min_confidence = float(d('min_confidence', 1.0).value)
        self.status_max_age = float(d('status_max_age_s', 10.0).value)
        self.confidence_max_age = float(d('confidence_max_age_s', 1.0).value)
        self.pose_max_age = float(d('tf_max_age_s', 0.5).value)
        xyz = d('t_base_imu_xyz', [0.0, 0.0, 0.556]).value
        rpy = d('t_base_imu_rpy', [0.0, 0.0, 0.0]).value
        if len(xyz) != 3 or len(rpy) != 3 or not all(math.isfinite(v) for v in [*xyz, *rpy]):
            raise ValueError('extrinsics must contain three finite xyz/rpy values')
        self.imu_to_base = SE3(xyz, rpy_to_quat(*rpy)).inverse()
        for value in [self.status_max_age, self.confidence_max_age, self.pose_max_age, self.min_confidence]:
            if not math.isfinite(value) or value <= 0:
                raise ValueError('adapter thresholds must be finite and positive')
        self.status = self.status_time = self.confidence = self.confidence_time = None
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.create_subscription(Int32, d('status_topic', '/lightning/loc_status').value, self.on_status, 10)
        self.create_subscription(Float32, d('confidence_topic', '/lightning/loc_confidence').value, self.on_confidence, 10)
        self.publisher = self.create_publisher(LocalizationEstimate,
            d('estimate_topic', '/localization/estimate').value, 10)
        self.create_timer(0.05, self.tick)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_status(self, msg):
        self.status, self.status_time = msg.data, self.now_s()

    def on_confidence(self, msg):
        self.confidence, self.confidence_time = msg.data, self.now_s()

    def tick(self):
        now = self.now_s()
        msg = LocalizationEstimate()
        msg.header.frame_id, msg.base_frame = self.map_frame, self.base_frame
        msg.state, msg.reason = msg.INITIALIZING, 'waiting for localization pose'
        try:
            tf = self.buffer.lookup_transform(self.map_frame, self.loc_child, rclpy.time.Time())
            pose = se3_from_tf_msg(tf) * self.imu_to_base
            msg.header.stamp = tf.header.stamp
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pose.t
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = pose.q
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            if not fresh(now, stamp, self.pose_max_age):
                msg.state, msg.reason = msg.LOST, 'Lightning pose stale or future-dated'
            elif self.status is None or self.confidence is None:
                msg.reason = 'waiting for Lightning status/confidence'
            elif not fresh(now, self.status_time, self.status_max_age):
                msg.state, msg.reason = msg.LOST, 'Lightning status expired'
            elif not fresh(now, self.confidence_time, self.confidence_max_age):
                msg.state, msg.reason = msg.LOST, 'Lightning confidence expired'
            elif not all(math.isfinite(v) for v in [*pose.t, *pose.q, self.confidence]):
                msg.state, msg.reason = msg.ERROR, 'non-finite Lightning estimate'
            elif self.status != self.required_status or self.confidence < self.min_confidence:
                msg.state, msg.reason = msg.DEGRADED, 'Lightning status/confidence rejected'
            else:
                msg.state, msg.reason = msg.VALID, 'Lightning tracking'
        except tf2_ros.TransformException:
            pass
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LightningAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
