#!/usr/bin/env python3
"""Optional derived topics: RViz PointCloud2 and SI IMU. Never changes raw inputs."""
import argparse,copy

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--execute',action='store_true');a=p.parse_args()
    if not a.execute:print('DRY-RUN: /livox/lidar -> /livox/points_rviz; /livox/imu (g) -> /livox/imu_si (m/s^2, orientation unavailable)');return
    import rclpy
    from rclpy.qos import QoSProfile
    from livox_ros_driver2.msg import CustomMsg
    from sensor_msgs.msg import Imu,PointField,PointCloud2
    from sensor_msgs_py.point_cloud2 import create_cloud
    rclpy.init();node=rclpy.create_node('mid360_derived');qos=QoSProfile(depth=10)
    cloud_pub=node.create_publisher(PointCloud2,'/livox/points_rviz',qos);imu_pub=node.create_publisher(Imu,'/livox/imu_si',qos)
    fields=[PointField(name=n,offset=4*i,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z','intensity'))]
    fields.append(PointField(name='offset_time',offset=16,datatype=PointField.UINT32,count=1))
    def cloud(m):cloud_pub.publish(create_cloud(m.header,fields,[(p.x,p.y,p.z,float(p.reflectivity),p.offset_time) for p in m.points]))
    def imu(m):
        out=copy.deepcopy(m)
        for k in ('x','y','z'):setattr(out.linear_acceleration,k,getattr(m.linear_acceleration,k)*9.80665)
        out.linear_acceleration_covariance=[v*9.80665**2 for v in m.linear_acceleration_covariance]
        out.orientation_covariance[0]=-1.0
        imu_pub.publish(out)
    subscriptions=[node.create_subscription(CustomMsg,'/livox/lidar',cloud,qos),node.create_subscription(Imu,'/livox/imu',imu,qos)]
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
