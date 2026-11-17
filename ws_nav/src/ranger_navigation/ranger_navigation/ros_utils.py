"""ROS conversions, kept out of the optimizer and deterministic task core."""
import math
import numpy as np
from geometry_msgs.msg import PoseStamped,Twist
from nav_msgs.msg import Path


def stamp_s(stamp):
    return stamp.sec+stamp.nanosec*1e-9


def stamp_ns(stamp):
    return stamp.sec*10**9+stamp.nanosec


def pose2(pose):
    p,q = pose.position,pose.orientation
    vals = [p.x,p.y,p.z,q.x,q.y,q.z,q.w]
    if not np.isfinite(vals).all() or abs(np.linalg.norm(vals[3:])-1)>1e-3:
        raise ValueError('invalid_pose_quaternion')
    return np.array([p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])


def twist_array(msg):
    return np.array([msg.linear.x,msg.linear.y,msg.angular.z])


def twist_msg(array):
    msg = Twist()
    msg.linear.x,msg.linear.y,msg.angular.z = map(float,array)
    return msg


def path_msg(poses,stamp,frame='map'):
    msg = Path()
    msg.header.frame_id,msg.header.stamp = frame,stamp
    for p in poses:
        row = PoseStamped()
        row.header = msg.header
        row.pose.position.x,row.pose.position.y = map(float,p[:2])
        row.pose.orientation.z,row.pose.orientation.w = math.sin(p[2]/2),math.cos(p[2]/2)
        msg.poses.append(row)
    return msg


def transform2(points,transform):
    c,s = math.cos(transform[2]),math.sin(transform[2])
    return np.asarray(points).reshape(-1,2)@np.array([[c,s],[-s,c]])+np.asarray(transform)[:2]
