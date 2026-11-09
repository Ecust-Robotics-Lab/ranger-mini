#!/usr/bin/env python3
"""Give an IMU-reference map pose to the existing private Initialize service."""
import argparse
import math
import json
import os
import time
import uuid

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--xyz', nargs=3, type=float, required=True)
p.add_argument('--rpy', nargs=3, type=float, default=[0.,0.,0.], help='radians')
p.add_argument('--domain', type=int, default=215)
args = p.parse_args()
if not 1 <= args.domain <= 232 or not all(math.isfinite(v) for v in args.xyz+args.rpy):
    p.error('finite pose and isolated domain required')
os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1')
import rclpy
from ranger_fast_livo_localizer.msg import LocalizerEstimate
from ranger_fast_livo_localizer.srv import Initialize

rclpy.init(); node=rclpy.create_node('ranger_manual_initial_pose');latest=[]
node.create_subscription(LocalizerEstimate, '/fast_livo/localizer/estimate', latest.append, 1)
client=node.create_client(Initialize, '/fast_livo/localizer/initialize')
try:
    end=time.monotonic()+15
    def ready():
        return latest and latest[-1].map_uuid and json.loads(latest[-1].quality_json or '{}').get('inputs_ready')
    while time.monotonic()<end and not ready():rclpy.spin_once(node,timeout_sec=.1)
    if not ready():raise RuntimeError('map/session/input not ready')
    if not client.wait_for_service(timeout_sec=3):raise RuntimeError('Initialize service unavailable')
    message=latest[-1];request=Initialize.Request(request_id='manual-'+uuid.uuid4().hex,
        map_uuid=message.map_uuid, map_generation=message.map_generation,
        session_id=message.session_id, reference_body='fast_livo_imu')
    request.initial_pose.header.frame_id='map'  # stamp=0 binds next complete input pair
    pose=request.initial_pose.pose.pose
    pose.position.x,pose.position.y,pose.position.z=args.xyz
    r,pitch,y=args.rpy;cr,sr=math.cos(r/2),math.sin(r/2);cp,sp=math.cos(pitch/2),math.sin(pitch/2);cy,sy=math.cos(y/2),math.sin(y/2)
    pose.orientation.x=sr*cp*cy-cr*sp*sy;pose.orientation.y=cr*sp*cy+sr*cp*sy
    pose.orientation.z=cr*cp*sy-sr*sp*cy;pose.orientation.w=cr*cp*cy+sr*sp*sy
    for k in range(6):request.initial_pose.pose.covariance[k*7]=.01
    future=client.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=5)
    if not future.done():raise TimeoutError('Initialize response timeout')
    result=future.result();print('accepted=',result.accepted,'reason=',result.reason)
    if not result.accepted:raise SystemExit(2)
finally:
    node.destroy_node();rclpy.shutdown()
