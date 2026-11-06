"""Manual-mode functional check using a seed derived from the preceding auto run."""
import json
import os
from pathlib import Path
import time
import numpy as np
from scipy.spatial.transform import Rotation
os.environ.update(ROS_DOMAIN_ID='215', ROS_LOCALHOST_ONLY='1')
import rclpy
from nav_msgs.msg import Odometry
from ranger_fast_livo_localizer.msg import LocalizerEstimate
from ranger_fast_livo_localizer.srv import Initialize

OUT = Path(__file__).resolve().parent
seed = json.loads((OUT/'manual_seed.json').read_text())
A = np.asarray(seed['map_local_correction'])
rclpy.init(); node = rclpy.create_node('manual_mode_functional_check'); odom=[]; estimates=[]
node.create_subscription(Odometry, '/fast_livo/odometry', odom.append, 4)
node.create_subscription(LocalizerEstimate, '/fast_livo/localizer/estimate', estimates.append, 20)
client = node.create_client(Initialize, '/fast_livo/localizer/initialize')
def ns(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
def wait(predicate, timeout):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.03)
        if predicate():return
    raise TimeoutError('manual check timeout')
try:
    wait(lambda: odom and estimates and estimates[-1].map_uuid and ns(odom[-1])>=seed['target_sample_ns'],90)
    p=odom[-1].pose.pose;U=np.eye(4);U[:3,3]=[p.position.x,p.position.y,p.position.z]
    U[:3,:3]=Rotation.from_quat([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]).as_matrix()
    X=A@U;current=estimates[-1]
    request=Initialize.Request(request_id='manual-functional-check',map_uuid=current.map_uuid,
        map_generation=current.map_generation,session_id=current.session_id,reference_body='fast_livo_imu')
    request.initial_pose.header.frame_id='map'
    pose=request.initial_pose.pose.pose;pose.position.x,pose.position.y,pose.position.z=X[:3,3].tolist()
    pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=Rotation.from_matrix(X[:3,:3]).as_quat().tolist()
    for k in range(6):request.initial_pose.pose.covariance[k*7]=.01
    assert client.wait_for_service(timeout_sec=3)
    future=client.call_async(request);wait(future.done,5);assert future.result().accepted,future.result().reason
    wait(lambda: estimates[-1].pose_valid,20)
    sample=estimates[-1]
    result={'passed':True,'request_id':request.request_id,'first_valid_stamp_ns':ns(sample),
        'scan_id':sample.scan_id,'quality':json.loads(sample.quality_json),
        'independent_global_initialization':False,'note':seed['reference']}
    (OUT/'manual_result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
finally:
    node.destroy_node();rclpy.shutdown()
