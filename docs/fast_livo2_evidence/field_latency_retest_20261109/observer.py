import json,time,os
os.environ.update(ROS_DOMAIN_ID='215',ROS_LOCALHOST_ONLY='1')
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray
from ranger_fast_livo_localizer.msg import LocalizerEstimate
rclpy.init();n=Node('field05_readonly_observer');state={}
def health(m):
 if m.status:state['health']={v.key:v.value for v in m.status[0].values}
def estimate(m):state['estimate']={'state':m.state,'reason':m.reason,'valid':m.pose_valid,'stamp':m.last_confirmed_stamp_ns,'request':m.initialization_request_id}
n.create_subscription(Bool,'/navigation_ready',lambda m:state.update(ready=m.data),rclpy.qos.QoSProfile(depth=1,reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT))
n.create_subscription(DiagnosticArray,'/localization_health',health,1)
n.create_subscription(LocalizerEstimate,'/fast_livo/localizer/estimate',estimate,1)
end=time.monotonic()+110;last=0
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.1)
 if time.monotonic()-last>=1:
  last=time.monotonic();state.update(steady=last,graph={topic:[p.node_name for p in n.get_publishers_info_by_topic(topic)] for topic in ['/tf','/cmd_vel','/base/safe_cmd_vel','/wheel/odom']});print(json.dumps(state),flush=True)
n.destroy_node();rclpy.shutdown()
