"""ROS transport adapter. No direct velocity or mode publisher is exposed to HTTP."""
import asyncio
import json
import math
import time
from workbench.core import xyzpose, stamp

TOPICS={
 '/localization/estimate':'ranger_localization_msgs/msg/LocalizationEstimate',
 '/clock':'rosgraph_msgs/msg/Clock','/plan':'nav_msgs/msg/Path',
 '/navigation/route':'std_msgs/msg/String',
 '/navigation/prediction':'nav_msgs/msg/Path','/navigation/state':'std_msgs/msg/String',
 '/navigation/execution':'std_msgs/msg/String','/base/execution_state':'std_msgs/msg/String',
 '/navigation/control_owner':'std_msgs/msg/String','/navigation/actual_mode':'std_msgs/msg/UInt8',
 '/navigation/minimum_distance':'std_msgs/msg/Float32',
 '/navigation/control':'ranger_navigation_msgs/msg/LeasedControlCommand',
 '/navigation_ready':'std_msgs/msg/Bool','/navigation/safety_ready':'std_msgs/msg/Bool',
 '/ranger/motion_permitted':'std_msgs/msg/Bool','/base/safe_cmd_vel':'geometry_msgs/msg/Twist',
 '/wheel/odom':'nav_msgs/msg/Odometry','/fast_livo/navigation/wheel/odom':'nav_msgs/msg/Odometry',
 '/motion_state':'ranger_msgs/msg/MotionState','/system_state':'ranger_msgs/msg/SystemState'}

class RosBridge:
    def __init__(self, consume, on_result):
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rosidl_runtime_py.utilities import get_message
        from rosidl_runtime_py.convert import message_to_ordereddict
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        from std_msgs.msg import Bool
        from tf2_ros import Buffer, TransformListener
        from rclpy.executors import SingleThreadedExecutor
        from std_srvs.srv import Trigger
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.rclpy=rclpy; self.node=rclpy.create_node('ranger_navigation_workbench')
        self.executor=SingleThreadedExecutor();self.executor.add_node(self.node)
        from rclpy.parameter import Parameter
        self.node.set_parameters([Parameter('use_sim_time',value=True)])
        self.consume=consume; self.on_result=on_result
        self.enable=self.node.create_publisher(Bool,'/ranger/motion_enable',10)
        self.action=ActionClient(self.node,NavigateToPose,'/navigate_to_pose')
        self.reset_client=self.node.create_client(Trigger,'/navigation/reset')
        self.handles={}; self.pending=set(); self.closed=False
        for topic,typ in TOPICS.items():
            self.node.create_subscription(get_message(typ),topic,
                lambda msg,t=topic:consume(t,message_to_ordereddict(msg)),
                QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.tf=Buffer(); self.listener=TransformListener(self.tf,self.node)
        from ranger_navigation_msgs.msg import Observation
        self.last_observation=0
        def observation(msg):
            if time.monotonic()-self.last_observation<.2:return
            self.last_observation=time.monotonic()
            try:
                from rclpy.time import Time
                transform=self.tf.lookup_transform('map',msg.header.frame_id,Time.from_msg(msg.header.stamp))
                from workbench.transforms import observation as project
                consume('/workbench/environment',project(message_to_ordereddict(msg),message_to_ordereddict(transform)))
            except Exception as exc:
                consume('/workbench/environment',{'valid':False,'reason':'same_stamp_tf_missing: '+str(exc)[:160]})
        self.node.create_subscription(Observation,'/navigation/observation',observation,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
    def spin(self):
        # Many independent topics exceed 100 callbacks/s. One callback per
        # owner tick left even depth-one clock/pose subscriptions behind.
        # Drain a bounded batch without starving IPC, cancel or heartbeat work.
        end=time.monotonic()+.003
        for _ in range(32):
            self.executor.spin_once(timeout_sec=0.)
            if time.monotonic()>=end:break
    def arm(self,value):
        from std_msgs.msg import Bool
        m=Bool(); m.data=bool(value); self.enable.publish(m)
    async def future(self,f,timeout=4):
        end=time.monotonic()+timeout
        while not f.done():
            if time.monotonic()>end: raise TimeoutError('ROS 请求超时')
            await asyncio.sleep(.01)
        return f.result()
    async def goal(self,task_id,target):
        from nav2_msgs.action import NavigateToPose
        if not self.action.server_is_ready(): raise RuntimeError('导航 Action 尚未就绪')
        g=NavigateToPose.Goal(); g.pose.header.frame_id='map'; g.pose.header.stamp=self.node.get_clock().now().to_msg()
        g.pose.pose.position.x,g.pose.pose.position.y=target[:2]
        g.pose.pose.orientation.z=math.sin(target[2]/2);g.pose.pose.orientation.w=math.cos(target[2]/2)
        f=self.action.send_goal_async(g)
        try: handle=await self.future(f)
        except Exception:
            # A late accepted request must never become an unowned moving task.
            def cancel_late(done):
                try:
                    h=done.result()
                    if h.accepted:h.cancel_goal_async()
                except Exception: pass
            f.add_done_callback(cancel_late)
            raise
        if not handle.accepted: raise RuntimeError('目标被导航器拒绝')
        self.handles[task_id]=handle
        def finished(future):
            self.handles.pop(task_id,None)
            result=future.result()
            self.on_result(task_id,{4:'SUCCEEDED',5:'CANCELED',6:'FAILED'}.get(result.status,'FAILED'))
        handle.get_result_async().add_done_callback(finished)
    async def cancel(self,task_id):
        h=self.handles.get(task_id)
        if h: await self.future(h.cancel_goal_async())
    async def reset(self,task_id=None):
        from std_srvs.srv import Trigger
        if task_id in self.handles:
            await self.cancel(task_id)
            end=time.monotonic()+4
            while task_id in self.handles:
                if time.monotonic()>end:raise TimeoutError('旧目标尚未结束，暂未重置规划控制')
                await asyncio.sleep(.01)
        end=time.monotonic()+3
        while not self.reset_client.service_is_ready():
            if time.monotonic()>end:raise RuntimeError('导航重置服务尚未就绪；请加载新版导航会话')
            await asyncio.sleep(.02)
        response=await self.future(self.reset_client.call_async(Trigger.Request()))
        if not response.success:raise RuntimeError('规划控制未重置：'+response.message)
        return json.loads(response.message)
    def close(self):
        if self.handles:self.arm(False)
        for h in self.handles.values():h.cancel_goal_async()
        for _ in range(5):self.spin();time.sleep(.02)
        self.executor.remove_node(self.node);self.executor.shutdown()
        self.node.destroy_node();self.rclpy.shutdown()
