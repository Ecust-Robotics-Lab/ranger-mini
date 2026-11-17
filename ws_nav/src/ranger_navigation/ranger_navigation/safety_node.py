"""Independent 50Hz fail-closed output process; never imports JAX."""
import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool,String,UInt8
from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
from ranger_msgs.msg import SystemState,MotionState
from ranger_navigation_msgs.msg import Observation,ControlCommand
from .geometry import load_geometry
from .models import Limits
from .obstacles import ObstacleMemory
from .safety_core import Safety
from .ros_utils import stamp_s,pose2,twist_array,twist_msg


class SafetyNode(Node):
    def __init__(self):
        super().__init__('ranger_motion_gate')
        d = self.declare_parameter
        body,_,_ = load_geometry(d('geometry_config','').value,float(d('clearance',.04).value))
        self.core = Safety(body,Limits(speed=d('max_speed',.30).value,yaw_rate=d('max_yaw_rate',.50).value,
                                      acceleration=d('acceleration',.30).value,yaw_acceleration=d('yaw_acceleration',.50).value,
                                      min_radius=d('min_radius',.55).value))
        self.last_clock,self.session,self.nav_session,self.nav_generation = None,None,None,-1
        self.teleop_mode,self.teleop_mode_received = None,None
        self.retired_nav_sessions = set()
        self.visibility = ObstacleMemory(body)
        self.create_subscription(Bool,'/ranger/motion_enable',lambda m:setattr(self.core,'enabled',m.data),10)
        self.create_subscription(Bool,'/ranger/soft_estop',lambda m:setattr(self.core,'estop',m.data),10)
        self.create_subscription(Bool,'/navigation_ready',self.ready,10)
        self.create_subscription(Odometry,d('wheel_topic','/fast_livo/navigation/wheel/odom').value,self.wheel,10)
        self.create_subscription(SystemState,'/system_state',self.system,10)
        self.create_subscription(MotionState,'/motion_state',self.mode,10)
        self.create_subscription(Observation,'/navigation/observation',self.observation,2)
        self.create_subscription(ControlCommand,'/navigation/control',self.nav,2)
        self.create_subscription(ControlCommand,'/teleop/control',self.teleop,2)
        self.create_subscription(Twist,'/teleop/cmd_vel',self.legacy_teleop,2)
        self.create_subscription(UInt8,'/teleop/motion_mode',self.legacy_mode,2)
        self.out = self.create_publisher(Twist,'/base/safe_cmd_vel',1)
        self.gated = self.create_publisher(Twist,'/cmd_vel_after_gate',1)
        self.request = self.create_publisher(UInt8,'/base/motion_mode_request',1)
        self.owner = self.create_publisher(String,'/navigation/control_owner',1)
        self.actual = self.create_publisher(UInt8,'/navigation/actual_mode',1)
        self.permitted = self.create_publisher(Bool,'/ranger/motion_permitted',1)
        self.health = self.create_publisher(Bool,'/navigation/safety_ready',1)
        self.diag = self.create_publisher(DiagnosticArray,'/diagnostics',1)
        self.create_timer(.02,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def ready(self,m):
        self.core.ready = bool(m.data)
        self.core.feed('ready',self.now(),time.monotonic())

    def wheel(self,m):
        try:
            if m.header.frame_id!='odom' or m.child_frame_id!='base_link':
                raise ValueError('wheel_frame')
            self.core.pose,self.core.velocity = pose2(m.pose.pose),twist_array(m.twist.twist)
            self.core.feed('wheel',stamp_s(m.header.stamp),time.monotonic())
        except ValueError:
            self.core.inputs.pop('wheel',None)

    def host_feedback(self,name,m):
        # Driver feedback has host stamps; navigation uses the managed sensor
        # clock. Verify host sample age first, then lease only its remaining age.
        age = time.time()-stamp_s(m.header.stamp)
        if 0<=age<=.25:
            self.core.feed(name,self.now()-age,time.monotonic()-age)
            return True
        self.core.inputs.pop(name,None)
        return False

    def system(self,m):
        if self.host_feedback('system',m):
            self.core.vehicle_ok = m.vehicle_state==m.VEHICLE_STATE_NORMAL and m.error_code==0
            self.core.can_control = m.control_mode==m.CONTROL_MODE_CAN

    def mode(self,m):
        if self.host_feedback('mode',m):
            self.core.actual_mode = int(m.motion_mode)

    def observation(self,m):
        if self.session is not None and self.session!=m.session_id:
            self.core.commands.clear()
            self.core.enabled = False
        self.session = m.session_id
        if not m.valid or m.header.frame_id!='odom' or len(m.visible_ranges)!=180:
            self.core.inputs.pop('cloud',None)
            return
        pts = np.array([[p.x,p.y] for p in m.obstacles]).reshape(-1,2)
        if not np.isfinite(pts).all() or not np.isfinite(m.visible_ranges).all():
            self.core.inputs.pop('cloud',None)
            return
        self.core.obstacles = pts
        self.visibility.set_visibility([m.sensor_pose.x,m.sensor_pose.y,m.sensor_pose.theta],m.visible_ranges,m.observed_cells)
        self.core.coverage = self.visibility.covers
        self.core.feed('cloud',stamp_s(m.header.stamp),time.monotonic())

    def nav(self,m):
        if m.session_id in self.retired_nav_sessions:
            return
        if self.nav_session!=m.session_id:
            if self.nav_session is not None:
                self.retired_nav_sessions.add(self.nav_session)
            self.nav_session,self.nav_generation = m.session_id,-1
            self.core.commands.pop('navigation',None)
        if m.generation<self.nav_generation:
            return
        self.nav_generation = m.generation
        self.core.command('navigation',twist_array(m.twist),m.mode,stamp_s(m.header.stamp),time.monotonic(),m.valid)

    def teleop(self,m):
        self.core.command('teleop',twist_array(m.twist),m.mode,stamp_s(m.header.stamp),time.monotonic(),m.valid)

    def legacy_mode(self,m):
        self.teleop_mode,self.teleop_mode_received = int(m.data),time.monotonic()

    def legacy_teleop(self,m):
        now = time.monotonic()
        valid = self.teleop_mode_received is not None and 0<=now-self.teleop_mode_received<=.15
        self.core.command('teleop',twist_array(m),self.teleop_mode,self.now(),now,valid)

    def tick(self):
        now,sample = time.monotonic(),self.now()
        if self.last_clock is not None and sample<self.last_clock:
            self.core.enabled = False
            self.core.inputs.clear()
            self.core.commands.clear()
        self.last_clock = sample
        command,request = self.core.evaluate(sample,now)
        self.gated.publish(twist_msg(command))
        command,minimum = self.core.collision(command)
        self.out.publish(twist_msg(command))
        if request is not None:
            self.request.publish(UInt8(data=int(request)))
        self.owner.publish(String(data=self.core.owner))
        if self.core.actual_mode is not None and self.core.fresh('mode',sample,now,.25):
            self.actual.publish(UInt8(data=int(self.core.actual_mode)))
        self.permitted.publish(Bool(data=self.core.reason=='passing'))
        self.health.publish(Bool(data=self.core.inputs_ready(sample,now)))
        st = DiagnosticStatus(name='ranger_motion_gate',level=bytes([0 if self.core.reason=='passing' else 1]),
                              message=self.core.reason)
        st.values = [KeyValue(key='owner',value=self.core.owner),KeyValue(key='mode',value=str(self.core.actual_mode)),
                     KeyValue(key='minimum_distance',value=str(minimum))]
        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        diag.status = [st]
        self.diag.publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
