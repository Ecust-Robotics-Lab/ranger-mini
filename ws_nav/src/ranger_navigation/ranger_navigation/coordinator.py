"""NavigateToPose adapter for SE(2)/EXACT-MPPI; no device startup or public TF."""
import json
import copy
import math
import re
import tempfile
import time
import uuid
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.action import ActionServer,GoalResponse,CancelResponse
from rclpy.task import Future
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import Point32,Twist
from nav_msgs.msg import Odometry,OccupancyGrid,Path as PathMsg
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool,String,UInt8,Float32
from ranger_localization_msgs.msg import LocalizationEstimate
from ranger_navigation_msgs.msg import Observation,ControlCommand
from ranger_msgs.msg import MotionState
from ranger_localization_bridge.alignment import PoseHistory
from ranger_localization_bridge.geometry import SE3
import tf2_ros
from .geometry import load_geometry,validate_sweep,wrap,densify
from .models import Limits,Mode,rollout,braking_commands,stopped
from .obstacles import ObstacleMemory
from .planner import Grid,Segment,reanchor_parallel,parallel_anchor_close
from .task import Task
from .worker import WorkerClient
from .ros_utils import stamp_s,stamp_ns,pose2,twist_array,twist_msg,path_msg,transform2


def cloud_xyz(msg):
    fields = {f.name:f for f in msg.fields}
    if not all(name in fields and fields[name].datatype in (7,8) for name in ('x','y','z')):
        raise ValueError('cloud_xyz_fields_missing')
    endian = '>' if msg.is_bigendian else '<'
    dtype = np.dtype({'names':['x','y','z'],'formats':[endian+('f4' if fields[k].datatype==7 else 'f8') for k in ('x','y','z')],
                      'offsets':[fields[k].offset for k in ('x','y','z')],'itemsize':msg.point_step})
    if msg.row_step<msg.width*msg.point_step or len(msg.data)<msg.height*msg.row_step:
        raise ValueError('cloud_layout_truncated')
    arr = np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step))
    return np.column_stack([arr[k].ravel() for k in ('x','y','z')])


class Coordinator(Node):
    def __init__(self):
        super().__init__('ranger_exact_navigation')
        d = self.declare_parameter
        geometry = str(d('geometry_config','').value)
        self.body,self.r_imu,self.t_imu = load_geometry(geometry,float(d('clearance',.04).value))
        self.static_reserve = float(d('static_map_reserve',.04).value)
        if not np.isfinite(self.static_reserve) or not 0 <= self.static_reserve <= .10:
            raise ValueError('static_map_reserve must be finite and in [0, .10]m')
        self.visibility_reserve = float(d('visibility_reserve',.10).value)
        self.prefer_parallel_in_narrow = bool(d('prefer_parallel_in_narrow',False).value)
        self.global_visibility = bool(d('global_visibility',True).value)
        if not np.isfinite(self.visibility_reserve) or not .015 <= self.visibility_reserve <= .20:
            raise ValueError('visibility_reserve must be finite and in [.015, .20]m')
        self.limits = Limits(speed=float(d('max_speed',.30).value),yaw_rate=float(d('max_yaw_rate',.50).value),
                             acceleration=float(d('acceleration',.30).value),yaw_acceleration=float(d('yaw_acceleration',.50).value),
                             min_radius=float(d('min_radius',.55).value))
        self.memory,self.history,self.task = ObstacleMemory(self.body,
            sectors=int(d('visibility_sectors',180).value),
            angular_guard=int(d('visibility_angular_guard',1).value),
            control_memory=float(d('visibility_control_memory',0.).value)),PoseHistory(),Task(
            self.limits,segment_yaw_tolerance=float(d('segment_yaw_tolerance',.06).value),
            goal_position_tolerance=float(d('goal_position_tolerance',.15).value),
            goal_yaw_tolerance=float(d('goal_yaw_tolerance',.15).value))
        self.session,self.front_gid,self.loc_session = uuid.uuid4().hex,None,None
        self.retired_front,self.retired_loc = set(),set()
        self.front_locked = False
        self.ready,self.owner = False,'none'
        self.safety_ready,self.safety_received = False,None
        self.ready_received = self.owner_received = self.estimate_received = None
        self.wheel,self.reference_wheel,self.estimate,self.grid = None,None,None,None
        self.wheel_samples = []
        self.wheel_received,self.cloud_received = None,None
        self.cloud_stamp,self.cloud_message = None,None
        self.fast_stamps,self.clouds = {},{}
        self.actual_mode = self.actual_mode_received = None
        self.command,self.last_clock,self.last_plan = None,None,-1e9
        self.last_control = self.last_state_publish = -1e9
        self.metrics = {}
        self.input_timing = {}
        self.goal_handle,self.result_future = None,None
        self.goal_futures = {}
        self.expected_map = str(d('expected_map_uuid','').value)
        self.expected_manifest = str(d('expected_manifest_sha256','').value)
        expected_yaml = str(d('map_yaml','').value)
        self.expected_grid = Grid.from_yaml(expected_yaml) if expected_yaml else None
        root = Path(str(d('session_root',str(Path.cwd()/'.runtime/exact_navigation')).value))
        root.mkdir(parents=True,exist_ok=True)
        folder = Path(tempfile.mkdtemp(prefix='run-',dir=root))
        self.worker = WorkerClient(folder,d('optimizer_python','').value,geometry,
                                   d('compute_backend','gpu').value,int(d('samples',256).value),
                                   {'speed':self.limits.speed,'yaw_rate':self.limits.yaw_rate,
                                    'acceleration':self.limits.acceleration,'yaw_acceleration':self.limits.yaw_acceleration,
                                    'min_radius':self.limits.min_radius},clearance=self.body.clearance,
                                   static_reserve=self.static_reserve,visibility_reserve=self.visibility_reserve,
                                   prefer_parallel_in_narrow=self.prefer_parallel_in_narrow)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer,self)
        self.create_subscription(Odometry,d('wheel_topic','/fast_livo/navigation/wheel/odom').value,self.on_wheel,50)
        self.create_subscription(Odometry,'/fast_livo/odometry',self.on_fast,10)
        self.create_subscription(PointCloud2,'/fast_livo/deskewed_imu',self.on_cloud,2)
        self.create_subscription(LocalizationEstimate,'/localization/estimate',self.on_estimate,10)
        self.create_subscription(Bool,'/navigation_ready',self.on_ready,10)
        self.create_subscription(String,'/navigation/control_owner',self.on_owner,10)
        self.create_subscription(MotionState,'/motion_state',self.on_mode,10)
        self.create_subscription(Bool,'/navigation/safety_ready',self.on_safety,10)
        map_qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,map_qos)
        self.obs_pub = self.create_publisher(Observation,'/navigation/observation',2)
        self.command_pub = self.create_publisher(ControlCommand,'/navigation/control',1)
        self.velocity_pub = self.create_publisher(Twist,'/cmd_vel_nav',1)
        self.path_pub = self.create_publisher(PathMsg,'/plan',1)
        self.local_pub = self.create_publisher(PathMsg,'/navigation/prediction',1)
        self.state_pub = self.create_publisher(String,'/navigation/state',1)
        self.minimum_pub = self.create_publisher(Float32,'/navigation/minimum_distance',1)
        self.action = ActionServer(self,NavigateToPose,'navigate_to_pose',execute_callback=self.execute,
                                  goal_callback=self.goal,cancel_callback=self.cancel,
                                  handle_accepted_callback=self.accepted)
        # Poll asynchronously at 200Hz; submit optimization at 20Hz. A 50ms
        # result-poll period alone would consume the entire latency target.
        self.create_timer(.005,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def invalidate(self,reason,clear=False,transient=False):
        if self.task.goal is not None and self.task.state not in Task.TERMINAL:
            self.get_logger().warning('navigation invalidated: '+reason)
        self.command = None
        if self.task.goal is not None and self.task.state not in Task.TERMINAL:
            if transient and not clear:
                self.task.pause_inputs(reason,time.monotonic())
            else:
                self.task.invalidate(reason)
        if clear:
            self.memory.reset()
            self.history.clear()
            self.fast_stamps.clear()
            self.clouds.clear()
            self.wheel = None
            self.wheel_samples.clear()

    def on_mode(self,m):
        age = time.time()-stamp_s(m.header.stamp)
        self.actual_mode,self.actual_mode_received = (int(m.motion_mode),time.monotonic()-age) if 0<=age<=.25 else (None,None)

    def on_safety(self,m):
        self.safety_ready,self.safety_received = bool(m.data),time.monotonic()

    def on_ready(self,m):
        if self.ready and not m.data:
            self.invalidate('localization_withdrawn',transient=True)
        self.ready,self.ready_received = bool(m.data),time.monotonic()

    def on_owner(self,m):
        if self.owner!=m.data:
            self.invalidate('control_owner_changed')
        self.owner,self.owner_received = m.data,time.monotonic()

    def on_map(self,m):
        if m.header.frame_id!='map':
            return
        grid = Grid(np.asarray(m.data).reshape(m.info.height,m.info.width),m.info.resolution,
                    pose2(m.info.origin))
        if self.expected_grid is not None and (not np.array_equal(grid.cells,self.expected_grid.cells)
                or not np.allclose(grid.origin,self.expected_grid.origin,atol=1e-7)
                or abs(grid.resolution-self.expected_grid.resolution)>1e-7):
            self.grid = None
            self.invalidate('map_service_differs_from_bound_asset')
            return
        if self.grid is not None and (not np.array_equal(grid.cells,self.grid.cells)
                or grid.resolution!=self.grid.resolution or not np.array_equal(grid.origin,self.grid.origin)):
            self.invalidate('map_changed',clear=True)
        self.grid = grid

    def on_estimate(self,m):
        match = re.search(r'session=([^; ]+)',m.reason)
        identity = match.group(1) if match else None
        if identity is not None and identity in self.retired_loc:
            return
        # A fault heartbeat may omit session metadata. It withdraws readiness,
        # but cannot retire the active localization session: otherwise valid
        # recovery messages from that very session are rejected forever.
        if identity is not None and identity!=self.loc_session:
            if self.loc_session is not None:
                self.retired_loc.add(self.loc_session)
            self.loc_session = identity
            self.invalidate('localization_session_changed',clear=True)
        if m.state!=m.VALID:
            self.invalidate('localization_invalid',transient=m.state==m.DEGRADED)
        self.estimate,self.estimate_received = m,time.monotonic()

    def on_wheel(self,m):
        try:
            if m.header.frame_id!='odom' or m.child_frame_id!='base_link':
                raise ValueError('wheel_frame')
            p,q = m.pose.pose.position,m.pose.pose.orientation
            pose2(m.pose.pose)
            if not np.isfinite(twist_array(m.twist.twist)).all():
                raise ValueError('wheel_velocity')
            added = self.history.add(stamp_ns(m.header.stamp),SE3((p.x,p.y,p.z),(q.x,q.y,q.z,q.w)))
            if added=='regressed':
                self.invalidate('wheel_time_regressed',clear=True)
                self.front_locked = True
                return
            if added!='duplicate':
                self.wheel,self.wheel_received = m,time.monotonic()
                self.wheel_samples.append(m)
                self.wheel_samples = self.wheel_samples[-30:]
        except ValueError:
            self.wheel = None

    def on_fast(self,m):
        # Humble rclpy does not pass MessageInfo to subscription callbacks.
        publishers = self.get_publishers_info_by_topic('/fast_livo/odometry')
        if len(publishers)!=1:
            self.front_locked = True
            self.invalidate('frontend_publisher_nonunique')
            return
        gid = bytes(publishers[0].endpoint_gid).hex()
        if gid in self.retired_front:
            return
        if self.front_gid!=gid:
            if self.front_gid is not None:
                self.retired_front.add(self.front_gid)
            self.front_gid,self.front_locked = gid,False
            self.invalidate('frontend_session_changed',clear=True)
        if m.header.frame_id!='fast_livo_local' or m.child_frame_id!='fast_livo_imu':
            return
        ns = stamp_ns(m.header.stamp)
        if self.fast_stamps and ns<max(self.fast_stamps):
            self.front_locked = True
            self.invalidate('frontend_clock_regressed',clear=True)
            return
        self.fast_stamps[ns] = time.monotonic()
        while len(self.fast_stamps)>30:
            del self.fast_stamps[min(self.fast_stamps)]

    def on_cloud(self,m):
        if m.header.frame_id=='fast_livo_imu':
            self.clouds[stamp_ns(m.header.stamp)] = m
            while len(self.clouds)>4:
                del self.clouds[min(self.clouds)]

    def process_cloud(self):
        if self.front_locked:
            return
        for ns in sorted(list(self.clouds)):
            # Sensor output can arrive a few milliseconds before the matching
            # /clock update. Keep it queued until its original sample time;
            # replacing valid memory with future data would repeatedly revoke
            # readiness even though an earlier aligned observation is fresh.
            if ns>self.get_clock().now().nanoseconds:
                continue
            if self.now()-ns*1e-9>.25:
                del self.clouds[ns]
                continue
            if ns not in self.fast_stamps:
                continue
            pose,reason = self.history.lookup(ns)
            if pose is None:
                if reason=='waiting for odometry bracket':
                    continue
                del self.clouds[ns]
                continue
            msg = self.clouds.pop(ns)
            started = time.monotonic()
            try:
                xyz = cloud_xyz(msg)
                okay = self.memory.update(xyz,self.r_imu,self.t_imu,[*pose.t[:2],pose.yaw()],ns*1e-9)
            except ValueError:
                okay = False
            out = Observation()
            out.header.stamp,out.header.frame_id,out.session_id = msg.header.stamp,'odom',self.session
            out.valid,out.reason = bool(okay),self.memory.error or 'valid'
            if okay:
                out.obstacles = [Point32(x=float(p[0]),y=float(p[1]),z=.4) for p in self.memory.points()]
                out.visible_ranges = self.memory.ranges.astype(float).tolist()
                out.sensor_pose.x,out.sensor_pose.y,out.sensor_pose.theta = map(float,self.memory.visibility_pose)
                out.voxel_size = self.memory.voxel
                out.observed_cells = self.memory._visible_grid.cells.astype(np.int8).ravel().tolist()
                self.cloud_received,self.cloud_stamp = time.monotonic(),msg.header.stamp
            self.obs_pub.publish(out)
            self.input_timing = dict(observation_build_publish_ms=(time.monotonic()-started)*1000.,
                                     observation_sample_age_ms=(self.now()-ns*1e-9)*1000.)

    def goal(self,request):
        try:
            p = pose2(request.pose.pose)
            if request.pose.header.frame_id!='map' or not np.isfinite(p).all() or request.behavior_tree:
                return GoalResponse.REJECT
        except ValueError:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def finish(self,state):
        handle,future = self.goal_handle,self.result_future
        if handle is None:
            return
        if handle.is_active:
            if state=='SUCCEEDED':
                handle.succeed()
            elif state=='CANCELED' and handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
        if future is not None and not future.done():
            future.set_result(NavigateToPose.Result())
        self.goal_handle,self.result_future = None,None

    def accepted(self,handle):
        if self.goal_handle is not None:
            self.finish('REPLACED')
        self.command = None
        self.task.begin(pose2(handle.request.pose.pose),time.monotonic())
        self.goal_handle,self.result_future = handle,Future()
        self.goal_futures[bytes(handle.goal_id.uuid)] = self.result_future
        handle.execute()

    async def execute(self,handle):
        # Capture this goal's future before a replacement can change self.
        key = bytes(handle.goal_id.uuid)
        future = self.goal_futures.get(key)
        if future is None:
            return NavigateToPose.Result()
        result = await future
        self.goal_futures.pop(key,None)
        return result

    def cancel(self,handle):
        if handle==self.goal_handle:
            self.task.cancel()
            self.command = None
        return CancelResponse.ACCEPT

    def sample(self,now):
        self.input_reason = "missing_ready_wheel_estimate_map_or_frontend_locked"
        if not self.ready or not self.safety_ready or self.wheel is None or self.estimate is None or self.grid is None or self.front_locked:
            return None
        for name,received in (("ready",self.ready_received),("wheel",self.wheel_received),("cloud",self.cloud_received),("estimate",self.estimate_received),("owner",self.owner_received),("mode",self.actual_mode_received),("safety",self.safety_received)):
            if received is None or not 0<=now-received<=.25:
                self.input_reason = name+"_receipt_stale"
                return None
        sample_now = self.now()
        self.input_reason = "estimate_contract_or_observation_invalid:"+str(self.memory.error)
        if (self.estimate.state!=self.estimate.VALID or self.estimate.header.frame_id!='map'
                or self.estimate.base_frame!='base_link' or self.memory.error is not None):
            return None
        if self.expected_map and ('map='+self.expected_map not in self.estimate.reason):
            return None
        self.input_reason = "sampling_stamp_stale_or_future"
        for name,stamp in (("estimate",stamp_s(self.estimate.header.stamp)),("observation",self.memory.last_stamp)):
            if stamp is None or not 0<=sample_now-stamp<=.25:
                self.input_reason += ":"+name+":"+str(None if stamp is None else sample_now-stamp)
                return None
        self.input_reason = "same_sample_tf_unavailable"
        # New wheel callbacks can precede the matching public TF callback. Use
        # the newest *fully aligned* original sample within the existing age
        # bound; never query latest TF or extrapolate a missing transform.
        for reference in reversed(self.wheel_samples):
            if not 0<=sample_now-stamp_s(reference.header.stamp)<=.25:
                continue
            try:
                transform = self.buffer.lookup_transform('map','odom',Time(nanoseconds=stamp_ns(reference.header.stamp)))
                p,q = transform.transform.translation,transform.transform.rotation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                correction = np.array([p.x,p.y,yaw])
                odom = pose2(reference.pose.pose)
                pose = np.array([*transform2(odom[None,:2],correction)[0],float(wrap(odom[2]+yaw))])
                previous = getattr(self,'last_correction',None)
                if previous is not None and (np.linalg.norm(correction[:2]-previous[:2])>.15 or abs(wrap(yaw-previous[2]))>.15):
                    self.invalidate('localization_correction_jump')
                self.last_correction,self.reference_wheel = correction,reference
                self.input_reason = "valid"
                return pose,twist_array(reference.twist.twist),correction
            except (tf2_ros.TransformException,ValueError):
                continue
        return None

    def tick(self):
        now,sample_now = time.monotonic(),self.now()
        cycle_started = now
        if self.last_clock is not None and sample_now<self.last_clock:
            self.invalidate('clock_regressed',clear=True)
            self.front_locked = True
        self.last_clock = sample_now
        self.process_cloud()
        # Cloud conversion records its receipt after computation. Refresh the
        # steady reference before comparing ages; the earlier tick start would
        # make that fresh receipt appear to be in the future once per cloud.
        now = time.monotonic()
        sample = self.sample(now)
        status = self.worker.status()
        healthy = sample is not None and status.get('ready',False)
        if status.get('error')=='worker_exited' and self.task.goal is not None and self.task.state not in Task.TERMINAL:
            self.task.invalidate('worker_exited','FAILED')
        if (healthy and self.task.state in ('SWITCHING','OBSERVING','PAUSED') and self.task.index<len(self.task.segments)
                and stopped(sample[1],self.limits)):
            candidate=self.task.segments[self.task.index]
            error=sample[0]-candidate.poses[0]
            if candidate.mode==Mode.PARALLEL and (np.linalg.norm(error[:2])>.003 or abs(wrap(error[2]))>.003):
                anchored=reanchor_parallel(candidate,sample[0],self.body,self.grid,
                    transform2(self.memory.points(),sample[2]),self.static_reserve)
                if anchored is not None:
                    self.task.segments[self.task.index]=anchored
        segment = self.task.tick(*(sample[:2] if sample is not None else (np.zeros(3),np.zeros(3))),
                                 self.actual_mode,healthy,self.owner,now)
        if segment is not None:
            check_id = (self.memory.last_stamp,self.task.generation,self.task.index)
            if getattr(self,'last_path_check',None)!=check_id:
                path = segment.poses
                nearest = int(np.argmin(np.linalg.norm(path[:,:2]-sample[0][:2],axis=1)))
                short = path[nearest:]
                distance = np.r_[0.,np.cumsum(np.linalg.norm(np.diff(short[:,:2],axis=0),axis=1))]
                short = short[:max(2,int(np.searchsorted(distance,.75))+1)]
                okay,_,_ = validate_sweep(short,transform2(self.memory.points(),sample[2]),self.body,
                                           margin=self.body.clearance+.04)
                self.last_path_check = check_id
                if not okay:
                    self.command,segment = None,None
                    self.task.block('guidance_blocked_replan',now)
        if not healthy or self.task.state!='EXECUTING':
            self.command = None
        result = self.worker.poll()
        if result is not None:
            meta,arrays = result
            if meta['generation']==self.task.generation and healthy:
                if meta.get('error'):
                    self.command = None
                    if meta['kind']=='control' and meta['error'] in ('worker_timeout','result_deadline_exceeded'):
                        self.task.hold_observation('control_compute_gap')
                    elif meta['kind']=='control' and meta['error'] in (
                            'feedback_drive_steering_transition','feedback_does_not_match_mode',
                            'acceleration_constraint_failure'):
                        # A measured transition residual needs a stopped,
                        # freshly validated rollout, not a different route.
                        self.task.hold_observation('feedback_settling')
                    else:
                        self.task.block(meta['error'],now)
                elif meta['kind']=='plan':
                    segments = [Segment(Mode(s['mode']),arrays['segment_'+str(i)],s['reverse']) for i,s in enumerate(meta['segments'])]
                    self.task.plan(segments,meta['generation'],now)
                    self.path_pub.publish(path_msg(np.vstack([s.poses for s in segments]) if segments else [sample[0]],self.reference_wheel.header.stamp))
                elif segment is not None and self.task.state=='EXECUTING':
                    pose,speed,correction = sample
                    def coverage(p,b,margin):
                        c,s = math.cos(correction[2]),math.sin(correction[2])
                        xy = (np.asarray(p[:2])-correction[:2])@np.array([[c,-s],[s,c]])
                        return self.memory.covers(np.array([*xy,p[2]-correction[2]]),b,margin)
                    points = transform2(self.memory.points(),correction)
                    trajectory = arrays['trajectory']
                    tail = rollout(trajectory[-1],braking_commands(arrays['controls'][-1],self.limits),.05)
                    okay,minimum,reason = validate_sweep(np.vstack((trajectory,tail[1:])),points,self.body,
                                                        self.grid,coverage,margin=self.body.clearance+.04,
                                                        map_margin=self.body.clearance+self.static_reserve)
                    elapsed = time.monotonic()-meta.get('input_started',meta['submitted'])
                    self.metrics = dict(request_id=meta['id'],compute_s=meta.get('compute_s'),end_to_end_s=elapsed,valid=okay,validation_reason=reason)
                    if elapsed>.1:
                        okay,reason = False,'validation_deadline_exceeded'
                    if okay:
                        self.command = meta,arrays['command'],meta['sample_stamp_ns']
                        prediction_stamp = copy.deepcopy(self.reference_wheel.header.stamp)
                        prediction_stamp.sec,prediction_stamp.nanosec = divmod(meta['sample_stamp_ns'],10**9)
                        self.local_pub.publish(path_msg(trajectory,prediction_stamp))
                        self.minimum_pub.publish(Float32(data=float(minimum)))
                    else:
                        self.command = None
                        if reason=='validation_deadline_exceeded':
                            self.task.hold_observation('control_compute_gap')
                        else:
                            self.task.block(reason,now)
        if healthy and self.task.goal is not None and self.task.state not in Task.TERMINAL:
            pose,speed,correction = sample
            # Include input alignment/cloud conversion and path checks in the
            # control-cycle measurement, not only JAX and result validation.
            control_cycle_started = cycle_started
            if self.worker.pending is None and self.task.state in ('PLANNING','BLOCKED') and now-self.last_plan>.5:
                self.command = None
                if self.worker.submit('plan',self.task.generation,dict(cells=self.grid.cells,pose=pose,goal=self.task.goal,
                        points=transform2(self.memory.points(),correction),visibility_cells=self.memory.control_grid.cells,
                        observer_pose=np.array([*transform2(self.memory.visibility_pose[None,:2],correction)[0],self.memory.visibility_pose[2]+correction[2]])),
                        dict(resolution=self.grid.resolution,origin=self.grid.origin.tolist(),actual_mode=self.actual_mode,
                             global_visibility=self.global_visibility),budget=5.1):
                    self.last_plan = now
            elif (self.worker.pending is None and segment is not None
                    and self.task.state in ('EXECUTING','OBSERVING') and now-self.last_control>=.05):
                # Preparation can be much more expensive than polling. Run it
                # at the control rate, including when coverage is insufficient;
                # a busy worker cannot accept another input anyway. Keep result
                # polling, input withdrawal and zero output on the 5ms timer.
                self.last_control = now
                path = densify(segment.poses,self.body,.025)
                nearest = int(np.argmin(np.linalg.norm(path[:,:2]-pose[:2],axis=1)+.15*np.abs(wrap(path[:,2]-pose[2]))))
                c,s = math.cos(correction[2]),math.sin(correction[2])
                known_end = nearest
                distance = 0.
                for i in range(nearest,len(path)):
                    if i>nearest:
                        distance += np.linalg.norm(path[i,:2]-path[i-1,:2])
                    if distance>.75:
                        break
                    xy = (path[i,:2]-correction[:2])@np.array([[c,-s],[s,c]])-self.memory.visibility_pose[:2]
                    if not self.memory.control_grid.free([*xy,path[i,2]-correction[2]],self.body,self.body.clearance+.015):
                        break
                    known_end = i
                if known_end<=nearest and nearest<len(path)-1:
                    self.task.hold_observation()
                    self.command = None
                    path = None
                else:
                    # Keep the original segment start to preserve its fixed
                    # parallel steering axis, then let the optimizer trim it.
                    path = path[:known_end+1]
                    if self.task.state=='OBSERVING':
                        # The route is still valid, but restart only at rest
                        # with the same confirmed chassis mode. The new control
                        # request uses measured velocity and full validation.
                        if not stopped(speed,self.limits) or self.actual_mode!=segment.mode:
                            path = None
                        else:
                            error=pose-segment.poses[0]
                            if segment.mode==Mode.PARALLEL and not parallel_anchor_close(segment,pose):
                                self.task.invalidate('observation_resume_pose_changed','PLANNING')
                                path = None
                            elif segment.mode==Mode.DRIVE:
                                original=densify(segment.poses,self.body,.015)
                                index=int(np.argmin(np.linalg.norm(original[:,:2]-pose[:2],axis=1)))
                                if (np.linalg.norm(original[index,:2]-pose[:2])>.03
                                        or abs(wrap(original[index,2]-pose[2]))>.03):
                                    self.task.invalidate('observation_resume_pose_changed','PLANNING')
                                    path = None
                                else:
                                    self.task.state,self.task.reason='EXECUTING','observation_resumed'
                            else:
                                self.task.state,self.task.reason='EXECUTING','observation_resumed'
                if path is not None:
                    points = transform2(self.memory.sampled(512),correction)
                    self.worker.submit('control',self.task.generation,dict(pose=pose,speed=speed,path=path,points=points,
                        visibility_cells=self.memory.control_grid.cells,
                        observer_pose=np.array([*transform2(self.memory.visibility_pose[None,:2],correction)[0],self.memory.visibility_pose[2]+correction[2]])),
                        dict(mode=int(segment.mode),segment=self.task.index,sample_stamp=stamp_s(self.reference_wheel.header.stamp),
                             input_started=control_cycle_started,sample_stamp_ns=stamp_ns(self.reference_wheel.header.stamp)),budget=.1)
        out = ControlCommand()
        out.session_id,out.generation,out.reason = self.session,self.task.generation,self.task.reason
        out.header.frame_id = 'base_link'
        out.header.stamp = copy.deepcopy(self.reference_wheel.header.stamp) if self.reference_wheel is not None else self.get_clock().now().to_msg()
        if self.task.index<len(self.task.segments):
            out.mode = int(self.task.segments[self.task.index].mode)
        out.valid = bool(healthy and self.owner=='navigation' and self.task.state in ('EXECUTING','SWITCHING','STOPPING','OBSERVING'))
        if out.valid and self.command is not None:
            meta,velocity,original_stamp = self.command
            if meta['generation']==self.task.generation and now-meta['submitted']<=.1:
                out.twist = twist_msg(velocity)
                ns = original_stamp
                out.header.stamp.sec,out.header.stamp.nanosec = divmod(ns,10**9)
        self.command_pub.publish(out)
        self.velocity_pub.publish(out.twist)
        publish_state = now-self.last_state_publish>=.05
        if publish_state:
            self.state_pub.publish(String(data=json.dumps(dict(state=self.task.state,reason=self.task.reason,
                                      generation=self.task.generation,worker=status,inputs_valid=healthy,input_reason=self.input_reason,metrics=self.metrics,
                                      input_timing=getattr(self,'input_timing',{}),
                                      segment_index=self.task.index,segment_count=len(self.task.segments),
                                      segment_mode=(int(self.task.segments[self.task.index].mode) if self.task.index<len(self.task.segments) else None),
                                      segment_target=(self.task.segments[self.task.index].poses[-1].tolist() if self.task.index<len(self.task.segments) else None)))))
            self.last_state_publish = now
        if self.task.reason=='inputs_invalid':
            self.get_logger().warning('input health gap: '+str(getattr(self,'input_reason','unknown')))
        if self.goal_handle is not None:
            if self.task.state in Task.TERMINAL:
                self.finish(self.task.state)
            elif sample is not None and publish_state:
                feedback = NavigateToPose.Feedback()
                feedback.current_pose = path_msg([sample[0]],self.reference_wheel.header.stamp).poses[0]
                feedback.distance_remaining = float(np.linalg.norm(sample[0][:2]-self.task.goal[:2]))
                self.goal_handle.publish_feedback(feedback)

    def destroy_node(self):
        self.worker.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Coordinator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
