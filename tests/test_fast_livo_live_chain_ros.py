"""Synthetic confirmed C1 -> actual adapter + live wheel clock -> actual bridge."""
import json,os,signal,subprocess,sys,tempfile,time,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'scripts/fast_livo2/navigation'),str(ROOT/'ws_nav/src/ranger_localization_bridge')]
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String,Bool
from diagnostic_msgs.msg import DiagnosticArray
from ranger_fast_livo_localizer.msg import LocalizerEstimate
from wheel_clock_node import WheelClockNode
from fast_livo2.mapping_monitor import Monitor
import tf2_ros

class LiveChain(unittest.TestCase):
    def test_conversion_tf_loss_recovery_and_clock_fault(self):
        rclpy.init();children=[];logs=[]
        with tempfile.TemporaryDirectory() as folder:
            driver=Node('live_chain_synthetic_inputs');converter=WheelClockNode(Path(folder))
            os.environ['RANGER_SESSION_HEALTH_FILE']=str(Path(folder)/'health.json')
            os.environ['RANGER_LIVO_TIMESHARE']=str(Path(folder)/'no_hardware_timeshare')
            monitor=Monitor(Path(folder),False,True)
            wheel=driver.create_publisher(Odometry,'/wheel/odom',100)
            receipts=driver.create_publisher(String,'/fast_livo/capture/input_receipts',10)
            loc=driver.create_publisher(LocalizerEstimate,'/fast_livo/localizer/estimate',10)
            ready=[];driver.create_subscription(Bool,'/navigation_ready',lambda m:ready.append(m.data),1)
            health=[];driver.create_subscription(DiagnosticArray,'/localization_health',health.append,1)
            buffer=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buffer,driver)
            anchor_s=time.monotonic_ns()-1_100_000_000;anchor_t=1_590_000_000_000_000_000
            count=12
            def pump(duration,wheel_on=True,receipts_on=True,bad_anchor=False):
                nonlocal count
                end=time.monotonic()+duration;next_send=0
                while time.monotonic()<end:
                    if time.monotonic()>=next_send:
                        next_send=time.monotonic()+.02
                        steady=time.monotonic_ns();sensor=anchor_t+steady-anchor_s
                        if receipts_on:
                            count+=1;r=String();r.data=json.dumps({'schema_version':1,'context':folder,'topics':{'/livox/lidar':{
                                'count':count,'first_header_ns':anchor_t-2_900_000_000+(1 if bad_anchor else 0),'first_steady_ns':anchor_s,
                                'last_header_ns':sensor,'last_steady_ns':steady}}});receipts.publish(r)
                        if wheel_on:
                            m=Odometry();m.header.frame_id='odom';m.child_frame_id='base_link'
                            m.header.stamp.sec,m.header.stamp.nanosec=divmod(time.time_ns()-10_000_000,10**9)
                            m.pose.pose.orientation.w=1.;wheel.publish(m)
                        msg=LocalizerEstimate();msg.header.frame_id='map';msg.imu_frame='fast_livo_imu'
                        stamp=sensor-50_000_000;msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(stamp,10**9)
                        msg.pose.position.x=2.;msg.pose.orientation.w=1.;msg.pose_valid=True;msg.state=msg.VALID
                        msg.map_uuid='synthetic-map';msg.manifest_sha256='a'*64;msg.graph_version=1;msg.map_generation=1
                        msg.session_id='synthetic-confirmed-c1';msg.request_generation=1;msg.initialization_request_id='synthetic'
                        msg.last_confirmed_stamp_ns=stamp;msg.latest_scan_stamp_ns=stamp;loc.publish(msg)
                    rclpy.spin_once(driver,timeout_sec=.002);rclpy.spin_once(converter,timeout_sec=.002)
                    rclpy.spin_once(monitor,timeout_sec=.002)
            try:
                for module,extra in [('fast_livo_localizer_adapter',['-p','t_base_imu_xyz:=[0.0,0.0,0.0]']),
                    ('bridge_node',['-p','adapter_backend:=external','-p','latch_on_innovation_failure:=true','-r','/wheel/odom:=/fast_livo/navigation/wheel/odom'])]:
                    log=(Path(folder)/(module+'.log')).open('w');logs.append(log)
                    children.append(subprocess.Popen([sys.executable,'-m','ranger_localization_bridge.'+module,'--ros-args','-p','use_sim_time:=true',*extra],stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
                pump(3)
                self.assertTrue(ready and ready[-1],ready[-10:])
                transform=buffer.lookup_transform('map','odom',rclpy.time.Time())
                self.assertAlmostEqual(transform.transform.translation.x,2.,places=6)
                self.assertEqual(len(driver.get_publishers_info_by_topic('/tf')),1)
                for topic in ['/cmd_vel','/base/safe_cmd_vel']:self.assertFalse(driver.get_publishers_info_by_topic(topic))
                pump(.9,wheel_on=False);self.assertFalse(ready[-1])
                pump(.6);self.assertTrue(ready[-1])
                pump(1.1,receipts_on=False);self.assertFalse(ready[-1])
                pump(.7);self.assertTrue(ready[-1])
                pump(.8,bad_anchor=True);self.assertFalse(ready[-1]);self.assertIn('anchor changed',converter.mapping.error)
                pump(.6);self.assertFalse(ready[-1])
                print('Verified ready -> wheel loss -> recovery -> receipts loss -> recovery -> anchor fault stays not ready',flush=True)
            finally:
                failed=sys.exc_info()[0] is not None
                if failed:
                    print('wheel clock',converter.mapping.status(time.time_ns(),time.monotonic_ns()),flush=True)
                    print('ready tail',ready[-15:],flush=True)
                    if health:print('bridge health',str(health[-1]),flush=True)
                for child in children:
                    if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
                for child in children:
                    try:child.wait(timeout=8)
                    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
                for log in logs:log.close()
                if failed or any(child.returncode for child in children):
                    for p in Path(folder).glob('*.log'):print(p.name,p.read_text())
                driver.destroy_node();converter.destroy_node();monitor.destroy_node();rclpy.shutdown()
if __name__=='__main__':unittest.main(verbosity=2)
