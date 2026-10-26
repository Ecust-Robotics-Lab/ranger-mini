#!/usr/bin/env python3
"""Managed field+Nav2 lifecycle test with synthetic sensors, no chassis driver."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
os.environ['ROS_DOMAIN_ID']='184'
os.environ['ROS_LOCALHOST_ONLY']='1'
os.environ['PYTHONDONTWRITEBYTECODE']='1'
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from livox_ros_driver2.msg import CustomMsg
from ranger_localization_msgs.msg import LocalizationEstimate
from session_manager import read_session, same_process


class SyntheticSensors(Node):
    def __init__(self):
        super().__init__('managed_synthetic_sensors')
        self.cloud=self.create_publisher(CustomMsg,'/livox/lidar',10)
        self.imu=self.create_publisher(Imu,'/livox/imu',10)
        self.wheel=self.create_publisher(Odometry,'/wheel/odom',10)
        self.pose=self.create_publisher(LocalizationEstimate,'/localization/estimate',10)
        self.final=[];self.ready=False;self.sensor_on=True
        self.create_subscription(Twist,'/base/safe_cmd_vel',self.final.append,10)
        self.create_subscription(Bool,'/navigation_ready',lambda m:setattr(self,'ready',m.data),10)
        self.create_timer(.04,self.tick)

    def tick(self):
        if not self.sensor_on:return
        now=self.get_clock().now()
        c=CustomMsg();c.header.stamp=now.to_msg();c.header.frame_id='livox_frame';self.cloud.publish(c)
        i=Imu();i.header=c.header;self.imu.publish(i)
        o=Odometry();o.header.stamp=now.to_msg();o.header.frame_id='odom';o.child_frame_id='base_link'
        o.pose.pose.orientation.w=1.;self.wheel.publish(o)
        m=LocalizationEstimate();m.header.stamp=(now-rclpy.duration.Duration(seconds=.12)).to_msg()
        m.header.frame_id='map';m.base_frame='base_link';m.pose.orientation.w=1.;m.state=m.VALID
        m.reason='synthetic managed test, no physical drivers';self.pose.publish(m)


def wait_for(node,predicate,timeout=15):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.01)
        try:
            if predicate():return True
        except (KeyError,TypeError,FileNotFoundError):pass
    return False


class ManagedRosTests(unittest.TestCase):
    def test_start_lifecycle_and_dependency_loss(self):
        with tempfile.TemporaryDirectory(prefix='ranger-managed-ros-') as tmp:
            base=Path(tmp);sessions=base/'sessions'
            cfg=yaml.safe_load((ROOT/'config/runtime.yaml').read_text())
            bundle=yaml.safe_load((ROOT/'config/maps/field_20260928.yaml').read_text())
            cfg['localization_backend']='standard';bundle['localization_backend']='standard'
            bundle_path=base/'map.yaml';bundle_path.write_text(yaml.safe_dump(bundle))
            cfg['map_bundle']=str(bundle_path);config_path=base/'runtime.yaml'
            config_path.write_text(yaml.safe_dump(cfg))
            env=dict(os.environ,RANGER_SESSION_ROOT=str(sessions),RANGER_RUNTIME_CONFIG=str(config_path),
                     ENABLE_BASE='0',ENABLE_LIDAR='0',ENABLE_LOC='0',ENABLE_AUX='1',ENABLE_BRIDGE='1',
                     ENABLE_FOXGLOVE='0',ENABLE_DERIVED='0',ENABLE_MAP_PUB='0')
            rclpy.init();node=SyntheticSensors();processes=[];logs=[]
            def state(mode):return read_session(sessions/mode)[1]
            def start(script,flags,name):
                log=(base/(name+'.log')).open('w');logs.append(log)
                p=subprocess.Popen(['bash',str(ROOT/'scripts'/script),*flags],env=env,
                    stdout=log,stderr=subprocess.STDOUT)
                processes.append(p);return p
            try:
                # This field profile contains only probe, static sensor TF and
                # generic bridge. No driver or native algorithm is started.
                field=start('field_session.sh',['--execute','--startup-timeout','20'],'field')
                self.assertTrue(wait_for(node,lambda:state('field') and state('field')['state']=='RUNNING',25))
                names=set(state('field')['components'])
                self.assertFalse(names & {'base','lidar','localization'})
                self.assertTrue(node.ready)
                nav=start('run_navigation.sh',['--execute','--control-authorized','--startup-timeout','25'],'navigation')
                self.assertTrue(wait_for(node,lambda:state('navigation') and state('navigation')['state']=='RUNNING',30))
                self.assertTrue(wait_for(node,lambda:len(node.final)>8,5))
                self.assertTrue(all(abs(m.linear.x)<1e-9 and abs(m.angular.z)<1e-9 for m in node.final))
                nav_state=state('navigation')
                nav_run=Path(nav_state['run_directory'])
                health=json.loads((nav_run/'health.json').read_text())
                self.assertTrue(all(v[0]==3 for v in health['lifecycle_states'].values()))
                # Removing synthetic inputs retracts readiness. Navigation's
                # health-loss watchdog must tear down this process tree.
                node.sensor_on=False
                self.assertTrue(wait_for(node,lambda:state('navigation')['state']=='FAILED',12))
                self.assertTrue(wait_for(node,lambda:nav.poll() is not None,8))
                self.assertEqual(nav.returncode,2)
                self.assertEqual(state('navigation')['owned'],[])
                self.assertIsNone(field.poll())  # no automatic restart or unrelated-session stop
                self.assertEqual(state('field')['state'],'DEGRADED')
                p=subprocess.run(['bash',str(ROOT/'scripts/field_session.sh'),'--stop'],env=env,
                                 capture_output=True,text=True,timeout=20)
                self.assertEqual(p.returncode,0,p.stderr)
                self.assertEqual(field.wait(timeout=5),0)
                self.assertEqual(state('field')['owned'],[])
            finally:
                for script in ['run_navigation.sh','field_session.sh']:
                    subprocess.run(['bash',str(ROOT/'scripts'/script),'--stop'],env=env,
                                   capture_output=True,text=True,timeout=25)
                for p in processes:p.wait(timeout=5)
                node.destroy_node();rclpy.shutdown()
                for log in logs:log.close()
                evidence=os.environ.get('EVIDENCE_DIR')
                for mode in ['field','navigation']:
                    run,s=read_session(sessions/mode)
                    if s:
                        print(mode,'final state',s['state'],'reason',s['reason'],'owned',s['owned'])
                        if evidence:
                            dest=Path(evidence)/mode;dest.mkdir(parents=True,exist_ok=True)
                            for file in run.iterdir():
                                if file.is_file():shutil.copy2(file,dest/file.name)
                for log in base.glob('*.log'):
                    print(log.name,log.read_text())
                    if evidence:shutil.copy2(log,Path(evidence)/log.name)


if __name__=='__main__':unittest.main(verbosity=2)
