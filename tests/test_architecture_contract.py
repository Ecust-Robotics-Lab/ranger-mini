#!/usr/bin/env python3
"""Runtime bundle and normalized localization contract tests; no actuators."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ws_nav/src/ranger_localization_bridge'))
os.environ['ROS_DOMAIN_ID'] = '181'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import yaml
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray
from ranger_localization_msgs.msg import LocalizationEstimate
from ranger_localization_bridge.runtime_config import resolve
from ranger_localization_bridge.health import readiness
import tf2_ros


class RuntimeTests(unittest.TestCase):
    def test_map_pair_and_seed_override(self):
        cfg = resolve(ROOT, {'INIT_X': '1.25'})
        self.assertEqual(Path(cfg['MAP_YAML']).parent, Path(cfg['MAP_DIR']))
        self.assertIn('2lx5kkho', cfg['LOC_CONFIG'] if '2lx5kkho' in cfg['LOC_CONFIG'] else
                      yaml.safe_load(Path(cfg['LOC_CONFIG']).read_text())['system']['map_path'])
        self.assertEqual(cfg['INIT_X'], '1.25')
        self.assertEqual(cfg['TF_MAX_AGE_S'], '0.5')

    def test_conflicting_legacy_map_rejected(self):
        with self.assertRaisesRegex(ValueError, 'conflicts'):
            resolve(ROOT, {'MAP_DIR': 'maps/lightning-offline-suej7qsz/data/new_map'})

    def test_invalid_overrides_rejected(self):
        for env in [{'TF_MAX_AGE_S': 'nan'}, {'INIT_X': '$(touch /tmp/never)'},
                    {'LIDAR_SECONDS': '7200'}, {'T_BASE_IMU_XYZ': '[0, 0]'},
                    {'INIT_QX':'0', 'INIT_QY':'0', 'INIT_QZ':'0', 'INIT_QW':'0'}]:
            with self.subTest(env=env), self.assertRaises(ValueError):
                resolve(ROOT, env)

    def test_legacy_arguments_are_validated(self):
        cfg = resolve(ROOT, {'BRIDGE_EXTRA_ARGS': 'tf_max_age_s:=0.8'})
        self.assertEqual(cfg['TF_MAX_AGE_S'], '0.8')
        self.assertEqual(cfg['BRIDGE_EXTRA_ARGS'], '')
        for value in ['unknown:=1', 'tf_max_age_s:=nan', 'tf_max_age_s:=0.8; touch /tmp/never']:
            with self.subTest(value=value), self.assertRaises(ValueError):
                resolve(ROOT, {'BRIDGE_EXTRA_ARGS': value})

    def test_backend_and_native_map_mismatch_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            bundle = yaml.safe_load((ROOT/'config/maps/field_20260928.yaml').read_text())
            path = Path(tmp)/'bundle.yaml'
            for change in [{'localization_backend': 'different'},
                           {'localization_map': 'maps/lightning-offline-suej7qsz/data/new_map'}]:
                bad = dict(bundle, **change)
                path.write_text(yaml.safe_dump(bad))
                with self.assertRaisesRegex(ValueError, 'mismatch|different native map'):
                    resolve(ROOT, {'MAP_BUNDLE': str(path)})

    def test_map_provenance_mismatch_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            bundle = yaml.safe_load((ROOT/'config/maps/field_20260928.yaml').read_text())
            bundle['navigation_map'] = 'maps/lightning-offline-suej7qsz/data/new_map/map.yaml'
            path = Path(tmp)/'bundle.yaml'; path.write_text(yaml.safe_dump(bundle))
            with self.assertRaisesRegex(ValueError, 'provenance'):
                resolve(ROOT, {'MAP_BUNDLE': str(path)})

    def test_safe_shell_roundtrip(self):
        # Exercise actual resolver output through bash with a quoted config path.
        with tempfile.TemporaryDirectory(prefix="ranger cfg '$HOME ") as tmp:
            config = Path(tmp)/'runtime.yaml'
            config.write_text((ROOT/'config/runtime.yaml').read_text())
            env = dict(os.environ, RANGER_RUNTIME_CONFIG=str(config), INIT_X='1.25\n')
            p = subprocess.run([sys.executable, str(ROOT/'scripts/runtime_config.py'), '--shell'],
                               text=True, capture_output=True, check=True, env=env)
            command = p.stdout + "\npython3 -c 'import os; print(os.environ[\"RANGER_RUNTIME_CONFIG\"]); print(os.environ[\"INIT_X\"])'"
            result = subprocess.run(['bash', '-c', command], text=True, capture_output=True, check=True)
            self.assertEqual(result.stdout.splitlines(), [str(config), '1.25'])

    def test_health_sample_time_not_heartbeat(self):
        m = LocalizationEstimate(); m.state=m.VALID
        m.header.stamp.sec=90
        ready, checks=readiness(100, m, 99.99, 99.99, .5, .5, .5, True)
        self.assertFalse(ready); self.assertFalse(checks['loc_tf_age_ok'])
        m.header.stamp.sec=101
        self.assertFalse(readiness(100,m,99.99,99.99,.5,.5,.5,True)[0])


class ExternalWorld(Node):
    def __init__(self):
        super().__init__('external_contract_world')
        self.pose_pub=self.create_publisher(LocalizationEstimate, '/localization/estimate',10)
        self.odom_pub=self.create_publisher(Odometry,'/wheel/odom',10)
        self.ready=None; self.diag={}; self.last_true=0; self.true_count=0
        self.create_subscription(Bool,'/navigation_ready',self.on_ready,10)
        self.create_subscription(DiagnosticArray,'/localization_health',self.on_diag,10)
        self.buf=tf2_ros.Buffer(); self.listener=tf2_ros.TransformListener(self.buf,self)
        self.mode='valid'; self.odom_on=True; self.estimate_on=True; self.frozen=None
        self.create_timer(.04,self.tick)

    def on_ready(self,m):
        self.ready=m.data
        if m.data:self.last_true=time.monotonic(); self.true_count+=1

    def on_diag(self,m):
        if m.status:self.diag={v.key:v.value for v in m.status[0].values}

    def tick(self):
        stamp=self.get_clock().now().to_msg()
        if self.odom_on:
            m=Odometry();m.header.stamp=stamp;m.header.frame_id='odom';m.child_frame_id='base_link'
            m.pose.pose.orientation.w=1.;self.odom_pub.publish(m)
        if self.estimate_on:
            m=LocalizationEstimate();m.header.stamp=stamp;m.header.frame_id='map';m.base_frame='base_link'
            m.pose.position.x=1.;m.pose.position.y=2.;m.pose.orientation.w=1.
            m.state=m.VALID;m.reason='independent synthetic backend'
            if self.mode in ['INITIALIZING','DEGRADED','LOST','ERROR']:m.state=getattr(m,self.mode)
            if self.mode=='frozen':
                if self.frozen is None:self.frozen=stamp
                m.header.stamp=self.frozen
            if self.mode=='future':m.header.stamp.sec+=5
            if self.mode=='frame':m.header.frame_id='wrong_map'
            if self.mode=='body':m.base_frame='imu_link'
            if self.mode=='nan':m.pose.position.x=float('nan')
            if self.mode=='zero_quat':m.pose.orientation.w=0.
            if self.mode=='nonunit':m.pose.orientation.w=2.
            if self.mode=='unknown':m.state=255
            if self.mode=='old':m.header.stamp.sec-=3
            self.pose_pub.publish(m)


def wait_for(node, predicate, timeout=6):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.02)
        if predicate():return True
    return False


class ExternalContractTests(unittest.TestCase):
    def test_external_backend_and_failures(self):
        # Separate process, isolated domain, direct generated-message input.
        # No Lightning TF/status/confidence producer exists in this test.
        rclpy.init()
        with tempfile.TemporaryDirectory() as tmp:
            log=open(Path(tmp)/'bridge.log','w')
            proc=subprocess.Popen([sys.executable,'-m','ranger_localization_bridge.bridge_node',
                '--ros-args','-p','adapter_backend:=external'],stdout=log,stderr=subprocess.STDOUT,
                start_new_session=True)
            node=ExternalWorld()
            try:
                self.assertTrue(wait_for(node,lambda:node.true_count>=5,12),'external backend never ready')
                tf=node.buf.lookup_transform('map','odom',rclpy.time.Time())
                self.assertAlmostEqual(tf.transform.translation.x,1.,places=5)
                self.assertAlmostEqual(tf.transform.translation.y,2.,places=5)
                self.assertNotIn('ranger_lightning_adapter',node.get_node_names())
                for mode in ['INITIALIZING','DEGRADED','LOST','ERROR','frame','body','nan','zero_quat','nonunit','unknown','old','future','frozen']:
                    with self.subTest(mode=mode):
                        node.mode=mode; node.frozen=None
                        self.assertTrue(wait_for(node,lambda:node.ready is False and (
                            node.diag.get('loc_tf_age_ok')=='0' if mode=='frozen' else
                            node.diag.get('backend_valid')=='0'),3),mode)
                        if mode=='frozen':
                            self.assertEqual(node.diag.get('loc_tf_age_ok'),'0')
                        # Hold invalid input, checking actual ready messages never re-enable.
                        deadline=time.monotonic()+.4; true_before=node.true_count
                        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
                        self.assertEqual(node.true_count,true_before,mode)
                        node.mode='valid'; before=node.true_count
                        self.assertTrue(wait_for(node,lambda:node.true_count>=before+3),mode+' recovery')
                node.estimate_on=False
                self.assertTrue(wait_for(node,lambda:node.ready is False and node.diag.get('backend_alive')=='0',3))
                node.estimate_on=True; before=node.true_count
                self.assertTrue(wait_for(node,lambda:node.true_count>=before+3))
                node.odom_on=False
                self.assertTrue(wait_for(node,lambda:node.ready is False and node.diag.get('odom_present')=='0',3))
                self.assertIsNone(proc.poll())
            finally:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                node.destroy_node();rclpy.shutdown();log.close()
                print((Path(tmp)/'bridge.log').read_text())


class LightningAdapterTests(unittest.TestCase):
    def test_extrinsics_and_backend_health(self):
        from std_msgs.msg import Int32, Float32
        from ranger_localization_bridge.geometry import SE3, rpy_to_quat, tf_msg_from_se3
        os.environ['ROS_DOMAIN_ID']='182'
        rclpy.init()
        node=Node('adapter_contract_world')
        br=tf2_ros.TransformBroadcaster(node)
        status=node.create_publisher(Int32,'/lightning/loc_status',10)
        confidence=node.create_publisher(Float32,'/lightning/loc_confidence',10)
        seen=[]
        node.create_subscription(LocalizationEstimate,'/localization/estimate',seen.append,10)
        mode=['valid']
        base=SE3((1.,2.,0.),rpy_to_quat(.1,.2,.3))
        mounting=SE3((.2,-.1,.556),rpy_to_quat(0.,0.,.4))
        def tick():
            br.sendTransform(tf_msg_from_se3(base*mounting,'map','imu_link',node.get_clock().now().to_msg()))
            if mode[0]!='no_status':status.publish(Int32(data=2))
            confidence.publish(Float32(data=0. if mode[0]=='weak' else float('nan') if mode[0]=='nan' else 3.))
        node.create_timer(.02,tick)
        with tempfile.TemporaryDirectory() as tmp:
            log=open(Path(tmp)/'adapter.log','w')
            proc=subprocess.Popen([sys.executable,'-m','ranger_localization_bridge.lightning_adapter',
                '--ros-args','-p','t_base_imu_xyz:=[0.2,-0.1,0.556]',
                '-p','t_base_imu_rpy:=[0.0,0.0,0.4]','-p','status_max_age_s:=0.3'],
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                self.assertTrue(wait_for(node,lambda:seen and seen[-1].state==LocalizationEstimate.VALID,12))
                m=seen[-1]
                for actual, expected in zip([m.pose.position.x,m.pose.position.y,m.pose.position.z],base.t):
                    self.assertAlmostEqual(actual,expected,places=6)
                for actual, expected in zip([m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w],base.q):
                    self.assertAlmostEqual(actual,expected,places=6)
                self.assertEqual(m.base_frame,'base_link');self.assertEqual(m.header.frame_id,'map')
                for phase, state in [('weak',m.DEGRADED),('nan',m.ERROR),('no_status',m.LOST)]:
                    mode[0]=phase
                    self.assertTrue(wait_for(node,lambda:seen[-1].state==state,3),phase)
                    mode[0]='valid'
                    self.assertTrue(wait_for(node,lambda:seen[-1].state==m.VALID,3))
                self.assertIsNone(proc.poll())
            finally:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                node.destroy_node();rclpy.shutdown();log.close()
                os.environ['ROS_DOMAIN_ID']='181'
                print((Path(tmp)/'adapter.log').read_text())


if __name__=='__main__':
    unittest.main(verbosity=2)
