"""Real C2 -> C1 static initialization in an isolated synthetic ROS world."""
import json
import os
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
import test_fast_livo_localizer_ros as c1

c1.EXECUTABLE=c1.ROOT/'ws_fast_livo/online_localization_install/ranger_fast_livo_localizer/lib/ranger_fast_livo_localizer/localizer_node'
c1.EVIDENCE=Path(os.environ.get('RANGER_LOCALIZER_TEST_EVIDENCE',str(c1.ROOT/'.runtime/validation/workbench/static_confirmation/ros')))
c1.EVIDENCE.mkdir(exist_ok=True,parents=True)

class StaticStartupRos(unittest.TestCase):
    setUp=c1.C1Ros.setUp
    tearDown=c1.C1Ros.tearDown
    spawn=c1.C1Ros.spawn
    pump=c1.C1Ros.pump
    wait=c1.C1Ros.wait
    sensors=c1.C1Ros.sensors
    send=c1.C1Ros.send
    on_health=c1.C1Ros.on_health

    def wheel_pose(self,ns):
        return c1.SE3((0.,0.,0.),(0.,0.,0.,1.))

    def test_online_eight_frame_static_search_then_independent_c1(self):
        self.frontend_origin=c1.SE3((100.,-50.,20.),c1.rpy_to_quat(.7,-.6,2.3))
        self.spawn([str(c1.EXECUTABLE),'--ros-args','--params-file',str(c1.ROOT/'config/fast_livo2/localizer.yaml'),
            '--params-file',str(c1.ROOT/'config/fast_livo2/relocalizer.yaml'),
            '-p','archive_path:='+str(self.archive),'-p','compatibility_path:='+str(self.compatibility),
            '-p','online_scheduling:=true','-p','t_base_imu_xyz:=[0.2,-0.1,0.4]',
            '-p','t_base_imu_rpy:=[0.1,-0.08,0.2]'],'static_online')
        self.wait(lambda:self.private and self.private[-1].map_uuid and json.loads(self.private[-1].quality_json).get('inputs_ready'))
        for _ in range(64):
            message=self.send(wheel=False)
            if message.pose_valid:break
        self.assertTrue(message.pose_valid,message.reason+' '+message.quality_json)
        self.assertTrue(message.initialization_request_id.startswith('global-'))
        handoffs=[json.loads(m.quality_json)['global_handoff'] for m in self.private if 'global_handoff' in json.loads(m.quality_json)]
        self.assertTrue(handoffs)
        handoff=handoffs[0]
        self.assertEqual(handoff['confirmation_mode'],'stationary_strong_geometry')
        self.assertGreaterEqual(handoff['confirmed_support_queries'],3)
        self.assertEqual(handoff['query_window_frames'],8)
        self.assertGreaterEqual(handoff['support_elapsed_s'],1.)
        self.assertTrue(all(not m.pose_valid for m in self.private if m.reason=='searching_geometric_registration'))
        sys.path.insert(0,str(c1.ROOT/'scripts'))
        from fast_livo2.mapping_monitor import Monitor
        # Persist a real node's confirmation telemetry without starting a monitor.
        sample=next(m for m in self.private if 'support_queries' in json.loads(m.quality_json))
        monitor=SimpleNamespace(folder=self.directory,localization=True,stamp=Monitor.stamp,get_clock=self.node.get_clock)
        Monitor.estimate(monitor,sample)
        persisted=json.loads((self.directory/'localization_samples.jsonl').read_text())
        self.assertIsNotNone(persisted['metrics']['support_queries'])
        self.assertTrue(persisted['metrics']['stationary_strong_geometry'])
        self.assertNotIn('candidates',persisted['metrics'])
        for topic in ['/cmd_vel','/cmd_vel_nav','/base/safe_cmd_vel']:
            self.assertFalse(self.node.get_publishers_info_by_topic(topic),topic)
        (self.directory/'result.json').write_text(json.dumps({'pass':True,'synthetic_static_no_hardware':True,
            'query_window_frames':8,'support_queries':handoff['confirmed_support_queries'],
            'support_elapsed_s':handoff['support_elapsed_s'],
            'phases':sorted({m.reason for m in self.private})},indent=2)+'\n')


if __name__=='__main__':unittest.main()
