"""Real GICP C1 + native map_server + existing TF bridge, isolated synthetic time."""
import json
import math
import os
from pathlib import Path
import sys
import unittest
import numpy as np
from PIL import Image
import yaml

import test_fast_livo_localizer_ros as fixture
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile,DurabilityPolicy
from ranger_localization_msgs.msg import LocalizationEstimate
from ranger_localization_bridge.geometry import se3_from_tf_msg
import rclpy

ROOT=fixture.ROOT
EVIDENCE=ROOT/'docs/fast_livo2_evidence/navigation_prep_20261106'
fixture.EVIDENCE=EVIDENCE
fixture.EXECUTABLE=ROOT/'ws_fast_livo/online_localization_install/ranger_fast_livo_localizer/lib/ranger_fast_livo_localizer/localizer_node'
sys.path.insert(0,str(ROOT/'scripts/fast_livo2/navigation'))
from asset import load_map,sha


def projection(folder,archive):
    folder.mkdir()
    pixels=np.array([[0,205,254,0],[254,0,205,254],[205,254,0,205]],dtype=np.uint8)
    Image.fromarray(pixels).save(folder/'map.pgm')
    config={'image':'map.pgm','mode':'trinary','resolution':.1,'origin':[-1.,-2.,0.],
            'negate':0,'occupied_thresh':.65,'free_thresh':.196}
    (folder/'map.yaml').write_text(yaml.safe_dump(config))
    source=json.loads((archive/'manifest.json').read_text())
    manifest={'schema_version':1,'kind':'fast_livo_navigation_map_candidate','source_archive':str(archive),
        'source_manifest_sha256':sha(archive/'manifest.json'),'map_uuid':source['map_uuid'],
        'graph_version':source['graph_version'],'frame':'map','resolution':.1,'origin':config['origin'],
        'map_image_sha256':sha(folder/'map.pgm'),'map_yaml_sha256':sha(folder/'map.yaml')}
    (folder/'manifest.json').write_text(json.dumps(manifest))
    (folder/'COMMITTED').write_text(json.dumps({'manifest_sha256':sha(folder/'manifest.json')}))
    return np.flipud(np.where(pixels==0,100,np.where(pixels==254,0,-1))).ravel().tolist()


class NavigationROS(unittest.TestCase):
    def test_managed_map_binding_gicp_and_same_sample_wheel(self):
        from session_manager import read_session,session_live
        session=ROOT/'.runtime/sessions/fast_livo_navigation_prep'
        self.assertFalse(session_live(read_session(session)[1]),'preserve existing session')
        world=fixture.C1Ros('test_initialization_external_base_and_same_time_bridge_latch')
        world.setUp()
        try:
            world.launch('small_gicp')
            navmap=world.directory/'navigation_map'
            expected_pixels=projection(navmap,world.archive)
            geometry=world.directory/'geometry.yaml'
            geometry.write_text(yaml.safe_dump({'installation':{'t_base_imu_xyz':list(world.base_imu.t),'t_base_imu_rpy':[.1,-.08,.2]}}))
            maps=[]
            world.node.create_subscription(OccupancyGrid,'/map',maps.append,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            managed=world.spawn(['bash',str(ROOT/'scripts/bridge_fast_livo2_offline.sh'),'--execute',
                '--domain','196','--navigation-map',str(navmap),'--geometry-config',str(geometry)],'managed_bridge')
            world.wait(lambda: (read_session(session)[1] or {}).get('components',{}).get('bridge',{}).get('state')=='READY',timeout=35)
            world.wait(lambda: bool(maps))
            self.assertEqual(list(maps[-1].data),expected_pixels)
            self.assertEqual(maps[-1].header.frame_id,'map')
            self.assertAlmostEqual(maps[-1].info.origin.position.x,-1.)
            result,_=world.track()
            world.wait(lambda:world.ready)
            correction=se3_from_tf_msg(world.buffer.lookup_transform('map','odom',rclpy.time.Time()))
            error=world.correction.inverse()*correction
            self.assertLess(math.dist(error.t,(0,0,0)),.02)
            self.assertLess(abs(error.yaw()),.01)
            self.assertEqual(len(world.node.get_publishers_info_by_topic('/tf')),1)
            self.assertFalse(world.node.get_publishers_info_by_topic('/cmd_vel'))
            self.assertFalse(world.node.get_publishers_info_by_topic('/base/safe_cmd_vel'))
            for _ in range(3):world.send(wheel=False)
            self.assertFalse(world.ready)
            for _ in range(7):world.send()
            world.wait(lambda:world.ready)
            # A valid C1 pose in a different map may not drive this bound navigation map.
            other=world.directory/'other_map'
            compat=fixture.write_fixture(other,world.points,world.correction*world.base_imu)
            world.load(other,compat,'different-map')
            world.wait(lambda:world.private[-1].map_uuid!=result.map_uuid)
            world.track('different-map-initial')
            world.wait(lambda:world.standard[-1].state==LocalizationEstimate.ERROR)
            self.assertIn('bound navigation map',world.standard[-1].reason)
            self.assertFalse(world.ready)
            self.assertIsNone(managed.poll())
        finally:world.tearDown()
        self.assertFalse(session_live(read_session(session)[1]))

    def test_formal_projection_integrity_and_image_orientation(self):
        folder=ROOT/'outputs/fast-livo-demo-navigation-prep-20261106/navigation_map_final'
        manifest=load_map(folder)
        pixels=np.asarray(Image.open(folder/'map.pgm'))
        self.assertEqual((pixels.shape[1],pixels.shape[0]),(manifest['width'],manifest['height']))
        # Nav2 trinary semantics, checked by real map_server in the managed test above.
        self.assertEqual(int((pixels==0).sum()),manifest['cells']['occupied'])
        self.assertEqual(int((pixels==254).sum()),manifest['cells']['free'])
        self.assertEqual(int((pixels==205).sum()),manifest['cells']['unknown'])
        world=fixture.C1Ros('test_initialization_external_base_and_same_time_bridge_latch')
        world.setUp()
        try:
            maps=[]
            world.node.create_subscription(OccupancyGrid,'/map',maps.append,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            world.spawn(['bash',str(ROOT/'scripts/bridge_fast_livo2_offline.sh'),'--execute',
                         '--domain','196','--navigation-map',str(folder)],'formal_map_bridge')
            world.wait(lambda:bool(maps),timeout=35)
            grid=maps[-1]
            expected=np.flipud(np.where(pixels==0,100,np.where(pixels==254,0,-1))).ravel()
            np.testing.assert_array_equal(np.array(grid.data),expected)
            self.assertAlmostEqual(grid.info.resolution,manifest['resolution'])
            world.wait(lambda:world.buffer.can_transform('base_link','fast_livo_imu',rclpy.time.Time()))
            imu=se3_from_tf_msg(world.buffer.lookup_transform('base_link','fast_livo_imu',rclpy.time.Time()))
            np.testing.assert_allclose(imu.t,[.11485917776401003,-.023945937986232936,.5552533734244779],atol=1e-9)
            world.wait(lambda:world.buffer.can_transform('base_link','livox_frame',rclpy.time.Time()))
            lidar=se3_from_tf_msg(world.buffer.lookup_transform('base_link','livox_frame',rclpy.time.Time()))
            np.testing.assert_allclose(lidar.t,[.109,0.,.600],atol=1e-9)
            self.assertFalse(world.ready)
        finally:world.tearDown()


if __name__=='__main__':
    EVIDENCE.mkdir(parents=True,exist_ok=True)
    unittest.main(verbosity=2)
