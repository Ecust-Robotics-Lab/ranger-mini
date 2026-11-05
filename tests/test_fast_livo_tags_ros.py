#!/usr/bin/env python3
"""Synthetic offline image -> official detector -> existing C2/C1, no devices."""
import ctypes
import hashlib
import json
import os
from pathlib import Path
import subprocess
import unittest
import cv2
import numpy as np
import test_fast_livo_localizer_ros as c1
from std_msgs.msg import String
from sensor_msgs.msg import Image
from ranger_fast_livo_localizer.srv import Relocalize
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data

ROOT = c1.ROOT
c1.EVIDENCE = Path(os.environ.get('RANGER_TAG_TEST_EVIDENCE', str(ROOT/'docs/fast_livo2_evidence/apriltag_20261029/ros'))).resolve()
c1.EVIDENCE.mkdir(parents=True, exist_ok=True)


def matrix(p):
    from scipy.spatial.transform import Rotation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(p.q).as_matrix()
    T[:3, 3] = p.t
    return T


def project(T, size, K):
    points = np.asarray([[-size/2, size/2, 0], [size/2, size/2, 0],
                         [size/2, -size/2, 0], [-size/2, -size/2, 0]])
    camera = points @ T[:3, :3].T+T[:3, 3]
    normalized = camera[:, :2]/camera[:, 2:3]
    return normalized*np.asarray([K[0, 0], K[1, 1]])+np.asarray([K[0, 2], K[1, 2]])


def official_texture():
    class U8(ctypes.Structure):
        _fields_ = [('width', ctypes.c_int), ('height', ctypes.c_int), ('stride', ctypes.c_int),
                    ('buf', ctypes.POINTER(ctypes.c_uint8))]
    lib = ctypes.CDLL(str(ROOT/'ws_fast_livo/apriltag_deps/lib/libapriltag.so'))
    lib.tag36h11_create.restype = ctypes.c_void_p
    lib.apriltag_to_image.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.apriltag_to_image.restype = ctypes.POINTER(U8)
    lib.image_u8_destroy.argtypes = [ctypes.POINTER(U8)]
    lib.tag36h11_destroy.argtypes = [ctypes.c_void_p]
    family = lib.tag36h11_create()
    image = lib.apriltag_to_image(family, 0)
    view = np.ctypeslib.as_array(image.contents.buf, shape=(image.contents.stride*image.contents.height,))
    texture = view.reshape(image.contents.height, image.contents.stride)[:, :image.contents.width].copy()
    lib.image_u8_destroy(image)
    lib.tag36h11_destroy(family)
    return cv2.resize(texture, (200, 200), interpolation=cv2.INTER_NEAREST)


class TagRos(unittest.TestCase):
    tearDown = c1.C1Ros.tearDown
    spawn = c1.C1Ros.spawn
    pump = c1.C1Ros.pump
    wait = c1.C1Ros.wait
    send = c1.C1Ros.send
    sensors = c1.C1Ros.sensors
    wheel_pose = c1.C1Ros.wheel_pose
    on_health = c1.C1Ros.on_health
    initialize = c1.C1Ros.initialize

    def setUp(self):
        c1.C1Ros.setUp(self)
        self.E = c1.SE3((.1, -.04, .12), c1.rpy_to_quat(.03, -.07, .1))
        self.K = np.asarray([[420., 0., 320.], [0., 425., 240.], [0., 0., 1.]])
        self.config = {'schema_version': 1, 'calibration_id': 'synthetic-camera-ros-v1',
                       'time_mapping_id': 'synthetic-exposure-ros-v1', 'time_source': 'synthetic exact exposure midpoint',
                       'time_verified': True, 'offset_ns': 0, 'uncertainty_ns': 0,
                       'T_imu_camera': c1.pose_json(self.E), 'pixel_rmse': 4., 'pixel_max': 8.,
                       'camera': {'width': 640, 'height': 480, 'frame': 'front_optical_frame',
                                  'pixel_domain': 'raw', 'distortion_model': 'plumb_bob',
                                  'K': self.K.reshape(-1).tolist(), 'D': [0., 0., 0., 0., 0.]}}
        self.config_path = self.directory/'camera.json'
        self.config_path.write_text(json.dumps(self.config, indent=2)+'\n')
        self.F = self.correction*self.base_imu*self.E*c1.SE3((0., 0., 2.5), c1.rpy_to_quat(.2, -.2, .1))
        self.manifest = json.loads((self.archive/'manifest.json').read_text())
        self.source_sha = c1.sha(self.archive/'manifest.json')
        self.landmarks = self.directory/'landmarks'
        self.build_enrollment()
        self.context = None
        self.detected = []
        self.node.create_subscription(String, '/fast_livo/tags/context',
                                      lambda msg: setattr(self, 'context', json.loads(msg.data)),
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(String, '/fast_livo/tags/observations',
                                      lambda msg: self.detected.append(json.loads(msg.data)), qos_profile_sensor_data)
        self.images = self.node.create_publisher(Image, '/front/image_sync', qos_profile_sensor_data)
        self.tags = self.node.create_publisher(String, '/fast_livo/tags/observations', qos_profile_sensor_data)
        self.texture = official_texture()

    def observation(self, camera, stamp, image, context='enrollment'):
        T = matrix(camera.inverse()*self.F)
        return {'schema_version': 1, 'observation_id': image+':0', 'image_id': image, 'source_id': 'independent-synthetic-truth',
                'context': context, 'family': 'tag36h11', 'id': 0, 'size_m': .6,
                'canonical_frame': 'apriltag-native-xy-down-z-in-v1', 'image_stamp_ns': stamp, 'observation_stamp_ns': stamp,
                'calibration_id': self.config['calibration_id'], 'time_mapping_id': self.config['time_mapping_id'],
                'hamming': 0, 'decision_margin': 100., 'corners': project(T, .6, self.K).tolist(),
                'hypotheses': [{'hypothesis_id': 0, 'T_camera_tag': c1.pose_json(camera.inverse()*self.F), 'pixel_rmse': 0., 'pixel_max': 0.}],
                'provenance': {'size_source': 'synthetic exact metric', 'image_reference': 'synthetic:'+image,
                               'camera_model': self.config['camera'] | {'R': [1., 0., 0., 0., 1., 0., 0., 0., 1.]}}, 'quality': {}}

    def build_enrollment(self):
        records = []
        correction = self.correction*self.base_imu
        for k in range(4):
            ns = self.start_ns+10_000_000_000+k*1_000_000_000
            U = c1.SE3((.2*k, 0., 0.), c1.rpy_to_quat(0., .01*k, 0.))
            X = correction*U
            o = self.observation(X*self.E, ns, 'enrollment-image-'+str(k))
            segment = [{'stamp_ns': ns-delta, 'context': 'enrollment', 'correction': c1.pose_json(correction)}
                       for delta in [200_000_000, 100_000_000, 0]]
            records.append({'schema_version': 1, 'map_uuid': self.manifest['map_uuid'], 'source_manifest_sha256': self.source_sha,
                            'graph_version': self.manifest['graph_version'], 'state': 1, 'pose_valid': True,
                            'reason': 'map_match_confirmed', 'scan_stamp_ns': ns, 'last_confirmed_stamp_ns': ns,
                            'context': 'enrollment', 'visit_id': 'independent-synthetic-enrollment', 'segment_verified': True,
                            'segment_support': segment, 'T_map_imu': c1.pose_json(X),
                            'local_history': [{'stamp_ns': ns, 'T_local_imu': c1.pose_json(U)}], 'observation': o})
        source = self.directory/'enrollment.jsonl'
        source.write_text(''.join(json.dumps(row)+'\n' for row in records))
        job = {'archive_path': str(self.archive), 'compatibility_path': str(self.compatibility),
               'config_path': str(self.config_path), 'method': 'c1', 'records_path': str(source), 'destination': str(self.landmarks)}
        path = self.directory/'build_job.json'
        path.write_text(json.dumps(job))
        executable = ROOT/'ws_fast_livo/backend_install/ranger_fast_livo_localizer/lib/ranger_fast_livo_localizer/tag_map_tool'
        result = subprocess.run([str(executable), '--execute', str(path)], capture_output=True, text=True)
        (self.directory/'builder.log').write_text(result.stdout+result.stderr)
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        self.assertEqual(json.loads((self.landmarks/'landmarks.json').read_text())[0]['status'], 'VALID')

    def launch(self, broken=False, recording=False):
        if broken:
            with (self.landmarks/'landmarks.json').open('a') as output:
                output.write('tampered')
        command = [str(c1.EXECUTABLE), '--ros-args', '--params-file', str(ROOT/'config/fast_livo2/localizer.yaml'),
                   '--params-file', str(ROOT/'config/fast_livo2/relocalizer.yaml'), '-p', 'archive_path:='+str(self.archive),
                   '-p', 'compatibility_path:='+str(self.compatibility), '-p', 'tag_config_path:='+str(self.config_path),
                   '-p', 'tag_landmarks_path:='+str(self.landmarks), '-p', 'enable_tag_relocalization:=true', '-p', 'enable_tag_monitor:=true']
        if recording:
            command += ['-p', 'landmark_recording_mode:=c1', '-p', 'tag_record_path:='+str(self.directory/'confirmed.jsonl')]
        process = self.spawn(command, 'tag_localizer')
        self.wait(lambda: self.context and self.private and self.private[-1].map_uuid and json.loads(self.private[-1].quality_json).get('inputs_ready'))
        return process

    def image_at(self, stamp):
        camera = self.correction*self.wheel_pose(stamp)*self.base_imu*self.E
        corners = project(matrix(camera.inverse()*self.F), .6, self.K)
        H = cv2.getPerspectiveTransform(np.float32([[20, 180], [180, 180], [180, 20], [20, 20]]), corners.astype(np.float32))
        pixels = cv2.warpPerspective(self.texture, H, (640, 480), flags=cv2.INTER_NEAREST, borderValue=255)
        image = Image()
        image.header.frame_id = 'front_optical_frame'
        image.header.stamp.sec, image.header.stamp.nanosec = divmod(stamp, 10**9)
        image.width, image.height, image.step, image.encoding = 640, 480, 640, 'mono8'
        image.data = pixels.tobytes()
        return image

    def test_image_detector_c2_monitor_and_observer_exit_fallback(self):
        self.launch(recording=True)
        observer = self.spawn([str(ROOT/'ws_fast_livo/backend_install/ranger_fast_livo_backend/lib/ranger_fast_livo_backend/tag_observer_node'),
                               '--ros-args', '-p', 'use_sim_time:=true', '-p', 'tag_config_path:='+str(self.config_path),
                               '-p', 'tag_sizes_json:=\'{"0":0.6}\''], 'observer')
        self.wait(lambda: bool(self.images.get_subscription_count()))
        valid = False
        for _ in range(20):
            # Exposure lies between the previous and next scan. No fake sync.
            exposure = self.ns+20_000_000
            self.images.publish(self.image_at(exposure))
            self.pump(.08)
            result = self.send(wheel=False)
            valid |= result.pose_valid
        self.assertTrue(valid, result.reason)
        self.assertTrue(any(row.get('observations') for row in self.detected))
        qualities = [json.loads(m.quality_json or '{}') for m in self.private]
        self.assertTrue(any(q.get('tag_candidates') for q in qualities), str(qualities[-3:]))
        self.assertTrue(any(q.get('tag_monitor', {}).get('status') == 'CONSISTENT' for q in qualities))
        self.assertTrue((self.directory/'confirmed.jsonl').stat().st_size > 0)
        observer.terminate()
        observer.wait(timeout=5)
        for _ in range(4):
            self.assertTrue(self.send(wheel=False).pose_valid)
        for topic in ['/tf', '/tf_static', '/cmd_vel', '/base/safe_cmd_vel']:
            self.assertFalse(self.node.get_publishers_info_by_topic(topic), topic)
        self.assertEqual(c1.sha(self.archive/'manifest.json'), self.source_sha)

    def test_corrupt_landmarks_disable_only_tag_and_unknown_observation_is_safe(self):
        self.launch(broken=True)
        for _ in range(20):
            result = self.send(wheel=False)
            if result.pose_valid:
                break
        self.assertTrue(result.pose_valid, result.reason)
        self.assertIn('tag_branch_disabled', json.loads(result.quality_json)['tag_status'])
        camera = self.correction*self.wheel_pose(self.ns)*self.base_imu*self.E
        stale = self.observation(camera, self.ns, 'old-query', context='retired-context')
        self.tags.publish(String(data=json.dumps({'schema_version': 1, 'context': 'retired-context', 'observations': [stale]})))
        self.pump(.05)
        self.assertTrue(self.send(wheel=False).pose_valid)
        self.assertIn('stale_context', json.loads(self.private[-1].quality_json)['tag_status'])

    def test_pgo_each_optimized_keyframe_then_new_disabled_revision(self):
        def pcd(path, xyz):
            values = np.column_stack((xyz, np.arange(len(xyz)))).astype('<f4')
            header = (f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
                      f'WIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(xyz)}\nDATA binary\n').encode()
            path.write_bytes(header+values.tobytes())
        records = []
        frames = []
        for k in range(4):
            X = self.correction*self.base_imu*c1.SE3((.2*k, .01*k, 0.), c1.rpy_to_quat(.01*k, .02*k, .01*k))
            U = c1.SE3((3*k, -k, .2*k), c1.rpy_to_quat(.1*k, -.1*k, .2*k))
            stamp = 10_000_000_000+k*1_000_000_000
            shard = self.archive/f'keyframes/{k}'
            shard.mkdir(exist_ok=True)
            for name in ['full.pcd', 'match.pcd']:
                pcd(shard/name, c1.transform(self.points, X.inverse()))
            (shard/'scan.cdr').write_bytes(b'opaque synthetic PGO CDR')
            frames.append({'id': k, 'scan_id': k, 'stamp_ns': stamp, 'directory': f'keyframes/{k}',
                           'U_local_imu': c1.pose_json(U), 'X_map_imu': c1.pose_json(X), 'points': len(self.points)})
            records.append({'event': 'tag_attachment', 'session_id': self.manifest['session_id'], 'keyframe_id': k,
                            'scan_id': k, 'scan_stamp_ns': stamp, 'T_keyframe_camera': c1.pose_json(self.E),
                            'observation': self.observation(X*self.E, stamp, 'pgo-image-'+str(k), self.manifest['session_id'])})
        pcd(self.archive/'map.pcd', np.concatenate([self.points]*4))
        self.manifest['keyframes'] = frames
        self.manifest['graph_version'] = 9
        self.manifest['files'] = {str(p.relative_to(self.archive)): c1.sha(p) for p in self.archive.rglob('*')
                                  if p.is_file() and p.name not in ['manifest.json', 'COMMITTED']}
        (self.archive/'manifest.json').write_text(json.dumps(self.manifest, indent=2)+'\n')
        self.source_sha = c1.sha(self.archive/'manifest.json')
        (self.archive/'COMMITTED').write_text(json.dumps({'manifest_sha256': self.source_sha})+'\n')
        self.compatibility.write_text(json.dumps(c1.declaration(self.archive, [self.archive/'manifest.json']), indent=2)+'\n')
        source = self.directory/'pgo_observations.jsonl'
        source.write_text(''.join(json.dumps(row)+'\n' for row in records))
        destination = self.directory/'pgo-landmarks'
        job = {'archive_path': str(self.archive), 'compatibility_path': str(self.compatibility),
               'config_path': str(self.config_path), 'method': 'pgo', 'records_path': str(source), 'destination': str(destination)}
        job_path = self.directory/'pgo_job.json'
        job_path.write_text(json.dumps(job))
        tool = str(ROOT/'ws_fast_livo/backend_install/ranger_fast_livo_localizer/lib/ranger_fast_livo_localizer/tag_map_tool')
        result = subprocess.run([tool, '--execute', str(job_path)], capture_output=True, text=True)
        (self.directory/'pgo_builder.log').write_text(result.stdout+result.stderr)
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        anchor = json.loads((destination/'landmarks.json').read_text())[0]
        self.assertEqual(anchor['status'], 'VALID')
        self.assertLess(np.linalg.norm(np.asarray(anchor['T_map_tag']['xyz'])-np.asarray(self.F.t)), 1e-8)
        existing = subprocess.run([tool, '--execute', str(job_path)], capture_output=True, text=True)
        self.assertNotEqual(existing.returncode, 0)
        job.update(method='disable', landmarks_path=str(destination), disable=['tag36h11:0'],
                   reason='synthetic operator disable test', destination=str(self.directory/'disabled-landmarks'))
        job_path.write_text(json.dumps(job))
        disabled = subprocess.run([tool, '--execute', str(job_path)], capture_output=True, text=True)
        self.assertEqual(disabled.returncode, 0, disabled.stdout+disabled.stderr)
        self.assertEqual(json.loads((self.directory/'disabled-landmarks/landmarks.json').read_text())[0]['status'], 'DISABLED')
        self.assertEqual(json.loads((destination/'landmarks.json').read_text())[0]['status'], 'VALID')
        self.assertEqual(c1.sha(self.archive/'manifest.json'), self.source_sha)
        blocked_parent = self.directory/'write-denied'
        blocked_parent.mkdir()
        if os.geteuid() != 0:
            job['destination'] = str(blocked_parent/'must-not-commit')
            job_path.write_text(json.dumps(job))
            os.chmod(blocked_parent, 0o500)
            try:
                failed = subprocess.run([tool, '--execute', str(job_path)], capture_output=True, text=True)
                self.assertNotEqual(failed.returncode, 0)
                self.assertFalse((blocked_parent/'must-not-commit').exists())
                (self.directory/'write_failure.log').write_text(failed.stdout+failed.stderr)
            finally:
                os.chmod(blocked_parent, 0o700)

    def test_tag_recovery_does_not_unlock_public_innovation_latch(self):
        self.launch()
        extrinsics = ['-p', 't_base_imu_xyz:=[0.2,-0.1,0.4]', '-p', 't_base_imu_rpy:=[0.1,-0.08,0.2]']
        self.spawn([c1.sys.executable, '-m', 'ranger_localization_bridge.fast_livo_localizer_adapter', '--ros-args',
                    '-p', 'use_sim_time:=true', *extrinsics], 'adapter')
        self.spawn([c1.sys.executable, '-m', 'ranger_localization_bridge.bridge_node', '--ros-args',
                    '-p', 'use_sim_time:=true', '-p', 'adapter_backend:=external',
                    '-p', 'latch_on_innovation_failure:=true'], 'bridge')

        def tagged_sample():
            exposure = self.ns+20_000_000
            camera = self.correction*self.wheel_pose(exposure)*self.base_imu*self.E
            o = self.observation(camera, exposure, 'query-'+str(exposure), self.context['context'])
            self.tags.publish(String(data=json.dumps({'schema_version': 1, 'context': self.context['context'], 'observations': [o]})))
            self.pump(.04)
            return self.send()

        for _ in range(20):
            tagged_sample()
            if self.ready:
                break
        self.assertTrue(self.ready)
        # Innovation compares an established >=0.5 s motion interval. The first
        # accepted pose intentionally cannot supply that interval by itself.
        for _ in range(7):
            self.assertTrue(tagged_sample().pose_valid)
        self.assertTrue(self.send(wheel_jump=10.).pose_valid)
        self.wait(lambda: self.health.get('innovation_unlocked') == '0')
        old_context = self.context['context']
        current = self.private[-1]
        client = self.node.create_client(Relocalize, '/fast_livo/localizer/relocalize')
        self.assertTrue(client.wait_for_service(timeout_sec=2))
        request = Relocalize.Request(request_id='explicit-tag-recovery', enable=True, map_uuid=current.map_uuid,
                                     map_generation=current.map_generation, session_id=current.session_id)
        future = client.call_async(request)
        self.wait(future.done)
        self.assertTrue(future.result().accepted, future.result().reason)
        self.wait(lambda: self.context['context'] != old_context)
        accepted = False
        for _ in range(20):
            result = tagged_sample()
            if result.pose_valid:
                accepted = True
                break
        self.assertTrue(accepted, result.reason)
        self.assertFalse(self.ready)
        self.assertEqual(self.health.get('innovation_unlocked'), '0')
        self.assertEqual(len(self.node.get_publishers_info_by_topic('/tf')), 1)

    def test_consumer_queue_age_uses_steady_time_when_sim_clock_pauses(self):
        self.launch()
        for _ in range(20):
            result = self.send(wheel=False)
            if result.pose_valid:
                break
        self.assertTrue(result.pose_valid)
        exposure = self.ns+20_000_000
        camera = self.correction*self.wheel_pose(exposure)*self.base_imu*self.E
        o = self.observation(camera, exposure, 'delayed-query', self.context['context'])
        self.tags.publish(String(data=json.dumps({'schema_version': 1, 'context': self.context['context'], 'observations': [o]})))
        self.pump(.7)  # ROS /clock stays fixed; wall-clock queue age still expires.
        result = self.send(wheel=False)
        self.assertTrue(result.pose_valid, result.reason)
        self.assertIn('consumer_queue_expired', json.loads(result.quality_json)['tag_status'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
