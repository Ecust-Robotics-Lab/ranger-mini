#!/usr/bin/env python3
"""Backend routing and relocation contracts. Never starts physical drivers."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'ws_nav/src/ranger_localization_bridge'))
from ranger_localization_bridge.runtime_config import resolve


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


class BackendRouting(unittest.TestCase):
    def profile(self, path):
        return resolve(ROOT, {'RANGER_RUNTIME_CONFIG': str(path)})

    def test_production_defaults_unchanged(self):
        default = resolve(ROOT, {})
        fast = self.profile('config/profiles/fast_livo2_frontend.yaml')
        self.assertEqual(default['MAPPING_BACKEND'], 'lightning')
        self.assertEqual(default['LOCALIZATION_BACKEND'], 'lightning')
        self.assertEqual(fast['MAPPING_BACKEND'], 'fast_livo2')
        for key in ('LOC_CONFIG', 'MAP_YAML', 'MAP_BUNDLE', 'LOCALIZATION_BACKEND', 'LOC_CHILD_FRAME'):
            self.assertEqual(default[key], fast[key], key)

    def test_routes_both_modes_to_correct_existing_entry(self):
        runner = module('mapping_session_test', ROOT/'scripts/mapping_session.py')
        for profile, expected in [('lightning.yaml', ('run_slam_online.sh', 'run_slam_offline.sh')),
                                  ('fast_livo2_frontend.yaml', ('run_fast_livo2_live.sh', 'run_fast_livo2_replay.sh'))]:
            cfg = self.profile('config/profiles/'+profile)
            for mode, name in zip(('online', 'offline'), expected):
                dry = runner.command(mode, cfg)
                self.assertEqual(Path(dry[1]).name, name)
                self.assertNotIn('--execute', dry)
                self.assertIn('--execute', runner.command(mode, cfg, execute=True))

    def test_nested_profile_changes_reach_real_sensor_plan(self):
        session = module('online_session_test', ROOT/'scripts/fast_livo2/online_session.py')
        with tempfile.TemporaryDirectory(prefix='ranger profile ') as tmp:
            config = Path(tmp)/'frontend with spaces.yaml'
            config.write_bytes((ROOT/'config/fast_livo2/ranger_mid360s_hik_trial.yaml').read_bytes())
            profile = Path(tmp)/'runtime.yaml'
            profile.write_text(yaml.safe_dump({'extends':'config/profiles/fast_livo2_frontend.yaml',
                'mapping':{'fast_livo2':{'config':str(config),'ros_domain_id':179}}}))
            cfg = self.profile(profile)
            old_share = os.environ.get('RANGER_LIVO_TIMESHARE')
            os.environ['RANGER_LIVO_TIMESHARE'] = str(Path(tmp)/'private timeshare')
            try:
                specs = {c.name:c for c in session.specs(cfg)}
                self.assertIn(str(config), specs['frontend'].argv)
                self.assertIn('timeshare_path:='+os.environ['RANGER_LIVO_TIMESHARE'], specs['writer'].argv)
                self.assertIn('timeshare_path:='+os.environ['RANGER_LIVO_TIMESHARE'], specs['camera'].argv)
                self.assertIn(str(ROOT/'ws_camera/install/hik_camera_trigger_driver/lib/hik_camera_trigger_driver/hik_camera_trigger_driver_node'), specs['camera'].argv)
                self.assertEqual(cfg['FAST_LIVO_ROS_DOMAIN_ID'], '179')
            finally:
                if old_share is None:os.environ.pop('RANGER_LIVO_TIMESHARE')
                else:os.environ['RANGER_LIVO_TIMESHARE'] = old_share

    def test_cycle_missing_config_and_invalid_domain_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            profile = Path(tmp)/'runtime.yaml'
            profile.write_text(yaml.safe_dump({'extends':str(profile)}))
            with self.assertRaisesRegex(ValueError, 'cycle'):self.profile(profile)
            for change in ({'config':str(Path(tmp)/'missing.yaml')}, {'ros_domain_id':True}, {'localhost_only':'true'}):
                profile.write_text(yaml.safe_dump({'extends':'config/profiles/fast_livo2_frontend.yaml','mapping':{'fast_livo2':change}}))
                with self.assertRaises(ValueError):self.profile(profile)

    def test_cannot_disable_camera_imu_or_lidar(self):
        with tempfile.TemporaryDirectory() as tmp:
            profile, config = Path(tmp)/'runtime.yaml', Path(tmp)/'frontend.yaml'
            profile.write_text(yaml.safe_dump({'extends':'config/profiles/fast_livo2_frontend.yaml','mapping':{'fast_livo2':{'config':str(config)}}}))
            for group, key, disabled in [('common','img_en',0), ('common','lidar_en',0),
                                         ('imu','imu_en',False), ('common','require_livo',False)]:
                doc = yaml.safe_load((ROOT/'config/fast_livo2/ranger_mid360s_hik_trial.yaml').read_text())
                doc['/**']['ros__parameters'][group][key] = disabled
                config.write_text(yaml.safe_dump(doc))
                with self.assertRaisesRegex(ValueError, 'fusion'):self.profile(profile)

    def test_local_frontend_cannot_claim_global_localization(self):
        with tempfile.TemporaryDirectory() as tmp:
            profile = Path(tmp)/'bad.yaml'
            profile.write_text('extends: config/runtime.yaml\nlocalization_backend: fast_livo2\n')
            with self.assertRaisesRegex(ValueError, 'prior-map global localization is not implemented'):
                self.profile(profile)

    def test_direct_fast_entry_cannot_bypass_fusion_check(self):
        with tempfile.TemporaryDirectory() as tmp:
            profile, config = Path(tmp)/'runtime.yaml', Path(tmp)/'frontend.yaml'
            doc = yaml.safe_load((ROOT/'config/fast_livo2/ranger_mid360s_hik_trial.yaml').read_text())
            doc['/**']['ros__parameters']['common']['require_livo'] = False
            config.write_text(yaml.safe_dump(doc))
            profile.write_text(yaml.safe_dump({'extends':'config/runtime.yaml','mapping':{'fast_livo2':{'config':str(config)}}}))
            p = subprocess.run(['bash', str(ROOT/'scripts/run_fast_livo2_replay.sh'),'--runtime-config',str(profile)],capture_output=True,text=True)
            self.assertEqual(p.returncode,2)
            self.assertIn('fusion',p.stderr)

    def test_real_cli_defaults_to_dry_run(self):
        for profile in ('lightning.yaml', 'fast_livo2_frontend.yaml'):
            p = subprocess.run(['bash',str(ROOT/'scripts/run_mapping.sh'),'--runtime-config',
                'config/profiles/'+profile], check=True,capture_output=True,text=True)
            self.assertEqual(json.loads(p.stdout)['action'], 'DRY-RUN')


class CalibrationRelocation(unittest.TestCase):
    def test_historical_config_hash_and_rebased_inputs(self):
        common = module('calibration_common_test',ROOT/'calibration/lidar_camera_extrinsics/scripts/common.py')
        config = ROOT/'calibration/lidar_camera_extrinsics/config/scene_01/qr_params.yaml'
        original = config.read_bytes()
        parsed = common.read(config)
        original_bag = yaml.safe_load(original)['fast_calib']['ros__parameters']['bag_path']
        bag = parsed['fast_calib']['ros__parameters']['bag_path']
        self.assertTrue(original_bag.startswith('/home/lab307/calibration/'))
        self.assertTrue(bag.startswith(str(ROOT/'calibration')+'/'))
        self.assertTrue(Path(bag,'metadata.yaml').is_file())
        prov = common.read(str(config)+'.provenance.json')
        self.assertEqual(hashlib.sha256(original).hexdigest(), prov['config_sha256'])
        self.assertEqual(config.read_bytes(), original)
        self.assertEqual(common.sha(prov['intrinsics']['source']),prov['intrinsics']['sha256'])

    def test_calibration_commands_are_project_local_and_dry(self):
        for action in ('camera','intrinsics','capture','extrinsics','solve','sync'):
            p = subprocess.run([sys.executable,str(ROOT/'calibration/workflow.py'),action],
                               capture_output=True,text=True,check=True)
            result = json.loads(p.stdout)
            self.assertEqual(result['action'],'DRY-RUN')
            self.assertTrue(str(result['command'][1]).startswith(str(ROOT/'calibration')+'/'))
            self.assertNotIn('/home/lab307/calibration/',p.stdout)


if __name__ == '__main__':
    unittest.main(verbosity=2)
