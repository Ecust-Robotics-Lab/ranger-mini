"""Active runtime/source lock and exact predecessor migration; no devices."""
import copy,json,os,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts/fast_livo2'),str(ROOT/'scripts'),str(ROOT/'ws_nav/src/ranger_localization_bridge')]
import mapping_session as mapping
from fast_livo2.frontend_upgrade import declaration,verify_active_frontend,sha
from ranger_localization_bridge.runtime_config import resolve

class FrontendUpgrade(unittest.TestCase):
 def setUp(self):
  with patch.dict(os.environ,{'RANGER_RUNTIME_CONFIG':str(ROOT/'config/profiles/fast_livo2_frontend.yaml')}):self.cfg=resolve(ROOT)
  self.current=mapping.frontend_identity(self.cfg);self.legacy=mapping.frontend_identity(self.cfg,legacy=True)
  self.archive=ROOT/'outputs/fast-livo-demo-changed-scene-20261110/mapping_replay_01/map'
 def test_actual_source_and_map_migration_are_explicit(self):
  before=sha(self.archive/'manifest.json');lock=verify_active_frontend()
  self.assertEqual(mapping.FRONTEND,ROOT/lock['binary']);self.assertNotEqual(self.current['frontend_sources_sha256'],self.legacy['frontend_sources_sha256'])
  for k in ('sensor_configuration_sha256','time_contract_sha256'):self.assertEqual(self.current[k],self.legacy[k])
  dest=declaration(self.archive,self.current,self.legacy)
  data=json.loads(dest.read_text());self.assertEqual(data['mode'],'frontend_timing_upgrade')
  self.assertEqual(data['from_frontend_sources_sha256'],self.legacy['frontend_sources_sha256'])
  self.assertEqual(data['to_frontend_sources_sha256'],self.current['frontend_sources_sha256'])
  self.assertEqual(sha(self.archive/'manifest.json'),before)
 def test_foreign_sensor_time_or_binary_cannot_use_the_upgrade(self):
  for k in ('sensor_configuration_sha256','time_contract_sha256','frontend_sources_sha256'):
   wrong=copy.deepcopy(self.current);wrong[k]='e'*64
   with self.assertRaises(ValueError):declaration(self.archive,wrong,self.legacy,persist=False)
 def test_public_navigation_uses_400ms_pose_and_keeps_other_bounds(self):
  from fast_livo2.navigation.session import components
  specs=components(self.archive.parent.parent/'navigation_map_05',ROOT/'config/fast_livo2/navigation_prep.yaml')
  adapter=next(c for c in specs if c.name=='adapter').argv;bridge=next(c for c in specs if c.name=='bridge').argv
  self.assertIn('sample_max_age:=0.4',adapter);self.assertIn('heartbeat_max_age:=0.25',adapter);self.assertIn('tf_max_age_s:=0.4',bridge)
 def test_sensor_preview_and_recording_use_the_active_binary(self):
  from record_session import specs
  with patch.dict(os.environ,{'RANGER_LIVO_TIMESHARE':'/tmp/offline-test-timeshare'}):
   components=specs(self.cfg,visualization='none')
  self.assertEqual(next(c for c in components if c.name=='frontend').argv[0],str(mapping.FRONTEND))

if __name__=='__main__':unittest.main()
