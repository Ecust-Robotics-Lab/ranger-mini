"""Offline mapping stays capture-bound and preserves integer nanoseconds."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/fast_livo2/navigation'))
from capture_clock import CaptureClock


class ClockTests(unittest.TestCase):
    def test_scope_precision_and_metadata_change(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);capture=p/'capture';(capture/'raw').mkdir(parents=True)
            meta=capture/'raw/metadata.yaml';meta.write_bytes(b'fixture metadata')
            analysis=p/'analysis';analysis.mkdir()
            offset=203782752743349211
            info={'source_bag':str(capture/'raw'),'source_metadata_sha256':hashlib.sha256(meta.read_bytes()).hexdigest(),
                'candidate_sensor_to_host_offset_ns':offset,
                'sign_corrected_angular_fits':{'first_half':{'lag_seconds':.002},'second_half':{'lag_seconds':-.018}}}
            (analysis/'analysis.json').write_text(json.dumps(info))
            mapping=CaptureClock(analysis,capture)
            ns=1793974815789677819
            self.assertEqual(mapping.host_to_sensor(ns)+offset,ns)
            np.testing.assert_array_equal(mapping.host_to_sensor(np.array([ns,ns+1],dtype=np.int64)),[ns-offset,ns+1-offset])
            self.assertFalse(mapping.to_dict()['physical_sync_verified'])
            self.assertAlmostEqual(mapping.window_lag_spread_s,.02)
            with self.assertRaisesRegex(ValueError,'another capture'):CaptureClock(analysis,p/'other')
            meta.write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError,'metadata changed'):CaptureClock(analysis,capture)


if __name__=='__main__':unittest.main(verbosity=2)
