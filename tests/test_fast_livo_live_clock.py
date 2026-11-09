import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/fast_livo2/navigation'))
from live_clock import LiveClock, StartupAnchor

class LiveClockTests(unittest.TestCase):
    def receipt(self,c=12,t=1_100_000_000):
        return {'schema_version':1,'context':'session-a','topics':{'/livox/lidar':{
            'count':c,'first_header_ns':1_590_000_000_000_000_001,'first_steady_ns':10_000_000_000,
            'last_header_ns':1_590_000_000_000_000_001+t,'last_steady_ns':10_000_000_000+t}}}
    def clock(self):
        x=LiveClock('session-a');self.assertTrue(x.receipts(self.receipt(),1_793_000_001_100_000_000,11_100_000_000));return x
    def test_original_sample_precision(self):
        x=self.clock();stamp=1_793_000_001_090_000_007
        self.assertEqual(x.convert(stamp,1_793_000_001_100_000_000,11_100_000_000),1_590_000_001_090_000_008)
    def test_acquisition_and_receipt_loss(self):
        x=LiveClock('session-a');self.assertIsNone(x.convert(1,2,3))
        x=self.clock();self.assertFalse(x.ready(1_793_000_002_100_000_000,12_100_000_000))
        self.assertTrue(x.receipts(self.receipt(22,2_100_000_000),1_793_000_002_100_000_000,12_100_000_000))
        self.assertTrue(x.ready(1_793_000_002_100_000_000,12_100_000_000))
    def test_session_host_jump_and_sensor_divergence(self):
        x=self.clock();r=self.receipt();r['context']='session-b';self.assertFalse(x.receipts(r,0,0))
        self.assertFalse(x.ready(1_793_000_002_100_000_000,11_100_000_000));self.assertIn('stepped',x.error)
        x=self.clock();r=self.receipt(13,1_200_000_000);r['topics']['/livox/lidar']['last_header_ns']+=200_000_000
        self.assertFalse(x.receipts(r,1_793_000_001_200_000_000,11_200_000_000));self.assertIn('divergence',x.error)
    def test_anchor_counter_regression_and_future_wheel(self):
        x=self.clock();r=self.receipt();r['topics']['/livox/lidar']['first_header_ns']+=1
        self.assertFalse(x.receipts(r,1_793_000_001_100_000_000,11_100_000_000))
        x=self.clock();self.assertFalse(x.receipts(self.receipt(1),1_793_000_001_100_000_000,11_100_000_000))
        x=self.clock();self.assertIsNone(x.convert(1_793_000_001_100_000_001,1_793_000_001_100_000_000,11_100_000_000))
    def test_startup_burst_acquires_stable_window_once(self):
        x=StartupAnchor();row=self.receipt(1,0)['topics']['/livox/lidar']
        self.assertIsNone(x.observe(row,10_000_000_000))
        for i in range(1,14):
            row.update(count=i+1,last_header_ns=1_590_000_000_000_000_001+2_900_000_000+i*100_000_000,
                       last_steady_ns=10_000_000_000+i*100_000_000)
            x.observe(row,row['last_steady_ns'])
        self.assertIsNotNone(x.anchor);self.assertEqual(x.resets,2)
        saved=x.anchor;row['last_header_ns']+=5_000_000_000
        self.assertEqual(x.observe(row,row['last_steady_ns']),saved)
    def test_authority_uses_stable_anchor_and_checks_raw_session(self):
        x=LiveClock('session-a')
        source=[1_590_000_000_000_000_001,10_000_000_000]
        x.set_anchor([source[0]+2_900_000_000,source[1]],source,1_793_000_001_100_000_000,11_100_000_000)
        row=self.receipt();row['topics']['/livox/lidar']['last_header_ns']+=2_900_000_000
        self.assertTrue(x.receipts(row,1_793_000_001_100_000_000,11_100_000_000))
        self.assertTrue(x.ready(1_793_000_001_100_000_000,11_100_000_000))
        row['topics']['/livox/lidar']['first_header_ns']+=1
        self.assertFalse(x.receipts(row,1_793_000_001_100_000_000,11_100_000_000))
if __name__=='__main__':unittest.main()
