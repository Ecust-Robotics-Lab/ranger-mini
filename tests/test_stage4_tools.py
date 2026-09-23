#!/usr/bin/env python3
import importlib.util,json,subprocess,tempfile,unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('map_check',ROOT/'scripts/check_slam_map.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class Checks(unittest.TestCase):
    def test_lzf_backreference(self):
        self.assertEqual(m.lzf(bytes([2])+b'abc'+bytes([32,2]),6),b'abcabc')
        with self.assertRaises(ValueError):m.lzf(bytes([32,99]),3)
    def test_complete_and_missing_chunk(self):
        with tempfile.TemporaryDirectory() as d:
            exp=Path(d);folder=exp/'data/new_map';folder.mkdir(parents=True)
            text='VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA ascii\n1 2 3\n4 5 6\n'
            (folder/'global.pcd').write_text(text);(folder/'0.pcd').write_text(text)
            (folder/'index.txt').write_text('0 0 0\n0 0 0 ./data/new_map/0.pcd\n# functional points\nstart 0 0 0 0 0 0 1\n')
            (folder/'keyframes.txt').write_text('0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 1\n')
            (exp/'config.yaml').write_text('system: {with_g2p5: false}\n')
            self.assertEqual(m.inspect(exp)['global_points'],2)
            (folder/'0.pcd').unlink()
            with self.assertRaises(FileNotFoundError):m.inspect(exp)
    def test_default_dry_runs_and_gates(self):
        for script in ['run_slam_offline.sh','run_slam_online.sh','record_slam_loop.sh']:
            p=subprocess.run(['bash',str(ROOT/'scripts'/script)],capture_output=True,text=True)
            self.assertEqual(p.returncode,0,p.stderr)
        for args in [['offline','--execute'],['online','--execute']]:
            p=subprocess.run(['/usr/bin/python3',str(ROOT/'scripts/slam_run.py'),*args],capture_output=True,text=True)
            self.assertEqual(p.returncode,2,p.stderr)
if __name__=='__main__':unittest.main()
