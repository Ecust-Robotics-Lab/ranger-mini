"""Process lifecycle tests with dummy Python children only; no ROS/hardware startup."""
import importlib.util,os,subprocess,sys,unittest
from pathlib import Path
spec=importlib.util.spec_from_file_location('stage_b',Path(__file__).resolve().parents[1]/'scripts/stage_b.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class Supervisor(unittest.TestCase):
    def test_pid_identity_guard(self):
        d={'supervisor_pid':os.getpid(),'supervisor_starttime':m.identity(os.getpid())}
        self.assertTrue(m.active_state(d));d['supervisor_starttime']='wrong';self.assertFalse(m.active_state(d))
    def test_command_order_and_parameters(self):
        from types import SimpleNamespace
        a=SimpleNamespace(lidar_config=Path('/test/config.json'),exposure_us=20000)
        commands=m.commands(a,Path('/test/logs'))
        self.assertEqual([x[0] for x in commands],['lidar','writer','camera'])
        self.assertIn('csv:=/test/logs/timeshare_writer.csv',commands[1][1])
        self.assertIn('sync_csv:=/test/logs/camera_sync.csv',commands[2][1])
        self.assertIn('exposure_us:=20000.0',commands[2][1])
    def test_cleanup_owned_dummy_process_groups(self):
        children=[]
        try:
            for name in ['dummy1','dummy2']:
                p=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)'],start_new_session=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
                children.append((name,p))
            m.stop_children(children)
            self.assertTrue(all(p.poll() is not None for _,p in children))
            # Cleanup must not signal the unrelated test process.
            self.assertIsNotNone(m.identity(os.getpid()))
        finally:
            for _,p in children:
                if p.poll() is None:p.kill();p.wait()
if __name__=='__main__':unittest.main(verbosity=2)
