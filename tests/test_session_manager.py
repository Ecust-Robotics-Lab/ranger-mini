#!/usr/bin/env python3
"""Synthetic Linux process ownership/dependency tests. No ROS or hardware."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from session_manager import (Component, JsonHealth, atomic_json, identity, read_session,
                             run_guarded, same_process, stop_session)


def wait_for(predicate,timeout=8):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        try:
            if predicate():return True
        except (OSError,ValueError,KeyError):pass
        time.sleep(.03)
    return False


def worker(scenario,base):
    base=Path(base); session=base/'sessions'/('two' if scenario=='resource_two' else 'one')
    python=sys.executable
    sleeping="import time; time.sleep(120)"
    if scenario=='dependency':
        first="from pathlib import Path; import time; Path(%r).touch(); time.sleep(120)"%str(base/'first')
        second="from pathlib import Path; import time; Path(%r).touch(); time.sleep(120)"%str(base/'second')
        # Deliberately reversed declaration order to verify dependency sorting.
        specs=[Component('second',[python,'-c',second],depends=('first',)),
               Component('first',[python,'-c',first],ready_key='input_ok')]
        provider=lambda _:JsonHealth(base/'health.json')
    elif scenario=='escape':
        code=("import subprocess,sys; from pathlib import Path; "
              "p=subprocess.Popen([sys.executable,'-c',%r],start_new_session=True); "
              "Path(%r).write_text(str(p.pid)); sys.exit(7)")%(sleeping,str(base/'escaped'))
        specs=[Component('escape',[python,'-c',code])];provider=lambda _:lambda:{}
    elif scenario=='preflight':
        specs=[Component('probe',[python,'-c',sleeping],ready_key='probe_live'),
               Component('must_not_start',[python,'-c',sleeping],depends=('probe',),start_requires=('free',))]
        provider=lambda _:JsonHealth(base/'health.json')
    elif scenario=='optional':
        specs=[Component('critical',[python,'-c',sleeping]),
               Component('optional',[python,'-c','import sys; sys.exit(3)'],critical=False)]
        provider=lambda _:lambda:{}
    elif scenario=='health_loss':
        specs=[Component('critical',[python,'-c',sleeping],ready_key='input_ok')]
        provider=lambda _:JsonHealth(base/'health.json')
    elif scenario=='timeout':
        specs=[Component('waiting',[python,'-c',sleeping],requires=('absent',))];provider=lambda _:lambda:{}
    else:
        code=("import subprocess,sys,time; from pathlib import Path; "
              "p=subprocess.Popen([sys.executable,'-c',%r],start_new_session=True); "
              "Path(%r).write_text(str(p.pid)); time.sleep(120)")%(sleeping,str(base/'escaped'))
        specs=[Component('parent',[python,'-c',code],resource='shared')];provider=lambda _:lambda:{}
    return run_guarded(session,specs,health_factory=provider,startup_timeout=.8 if scenario=='timeout' else 5.,
                       cleanup_grace=.2,fail_on_health_loss=scenario=='health_loss',health_loss_grace=.2)


class ProcessTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix='ranger-session-test-')
        self.base=Path(self.temp.name);self.workers=[];self.logs=[]
        self.unrelated=subprocess.Popen([sys.executable,'-c','import time; time.sleep(120)'])

    def spawn(self,scenario):
        log=(self.base/(scenario+str(len(self.logs))+'.log')).open('w');self.logs.append(log)
        proc=subprocess.Popen([sys.executable,__file__,'--worker',scenario,str(self.base)],stdout=log,stderr=subprocess.STDOUT)
        self.workers.append(proc)
        return proc

    def state(self):
        return read_session(self.base/'sessions/one')[1]

    def tearDown(self):
        for directory in [self.base/'sessions/one',self.base/'sessions/two']:
            try:stop_session(directory,timeout=4)
            except (OSError,ValueError):pass
        for p in self.workers:
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:p.kill();p.wait()
        self.assertIsNone(self.unrelated.poll(),'cleanup killed unrelated process')
        self.unrelated.terminate();self.unrelated.wait()
        for f in self.logs:f.close()
        for p in self.base.glob('*.log'):
            print(p.name,p.read_text())
        self.temp.cleanup()

    def test_dependency_wait_and_normal_stop(self):
        p=self.spawn('dependency')
        self.assertTrue(wait_for(lambda:(self.base/'first').exists()))
        self.assertFalse((self.base/'second').exists())
        atomic_json(self.base/'health.json',{'monotonic':time.monotonic(),'checks':{'input_ok':True}})
        self.assertTrue(wait_for(lambda:(self.base/'second').exists()))
        self.assertTrue(stop_session(self.base/'sessions/one'))
        self.assertEqual(p.wait(timeout=5),0)
        self.assertEqual(self.state()['owned'],[])

    def test_startup_timeout(self):
        p=self.spawn('timeout');self.assertEqual(p.wait(timeout=6),2)
        self.assertEqual(self.state()['state'],'FAILED')
        self.assertEqual(self.state()['components']['waiting']['state'],'WAITING')
        self.assertEqual(self.state()['owned'],[])

    def test_escaped_child_after_parent_exit(self):
        p=self.spawn('escape');self.assertEqual(p.wait(timeout=8),2)
        child=int((self.base/'escaped').read_text())
        self.assertFalse(same_process(identity(child)))
        self.assertEqual(self.state()['owned'],[])

    def test_scheduler_sigkill_guardian_cleans_orphans(self):
        p=self.spawn('normal')
        self.assertTrue(wait_for(lambda:(self.base/'escaped').exists() and self.state()['state']=='RUNNING'))
        child=identity(int((self.base/'escaped').read_text()))
        owner=self.state()['owner'];os.kill(owner['pid'],signal.SIGKILL)
        self.assertEqual(p.wait(timeout=8),2)
        self.assertFalse(same_process(child))
        self.assertTrue(self.state()['guardian_cleanup_complete'])

    def test_guardian_sigkill_scheduler_cleans(self):
        p=self.spawn('normal')
        self.assertTrue(wait_for(lambda:(self.base/'escaped').exists() and self.state()['state']=='RUNNING'))
        owner=self.state()['owner'];child=identity(int((self.base/'escaped').read_text()))
        os.kill(p.pid,signal.SIGKILL);p.wait(timeout=3)
        self.assertTrue(wait_for(lambda:not same_process(owner) and not same_process(child),8))
        self.assertEqual(self.state()['owned'],[])

    def test_resource_lease_and_duplicate_session(self):
        p=self.spawn('normal')
        self.assertTrue(wait_for(lambda:(self.base/'escaped').exists() and self.state()['state']=='RUNNING'))
        duplicate=self.spawn('normal');self.assertNotEqual(duplicate.wait(timeout=4),0)
        competing=self.spawn('resource_two');self.assertNotEqual(competing.wait(timeout=4),0)
        self.assertIsNone(p.poll())
        self.assertTrue(stop_session(self.base/'sessions/one'))
        self.assertEqual(p.wait(timeout=5),0)
        restarted=self.spawn('normal')
        self.assertTrue(wait_for(lambda:self.state()['state']=='RUNNING' and self.state()['guardian']['pid']==restarted.pid))
        self.assertTrue(stop_session(self.base/'sessions/one'))
        self.assertEqual(restarted.wait(timeout=5),0)

    def test_stale_identity_does_not_signal_reused_pid(self):
        session=self.base/'sessions/one';run=session/'run-fake';run.mkdir(parents=True)
        stale=dict(identity(self.unrelated.pid));stale['start_ticks']-=1
        atomic_json(session/'current.json',{'run_directory':str(run)})
        atomic_json(run/'state.json',{'guardian':stale,'owner':None,'owned':[stale]})
        self.assertTrue(stop_session(session,timeout=.2))
        self.assertIsNone(self.unrelated.poll())

    def test_occupied_interface_rejected_before_components_start(self):
        atomic_json(self.base/'health.json',{'monotonic':time.monotonic(),'checks':{'probe_live':True,'free':False}})
        p=self.spawn('preflight')
        self.assertEqual(p.wait(timeout=6),2)
        run,state=read_session(self.base/'sessions/one')
        starts=[json.loads(line)['component'] for line in (run/'events.jsonl').read_text().splitlines()
                if json.loads(line)['kind']=='started']
        self.assertNotIn('must_not_start',starts)
        self.assertTrue(all(name=='probe' for name in starts))
        self.assertIn('occupied interface',state['reason'])

    def test_optional_exit_degrades_without_restarting(self):
        p=self.spawn('optional')
        self.assertTrue(wait_for(lambda:self.state() and self.state()['state']=='DEGRADED'))
        self.assertEqual(self.state()['components']['optional']['exit_code'],3)
        self.assertIsNone(p.poll())
        self.assertTrue(stop_session(self.base/'sessions/one'))
        self.assertEqual(p.wait(timeout=5),0)

    def test_critical_health_loss_stops_without_auto_restart(self):
        atomic_json(self.base/'health.json',{'monotonic':time.monotonic(),'checks':{'input_ok':True}})
        p=self.spawn('health_loss')
        self.assertTrue(wait_for(lambda:self.state() and self.state()['state']=='RUNNING'))
        atomic_json(self.base/'health.json',{'monotonic':time.monotonic(),'checks':{'input_ok':False}})
        self.assertEqual(p.wait(timeout=6),2)
        self.assertEqual(self.state()['owned'],[])
        self.assertEqual(self.state()['state'],'FAILED')

    def test_control_start_requires_explicit_flag(self):
        for mode in ['base','navigation']:
            env=dict(os.environ,RANGER_SESSION_ROOT=str(self.base/'sessions'))
            p=subprocess.run([sys.executable,str(ROOT/'scripts/robot_session.py'),mode,'--execute'],
                             env=env,text=True,capture_output=True)
            self.assertEqual(p.returncode,2)
            self.assertIn('--control-authorized',p.stderr)
        self.assertFalse((self.base/'sessions').exists())


if __name__=='__main__':
    if len(sys.argv)>1 and sys.argv[1]=='--worker':raise SystemExit(worker(sys.argv[2],sys.argv[3]))
    unittest.main(verbosity=2)
