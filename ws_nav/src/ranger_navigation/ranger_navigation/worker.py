"""JAX/planning process with a single atomic-file mailbox; imports no ROS.

The supervisor never waits on a compute call. Requests/results have a process
nonce, task generation, request ID and monotonic deadline. Files are kept in
the owned session directory for diagnosis, with a bounded pair of mailboxes.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time
import uuid
import numpy as np


def save(path,meta,arrays=None):
    tmp = path.with_suffix('.tmp')
    with tmp.open('wb') as out:
        np.savez(out,meta=json.dumps(meta),**(arrays or {}))
    tmp.replace(path)


def read(path):
    with np.load(path,allow_pickle=False) as data:
        meta = json.loads(str(data['meta']))
        return meta,{k:data[k].copy() for k in data.files if k!='meta'}


class WorkerClient:
    def __init__(self,folder,python,geometry,backend='gpu',samples=256,limits=None,clearance=.04,static_reserve=.04,visibility_reserve=.10,prefer_parallel_in_narrow=False):
        self.folder = Path(folder)
        self.folder.mkdir(parents=True,exist_ok=True)
        self.nonce,self.sequence,self.pending = uuid.uuid4().hex,0,None
        self.request,self.result = self.folder/'request.npz',self.folder/'result.npz'
        self.result_identity = None
        self.log = (self.folder/'worker.log').open('a')
        env = dict(os.environ,XLA_PYTHON_CLIENT_PREALLOCATE='false',JAX_PLATFORMS='cuda' if backend=='gpu' else backend)
        source = str(Path(__file__).resolve().parents[1])
        env['PYTHONPATH'] = source+os.pathsep+env.get('PYTHONPATH','')
        self.process = subprocess.Popen([str(python),'-m','ranger_navigation.worker',
            '--folder',str(self.folder),'--geometry',str(geometry),'--nonce',self.nonce,
            '--samples',str(samples),'--limits',json.dumps(limits or {}),'--clearance',str(clearance),
            '--static-reserve',str(static_reserve),'--visibility-reserve',str(visibility_reserve),
            *(['--prefer-parallel-in-narrow'] if prefer_parallel_in_narrow else [])],
            env=env,stdout=self.log,stderr=subprocess.STDOUT)

    def status(self):
        if self.process.poll() is not None:
            return {'ready':False,'error':'worker_exited','exit':self.process.returncode}
        try:
            value = json.loads((self.folder/'status.json').read_text())
            return value if value.get('nonce')==self.nonce else {'ready':False}
        except (FileNotFoundError,ValueError):
            return {'ready':False}

    def submit(self,kind,generation,arrays,extra=None,budget=.1):
        if self.pending is not None or not self.status().get('ready'):
            return None
        self.sequence += 1
        meta = dict(kind=kind,generation=generation,id=self.sequence,nonce=self.nonce,
                    submitted=time.monotonic(),budget=budget,**(extra or {}))
        save(self.request,meta,arrays)
        self.pending = meta
        return meta

    def poll(self):
        if self.pending is None:
            return None
        pending = self.pending
        try:
            stat = self.result.stat()
            identity = (stat.st_ino,stat.st_mtime_ns)
            if identity==self.result_identity:
                raise FileNotFoundError('unchanged result mailbox')
            meta,arrays = read(self.result)
            self.result_identity = identity
            if (meta['nonce'],meta['id'])==(pending['nonce'],pending['id']):
                self.pending = None
                meta['elapsed'] = time.monotonic()-pending['submitted']
                if meta['elapsed']>pending['budget']:
                    meta['error'] = 'result_deadline_exceeded'
                return meta,arrays
        except (FileNotFoundError,ValueError,KeyError):
            pass
        if time.monotonic()-pending['submitted']>pending['budget']:
            self.pending = None
            return dict(pending,error='worker_timeout'),{}
        if self.process.poll() is not None:
            self.pending = None
            return dict(pending,error='worker_exited'),{}
        return None

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2.)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2.)
        self.log.close()


def run(args):
    from .geometry import load_geometry
    from .models import Limits
    from .planner import Planner,Grid
    from .exact_controller import ExactController
    import jax
    folder = Path(args.folder)
    body,_,_ = load_geometry(args.geometry,args.clearance)
    limits = Limits(**json.loads(args.limits))
    controller = ExactController(body,limits,samples=args.samples,visibility_reserve=args.visibility_reserve)
    warmup = controller.prewarm()
    status = dict(ready=True,pid=os.getpid(),nonce=args.nonce,devices=[str(d) for d in jax.devices()],warmup_s=warmup)
    tmp = folder/'status.tmp'
    tmp.write_text(json.dumps(status))
    tmp.replace(folder/'status.json')
    last_id,identity,request_identity = 0,None,None
    while True:
        try:
            stat = (folder/'request.npz').stat()
            current = (stat.st_ino,stat.st_mtime_ns)
            if current==request_identity:
                time.sleep(.001)
                continue
            meta,arrays = read(folder/'request.npz')
            request_identity = current
        except FileNotFoundError:
            time.sleep(.002)
            continue
        if meta['nonce']!=args.nonce or meta['id']<=last_id:
            time.sleep(.002)
            continue
        last_id = meta['id']
        output = {}
        began = time.monotonic()
        try:
            if began-meta['submitted']>meta['budget']:
                raise RuntimeError('request_expired_before_compute')
            if meta['kind']=='plan':
                planner = Planner(body,limits,timeout=min(5.,meta['budget']),static_reserve=args.static_reserve,
                                  prefer_parallel_in_narrow=args.prefer_parallel_in_narrow)
                grid = Grid(arrays['cells'],meta['resolution'],meta['origin']).with_obstacles(arrays['points'])
                grid.guidance_visibility_reserve = args.visibility_reserve
                if 'visibility_cells' in arrays and meta.get('global_visibility',True):
                    visible = Grid(arrays['visibility_cells'],.05,(-2.,-2.,0.))
                    grid.local_visibility = (visible,arrays['observer_pose'],arrays['pose'],.8)
                segments = planner.plan(grid,arrays['pose'],arrays['goal'],meta.get('actual_mode',0))
                meta['segments'] = [{'mode':int(s.mode),'reverse':s.reverse} for s in segments]
                output = {'segment_'+str(i):s.poses for i,s in enumerate(segments)}
            else:
                token = (meta['generation'],meta['mode'],meta['segment'])
                if token!=identity:
                    controller.reset(token)
                    identity = token
                visibility = (arrays['visibility_cells'],arrays['observer_pose']) if 'visibility_cells' in arrays else None
                output = controller.compute(arrays['pose'],arrays['speed'],arrays['path'],meta['mode'],arrays['points'],visibility)
        except Exception as error:
            meta['error'] = str(error)
        meta['compute_s'] = time.monotonic()-began
        save(folder/'result.npz',meta,output)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--folder',required=True)
    p.add_argument('--geometry',required=True)
    p.add_argument('--nonce',required=True)
    p.add_argument('--samples',type=int,default=256)
    p.add_argument('--limits',default='{}')
    p.add_argument('--clearance',type=float,default=.04)
    p.add_argument('--static-reserve',type=float,default=.04)
    p.add_argument('--visibility-reserve',type=float,default=.10)
    p.add_argument('--prefer-parallel-in-narrow',action='store_true')
    run(p.parse_args())


if __name__=='__main__':
    main()
