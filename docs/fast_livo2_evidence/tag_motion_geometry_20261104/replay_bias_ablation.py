#!/usr/bin/env python3
"""Experimental initialization only, using independently observed initial stillness."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import yaml

ROOT=Path(__file__).resolve().parents[3]
EV=Path(__file__).resolve().parent
OUT=ROOT/'outputs/fast-livo-tag140-motion-geometry-20261104'

def main():
    if sys.argv[1:]!=['--execute']:
        print('DRY-RUN: experimental gyro initialization ablation, no install/hardware');return 0
    sys.path.insert(0,str(ROOT/'scripts/fast_livo2'))
    from online_session import check_sensor_processes
    check_sensor_processes()
    env=dict(os.environ,ROS_DOMAIN_ID='195',ROS_LOCALHOST_ONLY='1',OMP_NUM_THREADS='4')
    env['LD_LIBRARY_PATH']=str(ROOT/'ws_fast_livo/deps_install/lib')+':'+env.get('LD_LIBRARY_PATH','')
    runs=[]
    for mode,offset in [('lio',0),('livo',0),('livo',110000000)]:
        check_sensor_processes()
        folder=OUT/f'bias_{mode}_{offset}';folder.mkdir()
        config=yaml.safe_load((ROOT/'outputs/fast-livo-tag140-sync-diag02-replay-current-20261104/0/trial.yaml').read_text())
        p=config['/**']['ros__parameters'];p['time_offset']['img_time_offset']=offset*1e-9
        if mode=='lio':p['common'].update(img_en=0,require_livo=False)
        assert not p['publish']['native_tf'] and not p['pcd_save']['pcd_save_en']
        cfg=folder/'trial.yaml';cfg.write_text(yaml.safe_dump(config,sort_keys=False))
        binary=ROOT/'ws_fast_livo/build_motion_diag_20261104'/('ranger_motion_diagnostic' if mode=='lio' else 'ranger_livo_replay')
        argv=[str(binary),'--ros-args','--params-file',str(cfg),'-p','offline.bag:='+str(ROOT/'bags/fast-livo-tag140_sync_diag_02-35f_m8nd/raw'),'-p','offline.output:='+str(folder/'result'),'--log-level','warn']
        start=time.monotonic()
        with (folder/'replay.log').open('wb') as log:
            r=subprocess.run(['/usr/bin/time','-v','-o',str(folder/'resources.txt'),*argv],env=dict(env,ROS_LOG_DIR=str(folder/'ros_logs')),stdout=log,stderr=subprocess.STDOUT,timeout=180)
        row={'mode':mode,'offset_ns':offset,'argv':argv,'exit_code':r.returncode,'wall_seconds':time.monotonic()-start,
             'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),'config_sha256':hashlib.sha256(cfg.read_bytes()).hexdigest(),
             'diagnostic_source_sha256':hashlib.sha256((ROOT/'ws_fast_livo/motion_diag_source_20261104/src/IMU_Processing.cpp').read_bytes()).hexdigest(),
             'diagnostic_only':True,'initial_static_segment_verified_from_images':True}
        runs.append(row);(EV/'bias_runs.json').write_text(json.dumps(runs,indent=2)+'\n');print(json.dumps(row),flush=True)
        if r.returncode:raise RuntimeError('diagnostic replay failed, inspect log')
    return 0

if __name__=='__main__':raise SystemExit(main())
