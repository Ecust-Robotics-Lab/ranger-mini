#!/usr/bin/env python3
"""Offline independent captures; original gyro initialization, bounded isolated runs."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import yaml

ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
OUT=ROOT/'outputs/fast-livo-tag140-two-capture-20261104'
CAPTURES={'turn':'fast-livo-tag140_turn_03-bk8_pjyy','straight':'fast-livo-tag140_straight_04-ryzykq2l'}

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--mode',choices=['lio','livo'],required=True)
    parser.add_argument('--offset-ns',type=int,default=0)
    args=parser.parse_args()
    if not args.execute:
        print(json.dumps({'action':'DRY-RUN','mode':args.mode,'offset_ns':args.offset_ns}));return 0
    if args.mode=='lio' and args.offset_ns:raise ValueError('LIO has no image offset')
    sys.path.insert(0,str(ROOT/'scripts/fast_livo2'))
    from online_session import check_sensor_processes
    env=dict(os.environ,ROS_DOMAIN_ID='196',ROS_LOCALHOST_ONLY='1',OMP_NUM_THREADS='4',OPENBLAS_NUM_THREADS='1')
    env['LD_LIBRARY_PATH']=str(ROOT/'ws_fast_livo/deps_install/lib')+':'+env.get('LD_LIBRARY_PATH','')
    source=ROOT/'ws_fast_livo/independent_time_source_20261104'
    baseline=json.loads((HERE/'isolated_source.json').read_text())
    assert all(sha(source/name)==h for name,h in baseline['files_sha256'].items())
    exe=ROOT/'ws_fast_livo/build_independent_time_20261104'/('ranger_motion_diagnostic' if args.mode=='lio' else 'ranger_livo_replay')
    for name,capture_name in CAPTURES.items():
        check_sensor_processes()
        folder=OUT/name/(args.mode+'_'+str(args.offset_ns));folder.mkdir(parents=True,exist_ok=False)
        capture=ROOT/'bags'/capture_name;manifest=json.loads((capture/'manifest.json').read_text())
        entry=next(r for r in manifest['source_files'] if r['source'].endswith('/ranger_mid360s_hik_trial.yaml'))
        archived=capture/entry['file'];assert sha(archived)==entry['sha256']
        config=yaml.safe_load(archived.read_text());p=config['/**']['ros__parameters']
        p['time_offset']['img_time_offset']=args.offset_ns*1e-9
        if args.mode=='lio':p['common'].update(img_en=0,require_livo=False)
        else:assert p['common']['img_en']==1 and p['common']['require_livo'] is True
        assert p['uav']['gravity_align_en'] and not p['publish']['native_tf'] and not p['pcd_save']['pcd_save_en']
        cfg=folder/'trial.yaml';cfg.write_text(yaml.safe_dump(config,sort_keys=False))
        argv=[str(exe),'--ros-args','--params-file',str(cfg),'-p','offline.bag:='+str(capture/'raw'),'-p','offline.output:='+str(folder/'result'),'--log-level','warn']
        provenance={'argv':argv,'domain':196,'localhost_only':True,'capture':str(capture),
                    'source_config_sha256':entry['sha256'],'trial_config_sha256':sha(cfg),'binary_sha256':sha(exe),
                    'gyro_initialization':'original zero; no mean-bias change','hardware_started':False,
                    'mode':args.mode,'offset_ns':args.offset_ns,'physical_sync_proven':False}
        (folder/'manifest.json').write_text(json.dumps(provenance,indent=2)+'\n')
        started=time.monotonic()
        with (folder/'replay.log').open('wb') as log:
            run=subprocess.run(['/usr/bin/time','-v','-o',str(folder/'resources.txt'),*argv],env=dict(env,ROS_LOG_DIR=str(folder/'ros_logs')),stdout=log,stderr=subprocess.STDOUT,timeout=180)
        provenance.update(exit_code=run.returncode,wall_seconds=time.monotonic()-started)
        for file in ['mat_pre.txt','mat_out.txt','imu.txt']:
            if (source/'Log'/file).exists():shutil.copy2(source/'Log'/file,folder/('frontend_'+file))
        (folder/'manifest.json').write_text(json.dumps(provenance,indent=2)+'\n');print(json.dumps(provenance),flush=True)
        if run.returncode:raise RuntimeError('offline replay failed; see log')
    return 0

if __name__=='__main__':raise SystemExit(main())
