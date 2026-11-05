#!/usr/bin/env python3
"""Replay locked current source from an isolated build, never the installed frontend."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import yaml

ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
CAPTURE=ROOT/'bags/fast-livo-tag140_sync_diag_02-35f_m8nd'
OUTPUT=ROOT/'outputs/fast-livo-tag140-sync-diag02-replay-current-20261104'
BUILD=ROOT/'ws_fast_livo/build_sync_diag_current_20261104'
sys.path.insert(0,str(ROOT/'scripts/fast_livo2'))
from online_session import check_sensor_processes
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    if sys.argv[1:]!=['--execute']:
        print('DRY-RUN: isolated current-source offline replays, offsets 0 / +120 ms, domain 181')
        return 0
    check_sensor_processes()
    OUTPUT.mkdir(parents=True,exist_ok=False)
    manifest=json.loads((CAPTURE/'manifest.json').read_text())
    source=next(r for r in manifest['source_files'] if r['source'].endswith('/ranger_mid360s_hik_trial.yaml'))
    archived=CAPTURE/source['file'];assert sha(archived)==source['sha256']
    env=dict(os.environ,ROS_DOMAIN_ID='181',ROS_LOCALHOST_ONLY='1',OMP_NUM_THREADS='4')
    env['LD_LIBRARY_PATH']=str(ROOT/'ws_fast_livo/deps_install/lib')+':'+env.get('LD_LIBRARY_PATH','')
    exe=BUILD/'ranger_livo_replay'
    cases=[]
    for offset in (0,120000000):
        check_sensor_processes()
        case=OUTPUT/str(offset);case.mkdir()
        cfg=yaml.safe_load(archived.read_text());params=cfg['/**']['ros__parameters']
        params['time_offset']['img_time_offset']=offset*1e-9
        assert params['common']['img_en']==1 and params['pcd_save']['pcd_save_en'] is False
        assert params['publish']['native_tf'] is False
        config=case/'trial.yaml';config.write_text(yaml.safe_dump(cfg,sort_keys=False))
        args=[str(exe),'--ros-args','--params-file',str(config),
              '-p','offline.bag:='+str(CAPTURE/'raw'),'-p','offline.output:='+str(case/'result'),
              '--log-level','warn']
        provenance={'argv':args,'source_config_sha256':source['sha256'],'trial_config_sha256':sha(config),
                    'frontend_binary_sha256':sha(exe),'assumed_img_offset_ns':offset,'domain':181,
                    'localhost_only':True,'physical_sync_proven':False,'hardware_started':False,
                    'source_snapshot':str(ROOT/'ws_fast_livo/sync_diag_source_20261104')}
        (case/'manifest.json').write_text(json.dumps(provenance,indent=2)+'\n')
        start=time.monotonic()
        with (case/'frontend.log').open('wb') as log:
            run=subprocess.run(['/usr/bin/time','-v','-o',str(case/'resources.txt'),*args],
                env=dict(env,ROS_LOG_DIR=str(case/'ros_logs')),stdout=log,stderr=subprocess.STDOUT,timeout=600)
        cases.append({'offset_ns':offset,'exit_code':run.returncode,'wall_seconds':time.monotonic()-start,
                      'directory':str(case)})
        (OUTPUT/'runs.json').write_text(json.dumps(cases,indent=2)+'\n')
        print(json.dumps(cases[-1]),flush=True)
        if run.returncode:raise RuntimeError('replay failed; see case log')
        summary=json.loads((case/'result/summary.json').read_text())
        assert summary['gravity_alignment_enabled'] and summary['gravity_alignment_finished']
        assert summary['gravity_alignment_events']==1
    return 0

if __name__=='__main__':raise SystemExit(main())
