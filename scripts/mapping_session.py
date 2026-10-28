#!/usr/bin/env python3
"""Choose the existing mapping entrypoint from the shared runtime configuration."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ws_nav/src/ranger_localization_bridge'))
from ranger_localization_bridge.runtime_config import resolve


def command(mode,cfg,extra=(),execute=False):
    backend=cfg['MAPPING_BACKEND']
    if backend=='lightning':
        argv=['bash',str(ROOT/f'scripts/run_slam_{mode}.sh'),'--config',cfg['LIGHTNING_MAPPING_CONFIG']]
    elif backend=='fast_livo2':
        argv=['bash',str(ROOT/('scripts/run_fast_livo2_live.sh' if mode=='online' else 'scripts/run_fast_livo2_replay.sh'))]
    else:raise ValueError('Unknown mapping backend: '+backend)
    return [*argv,*(['--execute'] if execute else []),*extra]


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode',choices=['online','offline'],nargs='?',default='online')
    p.add_argument('--execute',action='store_true');p.add_argument('--runtime-config')
    args,extra=p.parse_known_args()
    env=dict(os.environ)
    if args.runtime_config:env['RANGER_RUNTIME_CONFIG']=args.runtime_config
    try:
        cfg=resolve(ROOT,env);argv=command(args.mode,cfg,extra,args.execute)
        if not args.execute:
            print(json.dumps({'action':'DRY-RUN','mapping_backend':cfg['MAPPING_BACKEND'],
                              'localization_backend':cfg['LOCALIZATION_BACKEND'],'mode':args.mode,'command':argv,
                              'global_localization_provided_by_mapping_frontend':False if cfg['MAPPING_BACKEND']=='fast_livo2' else None},indent=2));raise SystemExit(0)
        env.update(cfg)
        raise SystemExit(subprocess.call(argv,env=env))
    except (ValueError,KeyError,OSError,TypeError,yaml.YAMLError) as e:p.exit(2,str(e)+'\n')
