#!/usr/bin/env python3
"""Fetch immutable reviewed sources; never reset or overwrite an existing repo."""
import argparse
import json
import subprocess
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]


def run(args):
    print(' '.join(str(a) for a in args),flush=True)
    subprocess.run(args,check=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--all-references',action='store_true')
    parser.add_argument('--backend',action='store_true',help='also fetch the two locked stage-B dependencies')
    args=parser.parse_args()
    repos=json.loads((ROOT/'config/fast_livo2/sources.lock.json').read_text())['repositories']
    needed={'ros2_frontend','rpg_vikit','sophus'}
    if args.backend:needed.update({'gtsam','small_gicp'})
    for r in repos:
        if not args.all_references and r['name'] not in needed:continue
        dest=ROOT/r['path']
        if dest.exists():
            sha=subprocess.check_output(['git','-C',str(dest),'rev-parse','HEAD'],text=True).strip()
            if sha!=r['commit']:raise SystemExit('Existing source mismatch; inspect without reset: '+str(dest))
            print('Existing pinned repository: '+r['name']);continue
        print(f"Fetch {r['url']} at {r['commit']} -> {dest}",flush=True)
        if not args.execute:continue
        dest.parent.mkdir(parents=True,exist_ok=True)
        run(['git','clone','--no-checkout',r['url'],str(dest)])
        run(['git','-C',str(dest),'switch','--detach',r['commit']])
    if not args.execute:print('DRY-RUN; pass --execute to fetch missing repositories')
