#!/usr/bin/env python3
"""Project-local calibration procedures; plan by default, execute explicitly."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parent
TOOLS=ROOT/'lidar_camera_extrinsics/scripts'


def command(action,extra,execute=False):
    if action=='camera':return ['bash',str(ROOT/'camera_intrinsics/start_intrinsics_camera.sh'),*(['--execute'] if execute else []),*extra]
    if action=='intrinsics':return [sys.executable,str(ROOT/'camera_intrinsics/calibrate_intrinsics.py'),*extra]
    if action=='capture':return [sys.executable,str(ROOT/'camera_intrinsics/capture_checkerboard.py'),*extra]
    if action=='extrinsics':return [sys.executable,str(TOOLS/'calib.py'),*extra]
    if action=='solve':return [sys.executable,str(TOOLS/'solve.py'),*extra]
    if action=='sync':return ['bash',str(TOOLS/'start_stage_b.sh'),'--execute' if execute else '--dry-run',*extra]
    if action in ('status','stop'):return [sys.executable,str(TOOLS/'stage_b.py'),action]
    raise ValueError(action)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=['camera','intrinsics','capture','extrinsics','solve','sync','status','stop'])
    p.add_argument('--execute',action='store_true')
    args,extra=p.parse_known_args()
    if extra[:1]==['--']:extra=extra[1:]
    cmd=command(args.action,extra,args.execute)
    if not args.execute and args.action not in ('status','stop'):
        print(json.dumps({'action':'DRY-RUN','procedure':args.action,'command':cmd,
                          'starts_physical_drivers':args.action in ('camera','sync'),'calibration_acceptance':'unchanged; PARTIAL remains PARTIAL'},indent=2));raise SystemExit(0)
    raise SystemExit(subprocess.call(cmd))
