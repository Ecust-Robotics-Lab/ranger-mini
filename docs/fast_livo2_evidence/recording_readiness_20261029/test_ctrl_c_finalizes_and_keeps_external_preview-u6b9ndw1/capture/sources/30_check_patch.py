#!/usr/bin/env python3
"""Apply the tracked patch to pristine HEAD bytes and compare deployed sources."""
import hashlib
import json
import os
import subprocess
import tempfile
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]


if __name__=='__main__':
    p=ROOT/'vendor/fast_livo2_review/ros2_frontend'
    patch=ROOT/'patches/fast_livo2_stage_a.patch'
    files=subprocess.check_output(['git','-C',str(p),'diff','--name-only'],text=True).splitlines()
    if not files:raise SystemExit('Expected the reviewed local patch')
    with tempfile.TemporaryDirectory(prefix='ranger-livo-patch-check-') as temp:
        d=Path(temp)
        for name in files:
            f=d/name;f.parent.mkdir(parents=True,exist_ok=True)
            f.write_bytes(subprocess.check_output(['git','-C',str(p),'show','HEAD:'+name]))
            mode=subprocess.check_output(['git','-C',str(p),'ls-tree','HEAD',name],text=True).split()[0]
            os.chmod(f,int(mode,8)&0o777)
        subprocess.run(['git','-C',str(d),'apply','--check',str(patch)],check=True)
        subprocess.run(['git','-C',str(d),'apply',str(patch)],check=True)
        for name in files:
            if (d/name).read_bytes()!=(p/name).read_bytes():raise SystemExit('Patch result mismatch: '+name)
    result={'passed':True,'source_commit':subprocess.check_output(['git','-C',str(p),'rev-parse','HEAD'],text=True).strip(),
            'patch_sha256':hashlib.sha256(patch.read_bytes()).hexdigest(),'files':files,
            'verification':'apply reviewed patch to pristine HEAD bytes/modes and compare actual source'}
    print(json.dumps(result,indent=2))
