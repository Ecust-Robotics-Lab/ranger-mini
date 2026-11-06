"""Verify preserved assets against pre-move identities, and record integrated sources."""
import hashlib
import json
import os
from pathlib import Path
import subprocess

r=Path(__file__).resolve().parents[3];cal=r/'calibration';out=Path(__file__).resolve().parent
old=json.loads((out/'migration_tree_before.json').read_text())
checks={};problems=[]
for name,meta in old.items():
    protected=(name.startswith(('bags/','ws_calib/','camera_intrinsics/results/','camera_intrinsics/results_archive/','camera_intrinsics/images/','camera_intrinsics/session_','camera_intrinsics/combined_','lidar_camera_extrinsics/datasets/','lidar_camera_extrinsics/config/','lidar_camera_extrinsics/results/')) or name.endswith(('STAGE_B_REPORT.md','STAGE_C2_REPORT.md','TRIAL_CALIBRATION_REPORT.md')))
    if not protected:continue
    p=cal/name
    try:
        st=p.lstat();ok=st.st_ino==meta['inode'] and st.st_size==meta['size']
        if 'symlink' in meta:ok=ok and p.is_symlink() and os.readlink(p)==meta['symlink']
        else:ok=ok and st.st_mtime_ns==meta['mtime_ns']
    except FileNotFoundError:ok=False
    checks[name]=ok
    if not ok:problems.append(name)
output={'check':'original inode, size, mtime_ns or symlink target from before atomic move; protected capture/result/config/legacy workspace entries','entries_checked':len(checks),'all_unchanged':not problems,'problems':problems}
(out/'protected_assets.json').write_text(json.dumps(output,indent=2)+'\n')
print(output)
assert not problems
files=[]
for base in [cal/'camera_intrinsics',cal/'lidar_camera_extrinsics/scripts',cal/'lidar_camera_extrinsics/ws/src',r/'ws_camera/src/hik_camera_trigger_driver',r/'ws_camera/src/timeshare_writer']:
    for p in base.rglob('*'):
        if p.is_file() and not any(x in p.parts for x in ('hikSDK','results','images','results_archive','__pycache__')) and p.suffix in ('.cpp','.hpp','.h','.py','.sh','.xml'):
            files.append(p)
manifest={'integration_date':'2026-10-26','original_calibration_commit':'82936bbbd12c5d5d93cf9f9d2564ec6a5c37fac8','original_camera_commit':subprocess.check_output(['git','-C',str(r/'ws_camera/src/hik_camera_ros2_driver'),'rev-parse','HEAD'],text=True).strip(),'original_camera_local_changes_preserved':True,'calibration_upstream_versions':'calibration/lidar_camera_extrinsics/upstream/SOURCE_MANIFEST.json; unverified entries remain unverified','sdk':'existing ws_camera/src/hik_camera_ros2_driver/hikSDK; proprietary local dependency; new trigger package uses relative symlink','source_sha256':{str(p.relative_to(r)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}}
(r/'config/calibration_sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
