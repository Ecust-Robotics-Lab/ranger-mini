"""One-off migration verification; temporarily hides only the compatibility symlink."""
import json
import os
from pathlib import Path
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
alias = Path('/home/lab307/calibration')
assert alias.is_symlink() and alias.resolve() == ROOT/'calibration'
target = os.readlink(alias)
commands = {
    'camera_package_resolution': 'source /opt/ros/humble/setup.bash; source ws_camera/install/setup.bash; ros2 pkg prefix hik_camera_trigger_driver; ros2 pkg prefix timeshare_writer',
    'calibration_cpp_checks': 'source /opt/ros/humble/setup.bash; calibration/lidar_camera_extrinsics/ws/install_integrated/fast_calib/lib/fast_calib/offline_checks',
    'calibration_real_bag_reader': 'source /opt/ros/humble/setup.bash; calibration/lidar_camera_extrinsics/ws/install_integrated/fast_calib/lib/fast_calib/input_probe calibration/bags/stageB_sync_run2',
    'real_writer_synthetic_ros': 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1; python3 scripts/fast_livo2/test_timeshare_writer.py',
    'livo_cpp_checks': 'source /opt/ros/humble/setup.bash; export LD_LIBRARY_PATH="$PWD/ws_fast_livo/deps_install/lib:${LD_LIBRARY_PATH:-}"; ws_fast_livo/install/fast_livo/lib/fast_livo/ranger_livo_checks',
    'fast_live_plan': 'bash scripts/run_fast_livo2_live.sh --runtime-config config/profiles/fast_livo2_frontend.yaml',
    'calibration_sync_plan': 'bash scripts/run_calibration.sh sync --execute -- --dry-run',
    'fast_short_replay': 'bash scripts/run_mapping.sh offline --runtime-config config/profiles/fast_livo2_frontend.yaml --execute --max-frames 20',
}
results = {}
alias.unlink()
try:
    for name, command in commands.items():
        env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
        with (EVIDENCE/(name+'.log')).open('w') as log:
            p = subprocess.run(['bash','-e','-o','pipefail','-c',command], cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=100)
        results[name] = {'command':command,'exit_code':p.returncode,'legacy_alias_absent':not alias.exists()}
        print(name,p.returncode,flush=True)
finally:
    assert not alias.exists() and not alias.is_symlink()
    alias.symlink_to(target)
    (EVIDENCE/'relocation_verification.json').write_text(json.dumps({'alias_restored':alias.resolve()==ROOT/'calibration','results':results},indent=2)+'\n')
assert len(results)==len(commands) and all(v['exit_code']==0 for v in results.values())
