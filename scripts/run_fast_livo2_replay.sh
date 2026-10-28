#!/usr/bin/env bash
set -e -o pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
execute=0
input_bag="$project_root/calibration/bags/stageB_sync_run2"
max_frames=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --execute) execute=1; shift ;;
    --bag) input_bag="$2"; shift 2 ;;
    --max-frames) max_frames="$2"; shift 2 ;;
    --runtime-config) export RANGER_RUNTIME_CONFIG="$2"; shift 2 ;;
    *) echo "Unsupported argument: $1" >&2; exit 2 ;;
  esac
done
[[ "$max_frames" =~ ^[0-9]+$ ]] || { echo 'max-frames must be nonnegative integer' >&2; exit 2; }
runtime_exports="$(python3 "$project_root/scripts/runtime_config.py" --shell --check-fast-livo)"
eval "$runtime_exports"
if [[ $execute == 0 ]]; then
  echo "DRY-RUN: offline FAST-LIVO2 direct-callback rosbag2 replay: $input_bag"
  echo "LiDAR+IMU+camera required; zero time offsets; trial calibration; no public TF; max_frames=$max_frames"
  echo "Configuration: $FAST_LIVO_CONFIG"
  echo 'Execute: bash scripts/run_fast_livo2_replay.sh --execute [--bag PATH] [--max-frames N]'
  exit 0
fi
[[ -f "$input_bag/metadata.yaml" ]] || { echo 'Bag metadata missing' >&2; exit 2; }
source /opt/ros/humble/setup.bash
source "$project_root/ws_livox/install/setup.bash"
export LD_LIBRARY_PATH="$project_root/ws_fast_livo/deps_install/lib:${LD_LIBRARY_PATH:-}"
export ROS_LOCALHOST_ONLY="$FAST_LIVO_LOCALHOST_ONLY"
export ROS_DOMAIN_ID="$FAST_LIVO_ROS_DOMAIN_ID"
export OMP_NUM_THREADS=4
run_dir="$(mktemp -d "$project_root/docs/fast_livo2_evidence/replay-XXXXXXXX")"
data_dir="$(mktemp -d "$project_root/bags/fast-livo2-baseline-XXXXXXXX")"
export ROS_LOG_DIR="$run_dir/ros"
mkdir -p "$ROS_LOG_DIR"
python3 - "$run_dir" "$data_dir" "$input_bag" "$max_frames" "$FAST_LIVO_CONFIG" <<'PY'
import sys, json, hashlib, os
from pathlib import Path
run,data,bag,limit,config_path=sys.argv[1:]
config=Path(config_path)
manifest={'input':str(Path(bag).resolve()),'output':data+'/result','max_frames':int(limit),
          'mode':'offline_direct_callbacks','ROS_DOMAIN_ID':int(os.environ['ROS_DOMAIN_ID']),'ROS_LOCALHOST_ONLY':int(os.environ['ROS_LOCALHOST_ONLY']),
          'calibration_status':'PARTIAL','physical_sync_proven':False,
          'config':str(config),'config_sha256':hashlib.sha256(config.read_bytes()).hexdigest()}
Path(run,'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
PY
echo "Evidence: $run_dir"
echo "Data: $data_dir/result"
set +e
/usr/bin/time -v -o "$run_dir/resources.txt" \
  "$project_root/ws_fast_livo/install/fast_livo/lib/fast_livo/ranger_livo_replay" \
  --ros-args --params-file "$FAST_LIVO_CONFIG" \
  -p "offline.bag:=$input_bag" -p "offline.output:=$data_dir/result" \
  -p "offline.max_frames:=$max_frames" --log-level warn > "$run_dir/frontend.log" 2>&1
result=$?
set -e
echo "$result" > "$run_dir/exit_code.txt"
echo "Replay exit: $result"
exit "$result"
