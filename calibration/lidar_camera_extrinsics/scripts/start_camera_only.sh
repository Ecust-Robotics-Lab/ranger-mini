#!/usr/bin/env bash
set -eo pipefail
C2_ROOT=$(cd -- "$(dirname -- "$0")/.." && pwd)
MODE=${1:-free}
if [[ "$MODE" != free && "$MODE" != trigger ]]; then
  echo 'Usage: start_camera_only.sh [free|trigger] [--dry-run]'; exit 2
fi
if [[ "$MODE" == free ]]; then
  PARAMS="$C2_ROOT/config/camera_free_run.yaml"
else
  PARAMS="$C2_ROOT/../../ws_camera/install/hik_camera_trigger_driver/share/hik_camera_trigger_driver/config/camera_params_trigger.yaml"
fi
source /opt/ros/humble/setup.bash
source "$C2_ROOT/../../ws_camera/install/setup.bash"
RUN="$C2_ROOT/logs/camera_only_$(date +%Y%m%d_%H%M%S_%N)"
CMD=(ros2 launch hik_camera_trigger_driver trigger_launch.py "params_file:=$PARAMS" exposure_us:=20000.0 "sync_csv:=$RUN/camera_sync.csv")
if [[ "${2:-}" == --dry-run ]]; then
  printf '%q ' "${CMD[@]}"; echo; exit 0
fi
/usr/bin/python3 - "$C2_ROOT/scripts" <<'PY'
import sys
sys.path.insert(0,sys.argv[1])
from stage_b import conflicts
busy=[p for p in conflicts() if any('hik' in n.lower() and 'camera' in n.lower() for n in p['executables'])]
if busy:raise SystemExit('Camera driver already running; stop/coordinate its owner first: '+str(busy))
PY
mkdir -p "$RUN"
export ROS_LOG_DIR="$RUN/ros"
cd "$RUN"
echo "Camera-only mode=$MODE; logs=$RUN; Ctrl+C to stop."
exec "${CMD[@]}"
