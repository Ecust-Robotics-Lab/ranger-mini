#!/usr/bin/env bash
set -e
if [[ ${1:-} != --execute ]]; then
  echo 'DRY-RUN: camera-only intrinsic capture source /front/image_intrinsics (free run, not hardware sync). Add --execute to start the physical camera.'
  exit 0
fi
source /opt/ros/humble/setup.bash
C1_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$C1_PROJECT_ROOT/ws_camera/install/setup.bash"
C1_TOOL_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
C1_DRIVER_PREFIX="$(ros2 pkg prefix hik_camera_trigger_driver)"
export GENICAM_GENTL64_PATH="$C1_DRIVER_PREFIX/lib"
export LD_LIBRARY_PATH="$C1_DRIVER_PREFIX/lib:$C1_DRIVER_PREFIX/lib/ThirdParty${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
python3 - "$C1_PROJECT_ROOT/calibration/lidar_camera_extrinsics/scripts" <<'PY'
import sys
sys.path.insert(0,sys.argv[1])
from stage_b import conflicts
if any(any('hik' in name.lower() and 'camera' in name.lower() for name in p['executables']) for p in conflicts()):
    raise SystemExit('Camera already in use; coordinate the owning session first')
PY
mkdir -p "$C1_PROJECT_ROOT/calibration/logs"
C1_RUN_DIR="$(mktemp -d "$C1_PROJECT_ROOT/calibration/logs/intrinsics-camera-XXXXXXXX")"
export ROS_LOG_DIR="$C1_RUN_DIR/ros"
cd "$C1_RUN_DIR"
exec ros2 run hik_camera_trigger_driver hik_camera_trigger_driver_node --ros-args \
  --params-file "$C1_TOOL_DIR/camera_params_intrinsics.yaml" \
  -r __node:=hik_camera_intrinsics
