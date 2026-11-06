#!/usr/bin/env bash
set -eo pipefail
C2_ROOT=$(cd -- "$(dirname -- "$0")/.." && pwd)
source /opt/ros/humble/setup.bash
source "$C2_ROOT/../../ws_livox/install/setup.bash"
source "$C2_ROOT/../../ws_camera/install/setup.bash"
export ROS_LOG_DIR="$C2_ROOT/logs/ros"
exec /usr/bin/python3 "$C2_ROOT/scripts/stage_b.py" start "$@"
