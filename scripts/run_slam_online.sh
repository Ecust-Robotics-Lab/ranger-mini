#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/slam_env.sh"
if [[ -f "$SLAM_PROJECT_ROOT/ws_slam/install/setup.bash" ]]; then
  source "$SLAM_PROJECT_ROOT/ws_slam/install/setup.bash"
fi
exec /usr/bin/python3 "$SLAM_PROJECT_ROOT/scripts/slam_run.py" online "$@"
