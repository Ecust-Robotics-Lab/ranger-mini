#!/usr/bin/env bash
set -e -o pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$project_root/ws_camera/install/setup.bash"
source "$project_root/ws_livox/install/setup.bash"
exec python3 "$project_root/calibration/workflow.py" "$@"
