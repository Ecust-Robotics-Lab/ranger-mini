#!/usr/bin/env bash
# User-run interactive experiment. --dry-run never starts devices.
set -e -o pipefail
experiment_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "$experiment_dir/../.." && pwd)"
export PYTHONDONTWRITEBYTECODE=1
source /opt/ros/humble/setup.bash
source "$project_root/ws_livox/install/setup.bash"
source "$project_root/ws_camera/install/setup.bash"
source "$project_root/ws_fast_livo/online_install/setup.bash"
export PYTHONPATH="$project_root/scripts:$project_root/scripts/fast_livo2:$project_root/ws_nav/src/ranger_localization_bridge:${PYTHONPATH:-}"
exec /usr/bin/python3 "$experiment_dir/suite.py" "$@"
