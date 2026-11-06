#!/usr/bin/env bash
# Unified private mapping. Default dry-run; --execute starts the requested session.
set -e -o pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONDONTWRITEBYTECODE=1
source /opt/ros/humble/setup.bash
source "$project_root/ws_livox/install/setup.bash"
source "$project_root/ws_camera/install/setup.bash"
if [[ -f "$project_root/ws_fast_livo/online_install/setup.bash" ]]; then
  source "$project_root/ws_fast_livo/online_install/setup.bash"
fi
export PYTHONPATH="$project_root/scripts:$project_root/ws_nav/src/ranger_localization_bridge:${PYTHONPATH:-}"
exec /usr/bin/python3 "$project_root/scripts/fast_livo2/mapping_session.py" "$@"
