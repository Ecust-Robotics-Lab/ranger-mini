#!/usr/bin/env bash
set -e -o pipefail
export PYTHONDONTWRITEBYTECODE=1
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ " $* " == *" --execute "* ]]; then
  source /opt/ros/humble/setup.bash
  source "$project_root/ws_fast_livo/backend_install/setup.bash"
fi
export LD_LIBRARY_PATH="$project_root/ws_fast_livo/apriltag_deps/lib:$project_root/ws_fast_livo/backend_deps/lib:${LD_LIBRARY_PATH:-}"
exec python3 "$project_root/scripts/fast_livo2/tags/session.py" "$@"
