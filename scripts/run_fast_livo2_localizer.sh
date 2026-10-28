#!/usr/bin/env bash
set -e -o pipefail
export PYTHONDONTWRITEBYTECODE=1
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ " $* " == *" --execute "* ]]; then
  source /opt/ros/humble/setup.bash
  source "$project_root/ws_nav/install/setup.bash"
  source "$project_root/ws_fast_livo/backend_install/setup.bash"
fi
exec python3 "$project_root/scripts/fast_livo2/localizer/session.py" "$@"
