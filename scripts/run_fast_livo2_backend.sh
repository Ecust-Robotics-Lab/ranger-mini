#!/usr/bin/env bash
set -e -o pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ " $* " == *" --execute "* ]]; then
  source /opt/ros/humble/setup.bash
  source "$root/ws_nav/install/setup.bash"
  source "$root/ws_fast_livo/backend_install/setup.bash"
fi
exec python3 "$root/scripts/fast_livo2/backend/session.py" "$@"
