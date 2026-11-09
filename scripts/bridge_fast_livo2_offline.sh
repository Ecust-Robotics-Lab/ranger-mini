#!/usr/bin/env bash
set -e -o pipefail
export PYTHONDONTWRITEBYTECODE=1
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ " $* " == *" --execute "* ]]; then
  source /opt/ros/humble/setup.bash
  source "$root/ws_nav/install/setup.bash"
  source "$root/ws_fast_livo/online_localization_install/setup.bash"
fi
exec python3 "$root/scripts/fast_livo2/navigation/session.py" "$@"
