#!/usr/bin/env bash
# Private prior-map localization; dry-run unless --execute.
set -e -o pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONDONTWRITEBYTECODE=1
source /opt/ros/humble/setup.bash
source "$root/ws_livox/install/setup.bash"
source "$root/ws_camera/install/setup.bash"
if [[ " $* " == *" --auto-wheel "* ]]; then
  source "$root/ws_base/install_recording_rx/setup.bash"
fi
source "$root/ws_nav/install/setup.bash"
source "$root/ws_fast_livo/online_install/setup.bash"
if [[ -f "$root/ws_fast_livo/online_localization_install/setup.bash" ]]; then
  source "$root/ws_fast_livo/online_localization_install/setup.bash"
fi
export PYTHONPATH="$root/scripts:$root/ws_nav/src/ranger_localization_bridge:${PYTHONPATH:-}"
exec /usr/bin/python3 "$root/scripts/fast_livo2/localizer/online_session.py" "$@"
