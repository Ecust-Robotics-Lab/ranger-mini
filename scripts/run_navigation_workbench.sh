#!/usr/bin/env bash
# Only --execute starts the web/task services; no devices until the session button.
set -e -o pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"
export PYTHONPATH="$root:$root/scripts:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
if [[ " $* " != *" --demo "* ]]; then
  source /opt/ros/humble/setup.bash
  source "$root/ws_livox/install/setup.bash"
  source "$root/ws_camera/install/setup.bash"
  source "$root/ws_fast_livo/online_install/setup.bash"
  source "$root/ws_fast_livo/online_localization_install/setup.bash"
  source "$root/ws_nav/install_exact/setup.bash"
  source "$root/ws_base/install_exact/setup.bash"
fi
exec /usr/bin/python3 -m workbench.launch "$@"
