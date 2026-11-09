#!/usr/bin/env bash
set -e -o pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$root/ws_fast_livo/online_localization_install/setup.bash"
export LD_LIBRARY_PATH="$root/ws_fast_livo/backend_deps/lib:${LD_LIBRARY_PATH:-}"
export PYTHONDONTWRITEBYTECODE=1
exec python3 "$root/scripts/fast_livo2/navigation/export_map.py" "$@"
