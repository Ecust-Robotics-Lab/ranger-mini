#!/usr/bin/env bash
# Offline C2/C1 + diagnostic wheel/TF only. No device or controller startup.
set -e -o pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$root/ws_nav/install/setup.bash"
source "$root/ws_fast_livo/online_localization_install/setup.bash"
export LD_LIBRARY_PATH="$root/ws_fast_livo/backend_deps/lib:$root/ws_fast_livo/relocalization_deps/lib:${LD_LIBRARY_PATH:-}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
exec python3 "$root/scripts/fast_livo2/navigation/replay_localization.py" "$@"
