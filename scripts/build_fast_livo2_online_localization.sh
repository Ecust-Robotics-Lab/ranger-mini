#!/usr/bin/env bash
# Isolated C1/C2 build; reuse reviewed dependencies and the online backend.
set -e -o pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ${1:-} != --execute ]]; then
  echo 'DRY-RUN: build C1/C2 into online_localization_install; no devices.'
  exit 0
fi
source /opt/ros/humble/setup.bash
source "$root/ws_fast_livo/online_install/setup.bash"
export CMAKE_PREFIX_PATH="$root/ws_fast_livo/relocalization_deps:$root/ws_fast_livo/backend_deps:${CMAKE_PREFIX_PATH:-}"
export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2
cd "$root/ws_fast_livo"
colcon --log-base online_localization_log build --base-paths src --packages-select ranger_fast_livo_localizer \
  --build-base online_localization_build --install-base online_localization_install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DRANGER_ENABLE_KISS=ON -DBUILD_TESTING=ON
