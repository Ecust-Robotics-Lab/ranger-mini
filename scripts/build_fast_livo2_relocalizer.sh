#!/usr/bin/env bash
set -e -o pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ${1:-} != --execute ]]; then
  echo 'DRY-RUN: optional C2 KISS dependencies and C1/C2 offline localizer; no devices. See dependency notices (PMC GPLv3+).'
  exit 0
fi
cd "$project_root"
python3 scripts/fast_livo2/relocalization/build_kiss.py --execute
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH="$project_root/ws_fast_livo/relocalization_deps:$project_root/ws_fast_livo/backend_deps:${CMAKE_PREFIX_PATH:-}"
export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2
cd ws_fast_livo
colcon --log-base relocalizer_log build --base-paths src --packages-up-to ranger_fast_livo_localizer \
  --build-base backend_build/ros --install-base backend_install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DRANGER_ENABLE_KISS=ON
cd "$project_root/ws_nav"
colcon --log-base c2_adapter_log build --packages-select ranger_localization_bridge --executor sequential
