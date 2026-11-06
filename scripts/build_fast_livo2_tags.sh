#!/usr/bin/env bash
set -e -o pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ${1:-} != --execute ]]; then
  echo 'DRY-RUN: fixed local AprilTag 3.4.5, optional observer, existing B/C1/C2. No devices.'
  exit 0
fi
cd "$project_root"
tag_source=vendor/fast_livo2_review/apriltag
if [[ ! -d "$tag_source/.git" ]]; then
  git clone --branch v3.4.5 --depth 1 https://github.com/AprilRobotics/apriltag.git "$tag_source"
fi
test "$(git -C "$tag_source" rev-parse HEAD)" = 94be783968e5091bcc9972c72c84fd63efce2935
test -z "$(git -C "$tag_source" status --porcelain)"
cmake -S "$tag_source" -B ws_fast_livo/apriltag_build -DCMAKE_INSTALL_PREFIX="$project_root/ws_fast_livo/apriltag_deps" -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF -DBUILD_PYTHON_WRAPPER=OFF
cmake --build ws_fast_livo/apriltag_build -j2
cmake --install ws_fast_livo/apriltag_build
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH="$project_root/ws_fast_livo/apriltag_deps:$project_root/ws_fast_livo/relocalization_deps:$project_root/ws_fast_livo/backend_deps:${CMAKE_PREFIX_PATH:-}"
export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2
cd ws_fast_livo
colcon --log-base apriltag_log build --base-paths src --packages-up-to ranger_fast_livo_localizer \
  --build-base backend_build/ros --install-base backend_install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DRANGER_ENABLE_KISS=ON -DRANGER_ENABLE_APRILTAG=ON
