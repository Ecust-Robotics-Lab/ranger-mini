#!/usr/bin/env bash
# Rebuild the integrated external-trigger camera and writer; never opens devices.
set -e -o pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ${1:-} != --execute ]]; then
  echo 'DRY-RUN: build hik_camera_trigger_driver and timeshare_writer in ws_camera (no hardware access).'
  exit 0
fi
source /opt/ros/humble/setup.bash
sdk_link="$project_root/ws_camera/src/hik_camera_trigger_driver/hikSDK"
if [[ ! -e "$sdk_link" ]]; then
  [[ -d "$project_root/ws_camera/src/hik_camera_ros2_driver/hikSDK" ]] || {
    echo 'Missing local Hik SDK: ws_camera/src/hik_camera_ros2_driver/hikSDK (not vendored in Git).' >&2; exit 2;
  }
  [[ ! -L "$sdk_link" ]] || { echo 'Broken Hik SDK link; inspect before rebuilding.' >&2; exit 2; }
  ln -s ../hik_camera_ros2_driver/hikSDK "$sdk_link"
fi
cd "$project_root/ws_camera"
PYTHONDONTWRITEBYTECODE=1 colcon build --packages-select hik_camera_trigger_driver timeshare_writer \
  --executor sequential --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
  --event-handlers console_direct+
