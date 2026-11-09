#!/usr/bin/env bash
# Build the acquisition/publication split without opening the camera.
set -e -o pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ${1:-} != --execute ]]; then
  echo 'DRY-RUN: build camera into ws_camera/latency_install; no devices.'
  exit 0
fi
source /opt/ros/humble/setup.bash
cd "$root/ws_camera"
colcon --log-base latency_log build --packages-select hik_camera_trigger_driver \
  --build-base latency_build --install-base latency_install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
