#!/usr/bin/env bash
set -e -o pipefail
calib_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ${1:-} != --execute ]]; then
  echo 'DRY-RUN: configure/build/install the integrated FAST-Calib; no sensor access or real solving.'
  exit 0
fi
source /opt/ros/humble/setup.bash
mkdir -p "$calib_root/logs"
build_log_dir="$(mktemp -d "$calib_root/logs/build-integrated-XXXXXXXX")"
export ROS_LOG_DIR="$build_log_dir/ros"
cmake -S "$calib_root/ws/src/fast_calib" -B "$calib_root/ws/build_integrated" -DCMAKE_INSTALL_PREFIX="$calib_root/ws/install_integrated/fast_calib" 2>&1 | tee "$build_log_dir/configure.log"
cmake --build "$calib_root/ws/build_integrated" --parallel 1 2>&1 | tee "$build_log_dir/build.log"
cmake --install "$calib_root/ws/build_integrated" 2>&1 | tee "$build_log_dir/install.log"
