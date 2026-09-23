#!/usr/bin/env bash
# Build only. Never start a node, SDK sample, or CAN setup script.
set -eo pipefail
project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
case "${1:---dry-run}" in
  --dry-run)
    echo 'DRY-RUN: source ROS Humble; use project-local Asio; colcon build in ws_base.'
    echo 'Build ugv_sdk, ranger_msgs, ranger_base, ranger_bringup; BUILD_TESTING=OFF.'
    exit 0 ;;
  --execute) ;;
  *) echo 'Usage: bash scripts/build_base.sh [--dry-run|--execute]' >&2; exit 2 ;;
esac
test "$(git -C "$project_root/ws_base/src/ranger_ros2" rev-parse HEAD)" = b6ea21a275ca5e7168130cc6470e61474681d679
test "$(git -C "$project_root/ws_base/src/ugv_sdk" rev-parse HEAD)" = f2704eacdc90357078cd93ec60aae08bb4baab35
test -f "$project_root/vendor/base_deps/root/usr/include/asio.hpp"
source /opt/ros/humble/setup.bash
export CPLUS_INCLUDE_PATH="$project_root/vendor/base_deps/root/usr/include"
export PYTHONNOUSERSITE=1
export CMAKE_BUILD_PARALLEL_LEVEL=4
cd "$project_root/ws_base"
build_log=$(mktemp "$project_root/docs/02_evidence/build-XXXXXXXX.log")
echo "Build log: $build_log"
set +e
colcon build --executor sequential \
  --packages-select ugv_sdk ranger_msgs ranger_base ranger_bringup \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF \
  2>&1 | tee "$build_log"
build_exit=${PIPESTATUS[0]}
set -e
echo "COLCON_EXIT=$build_exit" | tee -a "$build_log"
exit "$build_exit"
