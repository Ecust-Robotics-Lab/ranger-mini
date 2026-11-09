#!/usr/bin/env bash
# Isolated receive-only-capable build. Never starts CAN or a driver.
set -e -o pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
case "${1:---dry-run}" in
  --dry-run) echo 'DRY-RUN: build ranger_base in ws_base/build_recording_rx and install_recording_rx; existing installation preserved.'; exit 0 ;;
  --execute) ;;
  *) echo 'Usage: bash scripts/build_base_recording.sh [--dry-run|--execute]' >&2; exit 2 ;;
esac
# Local stage-2 commits include the pinned upstream and existing transport/model fixes.
test "$(git -C "$root/ws_base/src/ranger_ros2" rev-parse HEAD)" = 7b75bc2443a1d6cbd42cc3ef3059cf4897242072
test "$(git -C "$root/ws_base/src/ugv_sdk" rev-parse HEAD)" = e2f5764acc00a2e556cf3bdc3ea3b206fa326e5b
source /opt/ros/humble/setup.bash
source "$root/ws_base/install/setup.bash"
export CPLUS_INCLUDE_PATH="$root/vendor/base_deps/root/usr/include${CPLUS_INCLUDE_PATH:+:$CPLUS_INCLUDE_PATH}"
export CMAKE_BUILD_PARALLEL_LEVEL=2
export PYTHONNOUSERSITE=1
cd "$root/ws_base"
colcon --log-base log_recording_rx build --build-base build_recording_rx --install-base install_recording_rx \
  --packages-select ranger_base --executor sequential --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF
