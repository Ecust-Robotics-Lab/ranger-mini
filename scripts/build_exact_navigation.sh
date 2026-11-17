#!/usr/bin/env bash
# Build only; never starts devices or CAN. Preserve all previous installations.
set -e -o pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
case "${1:---dry-run}" in
 --dry-run) echo 'DRY-RUN: build V2 driver/SDK and EXACT navigation into ws_base/install_exact and ws_nav/install_exact.'; exit 0;;
 --execute) ;;
 *) exit 2;;
esac
source /opt/ros/humble/setup.bash
export CPLUS_INCLUDE_PATH="$root/vendor/base_deps/root/usr/include${CPLUS_INCLUDE_PATH:+:$CPLUS_INCLUDE_PATH}"
export CMAKE_BUILD_PARALLEL_LEVEL=3 PYTHONNOUSERSITE=1
cd "$root/ws_base"
colcon --log-base log_exact build --build-base build_exact --install-base install_exact \
 --packages-select ugv_sdk ranger_msgs ranger_base ranger_bringup --executor sequential \
 --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF
source "$root/ws_base/install_exact/setup.bash"
cd "$root/ws_nav"
colcon --log-base log_exact build --build-base build_exact --install-base install_exact \
 --packages-select ranger_localization_msgs ranger_localization_bridge ranger_navigation_msgs ranger_navigation \
 --executor sequential --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=OFF
