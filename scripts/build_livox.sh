#!/usr/bin/env bash
set -Eeuo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
if [[ ${1:---dry-run} != --execute ]]; then
 echo 'DRY-RUN: build pinned SDK2 into ws_livox/sdk_install and ROS2 driver into ws_livox/install; no sudo, no cleaning, no device access.'; exit 0
fi
export PATH=/usr/bin:/bin
sdk=$root/vendor/livox/Livox-SDK2
driver=$root/ws_livox/src/livox_ros_driver2
[[ $(git -C "$sdk" rev-parse HEAD) == 08f523c930b2f0ba1e98a6afaa8d7476bf479908 ]]
[[ $(git -C "$driver" rev-parse HEAD) == 4a1def929e5b59c7a8122d19fce6efba581ce9f7 ]]
if [[ -f "$driver/package.xml" ]]; then cmp "$driver/package.xml" "$driver/package_ROS2.xml"; else cp "$driver/package_ROS2.xml" "$driver/package.xml"; fi
mkdir -p "$root/docs/03_evidence"
log=$(mktemp "$root/docs/03_evidence/build-XXXXXXXX.log")
exec > >(tee "$log") 2>&1
trap 'rc=$?; echo "BUILD_EXIT=$rc"' EXIT
echo "LOG=$log"
cmake -S "$sdk" -B "$root/ws_livox/sdk_build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$root/ws_livox/sdk_install"
cmake --build "$root/ws_livox/sdk_build" --parallel 4
cmake --install "$root/ws_livox/sdk_build"
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$root/ws_livox"
colcon build --packages-select livox_ros_driver2 --executor sequential --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_RPATH="$root/ws_livox/sdk_install/lib" -DCMAKE_PREFIX_PATH="$root/ws_livox/sdk_install;/opt/ros/humble" -DLIVOX_LIDAR_SDK_LIBRARY="$root/ws_livox/sdk_install/lib/liblivox_lidar_sdk_shared.so" -DLIVOX_LIDAR_SDK_INCLUDE_DIR="$root/ws_livox/sdk_install/include" --event-handlers console_direct+
