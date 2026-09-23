#!/usr/bin/env bash
# Source only the fixed external Livox interface and this workspace.
SLAM_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$SLAM_PROJECT_ROOT/ws_livox/install/setup.bash"
export CMAKE_PREFIX_PATH="$SLAM_PROJECT_ROOT/ws_slam/deps/pangolin_install:$SLAM_PROJECT_ROOT/ws_slam/deps/root/usr:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$SLAM_PROJECT_ROOT/ws_slam/deps/root/usr/lib/x86_64-linux-gnu:$SLAM_PROJECT_ROOT/ws_slam/deps/pangolin_install/lib:${LD_LIBRARY_PATH:-}"
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
