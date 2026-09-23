#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/slam_env.sh"
if [[ "${1:-}" != --execute ]]; then
  echo 'Dry-run: compile the offline RosbagIO/preprocess checker against Lightning.'
  exit 0
fi
cd "$SLAM_PROJECT_ROOT"
cmake -S tests/slam_reader -B ws_slam/reader_build -DCMAKE_BUILD_TYPE=Release
cmake --build ws_slam/reader_build --parallel 2
