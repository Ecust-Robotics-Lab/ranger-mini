#!/usr/bin/env bash
# Prerequisite: pinned project-local dependencies; see docs/04_evidence/*.json.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/slam_env.sh"
if [[ "${1:-}" != --execute ]]; then
  echo 'Dry-run: build fixed Lightning source with external Livox interfaces; parallel=2.'
  exit 0
fi
export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS="-j2"
cd "$SLAM_PROJECT_ROOT/ws_slam"
colcon --log-base log build --packages-select lightning --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  --event-handlers console_direct+
