#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd -- "$(dirname -- "$0")/.." && pwd)
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR="$ROOT/logs/ros"
for scene in scene_01 scene_02 scene_03; do
  /usr/bin/python3 "$ROOT/scripts/record_scene.py" "$scene" --seconds 8
done
