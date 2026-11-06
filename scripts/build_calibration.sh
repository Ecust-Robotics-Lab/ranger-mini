#!/usr/bin/env bash
set -e -o pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$project_root/calibration/lidar_camera_extrinsics/scripts/build.sh" "$@"
