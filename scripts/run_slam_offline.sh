#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/slam_env.sh"
exec /usr/bin/python3 "$SLAM_PROJECT_ROOT/scripts/slam_run.py" offline "$@"
