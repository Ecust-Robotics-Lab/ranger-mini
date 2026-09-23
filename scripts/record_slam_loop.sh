#!/usr/bin/env bash
# Human-controlled loop only. This starts a recorder, never a driver or motion.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/slam_env.sh"
exec /usr/bin/python3 "$SLAM_PROJECT_ROOT/scripts/record_mid360.py" --label manual-loop "$@"
