#!/usr/bin/env bash
# Dry-run by default. Records sensors; never starts chassis control or Nav2.
set -e -o pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONDONTWRITEBYTECODE=1
source /opt/ros/humble/setup.bash
source "$project_root/ws_livox/install/setup.bash"
source "$project_root/ws_camera/install/setup.bash"
exec /usr/bin/python3 "$project_root/scripts/fast_livo2/record_session.py" "$@"
