#!/usr/bin/env bash
# Records with its own sensors and FAST profile by default; --dry-run shows the plan.
# Default wheel feedback auto-start is receive-only; never starts chassis control or Nav2.
set -e -o pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONDONTWRITEBYTECODE=1
source /opt/ros/humble/setup.bash
source "$project_root/ws_livox/install/setup.bash"
source "$project_root/ws_camera/install/setup.bash"
source "$project_root/ws_base/install/setup.bash"
exec /usr/bin/python3 "$project_root/scripts/fast_livo2/record_session.py" "$@"
