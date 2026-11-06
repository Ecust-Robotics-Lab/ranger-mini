#!/usr/bin/env bash
set -eo pipefail
C2_ROOT=$(cd -- "$(dirname -- "$0")/.." && pwd)
source "$C2_ROOT/scripts/env.sh"
exec rviz2 -d "$C2_ROOT/config/stage_b_preview.rviz" "$@"
