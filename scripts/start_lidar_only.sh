#!/usr/bin/env bash
# Managed lidar-only session; dry-run default, no automatic restart.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
exec python3 "$ROOT/scripts/robot_session.py" lidar "$@"
