#!/usr/bin/env bash
# Managed chassis-only session; explicit fresh authorization is mandatory.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
exec python3 "$ROOT/scripts/robot_session.py" base "$@"
