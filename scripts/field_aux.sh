#!/usr/bin/env bash
# Auxiliary session: dry-run by default, explicit --execute starts components.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
exec python3 "$ROOT/scripts/robot_session.py" aux "$@"
