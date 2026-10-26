#!/usr/bin/env bash
# Dry-run by default. --execute runs a foreground supervised session.
# A start including the chassis also needs --control-authorized for this test.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
if [[ "${1:-}" != --stop && "${1:-}" != --status ]]; then
    ENV_FILE="${ENV_FILE:-$ROOT/config/field_session.env}"
    set -a
    source "$ENV_FILE"
    set +a
fi
exec python3 "$ROOT/scripts/robot_session.py" field "$@"
