#!/usr/bin/env bash
# Dry-run by default; --execute --control-authorized starts a managed stack.
# Existing healthy localization is required; motion permission starts disabled.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
exec python3 "$ROOT/scripts/robot_session.py" navigation "$@"
