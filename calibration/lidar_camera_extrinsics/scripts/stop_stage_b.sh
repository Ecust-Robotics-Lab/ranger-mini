#!/usr/bin/env bash
set -eo pipefail
C2_ROOT=$(cd -- "$(dirname -- "$0")/.." && pwd)
exec /usr/bin/python3 "$C2_ROOT/scripts/stage_b.py" stop
