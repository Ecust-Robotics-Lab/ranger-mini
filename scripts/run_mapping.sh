#!/usr/bin/env bash
set -e -o pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "$project_root/scripts/mapping_session.py" "$@"
