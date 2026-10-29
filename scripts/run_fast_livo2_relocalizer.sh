#!/usr/bin/env bash
set -e -o pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$project_root/scripts/run_fast_livo2_localizer.sh" --global-search --domain 190 "$@"
