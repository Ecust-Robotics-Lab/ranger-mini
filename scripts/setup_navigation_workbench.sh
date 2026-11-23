#!/usr/bin/env bash
# Project-local dependencies only. No apt, driver or network configuration changes.
set -e -o pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ " ${*:-} " != *" --execute "* ]]; then
  echo 'DRY-RUN: create .runtime/navigation_workbench_venv and install workbench/requirements.lock.txt'
  exit 0
fi
python3 -m venv --system-site-packages "$root/.runtime/navigation_workbench_venv"
"$root/.runtime/navigation_workbench_venv/bin/pip" install -r "$root/workbench/requirements.lock.txt"
