#!/usr/bin/env bash
set -e -o pipefail
export PYTHONDONTWRITEBYTECODE=1
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ${1:-} != --execute ]]; then
  echo 'DRY-RUN: build backend export/C1 into ws_fast_livo/backend_install and the optional ws_nav C1 adapter; no devices.'
  exit 0
fi
cd "$project_root"
python3 - <<'PY'
import json, subprocess
from pathlib import Path
for repository in json.loads(Path('config/fast_livo2/sources.lock.json').read_text())['repositories']:
    if repository['name'] not in ('gtsam', 'small_gicp'):
        continue
    prefix = ['git', '-C', repository['path']]
    if subprocess.check_output(prefix+['rev-parse', 'HEAD'], text=True).strip() != repository['commit']:
        raise SystemExit('Dependency SHA mismatch: '+repository['name'])
    if subprocess.check_output(prefix+['status', '--porcelain']):
        raise SystemExit('Preserve and review modified dependency: '+repository['name'])
if not Path('ws_fast_livo/backend_deps/lib').is_dir():
    raise SystemExit('Build locked project-local dependencies with build_fast_livo2_backend.sh first')
PY
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH="$project_root/ws_fast_livo/backend_deps:${CMAKE_PREFIX_PATH:-}"
export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2
cd "$project_root/ws_fast_livo"
colcon --log-base localizer_log build --base-paths src --packages-up-to ranger_fast_livo_localizer \
  --build-base backend_build/ros --install-base backend_install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cd "$project_root/ws_nav"
colcon --log-base c1_adapter_log build --packages-select ranger_localization_bridge --executor sequential
