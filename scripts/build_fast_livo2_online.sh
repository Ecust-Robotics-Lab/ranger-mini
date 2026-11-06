#!/usr/bin/env bash
# Build the online backend in an isolated prefix, reusing locked local dependencies.
set -e -o pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ${1:-} != --execute ]]; then
  echo 'DRY-RUN: build isolated frontend I/O and online backend; reuse locked local dependencies; no devices.'
  echo 'Execute: bash scripts/build_fast_livo2_online.sh --execute'
  exit 0
fi
source /opt/ros/humble/setup.bash
cd "$project_root"
python3 - <<'PY'
import json,subprocess
from pathlib import Path
for repository in json.loads(Path('config/fast_livo2/sources.lock.json').read_text())['repositories']:
    if repository['name'] not in ('gtsam','small_gicp','sophus','rpg_vikit'): continue
    actual=subprocess.check_output(['git','-C',repository['path'],'rev-parse','HEAD'],text=True).strip()
    if actual!=repository['commit']: raise SystemExit('Dependency SHA mismatch: '+repository['name'])
    if subprocess.check_output(['git','-C',repository['path'],'status','--porcelain']):
        raise SystemExit('Preserve and review modified dependency: '+repository['name'])
PY
source "$project_root/ws_livox/install/setup.bash"
python3 scripts/fast_livo2/prepare_online_frontend.py
export CMAKE_PREFIX_PATH="$project_root/ws_fast_livo/deps_install:${CMAKE_PREFIX_PATH:-}"
cmake -S ws_fast_livo/online_frontend_source -B ws_fast_livo/online_frontend_build \
  -DCMAKE_INSTALL_PREFIX="$project_root/ws_fast_livo/online_frontend_install/fast_livo" \
  -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DBUILD_TESTING=OFF
cmake --build ws_fast_livo/online_frontend_build -j2
cmake --install ws_fast_livo/online_frontend_build
export CMAKE_PREFIX_PATH="$project_root/ws_fast_livo/backend_deps:${CMAKE_PREFIX_PATH:-}"
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
cd "$project_root/ws_fast_livo"
colcon --log-base online_log build --base-paths src --packages-select ranger_fast_livo_backend \
  --build-base online_build --install-base online_install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DRANGER_ENABLE_APRILTAG=OFF -DBUILD_TESTING=ON
