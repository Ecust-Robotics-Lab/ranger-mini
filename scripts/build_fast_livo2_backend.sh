#!/usr/bin/env bash
set -e -o pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ${1:-} != --execute ]]; then
  echo 'DRY-RUN: build locked GTSAM/small_gicp into ws_fast_livo/backend_deps, then backend only; no devices.'
  echo 'Execute: bash scripts/build_fast_livo2_backend.sh --execute'
  exit 0
fi
cd "$root"
python3 - <<'PY'
import json,subprocess
from pathlib import Path
for r in json.loads(Path('config/fast_livo2/sources.lock.json').read_text())['repositories']:
    if r['name'] not in ('gtsam','small_gicp'):continue
    if subprocess.check_output(['git','-C',r['path'],'rev-parse','HEAD'],text=True).strip()!=r['commit']:raise SystemExit('Dependency SHA mismatch: '+r['name'])
    if subprocess.check_output(['git','-C',r['path'],'status','--porcelain']):raise SystemExit('Preserve and review modified dependency: '+r['name'])
PY
prefix="$root/ws_fast_livo/backend_deps"
cmake -S vendor/fast_livo2_review/gtsam -B ws_fast_livo/backend_build/gtsam \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$prefix" \
  -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_BUILD_PYTHON=OFF \
  -DGTSAM_UNSTABLE_BUILD_PYTHON=OFF -DGTSAM_INSTALL_MATLAB_TOOLBOX=OFF \
  -DGTSAM_UNSTABLE_INSTALL_MATLAB_TOOLBOX=OFF -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_TIMING_ALWAYS=OFF \
  -DGTSAM_WITH_TBB=OFF -DGTSAM_WITH_EIGEN_MKL=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build ws_fast_livo/backend_build/gtsam -j2
cmake --install ws_fast_livo/backend_build/gtsam
cmake -S vendor/fast_livo2_review/small_gicp -B ws_fast_livo/backend_build/small_gicp \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$prefix" \
  -DBUILD_WITH_MARCH_NATIVE=OFF -DBUILD_PYTHON_BINDINGS=OFF \
  -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build ws_fast_livo/backend_build/small_gicp -j2
cmake --install ws_fast_livo/backend_build/small_gicp
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH="$prefix:${CMAKE_PREFIX_PATH:-}"
cd ws_fast_livo
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
colcon --log-base backend_log build --base-paths src --packages-select ranger_fast_livo_backend \
  --build-base backend_build/ros --install-base backend_install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
