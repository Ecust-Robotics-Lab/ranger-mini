#!/usr/bin/env bash
# Local build only. Dependencies/sources must already match the tracked lock.
set -e -o pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
if [[ ${1:-} != --execute ]]; then
  echo 'DRY-RUN: verify locked sources, apply reviewed patch, build Sophus/vikit/FAST-LIVO2 locally.'
  echo 'Execute: bash scripts/build_fast_livo2.sh --execute'
  exit 0
fi
source /opt/ros/humble/setup.bash
source "$project_root/ws_livox/install/setup.bash"
python3 - <<'PY'
import json, subprocess
from pathlib import Path
root=Path.cwd()
for r in json.loads((root/'config/fast_livo2/sources.lock.json').read_text())['repositories']:
    if r['name'] not in {'ros2_frontend','rpg_vikit','sophus'}: continue
    p=root/r['path']
    sha=subprocess.check_output(['git','-C',str(p),'rev-parse','HEAD'],text=True).strip()
    if sha != r['commit']: raise SystemExit('Source version mismatch: '+str(p))
    if r['name'] != 'ros2_frontend' and subprocess.check_output(['git','-C',str(p),'diff','HEAD','--binary']):
        raise SystemExit('Unreviewed dependency changes; preserve and inspect them first: '+str(p))
front=root/'vendor/fast_livo2_review/ros2_frontend'
patch=root/'patches/fast_livo2_stage_a.patch'
if subprocess.run(['git','-C',str(front),'apply','--reverse','--check',str(patch)],capture_output=True).returncode:
    subprocess.run(['git','-C',str(front),'apply','--check',str(patch)],check=True)
    subprocess.run(['git','-C',str(front),'apply',str(patch)],check=True)
(front/'src/ranger_livo_replay.cpp').write_bytes((root/'scripts/fast_livo2/replay.cpp').read_bytes())
(front/'src/ranger_livo_checks.cpp').write_bytes((root/'scripts/fast_livo2/checks.cpp').read_bytes())
(front/'include/ranger_input_guard.h').write_bytes((root/'scripts/fast_livo2/input_guard.h').read_bytes())
if subprocess.check_output(['git','-C',str(front),'diff','--binary']) != patch.read_bytes():
    raise SystemExit('Frontend tracked changes differ from the reviewed patch; preserve and inspect them first')
PY
dep_prefix="$project_root/ws_fast_livo/deps_install"
build_root="$project_root/ws_fast_livo/build"
cmake -S vendor/fast_livo2_review/sophus -B "$build_root/sophus" \
  -DCMAKE_INSTALL_PREFIX="$dep_prefix" -DBUILD_SOPHUS_TESTS=OFF -DBUILD_SOPHUS_EXAMPLES=OFF \
  -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON
cmake --build "$build_root/sophus" -j2
cmake --install "$build_root/sophus"
export CMAKE_PREFIX_PATH="$dep_prefix:${CMAKE_PREFIX_PATH:-}"
cmake -S scripts/fast_livo2/vikit -B "$build_root/vikit" \
  -DVIKIT_SOURCE_DIR="$project_root/vendor/fast_livo2_review/rpg_vikit/vikit_common" \
  -DCMAKE_INSTALL_PREFIX="$dep_prefix" -DCMAKE_BUILD_TYPE=Release
cmake --build "$build_root/vikit" -j2
cmake --install "$build_root/vikit"
cmake -S vendor/fast_livo2_review/ros2_frontend -B "$build_root/fast_livo" \
  -DCMAKE_INSTALL_PREFIX="$project_root/ws_fast_livo/install/fast_livo" \
  -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON -DBUILD_TESTING=OFF
cmake --build "$build_root/fast_livo" -j2
cmake --install "$build_root/fast_livo"
