#!/usr/bin/env bash
# Isolated frontend candidate only. Never starts devices or changes runtime selection.
set -e -o pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ${1:---dry-run} != --execute ]]; then
  echo 'DRY-RUN: verify reviewed frontend, build baseline/candidate, install candidate in ws_fast_livo/latency_frontend_install. No devices or runtime switch.'
  exit 0
fi
source /opt/ros/humble/setup.bash
source "$root/ws_livox/install/setup.bash"
cd "$root"
python3 scripts/fast_livo2/prepare_online_frontend.py
export CMAKE_PREFIX_PATH="$root/ws_fast_livo/deps_install:${CMAKE_PREFIX_PATH:-}"
for variant in baseline candidate; do
  python3 scripts/fast_livo2/prepare_latency_frontend.py --variant "$variant"
  cmake -S "ws_fast_livo/latency_frontend_${variant}_source" -B "ws_fast_livo/latency_frontend_${variant}_build" \
    -DCMAKE_INSTALL_PREFIX="$root/ws_fast_livo/latency_frontend_install/fast_livo" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DBUILD_TESTING=OFF
  cmake --build "ws_fast_livo/latency_frontend_${variant}_build" --target ranger_livo_replay fastlivo_mapping ranger_livo_checks -j2
done
cmake --install ws_fast_livo/latency_frontend_candidate_build
