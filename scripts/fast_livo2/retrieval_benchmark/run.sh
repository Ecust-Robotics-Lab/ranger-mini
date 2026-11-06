#!/usr/bin/env bash
# Offline only: consumes frozen clouds, never starts ROS or devices.
set -e -o pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$root/ws_fast_livo/online_localization_install/setup.bash"
export LD_LIBRARY_PATH="$root/ws_fast_livo/backend_deps/lib:$root/ws_fast_livo/relocalization_deps/lib:${LD_LIBRARY_PATH:-}"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
cd "$root"
exec outputs/fast-livo-demo-retrieval-comparison-20261106/build/retrieval_comparison \
  --archive "$root/outputs/fast-livo-demo-online-mapping-20261105/full_replay_12/map" \
  --identity outputs/fast-livo-demo-online-localization-20261106/visit01_auto_pcl_01/source_identity.json \
  --queries outputs/fast-livo-demo-retrieval-comparison-20261106/queries/cases.json "$@"
