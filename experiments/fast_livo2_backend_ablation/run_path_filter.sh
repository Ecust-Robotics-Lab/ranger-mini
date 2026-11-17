#!/usr/bin/env bash
set -eo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$task_root"
if [[ ${1:-} != --execute ]]; then
  echo '离线：独立构建后端、执行核心检查、按0/3/5/10m重新配准五次原包；无设备。'
  echo 'bash experiments/fast_livo2_backend_ablation/run_path_filter.sh --execute'
  exit 0
fi
study_dir="$task_root/experiments/fast_livo2_backend_ablation"
study_group="${2:-$task_root/experiments/fast_livo2_p0_closure/runs/20261111_171817-liu6_7ya}"
study_output="$study_dir/results/path-$(date +%Y%m%d_%H%M%S)-$$"
mkdir -p "$study_output"
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH="$task_root/ws_fast_livo/backend_deps:${CMAKE_PREFIX_PATH:-}"
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
colcon --log-base "$study_output/colcon" build --base-paths ws_fast_livo/src \
  --packages-select ranger_fast_livo_backend --build-base "$study_dir/build_core_path" \
  --install-base "$study_dir/install_path" --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON > "$study_output/core_build.log" 2>&1
export LD_LIBRARY_PATH="$study_dir/install_path/ranger_fast_livo_backend/lib:$task_root/ws_fast_livo/backend_deps/lib:${LD_LIBRARY_PATH:-}"
ctest --test-dir "$study_dir/build_core_path/ranger_fast_livo_backend" --output-on-failure > "$study_output/tests.log" 2>&1
cmake -S "$study_dir" -B "$study_dir/build_path" -DCMAKE_BUILD_TYPE=Release -DRANGER_PATH_FILTER_ENABLED=ON \
  "-DCMAKE_PREFIX_PATH=$study_dir/install_path/ranger_fast_livo_backend;$task_root/ws_fast_livo/backend_deps;/opt/ros/humble" > "$study_output/tools_build.log" 2>&1
cmake --build "$study_dir/build_path" -j2 >> "$study_output/tools_build.log" 2>&1
python3 "$study_dir/run_native_study.py" --group "$study_group" --output "$study_output/replays" \
  --binary "$study_dir/build_path/backend_ablation" --path-study > "$study_output/study.log" 2>&1
echo "结果：$study_output"
