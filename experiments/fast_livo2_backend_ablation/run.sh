#!/usr/bin/env bash
set -eo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$task_root"
if [[ ${1:-} != --execute ]]; then
  echo '离线后端对照：--execute 后构建工具并执行因子消融、原包核对和关键帧完整后端回放。'
  echo '不启动ROS节点或设备。结果写入独立新目录。'
  echo 'bash experiments/fast_livo2_backend_ablation/run.sh --execute [实验组绝对路径]'
  exit 0
fi
study_group="${2:-$task_root/experiments/fast_livo2_p0_closure/runs/20261111_171817-liu6_7ya}"
study_dir="$task_root/experiments/fast_livo2_backend_ablation"
study_output="$study_dir/results/review-$(date +%Y%m%d_%H%M%S)-$$"
mkdir -p "$study_output"
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH="$task_root/ws_fast_livo/backend_deps/lib:$task_root/ws_fast_livo/online_install/ranger_fast_livo_backend/lib:${LD_LIBRARY_PATH:-}"
cmake -S "$study_dir" -B "$study_dir/build" -DCMAKE_BUILD_TYPE=Release \
  "-DCMAKE_PREFIX_PATH=$task_root/ws_fast_livo/backend_deps;$task_root/ws_fast_livo/online_install/ranger_fast_livo_backend;/opt/ros/humble" > "$study_output/configure.log" 2>&1
cmake --build "$study_dir/build" -j2 > "$study_output/build.log" 2>&1
python3 "$study_dir/run_factor_study.py" --group "$study_group" --output "$study_output/factors" > "$study_output/factors.log" 2>&1
python3 "$study_dir/run_native_study.py" --group "$study_group" --output "$study_output/native" > "$study_output/native.log" 2>&1
echo "结果：$study_output"
