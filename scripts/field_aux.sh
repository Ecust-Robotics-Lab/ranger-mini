#!/usr/bin/env bash
# 现场会话辅助节点聚合：二维地图发布 + 派生点云 + 静态外参TF + foxglove_bridge
# 由 scripts/field_session.sh 的 aux 标签页调用，也可单独运行。
# Ctrl-C 一并收尾；各组件日志在 /tmp/ranger_session/ 下。
set -o pipefail  # 注意不用 -u：ROS setup 脚本依赖未绑定变量属正常行为

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOGDIR=/tmp/ranger_session
mkdir -p "$LOGDIR"

MAP_DIR="${MAP_DIR:-$ROOT/maps/lightning-offline-suej7qsz/data/new_map}"
ENABLE_FOXGLOVE="${ENABLE_FOXGLOVE:-1}"

pids=()
cleanup() {
    echo "[aux] 收尾，停止 ${#pids[@]} 个辅助节点（INT→宽限3s→TERM→KILL）"
    for p in "${pids[@]:-}"; do
        [ -n "$p" ] && kill -INT "$p" 2>/dev/null
    done
    sleep 3   # 给 ros2 launch/驱动正常收尾的时间
    for p in "${pids[@]:-}"; do
        [ -n "$p" ] && kill -TERM "$p" 2>/dev/null
    done
    sleep 1   # static_transform_publisher 等不响应 INT 的节点靠 TERM
    for p in "${pids[@]:-}"; do
        [ -n "$p" ] && kill -KILL "$p" 2>/dev/null
    done
    wait 2>/dev/null
    echo "[aux] 已全部停止"
}
trap cleanup EXIT INT TERM

# ROS 环境：slam_env.sh 已含 Humble + ws_livox + 项目内依赖
source "$ROOT/scripts/slam_env.sh"

echo "[aux] 1/4 二维地图发布 (latched /map): $MAP_DIR"
python3 "$ROOT/scripts/publish_loc_map.py" --map-dir "$MAP_DIR" \
    > "$LOGDIR/map_publisher.log" 2>&1 &
pids+=($!)

echo "[aux] 2/4 派生点云 /livox/points_rviz"
python3 "$ROOT/scripts/mid360_derived.py" --execute \
    > "$LOGDIR/derived.log" 2>&1 &
pids+=($!)

echo "[aux] 3/4 静态TF imu_link->livox_frame（厂家名义外参）"
ros2 run tf2_ros static_transform_publisher \
    --x -0.011 --y -0.02329 --z 0.04412 \
    --frame-id imu_link --child-frame-id livox_frame \
    > "$LOGDIR/static_tf.log" 2>&1 &
pids+=($!)

if [ "$ENABLE_FOXGLOVE" = "1" ]; then
    echo "[aux] 4/4 foxglove_bridge (ws://<本机IP>:8765)"
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
        > "$LOGDIR/foxglove.log" 2>&1 &
    pids+=($!)
else
    echo "[aux] 4/4 foxglove_bridge 已按配置跳过"
fi

echo "[aux] 全部辅助节点已启动；日志在 $LOGDIR/；Ctrl-C 一并停止"
wait
