#!/usr/bin/env bash
# 阶段 6 单点导航启动入口（干跑默认；--execute 需现场控制授权）
export FASTRTPS_DEFAULT_PROFILES_FILE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/config/fastdds_udp_only.xml"   # 禁 SHM 传输，防幽灵端点
# 前提：field_session.sh --execute 已在跑（雷达/底盘/定位/桥接）。
#
# 用法:
#   bash scripts/run_navigation.sh            # 干跑：打印将启动的组件
#   bash scripts/run_navigation.sh --execute  # 实际启动（速度链输出零，直到 enable）
#   bash scripts/run_navigation.sh --status
#   bash scripts/run_navigation.sh --stop
#
# 放行顺序：启动后先 nav_safety.sh enable 授权，再 send_nav_goal.py 发目标。
set -o pipefail  # 注意不用 -u：ROS setup 脚本依赖未绑定变量属正常行为

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# 导航用副本地图（make_nav_map.py 生成：清除建图起点处 1 个不可能的自身占据像元，
# 该像元经膨胀会使起点致死、Smac 拒规划；原图 map.yaml 未改动）
MAP_YAML="${MAP_YAML:-$ROOT/maps/lightning-offline-8cw1afzl/data/new_map/map.yaml}"  # 2026-09-28 重录地图（起点净空0.326m无需副本）
CMD_PREFIX=(ros2 launch ranger_navigation navigation.launch.py
            execute:=true control_authorized:=true map_yaml:="$MAP_YAML")

ACTION="${1:-plan}"
case "$ACTION" in
    plan|--plan) EXEC=0 ;;
    --execute)   EXEC=1 ;;
    --status)    EXEC=-1 ;;
    --stop)      EXEC=-2 ;;
    *) echo "用法: $0 [--execute|--status|--stop]"; exit 1 ;;
esac

# 进程匹配用可执行文件路径锚定（对抗审查：裸组件名会误伤恰好包含该词的
# 编辑器/日志进程，且 vim 吸收 SIGINT 后会被逐级升级到 TERM/KILL）
declare -a PATTERNS=(
    "ranger_navigation/navigation.launch.py"
    "[l]ib/nav2_collision_monitor/collision_monitor"
    "[l]ib/ranger_navigation/motion_gate"
    "[l]ib/ranger_navigation/obstacle_filter"
    "[l]ib/twist_mux/twist_mux"
    "[l]ib/nav2_velocity_smoother/velocity_smoother"
    "[l]ib/nav2_controller/controller_server"
    "[l]ib/nav2_planner/planner_server"
    "[l]ib/nav2_bt_navigator/bt_navigator"
    "[l]ib/nav2_map_server/map_server"
    "lifecycle_manager_navigation"
)

if [ "$EXEC" = "0" ]; then
    echo "== 干跑（--execute 才会实际启动）=="
    echo "    ${CMD_PREFIX[*]}"
    echo "== 速度链: Nav2/遥控 -> twist_mux -> velocity_smoother -> ranger_motion_gate -> collision_monitor -> /base/safe_cmd_vel =="
    echo "== 放行: scripts/nav_safety.sh enable ；发目标: python3 scripts/send_nav_goal.py --x .. --y .. =="
    exit 0
fi

if [ "$EXEC" = "-1" ]; then
    for pat in "${PATTERNS[@]}"; do
        if pgrep -f "$pat" > /dev/null; then
            echo "[$pat] 运行中 (pid: $(pgrep -f "$pat" | tr '\n' ' '))"
        else
            echo "[$pat] 未运行"
        fi
    done
    echo "== 许可状态: bash scripts/nav_safety.sh status =="
    exit 0
fi

if [ "$EXEC" = "-2" ]; then
    for pat in "${PATTERNS[@]}"; do
        if pgrep -f "$pat" > /dev/null; then
            pkill -INT -f "$pat" && echo "[$pat] 已发 SIGINT"
        else
            echo "[$pat] 本来就没在运行"
        fi
    done
    sleep 3
    for pat in "${PATTERNS[@]}"; do
        pgrep -f "$pat" > /dev/null && { pkill -TERM -f "$pat"; echo "[$pat] 残留，已补 SIGTERM"; }
    done
    sleep 1
    for pat in "${PATTERNS[@]}"; do
        pgrep -f "$pat" > /dev/null && { pkill -KILL -f "$pat"; echo "[$pat] 仍残留，已 SIGKILL"; }
    done
    echo "提示：底盘驱动若被强停无发零看门狗（阶段2），确认遥控器已接管。"
    exit 0
fi

# --execute 前置检查 ------------------------------------------------------
MISSING=0
pgrep -f "ranger_localization_bridge" > /dev/null || {
    echo "[警告] 桥接(定位)未在运行：navigation_ready 门禁将阻断运动。"; MISSING=1; }
pgrep -f "run_loc_online" > /dev/null || \
    echo "[警告] Lightning 定位未在运行：无 map->imu_link TF。"
pgrep -f "base_mini_v2.launch.py" > /dev/null || \
    echo "[提示] 底盘驱动未在运行（纯离线验证可继续）。"
if pgrep -f "publish_loc_map.py" > /dev/null; then
    echo "[警告] 辅助地图发布(publish_loc_map.py)在跑：与 map_server 双发布 /map。"
    echo "       建议在 config/field_session.env 设 ENABLE_MAP_PUB=0 后重启 aux 标签页。"
fi

echo "== 启动导航栈（map: $MAP_YAML）=="
source /opt/ros/humble/setup.bash
source "$ROOT/ws_nav/install/setup.bash"
exec "${CMD_PREFIX[@]}"
