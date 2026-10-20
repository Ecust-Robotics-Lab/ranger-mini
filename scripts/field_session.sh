#!/usr/bin/env bash
# ranger_nav 现场会话一键启动：把原来的 A/B/C/D/E/F 多终端合并为一个
# gnome-terminal 窗口的多个标签页；参数集中在 config/field_session.env。
#
# 用法：
#   bash scripts/field_session.sh             # 干跑：只打印将启动的组件与命令
#   bash scripts/field_session.sh --execute   # 有桌面→标签页；SSH/无桌面→后台进程+日志
#   bash scripts/field_session.sh --status    # 查看各组件运行状态
#   bash scripts/field_session.sh --stop      # 按精确模式逐组件停止
#
# 安全提示：ENABLE_BASE=1 时启动底盘驱动会发送 CAN 控制模式配置（阶段2已知），
# 保持遥控器可接管。定位初始化需要车位于建图起点附近、朝向大致一致。
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

ENV_FILE="${ENV_FILE:-$ROOT/config/field_session.env}"
[ -f "$ENV_FILE" ] || { echo "缺少 $ENV_FILE"; exit 1; }
set -a; source "$ENV_FILE"; set +a

ACTION="${1:-plan}"
case "$ACTION" in
    plan|--plan) EXEC=0 ;;
    --execute)   EXEC=1 ;;
    --status)    EXEC=-1 ;;
    --stop)      EXEC=-2 ;;
    *) echo "用法: $0 [--execute|--status|--stop]"; exit 1 ;;
esac

if [ "$EXEC" -ge 0 ]; then
    source "$ROOT/scripts/load_runtime.sh" || exit 2
fi

# ---- 各组件命令（在各自标签页的 bash 内执行）--------------------------
tab_lidar() {  # 雷达驱动（含单实例锁与证据日志）
    echo "cd '$ROOT' && source scripts/slam_env.sh && python3 scripts/start_mid360.py --execute --seconds ${LIDAR_SECONDS:-1800}; exec bash"
}
tab_base() {   # 底盘驱动：/wheel/odom（启动即发 CAN 控制模式配置）
    echo "source /opt/ros/humble/setup.bash && source '$ROOT/ws_base/install/local_setup.bash' && ros2 launch '$ROOT/config/base_mini_v2.launch.py' execute:=true confirmed_model:=ranger_mini_v2 control_authorized:=true; exec bash"
}
tab_loc() {    # Lightning 在线定位（固定启动点初始位姿可调）
    [ "${LOCALIZATION_BACKEND:-lightning}" = lightning ] || { echo "echo 'standard backend: start external LocalizationEstimate producer'; exec bash"; return; }
    local loc_config_quoted
    printf -v loc_config_quoted '%q' "${LOC_CONFIG:-config/slam/ranger_mid360s_loc.yaml}"
    echo "cd '$ROOT' && source scripts/slam_env.sh && ws_slam/build/lightning/bin/run_loc_online --config=$loc_config_quoted --tf_child=${TF_CHILD:-imu_link} --init_x=${INIT_X:-0} --init_y=${INIT_Y:-0} --init_z=${INIT_Z:-0} --init_qx=${INIT_QX:-0} --init_qy=${INIT_QY:-0} --init_qz=${INIT_QZ:-0} --init_qw=${INIT_QW:-1}; exec bash"
}
tab_bridge() { # 桥接：map->odom + odom->base_link + navigation_ready
    echo "source /opt/ros/humble/setup.bash && source '$ROOT/ws_nav/install/setup.bash' && ros2 launch ranger_localization_bridge loc_bridge.launch.py t_base_imu_xyz:='${T_BASE_IMU_XYZ:-[0.0,0.0,0.556]}' t_base_imu_rpy:='${T_BASE_IMU_RPY:-[0.0,0.0,0.0]}' localization_backend:=${LOCALIZATION_BACKEND:-lightning} min_confidence:=${MIN_CONFIDENCE:-1.0} tf_max_age_s:=${TF_MAX_AGE_S:-0.5} odom_max_age_s:=${ODOM_MAX_AGE_S:-0.5} heartbeat_max_age_s:=${HEARTBEAT_MAX_AGE_S:-0.5} status_max_age_s:=${STATUS_MAX_AGE_S:-10.0} confidence_max_age_s:=${CONFIDENCE_MAX_AGE_S:-1.0} ${BRIDGE_EXTRA_ARGS:-}; exec bash"
}
tab_aux() {    # 辅助节点聚合（地图发布/派生点云/静态TF/foxglove_bridge）
    echo "cd '$ROOT' && bash scripts/field_aux.sh; exec bash"
}

declare -a NAMES CMDS
[ "${ENABLE_LIDAR:-1}"  = "1" ] && { NAMES+=("A-lidar");   CMDS+=("$(tab_lidar)"); }
[ "${ENABLE_BASE:-1}"   = "1" ] && { NAMES+=("B-base");    CMDS+=("$(tab_base)"); }
[ "${ENABLE_LOC:-1}"    = "1" ] && { NAMES+=("C-loc");     CMDS+=("$(tab_loc)"); }
[ "${ENABLE_BRIDGE:-1}" = "1" ] && { NAMES+=("D-bridge");  CMDS+=("$(tab_bridge)"); }
[ "${ENABLE_AUX:-1}"    = "1" ] && { NAMES+=("E-aux");     CMDS+=("$(tab_aux)"); }

# ---- 计划/干跑 ---------------------------------------------------------
if [ "$EXEC" = "0" ]; then
    echo "== 干跑（--execute 才会实际启动）=="
    for i in "${!NAMES[@]}"; do
        echo "--- [${NAMES[$i]}]"; echo "    ${CMDS[$i]}"
    done
    echo "== Foxglove 连接: ws://<本机IP>:8765；就绪检查: ros2 topic echo /navigation_ready --once =="
    exit 0
fi

# ---- 状态/停止（按精确进程模式，不误伤）-------------------------------
declare -A PATTERNS=(
    [A-lidar]="scripts/start_mid360.py"
    [B-base]="base_mini_v2.launch.py"
    [C-loc]="bin/run_loc_online"
    [D-bridge]="ranger_localization_bridge"
    [E-aux]="scripts/field_aux.sh"
)
# aux 子进程兜底模式（supervisor 先走，孤儿由这些模式兜住）
declare -a AUX_CHILD_PATTERNS=(
    "publish_loc_map.py"
    "[l]ib/ranger_localization_bridge/lightning_adapter"
    "mid360_derived.py"
    "static_transform_publisher.*imu_link.*livox_frame"
    "foxglove_bridge"
)
if [ "$EXEC" = "-1" ]; then
    for n in "${!PATTERNS[@]}"; do
        if pgrep -f "${PATTERNS[$n]}" > /dev/null; then
            echo "[${n}] 运行中 (pid: $(pgrep -f "${PATTERNS[$n]}" | tr '\n' ' '))"
        else
            echo "[${n}] 未运行"
        fi
    done
    exit 0
fi
if [ "$EXEC" = "-2" ]; then
    for n in "${!PATTERNS[@]}"; do
        if pgrep -f "${PATTERNS[$n]}" > /dev/null; then
            pkill -INT -f "${PATTERNS[$n]}" && echo "[${n}] 已发 SIGINT"
        else
            echo "[${n}] 本来就没在运行"
        fi
    done
    sleep 3
    for pat in "${AUX_CHILD_PATTERNS[@]}"; do
        pkill -INT -f "$pat" 2>/dev/null
    done
    sleep 1
    for n in "${!PATTERNS[@]}"; do
        pgrep -f "${PATTERNS[$n]}" > /dev/null && { pkill -TERM -f "${PATTERNS[$n]}"; echo "[${n}] 残留，已补 SIGTERM"; }
    done
    for pat in "${AUX_CHILD_PATTERNS[@]}"; do
        pgrep -f "$pat" > /dev/null && { pkill -TERM -f "$pat"; echo "[aux子进程:$pat] 残留，已补 SIGTERM"; }
    done
    sleep 1
    for n in "${!PATTERNS[@]}"; do
        pgrep -f "${PATTERNS[$n]}" > /dev/null && { pkill -KILL -f "${PATTERNS[$n]}"; echo "[${n}] 仍残留，已 SIGKILL"; }
    done
    echo "提示：底盘驱动若被强停，发零刷新看门狗不存在（阶段2结论），确认遥控器已接管。"
    exit 0
fi

# ---- 启动 --------------------------------------------------------------
# 无桌面/SSH 场景（车机常无显示）：gnome-terminal 不可用时自动改后台模式，
# 组件日志逐个落在 /tmp/ranger_session/<组件名>.log（--stop/--status 通用）。
LOGDIR=/tmp/ranger_session
mkdir -p "$LOGDIR"

if command -v gnome-terminal > /dev/null && [ -n "${DISPLAY:-}" ]; then
    echo "== 启动 ${#NAMES[@]} 个标签页：${NAMES[*]} =="
    echo "== 车应在建图起点附近；ENABLE_BASE 启动底盘驱动会发 CAN 控制模式配置 =="
    first=1
    for i in "${!NAMES[@]}"; do
        title="${NAMES[$i]}"
        cmd="${CMDS[$i]}"
        if [ "$first" = "1" ]; then
            gnome-terminal --window --title="$title" -- bash -c "$cmd"
            first=0
        else
            gnome-terminal --tab --title="$title" -- bash -c "$cmd"
        fi
        sleep 1   # 标签按依赖顺序错开启动，便于阅读各自日志
    done
else
    echo "== 无桌面会话（DISPLAY 未设置或无 gnome-terminal）→ 后台模式 =="
    echo "== 日志: $LOGDIR/<组件>.log；查看: tail -f $LOGDIR/B-base.log =="
    echo "== 车应在建图起点附近；ENABLE_BASE 启动底盘驱动会发 CAN 控制模式配置 =="
    for i in "${!NAMES[@]}"; do
        title="${NAMES[$i]}"
        # 去掉标签页专用的 'exec bash' 尾巴，避免后台残留交互 shell
        cmd="${CMDS[$i]%"; exec bash"}"
        nohup bash -c "$cmd" > "$LOGDIR/${title}.log" 2>&1 &
        echo "[${title}] 后台启动 pid=$! 日志=$LOGDIR/${title}.log"
        sleep 1   # 按依赖顺序错开启动
    done
fi
echo "== 完成。就绪检查: ros2 topic echo /navigation_ready --once（期望 data: true）=="
echo "== Foxglove: ws://<本机IP>:8765，Fixed Frame=map，加 /map、TF、/livox/points_rviz =="
