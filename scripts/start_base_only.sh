#!/usr/bin/env bash
# 单独补起底盘驱动（用于会话其余组件已在跑、只有 B-base 缺席时；
# 例如重启后 can0 刚重配完，不想重启整个会话弄丢已收敛的定位）。
# 前置：can0 已 UP/ERROR-ACTIVE（launch 自带护栏，不满足会拒绝）。
# 日志：/tmp/ranger_session/B-base.log
set -o pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG=/tmp/ranger_session/B-base.log
mkdir -p /tmp/ranger_session
nohup bash -c "source /opt/ros/humble/setup.bash && source '$ROOT/ws_base/install/local_setup.bash' && ros2 launch '$ROOT/config/base_mini_v2.launch.py' execute:=true confirmed_model:=ranger_mini_v2 control_authorized:=true" >> "$LOG" 2>&1 &
echo "B-base 后台启动 pid=$! 日志=$LOG（等待 ~5s 后: tail -5 $LOG 应见 Start listening to port: can0）"
