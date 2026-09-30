#!/usr/bin/env bash
# 单独补起雷达驱动（驱动 3600s 到时自动退出后，不想重启整个会话时用；
# C-loc 等组件会自动接上恢复的点云流）。日志 /tmp/ranger_session/A-lidar.log
export FASTRTPS_DEFAULT_PROFILES_FILE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/config/fastdds_udp_only.xml"
set -o pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG=/tmp/ranger_session/A-lidar.log
mkdir -p /tmp/ranger_session
SECS="${LIDAR_SECONDS:-3600}"
nohup bash -c "cd '$ROOT' && source scripts/slam_env.sh && python3 scripts/start_mid360.py --execute --seconds $SECS" >> "$LOG" 2>&1 &
echo "A-lidar 后台启动 pid=$! 时限 ${SECS}s 日志=$LOG"
