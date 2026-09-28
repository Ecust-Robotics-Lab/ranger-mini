#!/usr/bin/env bash
# Fast DDS 共享内存残留清理（阶段 6 运维发现）
#
# 背景：被 SIGKILL 的 ROS 2 进程会在 /dev/shm 留下 fastrtps_* 段，并使 DDS 图
# 出现永不超时的幽灵端点（阶段 6 测试证据 docs/06_evidence/safety-r4rz4zmi 等：
# 泄漏端点 + 残段导致跨轮次图污染与异常数据）。本脚本在"确认没有活 ROS 进程"
# 的前提下安全清理这些残留段。
#
# 用法: bash scripts/clean_dds_shm.sh [--force]
#   默认：检测到任何疑似活 ROS 进程则拒绝执行并列出它们
#   --force：跳过检测直接清理（自行承担风险）
set -o pipefail  # 注意不用 -u：ROS setup 脚本依赖未绑定变量属正常行为

if [ "${1:-}" != "--force" ]; then
    mapfile -t alive < <(pgrep -af 'ros2 |ros2$|/ros/|ranger_navigation|nav2_|fastrtps' 2>/dev/null | grep -av 'clean_dds_shm')
    if [ "${#alive[@]}" -gt 0 ]; then
        echo "检测到疑似活着的 ROS 进程，拒绝清理（--force 可强制）："
        printf '  %s\n' "${alive[@]}" | head -10
        exit 1
    fi
fi

n_before=$(ls /dev/shm 2>/dev/null | grep -c '^fastrtps_' || true)
rm -f /dev/shm/fastrtps_*
n_after=$(ls /dev/shm 2>/dev/null | grep -c '^fastrtps_' || true)
echo "已清除 fastrtps_* 残留段：$((n_before - n_after)) 个（剩 ${n_after}）"
