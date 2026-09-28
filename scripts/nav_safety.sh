#!/usr/bin/env bash
# 导航运动许可/软急停开关（阶段 6 安全链人工接口）
#
#   enable  -> /ranger/motion_enable=true   授权本次导航运动（门禁进程重启后需重新执行）
#   disable -> /ranger/motion_enable=false  立即撤销许可（门禁输出归零）
#   estop   -> /ranger/soft_estop=true      软急停（门禁内锁存，速度链立即归零）
#   clear   -> /ranger/soft_estop=false     解除软急停（许可仍需单独 enable）
#   status  -> 查看门禁当前判定
#
# 语义：VOLATILE 连发 5 次，不做 latched——重启后的门禁不会自动恢复许可。
set -uo pipefail
source /opt/ros/humble/setup.bash 2>/dev/null

MODE="${1:-status}"
case "$MODE" in
    enable)  TOPIC=/ranger/motion_enable; DATA=true ;;
    disable) TOPIC=/ranger/motion_enable; DATA=false ;;
    estop)   TOPIC=/ranger/soft_estop;    DATA=true ;;
    clear)   TOPIC=/ranger/soft_estop;    DATA=false ;;
    status)
        timeout 5 ros2 topic echo /ranger/motion_permitted --once 2>/dev/null \
          && echo "（true=门禁放行中；false=阻断）" \
          || echo "门禁未运行或 /ranger/motion_permitted 无数据"
        exit 0 ;;
    *) echo "用法: $0 enable|disable|estop|clear|status"; exit 1 ;;
esac

# timeout 兜底：门禁不在/环境错域时命令不至于静默挂起；失败如实报错
if ! timeout 10 ros2 topic pub -r 10 -t 5 "$TOPIC" std_msgs/msg/Bool "{data: $DATA}" >/dev/null; then
    echo "错误：发布失败（ROS 环境未 source / 门禁不在 / 域不匹配）"; exit 1
fi
echo "已发布 $TOPIC=$DATA（5 次 @10Hz，VOLATILE）"
[ "$DATA" = true ] && [ "$TOPIC" = /ranger/motion_enable ] && \
  echo "注意：许可不跨门禁进程重启；真机测试按 AGENTS.md 逐次授权。"
[ "$MODE" = clear ] && \
  echo "注意：clear 只解除软急停；若许可仍为 enable，运动将立即恢复（先 disable 再 clear 可保持静止）。"
