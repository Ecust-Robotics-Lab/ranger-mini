# 单 P0 在线闭环漂移实验

五次正式实验，只设置一个人工固定 P0。默认路线标签为“短、中、长、中、长”，开始时可改；各轮距离可以不同。人工驾驶，最后依照物理标记恢复底盘中心和相机朝向车头。脚本不启动底盘驱动、CAN、导航，不发速度；人工驾驶所需的遥控设备由操作者管理。

## 一键运行

```bash
cd /home/lab307/ranger_nav
bash experiments/fast_livo2_p0_closure/run.sh
```

运行这条命令并在提示处回车，会启动本轮雷达、外触发相机、三融合 FAST-LIVO2、B 建图后端与录包。相机使用当前 latency_install。设备已由其他会话占用时拒绝抢占。

默认自动读取最近有进度的实验组并跳过已完成轮次；误建但没有开始的空组不会遮住已有进度。第一次没有历史组时自动新建。五次全部完成后再次运行只提示查看结果；新做五次请加 `--new`。当前已有第1轮的组会自动从第2轮继续。

2026-11-11 启动时钟修复：B 与 C1/C2 共用启动稳定窗口，不再用首帧接收时刻直接固定锚点；前端等待 `/clock` 就绪后才启动。终端会显示时钟、相机、前端配对和后端状态，时钟失败时立即报告。旧实验组 `20261111_170356-pjs0bdu3` 在启动阶段失败，原包/日志保留；修复后请重新运行上面的命令创建新组，不能用 `--resume` 混用修复前后的来源身份。

每轮操作：

1. 在 P0 对齐中心和车头，输入路线标签或直接回车，启动本轮。
2. 等待初始化和 15 秒静止预热；第二次回车后静止 3 秒，出现“可以出发”再走。
3. 人工行走所选长/中/短路线。回到 P0，完成中心和车头对齐、停稳后，第三次回车。
4. 保持不动 15 秒；系统采集停稳后 1～3 秒与 11～13 秒两个固定窗口，随后自动封存地图、停本轮设备、计算结果。
5. 查看本轮结果，在下一轮提示回车继续。每轮重新初始化，不继承上一轮图。

每轮最长采集 1800 秒；启动超时 120 秒。中断用 Ctrl+C，等待封存与受管清理完成，失败和中断记录保留。完整核验可能需要几分钟，进度在终端显示，详细日志在本轮 mapping_console.log。

```bash
# 只显示计划，不开设备
bash experiments/fast_livo2_p0_closure/run.sh --dry-run
# 查询当前/最近实验组
bash experiments/fast_livo2_p0_closure/run.sh --status
# 另一终端请求停止本实验组（精确 boot/PID 身份）
bash experiments/fast_livo2_p0_closure/run.sh --stop
# 继续未完成次数；失败轮写入新的 attempt，不覆盖原数据
bash experiments/fast_livo2_p0_closure/run.sh --resume /绝对路径/实验组目录
# 显式重新做一组五次实验
bash experiments/fast_livo2_p0_closure/run.sh --new
```

## 数据与结果

所有数据放在本目录 `runs/日期时间-随机后缀/`，该目录不纳入 Git：

```text
SUMMARY.md                         五次状态与各自报告链接
config.json / source_hashes.json    冻结配置、来源与实际二进制 SHA
calibration.json                    本次车体外参快照
trial_01_attempt_01/
  events.json                      出发/返回的 steady 原时刻
  mapping_console.log              本轮启动、运行、封存日志
  capture/raw/                     原始三传感器、前端与在线后端 rosbag
  capture/map/                     冻结图、original.tum、optimized.tum、关键帧
  capture/closure_samples.jsonl    观察器原采样戳、接收时刻、状态、图版本
  analysis/result-时间戳/
    REPORT.md / metrics.json / metrics.csv
    frontend.csv / optimized_history.csv / online_backend.csv
    trajectories.png
```

`runs/latest.json` 记录最近使用的组；默认入口也扫描已有进度，避免空组遮住原结果。所有重算写新的结果目录。采集参数、逐轮采集过程或二进制发生变化后不能把新轮次加入旧组；只修正离线分析或组选择流程时可继续同组，原始来源哈希保留，修订版本另记于 `analysis_revisions/` 和结果JSON。

## 指标与口径

- 所有轨迹先按冻结的非零 XYZ/RPY 外参转换到 base_link 中心；不拟合整条轨迹、不调整尺度。
- 闭环位置误差为起终静止窗口位置中位数的 XY 距离；角度误差为窗口航向角的最短角差。
- 没有独立测距时，使用原始前端的同一估算行程，给三条轨迹计算“按估算路程归一化”的闭环漂移率。`relative_drift_rate_percent` 保持 null；估算值另存在 `estimated_relative_drift_rate_percent`。不把 wheel 或前端自估路程当成真值。
- 估算行程只统计出发到人工标记返回的轨迹，以 2cm 空间采样间隔减小静止抖动累加；仍包含前端误差及实际路线偏差。
- 若以后独立测得某一轮路程，用 `--analyze /绝对路径/trial_01_attempt_01 --length-m 80.0` 重算；只对这一轮生效。
- 原始前端来自原 bag；观察器作为缺包时的降级证据，不能据此报数据完整 PASS。
- 最终历史轨迹由冻结图和前端相对运动重建：`X(t)=X_k_final*inverse(U_k)*U(t)`，只取不晚于 t 的关键帧。它不代表所有采样点独立参加优化。
- 在线结果保留当时首次发布的采样及实际接收时刻；按新鲜度和窗口截止时间筛选，不用最终图替换在线历史，不拿重复心跳补帧。
- 起终窗口至少 10 个采样、跨度覆盖至少 70%、最大采样缺口不超过 0.5 秒。缺数据记 N/A；窗口步进超过 5cm/2°会标记跳变。起点前端静止范围大于 2cm/1°时数据验收为 PARTIAL。
- `PASS_DATA` 只表示采集/封存与计算所需数据条件通过，不是预设精度门槛通过。P0 复位由人工确认，实际误差包含复位误差、工作外参误差；不报告无真值的 APE/RPE。
- 单 P0 主要评价前端和 B 后端闭环返回一致性；不据此宣称全程地图精度或 C1/C2 重定位验收通过。短中长分别保留结果，不混算固定路线重复性。

## 软件验证（不连接设备）

```bash
source /opt/ros/humble/setup.bash
source ws_fast_livo/online_install/setup.bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m unittest discover \
  -s experiments/fast_livo2_p0_closure/tests -v
```

合成测试只验证数值、归档、时间/会话边界和入口计划，不代表实车成绩。
