# 单 P0 在线闭环漂移实验准备

日期：2026-11-11。软件准备 PASS；五次正式实车实验 NOT_RUN，物理精度验收 PARTIAL。

## 用户要求与实现

仅一个人工固定 P0，出发与返回时恢复底盘中心及相机朝向端车头。计划五次长/中/短距离的行走，用户没有独立测量路线长度。本次只准备脚本供用户执行，没有启动设备、CAN、底盘或运动。

独立目录：[experiments/fast_livo2_p0_closure](../experiments/fast_livo2_p0_closure/README.md)。`run.sh` 为交互运行入口；`suite.py` 组织五个独立会话、事件标记、身份核验与中断收尾；`mapping_entry.py` 以精确文件路径加载原FAST建图入口，局部复用当前latency相机及库，并加入只读观察器；`observer.py` 保存采样与接收时刻、原始位姿、图版本、状态；`analysis.py` 导出三条base_link轨迹及指标、CSV、PNG和中文报告。

默认路线标签短/中/长/中/长，可以逐轮修改。每轮独立初始化：就绪后静止15秒、起点静止3秒、人工驾驶、P0复位后回车、终点静止15秒；终点评估窗口固定为返回后1～3秒与11～13秒。每轮最长1800秒。不会启动底盘驱动、导航或发送速度；wheel只在同域已有源时被动录制，不自动启动receive-only驱动。

复用现有受管建图会话与资源锁，拒绝抢占已有设备；只向本次记录并匹配boot/PID/start_ticks的进程请求停止。每次采集与重算使用新目录，失败/中断attempt保留；`--resume` 可继续未完成轮次，不覆盖失败证据。参数、实验脚本与关键二进制SHA冻结，变化后不能混入同组。

## 指标口径

- 所有轨迹先按本次非零XYZ/RPY工作外参转换到base_link中心；不拟合全轨迹、不调尺度。起终点位置中位数XY距离与航向最短角差为主指标。
- 原始前端优先从原bag完整提取Odometry。最终历史轨迹按同一冻结图版本逐关键帧修正与前端相对运动重建，不能将关键帧TUM首尾直接当物理评估端点。
- 在线位姿来自当时实际接收记录；第一次发布、有效状态、新鲜度、原采样时刻、窗口截止接收时刻均保留。重复心跳不补样，最终图不追溯替换在线轨迹。
- 无独立测距，使用同一原始前端估算长度给三条轨迹归一化，另列estimated_relative_drift_rate_percent；relative_drift_rate_percent保持null。后续按轮次传入`--length-m`可重算，不覆盖原结果。
- 每个窗口至少10采样、至少70%跨度覆盖、最大缺口0.5秒；不满足则N/A。记录窗口跳变、起点静止范围、在线行程采样覆盖、最长缺口、接受回环事件/因子数。没有回环时不能宣称验证了回环修正。
- 不同距离逐次报告，不混算固定路线重复性；P0复位由人工确认，没有独立全程真值，不报告APE/RPE，不由这组结果替代C1/C2定位或绝对地图精度验收。

## 只读核对与软件验证

分支仍`refactor/pluggable-navigation`，HEAD为`e8245fa80310a4e91b5d2eb303f33883bb965216`；已有未提交工作保留。本次boot为`a6e6b3b6-da5f-4add-bc98-4a7d6546fb9a`，核对时未见设备/FAST/底盘运行进程。

前端固定来源仍humble分支`837b7bbc1431cb04cf936528e52c83c835efba8e`及本项目补丁，使用online_frontend_install；B使用online_install。当前前端SHA `83600c6f6781019466ff629f8fc8a1f820763004adc804dcbff2ad855a12900a`，B节点SHA `d642e69017932b7f54c16c856c5b796f06fdf856f921394ce72a245c6f60d972`。实验实际启动时重新冻结，不把本次快照当未来身份。

命令与退出码：

| 命令 | 退出码 | 结果 |
|---|---:|---|
| `bash -n experiments/fast_livo2_p0_closure/run.sh` 与Python AST解析 | 0 | 语法通过 |
| `bash experiments/fast_livo2_p0_closure/run.sh --dry-run` | 0 | 五次计划、hardware_started=false、无底盘/速度 |
| `python3 experiments/fast_livo2_p0_closure/mapping_entry.py --dry-run --visualization none --domain 191`（ROS环境） | 0 | 实际组件计划使用latency相机与独立观察器 |
| `/usr/bin/python3 -m unittest discover -s experiments/fast_livo2_p0_closure/tests -v`（Humble+B消息环境） | 0 | 16项通过 |
| `bash experiments/fast_livo2_p0_closure/run.sh --status` | 0 | 无实车实验组运行 |

首次入口计划测试暴露根目录通用mapping_session与FAST专用同名模块的导入冲突，已改为精确文件加载，最终检查通过。测试覆盖：非零外参、角度跨±π、最终图无未来参考、晚到/过期/重复在线输出、窗口缺样与跳变、原bag读取、三条合成轨迹差异、重算不覆盖、坏SHA/跨会话拒绝，以及真实假进程正常停止/中断保存证据。测试没有创建ROS节点或连接设备。

证据目录：[p0_closure_preparation_20261111](fast_livo2_evidence/p0_closure_preparation_20261111/)。本次没有构建新二进制，没有提交或推送。

## 最少人工操作

保持车在P0、中心与朝向对齐，在交互终端运行：

```bash
cd /home/lab307/ranger_nav
bash experiments/fast_livo2_p0_closure/run.sh
```

按提示启动、静止、出发、驾驶和复位；每轮最终回车后保持静止15秒。数据与汇总在实验目录的`runs/`内。五次真实数据尚待用户执行。
