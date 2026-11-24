# 当前项目状态

核对日期 2026-11-23。项目运行入口、资产与现场排查见 [接手指南](HANDOFF.md)，操作约束见 [AGENTS.md](../AGENTS.md)。

2026-11-24 底盘启动排查 **PARTIAL**。工作台会话 `run-4729a1m6` 仍运行，底盘 `run-q5uv3k6p` 等待新鲜 odom 超时，受管进程已退出。启动快照显示 can0 启用为 500 kbit/s，随后 `ip -details -statistics link show can0` 查到 ERROR-PASSIVE，`tc -s qdisc show dev can0` 显示积压。底盘原日志 `.runtime/sessions/base/run-q5uv3k6p/base.log` 记录 3178 条 `errno=105`（ENOBUFS）。本次只读检查退出 0，未改网络或重启设备。下一步现场确认底盘上电和 USB-CAN 到底盘的 CAN 接线，链路稳定后按当次授权验证新鲜 wheel/odom。

| 模块 | 当前能力 | 待验收 |
| --- | --- | --- |
| FAST-LIVO2 前端 | 正式 FAST 入口使用低延迟 LiDAR–IMU–Camera 前端，图像偏移 +100 ms，重力对齐 | 物理相位、长时稳定性 |
| B 建图 | 已实现独立回环 PGO、逐关键帧地图重建、地图归档 | 多场景误环、绝对精度 |
| C1/C2 定位 | FAST 在线 auto 使用自有 v2 检索、KISS 六自由度粗配准、small_gicp 与 C1 跟踪。已实现强几何静止确认和短时局部恢复 | 跨会话泛化、实机恢复、持续就绪与真值 |
| 底盘及导航 | SE(2) 全局搜索、EXACT-MPPI、V2 行驶/平移/旋转、独立门禁。已有现场单点成功 | 门口、实际制动、完整走廊回返与长时任务 |
| 工作台 | 受管启停、单点/替换/取消、规划重置、记录及只读回放、启动分阶段诊断 | 完整真实现场验收 |
| AprilTag | 可选软件路径已实现，默认关闭 | 实测时间、真实 VALID 地标表及恢复/回环 |

通用 `config/runtime.yaml` 默认采用 FAST-LIVO2、wheel 与 standard 定位接口。Lightning 需显式选择 `config/profiles/lightning.yaml`。导航使用 EXACT-MPPI。坡道与高程控制未纳入当前导航验收。

## 项目运行参数

配置源为 [exact_navigation.yaml](../ws_nav/src/ranger_navigation/config/exact_navigation.yaml)。使用者修改后，应以新会话的实际回报核对生效值。

| 项目 | 当前配置 |
| --- | --- |
| 速度和加速度 | 平移 0.50 m/s、旋转 0.80 rad/s、线加速度 0.40 m/s²、角加速度 0.60 rad/s² |
| 行驶几何 | `min_radius: 0.48` m |
| 停稳 | 线速度 0.05 m/s、角速度 0.025 rad/s、持续 0.5 秒 |
| 定位时限 | 原采样 400 ms、接收 250 ms |
| 其他时限 | wheel/云/底盘反馈各按输入检查，自主指令租约 100 ms |
| 静态图裕量 | 总 0.025 m，包含 0.015 m 扫掠界限 |
| 实时观测覆盖 | 总外扩和保护范围各 0.015 m，边界/内部未知格额度 16/6，`allow_boundary_unknowns: true` |

11月23日核对时，工作台、导航、底盘及 FAST 定位的最近受管会话均为 STOPPED。实际状态以 boot/PID 现查为准。 `00fcf93` 检查点验证通过 49 项核心/配置、66 项工作台/身份、40 项 CUDA/导航检查，底盘补丁按锁定前置与源码逐文件核对一致。

2026-11-24 精简首批软件检查 **PASS**。删除独立旧实验，建图、录包和标定入口单列。79 项检查通过，实车未执行。命令、版本和日志见 [精简记录](repository_cleanup_20261124.md)。

同日默认配置切换 **PASS**。FAST 默认不再校验 Lightning 地图包，Lightning 保留为显式基线。116 项软件检查通过，未进行实车测试。已从 `bags/` 移出 34 个过时空包目录，残留日志和元数据存入本机备份。 [配置与清理记录](repository_cleanup_20261124.md)

同日运行目录清理 **PASS**。移除约 738 MiB 可重建的测试缓存，工作台预演和 62 项软件检查通过。 [清理记录](repository_cleanup_20261124.md)

同日文档清理 **PASS**。删除已确认的旧文档及证据，现用回归输入和 Lightning 构建清单迁出 `docs/`。94 项回归检查、停车反馈检查和入口预演通过。 [清理记录](repository_cleanup_20261124.md)

同日第二批文档清理 **PASS**。延迟实录回放数据库已删除，`exact_mppi_evidence/` 仅保留前端来源锁引用的三份对照 JSON。73 项离线检查、入口预演和已有结果的可视化渲染通过。 [清理记录](repository_cleanup_20261124.md)
