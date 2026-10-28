# 新上下文接手指南

更新：2026-10-28。项目 `/home/lab307/ranger_nav`。C1 已完成离线范围并在后续指令下本地提交；当前按新指令开始 C2 离线实施，未启动硬件。接手先核对工作树和 [C1 报告](fast_livo2_stage_c1_report.md)，不要重复实现归档加载和初值定位。

## 阅读入口

1. [AGENTS](../AGENTS.md)：当前有效约束与授权边界。
2. [STATUS](STATUS.md)：完成情况、未验证能力和生产默认。
3. [整体实施计划](FAST_LIVO2_BACKEND_PLAN.md)：架构、参考仓库/版本/许可、C1/C2 详细步骤、数据与验收要求。下一阶段计划统一维护在此，交接提示词只引用它。
4. [C1 报告](fast_livo2_stage_c1_report.md)、[阶段 B 运动报告](fast_livo2_stage_b_motion_report.md)、[B 首次交付](fast_livo2_stage_b_report.md)：实际构建、命令与证据。
5. [前端契约](fast_livo2_interfaces.md)、[B 契约](fast_livo2_backend_interfaces.md)、[C1 契约](fast_livo2_localizer_interfaces.md)、[公共定位契约](localization_backend_contract.md)。
6. [工程结构](PROJECT_LAYOUT.md)、[根 README](../README.md)、[受管会话](managed_sessions.md)；涉及采集/标定时再读 [录包指南](fast_livo2_recording.md) 和 [标定入口](../calibration/README.md)。

## 当前代码与工作树

- 分支 `refactor/pluggable-navigation`；B 推送基线为 `46c2efad12c6cd065739a747f6edfa6a06ee52c1`。C1/既有成果随后按新指令本地分组提交，当前 HEAD 须以 git 现查，尚未推送新提交。此前三个 B 提交已按用户要求推送：`9e7c97e`（B 核心）、`b299078`（运动验证）、`46c2efa`（CSV LF）。未来接手仍需重新检查实际 Git 状态。
- 相机/标定整合、采集/视频、既有证据和 C1 已分组本地提交：`0758ac7`（driver）、`0ced46c`（整合/采集）、`a13fee4`（B 补充证据）、`1212b43`（C1）。文档收尾后建立 C2 开工基线，后续需检查新阶段工作树。保留用户原雷达文档修改和六份 driver 证据，不能 reset/clean、删除未跟踪目录或把所有修改扫进一次提交。
- B 自有包：`ws_fast_livo/src/ranger_fast_livo_backend`；依赖/构建/安装分别为 `ws_fast_livo/backend_deps`、`backend_build`、`backend_install`，不覆盖既有前端。
- 前端：`vendor/fast_livo2_review/ros2_frontend`，humble，`837b7bbc1431cb04cf936528e52c83c835efba8e` 加 `patches/fast_livo2_stage_a.patch`。全部构建锁见 `config/fast_livo2/sources.lock.json`。
- 自有 C1 包为 `ws_fast_livo/src/ranger_fast_livo_localizer`，包含严格归档读取、三维子图、初值/持续跟踪、请求代际和私有结果；复用 B 核心库，安装在既有 backend_install。`scripts/build_fast_livo2_localizer.sh` 还构建可选 ws_nav C1 适配器；Lightning 不依赖导入该模块才能启动。C2 尚未实现。
- `calibration/` 为正式路径，`~/calibration` 只是兼容链接；同步包位于 `ws_camera/src/`。SDK 沿用原本地文件，不随本轮 Git 分发。

## 已完成与主要限制

- A：前端本机构建、829 对静态基线、真实三融合预览及 RViz/Foxglove；预览已按用户指令停止。不要重新从安装/选型开始，也不能假设历史进程仍在运行。
- B：严格配对、关键帧、位置候选、PCL/GICP/VGICP、几何门控、Pose3 iSAM2、非关键帧传播、分片存储与一致性导出、标准适配器和隔离公共桥接已实现。
- C1：新进程读取 B 地图、显式离线来源兼容、人工初值、六自由度几何确认/跟踪、失效重初始化、独立消息/适配和受管入口。C++ 18 项、ROS 9 项及相关回归见 C1 报告；C1 源码和证据已在 `1212b43` 本地提交，未推送。
- C1 同包功能检查：PCL/GICP 各1362对，第366对给初值，分别994/988个有效定位采样，GICP另有6次不收敛拒绝，无LOST/ERROR。结果在 `outputs/fast-livo-c1-final-20261028-{pcl,gicp}/`，小型证据在 `docs/fast_livo2_evidence/stage_c1_20261028/`，不能当跨会话性能。
- 单份真实三传感器运动包：PCL/GICP 各 1362 对、49 关键帧、13 条接受并优化的回环约束；16 项既有验收命令及 3 项新增测试通过。完整证据在 `docs/fast_livo2_evidence/stage_b_motion_20261028/run-PYV5azso/`。
- PARTIAL：物理同步/曝光相位、实际 T_base_imu、独立真值精度、多环境误环泛化、实时 wheel 对齐与长期运行。现有后端运行使用逐帧确认节流，不是实时吞吐基准；厘米量级地图修正不是精度认证。
- 生产默认继续 Lightning 建图/定位、wheel、terrain disabled；保留 Nav2 与安全链。FAST B 只负责建图，不具备先验地图定位。

## 数据与演示资产

| 资产 | 路径与用途 |
| --- | --- |
| 原始运动包 | `bags/fast-livo-motion-w5phdosj/raw`，约 136.6 秒，保留原始内容；末尾发布者变化错误已记录，不改称完整 180 秒录制 |
| 前端派生输入 | `outputs/fast-livo-demo-20261028-fHJTbGCU/frontend/native_frames`，1362 对，用于已完成的 B 验证 |
| 前端展示视频 | 同一 demo 目录中的 `FAST-LIVO2_Demo_1080p.mp4` 和 `FAST-LIVO2_Demo_share_720p_2x.mp4`，见 [演示报告](fast_livo2_demo_20261028.md) |
| B 全部输出 | `outputs/fast-livo-backend-validation-20261028-pcl/` 与 `outputs/fast-livo-backend-validation-20261028-gicp/` |
| B 对比视频 | PCL 目录下 `comparison/comparison_1080p.mp4`，24 秒，原始/优化地图同尺度展示 |
| 静态基线 | `bags/fast-livo2-baseline-126a5fAo`、`bags/fast-livo2-dds-smoke-final`，不作为运动定位泛化证据 |
| 回退资产 | `.runtime/backups/consolidation_20261026/`、`docs_handoff_20261026/`、`stage_b_motion_20261028/`，均保留 |

大文件常被 Git 忽略，不代表可删除。第二份 180 秒 `ez09stbr` 此前按请求移入回收站；四份更早试录已按另次明确指令清空回收站，不能再说可恢复。旧操作清单分别在 `.runtime/capture_cleanup/20261028_122508_441848/manifest.json` 和 `.runtime/capture_cleanup/20261027_231026_079043/manifest.json`。本次不重复删除或扩大范围；失败启动 `58obp75g` 保留。

## 必须核对的接口

`/fast_livo/odometry` 为 VIO 后 IMU 位姿，`/fast_livo/deskewed_imu` 为同刻单区间 IMU 云。LiDAR–IMU 外参已经应用；不要用累计世界云或 VIO 前旧话题替代。前端内部 double 秒限制与去畸变语义见契约，不以同戳推断物理同步。

`T_map_base=T_map_imu×inverse(T_base_imu)`。唯一公共桥接按原采样时刻插值 wheel，不能把 FAST local 当 wheel odom。当前 sensor 使用过 2020 历元、host 为 2026，不能直接接实时 wheel 或重打 now 掩盖。离线使用 header-clock、use_sim_time、隔离 domain。当前地图不保证重力对齐。

## 安全接手与下一步

先运行只读检查，不自动加 `--execute`：

```bash
cd /home/lab307/ranger_nav
git status --short
git log -3 --oneline
git branch -vv
python3 scripts/runtime_config.py
bash scripts/run_fast_livo2_live.sh --status
bash scripts/build_fast_livo2_backend.sh
bash scripts/run_fast_livo2_backend.sh
bash scripts/build_fast_livo2_localizer.sh
bash scripts/run_fast_livo2_localizer.sh --status
bash scripts/run_fast_livo2_localizer.sh
```

设备状态必须现查；历史“已停止”不是运行时锁。`--offline-bridge` 仅用于明确启用的隔离联调。已有构建/验收复现命令在 B 运动报告，不为纯文档任务重复长时间回放。

当前已收到 [总计划](FAST_LIVO2_BACKEND_PLAN.md) 第 5 节 C2-0 至 C2-4 的离线实施指令；先提交既有成果，再自主完成源码/构建/离线验收，不要从 C1 地图加载重新开始。独立定位采集、同步/外参和 B 泛化补证按条件推进，真实能力不足保持 PARTIAL。不自动进入硬件、导航、高程或离线 BA。

C1 及其他既有成果已按后续明确指令本地提交；无新推送指令。C1 原报告中的“未提交”是交付当时事实，当前提交状态以此处及 Git 为准。开工差异和 3897 份既有文件身份清单在 `.runtime/backups/stage_c1_20261028_161707/`。既有用户修改、六份 driver 证据、原包、地图、标定、失败输出和旧备份保留。

本次已把临时预览/录制过程从 AGENTS 和交接入口移除，原始带日期报告和证据未删除。规划整理前的四份文档备份在 `.runtime/backups/planning_handoff_20261028/`。更广泛过时文档清理须逐项检查引用与证据价值，不批量删旧文件。
