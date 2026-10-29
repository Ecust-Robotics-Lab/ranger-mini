# 文档入口

更新：2026-10-29。当前操作与历史证据分开阅读；旧阶段文档中的“当前/下一步/未授权”仅指该报告日期，不能替代当前指令与 AGENTS。

## 接手与当前操作

| 文档 | 用途 |
| --- | --- |
| [HANDOFF](HANDOFF.md) | 新上下文快速接手、代码/数据/工作树及关键接口 |
| [STATUS](STATUS.md) | 当前能力、验收边界和最近验证日期 |
| [根 README](../README.md)、[AGENTS](../AGENTS.md) | 当前命令、默认配置与工程约束 |
| [PROJECT_LAYOUT](PROJECT_LAYOUT.md) | 整合后的文件职责、保留/删除范围 |
| [标定 README](../calibration/README.md) | 内参、外参、外触发同步与当前包位置 |
| [定位契约](localization_backend_contract.md)、[受管会话](managed_sessions.md) | 标准位姿/健康、同刻 TF、启动/清理/故障处理 |
| [FAST 前端接口](fast_livo2_interfaces.md) | 后端必须遵守的点云/位姿/时间语义 |
| [三传感器录包与远程预览](fast_livo2_recording.md) | 一键采集、日志归档、RViz/Foxglove及路线验收 |
| [后端与重定位计划](FAST_LIVO2_BACKEND_PLAN.md) | B/C1/C2 离线实现；D 与真实泛化仍待验 |
| [C1 接口与操作](fast_livo2_localizer_interfaces.md) | 归档加载、给定初值、持续定位、状态与隔离桥接 |
| [C2 接口与操作](fast_livo2_relocalizer_interfaces.md) | 全局检索、六自由度粗配准、独立运动确认、取消和恢复 |
| [重力对齐报告](fast_livo2_gravity_alignment_report.md) | 新前端 +Z 朝上；旧地图不追认对齐 |

## 最新实现与验证报告

- [阶段 C2 报告](fast_livo2_stage_c2_report.md)：可选离线全局重定位、30项数值/4项ROS、1362对同包检查；跨会话仍 PARTIAL。
- [阶段 C1 报告](fast_livo2_stage_c1_report.md)：只读先验地图、六自由度初值/跟踪、故障及隔离桥接；独立会话与真值精度仍 PARTIAL。
- [阶段B真实运动后端验证](fast_livo2_stage_b_motion_report.md)：PCL/GICP各1362对、13条回环约束、快照/传播/故障回归及优化前后对比视频，单包离线管线通过，真值精度仍PARTIAL。

- [2026-10-28 建图演示视频](fast_livo2_demo_20261028.md)：首份运动包离线重放、1080p完整/720p两倍速视频；保留原包，第二份180秒数据按请求移入回收站。

- [2026-10-27 采集入口报告](fast_livo2_recording_report.md)：完整录包脚本、故障测试与Foxglove收云；保留首次无图像故障及后续恢复记录，当前状态见STATUS。

- [阶段 B 报告](fast_livo2_stage_b_report.md)、[后端契约](fast_livo2_backend_interfaces.md)：独立回环/图优化、依赖、归档、故障与隔离桥接证据。

- [FAST 源码审查](fast_livo2_source_review.md)：固定分支/SHA、许可和各后端真实调用路径/缺陷。
- [阶段 A 离线报告](fast_livo2_stage_a_report.md)：构建、静态融合、视觉参与和 DDS 基线。
- [在线报告](fast_livo2_online_report.md)：真实三融合/RViz、同步 v2 修复与停止；当时源码旧路径后已迁移。
- [整合报告](consolidation_report.md)：清理、标定迁移、配置切换与离线回归。
- [10月26日文档整理报告](documentation_refresh_report.md)：当次整理、过时文件删除与引用检查。

## 历史阶段与证据

| 阶段 | 报告 |
| --- | --- |
| 早期环境与网络 | [环境审计](00_environment_audit.md)、[雷达连接处理](LIDAR_CONNECTION.md) |
| 底盘 | [源码/接口](02_base_interface.md)、[运动与退出记录](02_ros_motion_test.md)、[9月历史实验清单](02_base_manual_tests.md) |
| 雷达 | [Mid-360S 验证](03_mid360_validation.md) |
| Lightning / Nav2 | [建图](04_mapping_report.md)、[历史Lightning人工采集](04_manual_capture.md)、[定位适配](05_localization_interface.md)、[平地导航](06_flat_navigation_acceptance.md) |
| 架构批次 1–3 | [基线](architecture_baseline.md)、[解耦](architecture_batch2_report.md)、[时序与进程](architecture_batch3_report.md) |
| 仓库维护 | [发布/作者历史](github_publication.md) |

`*_evidence/`、地图 manifest/acceptance、标定原始结果保留作为证据，不因失败或接口过时删除。旧文档中的设备地址、参数、授权和“待执行”清单仅属于当时记录；当前操作以HANDOFF、STATUS和专用入口契约为准。
