# 文档入口

更新：2026-11-05。当前入口与带日期证据分开阅读；旧报告的“当前/下一步/未授权”只指当时快照，不能覆盖最新HANDOFF/STATUS或自动触发执行。

## 接手与当前操作

| 文档 | 用途 |
| --- | --- |
| [HANDOFF](HANDOFF.md) | 新上下文快速接手、代码/数据/工作树及关键接口 |
| [新Agent提示词](NEXT_AGENT_PROMPT.md)、[交接/清理报告](handoff_cleanup_20261105.md) | 可复制开场词、本次删除名单/原包和证据保留边界 |
| [STATUS](STATUS.md) | 当前能力、验收边界和最近验证日期 |
| [根 README](../README.md)、[AGENTS](../AGENTS.md) | 当前命令、默认配置与工程约束 |
| [PROJECT_LAYOUT](PROJECT_LAYOUT.md) | 整合后的文件职责、保留/删除范围 |
| [标定 README](../calibration/README.md) | 内参、外参、外触发同步与当前包位置 |
| [定位契约](localization_backend_contract.md)、[受管会话](managed_sessions.md) | 标准位姿/健康、同刻 TF、启动/清理/故障处理 |
| [FAST 前端接口](fast_livo2_interfaces.md) | 后端必须遵守的点云/位姿/时间语义 |
| [三传感器录包与远程预览](fast_livo2_recording.md) | 一键采集、日志归档、RViz/Foxglove及路线验收 |
| [后端与重定位计划](FAST_LIVO2_BACKEND_PLAN.md) | B/C1/C2 离线实现；D 与真实泛化仍待验 |
| [AprilTag 增量计划](FAST_LIVO2_APRILTAG_PLAN.md)、[接口与操作](fast_livo2_apriltag_interfaces.md) | 在既有 B/C1/C2 上增加观测、地标与候选，默认关闭 |
| [C1 接口与操作](fast_livo2_localizer_interfaces.md) | 归档加载、给定初值、持续定位、状态与隔离桥接 |
| [C2 接口与操作](fast_livo2_relocalizer_interfaces.md) | 全局检索、六自由度粗配准、独立运动确认、取消和恢复 |
| [重力对齐报告](fast_livo2_gravity_alignment_report.md) | 新前端 +Z 朝上；旧地图不追认对齐 |

## 最新实现与验证报告

- [20261105本地分组提交](local_git_checkpoint_20261105.md)：按用户后续指令保存7组本地检查点；记录完整性/静态检查和活动field会话区别，无新实验、设备操作或推送。

- [20261105交接与清理](handoff_cleanup_20261105.md)：精简当前交接/状态、修正旧配置与地图说明，删除3份过时未引用试录、3空目录和6处缓存约14.1GiB；当前源码、地图和后续必要原包保持，不运行新算法实验。

- [同戳与扫描相位核查](fast_livo2_timestamp_phase_report.md)：核实上游+100ms配置和作者的一帧差解释，两新包1800扫描/1843相机会话帧只读审查；按用户要求正式前端试验改为+0.1s，11项既有回归和dry-run通过，曝光参考点与Tag时间仍未实测。

- [C1 修正归因与 GICP 对照](fast_livo2_c1_correction_report.md)：525扫描×4控制、原PCL逐帧精确复现；既有GICP完整C2/C1运行及冻结旧Tag跨访问17.32→6.11px，6项新/3项相关测试通过。一次短暂DEGRADED按原规则拒绝，默认与部署未改，真实Tag时间/地标/回环仍待验。

- [两份新包与地图对照](fast_livo2_tag_two_capture_report.md)：两新包训练冻结/独立验证、转向新建局部图与独立直行C2→C1功能通过；同帧旧图16.21→新图6.23 px，Tag跨访问仍17.29 px、无VALID地标；C1计算与离线取消评估修正，74项相关测试通过，未部署或启动硬件。

- [Tag 运动几何与初始化对照](fast_livo2_tag_motion_geometry_report.md)：新增5次隔离重放、固定LiDAR–IMU时间扫描与图像稳定窗口诊断；时间/零偏单项调整仍未解决三融合残差，3项新和5项相关测试通过，准备分离动作独立验证，未部署或登记VALID。

- [独立包 C2 全局定位与标定复核](fast_livo2_cross_session_global_report.md)：纯LiDAR候选路径已实际运行；1195对输入、936有效采样，无LOST/ERROR，位置正确性待确认；独立静态板边界复核和5项新测试完成，Tag与公共桥接仍分开验收。

- [120 秒车载 Tag 同步与受控重放](fast_livo2_tag140_sync_diag02_report.md)：可靠 writer 本次实采无漏扫描；Tag/原始 gyro 与当前源码独立重放表明 +120 ms 候选改善尾部误差，仍未通过固定地标门槛；25 项相关测试通过，实机补偿未部署。

- [同步漏收缓解与车载诊断准备](fast_livo2_sync_capture_diagnostics_report.md)：writer 可靠订阅、DDS 来源/时序/事件和录包归档；7 项交付测试、4 项录包回归及共享协议检查通过，120 秒用户手动录制待执行，物理同步仍 PARTIAL。

- [140 mm 同步与固定地标一致性](fast_livo2_tag140_consistency_report.md)：writer 漏收 7 个已录扫描与重复戳对应；固定轨迹/时偏诊断仍未通过，5 项新测试，无驱动或时偏修改。

- [140 mm Tag 检查包](fast_livo2_tag140_check_report.md)：60 s 真实新采集，593 帧全量筛查、556 次检测；重复共享戳仍 PARTIAL，原包 SHA 不变，无地标注册。

- [录包就绪与报错修复](fast_livo2_recording_readiness_report.md)：启动前采集 IP 校验、未就绪组件/话题及断流错误直达终端；29 项离线测试通过，无硬件操作。

- [AprilTag 旧地图补录评估](fast_livo2_apriltag_landmark_assessment_report.md)：固定性用户确认，14 帧均有健康 C1 支持；双模式、约 4 cm 平移基线及跨帧回投不一致，暂不登记 VALID，6 项专项测试通过。

- [AprilTag 现有包全量报告](fast_livo2_apriltag_fullbag_report.md)：1367 帧、14 次 ID 1/28 支假设，原戳关联与日志核对，原包 SHA 不变；物理同步/真实地标仍 PARTIAL。

- [AprilTag 离线增量报告](fast_livo2_apriltag_report.md)：22 项新 C++、5 项 ROS/工具及原系统回归；真实样本 ID 1/0.155 m 双解检测，真实地标/跨访问验收仍 PARTIAL。

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
