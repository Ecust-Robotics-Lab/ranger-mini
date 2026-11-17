# 文档入口

更新：2026-11-10。当前状态只从HANDOFF/STATUS读取，带日期报告保留当时条件，不从旧报告的“下一步/未提交/未实测”自动续跑。

## 当前接手与计划

| 文档 | 用途 |
| --- | --- |
| [HANDOFF](HANDOFF.md)、[STATUS](STATUS.md)、[AGENTS](../AGENTS.md) | 当前成果、真实限制、安装/资产及操作约束 |
| [可复制新Agent提示词](NEXT_AGENT_PROMPT.md)、[文档清理](handoff_cleanup_20261110.md) | 本次快速接手、过时内容去除范围 |
| [规划控制前置计划](FAST_NAVIGATION_INTEGRATION_PLAN.md) | 历史输入准备和闭环顺序；具体算法已采用EXACT-MPPI计划 |
| [EXACT-MPPI计划](EXACT_MPPI_NAVIGATION_PLAN.md)、[实现/验收报告](exact_mppi_implementation_report_20261110.md)、[接口与复现](exact_mppi_interfaces.md) | 新导航源码入口、GPU、软件验证与真实验收边界 |
| [办公室—走廊实车准备](EXACT_MPPI_FIELD_TEST.md) | 候选路线、窄通道参数、地图门口缺项、分阶段实测与43话题录制 |
| [静态1.5cm与门口诊断](exact_mppi_static_margin_report_20261110.md) | 请求裕量对照、独立几何阻断、旧3D证据和重录前最少核对 |
| [首次定位优化](fast_livo2_initialization_optimization_report_20261109.md) | 已实现静止确认/预处理复用，2.800/4.220秒是回放成绩 |
| [最新实机](fast_livo2_field_latency_retest_report_20261109.md)、[相机/恢复实现](fast_livo2_failure_fix_report_20261109.md) | 现场连续就绪限制与已实现修复 |
| [最新Git检查点](git_checkpoint_initialization_upload_20261109.md) | 已上传源码基线e8245fa；之后文档变更见工作树 |
| [总计划](FAST_LIVO2_BACKEND_PLAN.md)、[Tag计划](FAST_LIVO2_APRILTAG_PLAN.md) | 设计/验收要求，完成度以STATUS为准 |
| [前端](fast_livo2_interfaces.md)、[B](fast_livo2_backend_interfaces.md)、[C1](fast_livo2_localizer_interfaces.md)、[C2](fast_livo2_relocalizer_interfaces.md) | 数据、坐标、时间、定位接口 |
| [在线建图](fast_livo2_online_mapping.md)、[在线定位](fast_livo2_online_localization.md)、[录包](fast_livo2_recording.md) | 当前专用入口；执行须相应授权 |
| [PROJECT_LAYOUT](PROJECT_LAYOUT.md)、[根README](../README.md)、[受管会话](managed_sessions.md) | 目录、分派和会话生命周期 |
| [标定](../calibration/README.md)、[公共定位契约](localization_backend_contract.md) | 标定来源、同刻TF和公共接口 |

## 历史实现报告（内容描述生成时条件）

- [20261109第三次现场复测与剩余阻塞](fast_livo2_field_gate_acceptance_report_20261109.md)

- [20261109第二次90秒现场联调](fast_livo2_field_retest_report_20261109.md)

- [20261109首次现场联调与启动锚点修正](fast_livo2_field_localization_report_20261109.md)

- [20261109在线公共TF联调与短测准备](fast_livo2_live_chain_report_20261109.md)

- [20261109在线每会话时间映射](fast_livo2_live_clock_report_20261109.md)

- [20261109同录包完整公共TF离线链路](fast_livo2_offline_wheel_chain_report_20261109.md)

- [20261109同录包与wheel接入](fast_livo2_wheel_alignment_report_20261109.md)

- [20261106录包自动wheel](fast_livo2_recording_wheel_report.md)

- [20261106当前wheel实物启动检查](wheel_start_report_20261106.md)

- [20261106导航前置接入](fast_livo2_navigation_prep_report.md)

- [20261106在线定位与重定位首轮交付](fast_livo2_online_localization_report.md)

- [20261106新查询包核验](fast_livo2_relocalization_visit01_capture_report.md)

- [20261105统一在线预建图](fast_livo2_online_mapping_report.md)

- [20261105走廊/教室PCD与轨迹](fast_livo2_corridor_classroom_mapping_report.md)

- [20261105走廊/教室录制核验](fast_livo2_corridor_classroom_capture_report.md)

- [20261105录包默认行为](fast_livo2_recording_defaults_report.md)

- [20261105本地分组提交](local_git_checkpoint_20261105.md)

- [20261105交接与清理](handoff_cleanup_20261105.md)

- [同戳与扫描相位核查](fast_livo2_timestamp_phase_report.md)

- [C1 修正归因与 GICP 对照](fast_livo2_c1_correction_report.md)

- [两份新包与地图对照](fast_livo2_tag_two_capture_report.md)

- [Tag 运动几何与初始化对照](fast_livo2_tag_motion_geometry_report.md)

- [独立包 C2 全局定位与标定复核](fast_livo2_cross_session_global_report.md)

- [120 秒车载 Tag 同步与受控重放](fast_livo2_tag140_sync_diag02_report.md)

- [同步漏收缓解与车载诊断准备](fast_livo2_sync_capture_diagnostics_report.md)

- [140 mm 同步与固定地标一致性](fast_livo2_tag140_consistency_report.md)

- [140 mm Tag 检查包](fast_livo2_tag140_check_report.md)

- [录包就绪与报错修复](fast_livo2_recording_readiness_report.md)

- [AprilTag 旧地图补录评估](fast_livo2_apriltag_landmark_assessment_report.md)

- [AprilTag 现有包全量报告](fast_livo2_apriltag_fullbag_report.md)

- [AprilTag 离线增量报告](fast_livo2_apriltag_report.md)

- [阶段 C2 报告](fast_livo2_stage_c2_report.md)
- [阶段 C1 报告](fast_livo2_stage_c1_report.md)
- [阶段B真实运动后端验证](fast_livo2_stage_b_motion_report.md)

- [2026-10-28 建图演示视频](fast_livo2_demo_20261028.md)

- [2026-10-27 采集入口报告](fast_livo2_recording_report.md)

- [阶段 B 报告](fast_livo2_stage_b_report.md)、[后端契约](fast_livo2_backend_interfaces.md)

- [FAST 源码审查](fast_livo2_source_review.md)
- [阶段 A 离线报告](fast_livo2_stage_a_report.md)
- [在线报告](fast_livo2_online_report.md)
- [整合报告](consolidation_report.md)
- [10月26日文档整理报告](documentation_refresh_report.md)

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

- [2026-11-06 Git分组保存与上传检查点](git_checkpoint_20261106.md)

- [自有检索v2、STD/Scan Context对照与GICP默认](fast_livo2_retrieval_comparison_report.md)

- [在线重定位本地Git检查点](git_checkpoint_online_localization_20261106.md)
- [FAST定位接入导航与闭环计划](FAST_NAVIGATION_INTEGRATION_PLAN.md)

- [2026-11-09 首次定位优化
