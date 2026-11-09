# 新 Agent 接手提示词

更新：2026-11-09。下方可直接复制。此次只要求接手核对，后续实施和设备操作按新的明确指令执行。

```text
请接手 /home/lab307/ranger_nav。先只读核对，简短汇报接手理解、现有成果、剩余问题和下一步，暂不继续实施或启动设备。

先读：AGENTS.md、docs/HANDOFF.md、docs/STATUS.md、docs/FAST_LIVO2_BACKEND_PLAN.md、docs/FAST_LIVO2_APRILTAG_PLAN.md。然后按当前重点读：
1. docs/fast_livo2_failure_fix_report_20261109.md（最新修复、构建、验证与限制）
2. docs/fast_livo2_failure_research_20261109.md（原因和解决方案）
3. docs/fast_livo2_field_gate_acceptance_report_20261109.md（最新真实现场结果）
4. docs/fast_livo2_field_retest_report_20261109.md
5. docs/fast_livo2_live_chain_report_20261109.md、docs/fast_livo2_live_clock_report_20261109.md
6. docs/fast_livo2_wheel_alignment_report_20261109.md、docs/fast_livo2_navigation_prep_report.md
7. docs/git_checkpoint_handoff_20261109.md（本次分组本地保存）
8. docs/handoff_cleanup_online_mapping_20261105.md（最近清理，仅为历史固定清单，禁止据此继续删除）
还须先读 docs/fast_livo2_field_latency_retest_report_20261109.md：修复后已完成co968xa3的90.044秒实机测试，921相机同步帧全OK，591个不同有效确认，无LOST；3次60–120ms就绪撤销，局部恢复未触发，设备已清理，本次新证据/报告未提交。
需要追溯时再按HANDOFF查旧报告，不直接执行历史命令。

现查Git分支/完整HEAD/工作树、会话boot/PID和进程归属、配置、实际二进制/动态库、原包与地图。保护未提交修改及全部原包、地图、失败证据、标定和.runtime/backups。本次交接已分组本地提交，没有推送；不要自动提交、推送、清理或升级依赖。原包/地图/构建主要保存在本机，Git包含源码、配置、补丁、报告和可版本化证据，Git并非这些大资产的完整备份。

当前进展：在线预建图软件链路已打通，390秒真实包原速DDS验证3900对、305关键帧、114接受回环。正式地图为 outputs/fast-livo-demo-online-mapping-20261105/full_replay_12/map，UUID 253f573d7a7eabbc363a225ab192cc14。候选导航占据图为 outputs/fast-livo-demo-navigation-prep-20261106/navigation_map_final。在线定位默认自有三维距离分布v2→KISS全6DoF粗配准→small_gicp→C1，Tag关闭。保持三融合、正式img_time_offset=+0.1秒和初始重力对齐。

当前重点是在线定位连续性。修复前90秒实机包为 bags/fast-livo-localization-88k5eclm；修复前得到58个不同有效确认、首次定位距首FAST约50.6秒。4200派生wheel无领先已记录clock，60个运行图快照动态TF唯一。相机USB3交付积压约0.53秒，迟到540帧被赋予不对应的较新scan时间，随后4帧BACKWARD，FAST输出间隔约0.66秒，C1过期后约33.58秒全局恢复。失败搜索单轮中位约63ms，大量候选在KISS粗配准内点门槛被拒，不能把34秒全部归咎small_gicp耗时。

最新两项修复（相机已做上述现场功能检查，局部恢复仍未现场触发）：
- 相机取帧与发布/日志分线程、两帧有限队列、采集时保存scan快照、设备tick/scan相对进度检查、CSV分段计时和批量flush。原CSV重检拒绝540–543、544通过。
- 仅显式C2在短时过期后保留已验证地点作为内部先验，以新鲜扫描局部几何重验；2秒/0.75m/0.35rad范围，至少两新采样/0.1秒一致，静止也可恢复。失败/超范围回全局搜索。旧候选不得当新有效修正，0.5秒新鲜度未放宽，独立C1人工初值语义保持。
36项核心测试、7项针对性ROS测试通过；失效处真实记录点云可用两帧局部恢复。原包点云实验使用已接受位姿先验，不是完整原速重跑或实车恢复耗时。软件/离线PASS，实车连续就绪、SDK/USB最初停顿根因、物理同步和绝对精度仍PARTIAL。

实际构建：C1/C2在 ws_fast_livo/online_localization_install；前端在 online_frontend_install。新相机在 ws_camera/latency_install，在线localize_fast_livo2.sh已指向它，原ws_camera/install保留。最新二进制/库完整SHA与依赖见 docs/fast_livo2_evidence/failure_fix_20261109/binary_hashes.json 及实施报告。新底盘只接收版本为 ws_base/install_recording_rx，接收角符号和关闭TF时的空发布端点已修；原安装保留。

用户已授权先用约RPY=[1°,21°,-179°]的工作外参；准确当前XYZ/RPY从 config/fast_livo2/navigation_prep.yaml 和FAST profile读，不能退回历史零旋转或当作实测标定。公共桥接唯一发布map→odom、odom→base_link，同一采样时刻使用wheel历史插值；现有时间映射含未知接收偏差，不是物理同步标定。

建议下一步：先复核上述改动和最新短测结果，分析首次全局搜索等待与三次局部几何拒绝；若需再验证现场局部恢复，先设计明确的短时故障场景。此次提示词不授权开设备；收到该次明确许可后才能启动相机/雷达/receive-only底盘。自动运动、速度、导航目标另须明确授权。历史设备已受管停止，但须按当前boot/PID核实，不能把旧快照当现状。不要因修复软件通过便宣称实车通过。

全局候选召回/长查询窗口和C1初始化预处理缓存尚未修改，按新证据决定优先级。用户准备另行大改规划控制，不自动继续旧Smac/MPPI方案。生产仍Lightning-LM、wheel和既有安全链。请用中文简洁汇报，开发偏快速、复用现有实现，避免过多防御性设计。
```

最新追加：先读 docs/fast_livo2_initialization_optimization_report_20261109.md；本轮C2/C1静止确认和预处理复用已构建，原速离线同包29.820→2.800秒，另包4.220秒。40核心/21后端/9ROS通过，源码及报告尚未提交，全部测试已退出，未再启硬件；勿用历史“初始化缓存未实施”描述当前状态。
