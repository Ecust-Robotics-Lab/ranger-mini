# Ranger Mini 导航系统

ROS 2 Humble / Ubuntu 22.04 / x86_64，实物为 **Ranger Mini V2 + Livox Mid-360S + 海康相机**。Lightning-LM 保持生产默认；FAST-LIVO2 三传感器前端、独立回环后端 B、C1先验定位与C2全局重定位已实现，已有不同采集包的离线功能对照；完整泛化、真值精度和实时接车仍待验。

**新上下文从 [交接指南](docs/HANDOFF.md) 开始**，再读 [当前状态](docs/STATUS.md)、[工程约束](AGENTS.md)、[后端与重定位计划](docs/FAST_LIVO2_BACKEND_PLAN.md)。全部文档见 [docs 索引](docs/README.md)。

2026-11-05交接已更新，复制 [新Agent提示词](docs/NEXT_AGENT_PROMPT.md) 可快速接手。正式前端试验现使用用户指定的 **+100ms**，完整新配置评估尚未重跑；历史+110ms结果保持原条件。过时试录清理名单与保留理由见 [清理报告](docs/handoff_cleanup_20261105.md)。

AprilTag 候选与地标增量已完成软件及离线检查，默认关闭；见 [细化计划](docs/FAST_LIVO2_APRILTAG_PLAN.md)、[接口与操作](docs/fast_livo2_apriltag_interfaces.md)、[交付报告](docs/fast_livo2_apriltag_report.md)。真实地标与跨访问性能仍待验，既有成果已按用户指令[分组本地提交](docs/local_git_checkpoint_20261105.md)，未推送。

## 当前能力

| 模块 | 已有结果与边界 |
| --- | --- |
| 生产导航路线 | Lightning、轮速、公共定位桥接、Nav2、安全速度链已有实现；完整真机导航仍 PARTIAL |
| 定位契约 | LocalizationEstimate 携带地图系车体位姿、原采样时刻和健康；按定位时刻插值轮速，唯一管理公共 TF |
| FAST-LIVO2 | 保留LiDAR–IMU–Camera更新；已有真实运动重放、B/C1/C2与Tag软件检查；FAST会话未运行，其他现场会话状态见HANDOFF |
| 配置切换 | mapping_backend 选择 Lightning / FAST-LIVO2 前端；localization_backend 独立选择 lightning / standard |
| 标定与同步 | 已整合到本工程；v2 共享戳保护已修复，物理同步、几何标定与实时历元仍待验 |
| 待验收 | C1/C2跨会话与真实负样本、同步/外参/真值、实时wheel与D接车；高程/坡道另立范围 |

当前保留旧136.6秒建图包、140mm同步/转向/直行诊断包及固定回归基线；具体用途和当前局部图见[HANDOFF](docs/HANDOFF.md)。旧包的[建图演示视频](docs/fast_livo2_demo_20261028.md)和后续图片仅展示各自条件的结果，不代表真值精度或真实Tag辅助验收。standard是外部标准消息接口，不是另一套现成定位算法。

## 目录与配置

```text
config/runtime.yaml     生产默认与后端/地图/阈值
config/profiles/        继承默认配置的算法选择
config/fast_livo2/      本机试验参数、标定来源、固定 SHA 锁
calibration/            标定工具、流程、原始数据与结果
scripts/、tests/        统一入口、会话管理、验证
ws_camera/src/          当前维护的触发相机与 timeshare_writer
ws_nav/src/             定位消息、公共桥接、导航与安全
ws_slam/                既有 Lightning 部署
ws_fast_livo/           FAST-LIVO2 隔离构建部署
ws_base/、ws_livox/     底盘与雷达工作空间
vendor/、patches/       第三方固定源码与实际补丁
bags/、maps/、docs/     数据、地图和验证证据
.runtime/               受管状态、日志与回退备份
```

正式标定根目录为 `calibration/`，`~/calibration` 仅是兼容链接；新入口已在临时无链接条件下通过离线验证。原相机驱动/SDK及用户修改保留。职责与清理范围见 [工程结构](docs/PROJECT_LAYOUT.md)、[整合报告](docs/consolidation_report.md)。

本地原始图片/rosbag、SDK、第三方工作空间、构建与 .runtime 不进入主仓库。相机只纳入两个维护包源码。A/整合、C1、C2、重力对齐与净化文档已有本地提交，本轮未推送；原始数据和本机部署仍须保留，远端副本不能代替当前工程；第三方完整 SHA/补丁来源见交接指南与版本锁。

## 统一入口

项目根目录执行，以下命令均只读取状态或显示计划：

```bash
python3 scripts/runtime_config.py
bash scripts/run_mapping.sh online
bash scripts/run_mapping.sh online --runtime-config config/profiles/fast_livo2_frontend.yaml
bash scripts/run_mapping.sh offline --runtime-config config/profiles/fast_livo2_frontend.yaml
bash scripts/run_calibration.sh sync
bash scripts/field_session.sh
bash scripts/run_navigation.sh
bash scripts/run_fast_livo2_live.sh --status
bash scripts/record_fast_livo2.sh --seconds 60
```

可修改 runtime 的 mapping_backend，或用 `--runtime-config` / `RANGER_RUNTIME_CONFIG` 选择 profile。profile 的 extends 与相对路径以工程根目录为准；地图通过 map_bundle 检查来源和后端兼容性。

实际执行须显式 `--execute`，online 会打开相应实物，不能从默认 dry-run 推断获准测试。底盘/导航还必须有该次用户授权及 `--control-authorized`，运动许可不跨重启；不自动恢复运动。完整流程见 [受管会话说明](docs/managed_sessions.md)。相机内参、外参和同步的入口/依赖见 [标定说明](calibration/README.md)。

FAST-LIVO2 前端与 B 不能设为先验地图定位后端；C1/C2 使用独立离线入口，未接管生产 runtime。FAST 局部世界系不等于 wheel odom。输入输出的参考点与时钟见 [前端接口](docs/fast_livo2_interfaces.md) 和 [C1 契约](docs/fast_livo2_localizer_interfaces.md)。

三传感器运动采集使用新入口 `record_fast_livo2.sh`，支持完整归档、定时/中断收尾及已有预览接入。`run_fast_livo2_live.sh --visualization rviz|foxglove|both|none` 选择显示，Foxglove经SSH隧道访问本机只读Bridge。命令、采集检查和历史故障记录见[录包指南](docs/fast_livo2_recording.md)、[采集入口报告](docs/fast_livo2_recording_report.md)。

## 公共定位与 TF

```text
Lightning 适配器 / 外部全局定位后端
    → /localization/estimate + /wheel/odom
    → 公共桥接：map→odom→base_link、/navigation_ready、/localization_health
    → Nav2 → 仲裁/平滑/门禁/Collision Monitor → /base/safe_cmd_vel
```

公共桥接用同一机体、同一原采样时刻的位姿计算修正，缓存默认 2 s，禁止外推；前端/参考后端不得同时抢发公共 TF。定位失效撤销就绪。安装外参仍有近似值，实时传感器旧历元尚须处理，未将隔离 FAST 展示接为导航定位。详见 [定位契约](docs/localization_backend_contract.md)。

## 构建与验证

下列构建不打开硬件，默认不带 --execute 的专用脚本只显示计划：

```bash
bash scripts/build_camera_sync.sh --execute
bash scripts/build_calibration.sh --execute
bash scripts/build_fast_livo2.sh --execute
bash scripts/build_fast_livo2_backend.sh --execute
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
cd ws_nav
PYTHONDONTWRITEBYTECODE=1 colcon build --packages-up-to ranger_navigation --executor sequential
```

这些是已有本机依赖下的构建入口，不是全新机器的一键安装程序。相机需要本地 Hik SDK；FAST 前端依赖在独立目录，来源与补丁见 [阶段 A](docs/fast_livo2_stage_a_report.md)。

最新检查及其适用范围统一见 [STATUS](docs/STATUS.md)；旧整合与Nav2报告保留各自验证日期，不能据此宣称本轮全量复跑或实车验收通过。

```bash
PYTHONDONTWRITEBYTECODE=1 python3 tests/test_project_consolidation.py
```

其他 ROS 测试先审查脚本并加载对应 workspace，使用隔离 domain/localhost。事实与日志见 [STATUS](docs/STATUS.md)、[整合报告](docs/consolidation_report.md)、[在线报告](docs/fast_livo2_online_report.md)。历史阶段报告作为追溯证据保留，不将其旧状态当当前操作指令。

后端入口（下列命令均为 dry-run）：

```bash
bash scripts/run_fast_livo2_backend.sh
bash scripts/run_fast_livo2_backend.sh --offline-bridge
```

阶段 B 交付见 [报告](docs/fast_livo2_stage_b_report.md) 和 [后端接口](docs/fast_livo2_backend_interfaces.md)。C1 见 [报告](docs/fast_livo2_stage_c1_report.md) 和 [接口/操作](docs/fast_livo2_localizer_interfaces.md)：`bash scripts/build_fast_livo2_localizer.sh`、`bash scripts/run_fast_livo2_localizer.sh` 默认 dry-run，显式 `--offline-bridge` 才接隔离公共桥接。C2 见 [报告](docs/fast_livo2_stage_c2_report.md) 与 [接口](docs/fast_livo2_relocalizer_interfaces.md)，使用独立 dry-run 入口 `scripts/run_fast_livo2_relocalizer.sh`。不自动进入真机或高程阶段。

项目维护：[ujd66](https://github.com/ujd66) · [Ecust Robotics Lab](https://github.com/Ecust-Robotics-Lab)。
