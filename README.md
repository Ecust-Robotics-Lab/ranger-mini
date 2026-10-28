# Ranger Mini 导航系统

ROS 2 Humble / Ubuntu 22.04 / x86_64，实物为 **Ranger Mini V2 + Livox Mid-360S + 海康相机**。Lightning-LM 保持生产默认；FAST-LIVO2 三传感器前端、独立回环后端 B 和给定初值的 C1 离线先验定位已实现。C2 无初值重定位未实施，实车精度和实时接车仍待验。

**新上下文从 [交接指南](docs/HANDOFF.md) 开始**，再读 [当前状态](docs/STATUS.md)、[工程约束](AGENTS.md)、[后端与重定位计划](docs/FAST_LIVO2_BACKEND_PLAN.md)。全部文档见 [docs 索引](docs/README.md)。

## 当前能力

| 模块 | 已有结果与边界 |
| --- | --- |
| 生产导航路线 | Lightning、轮速、公共定位桥接、Nav2、安全速度链已有实现；完整真机导航仍 PARTIAL |
| 定位契约 | LocalizationEstimate 携带地图系车体位姿、原采样时刻和健康；按定位时刻插值轮速，唯一管理公共 TF |
| FAST-LIVO2 | 保留 LiDAR–IMU–Camera 更新；静态基线与在线 RViz 通过，用户要求停止后会话保持停止 |
| 配置切换 | mapping_backend 选择 Lightning / FAST-LIVO2 前端；localization_backend 独立选择 lightning / standard |
| 标定与同步 | 已整合到本工程；v2 共享戳保护已修复，物理同步、几何标定与实时历元仍待验 |
| 后续目标 | 独立在线回环/PGO → 优化地图包 → 给定初值定位 → 无初值自动重定位；高程/坡道另行分阶段 |

目前保留一份约136.6秒三传感器运动采集包，已生成[建图演示视频](docs/fast_livo2_demo_20261028.md)；画面与合成测试不代表真值精度、真实回环或自动定位验收。standard 是外部标准消息接口，不是另一套现成定位算法。

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

本地原始图片/rosbag、SDK、第三方工作空间、构建与 .runtime 不进入主仓库。相机只纳入两个维护包源码。当前阶段 A/整合成果尚有大量未提交文件，克隆远端不能代替当前工作树；第三方完整 SHA/补丁来源见交接指南与版本锁。

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

FAST-LIVO2 前端与 B 不能设为先验地图定位后端；C1 使用独立离线入口和私有初值接口，未接管生产 runtime。FAST 局部世界系不等于 wheel odom。输入输出的参考点与时钟见 [前端接口](docs/fast_livo2_interfaces.md) 和 [C1 契约](docs/fast_livo2_localizer_interfaces.md)。

三传感器运动采集使用新入口 `record_fast_livo2.sh`，支持完整归档、定时/中断收尾及已有预览接入。`run_fast_livo2_live.sh --visualization rviz|foxglove|both|none` 选择显示，Foxglove经SSH隧道访问本机只读Bridge。命令与当前相机外触发阻塞见[录包指南](docs/fast_livo2_recording.md)、[采集入口报告](docs/fast_livo2_recording_report.md)。

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

2026-10-26 最近整合验证：配置/迁移 10/10、标定接口 11/11、时间对齐 7/7、进程管理 11/11、定位契约 11/11、前端 C++ 13/13；静态短回放 20 对且视觉参与。完整静态基线 829 对，在线约 10/200/10 Hz。Nav2 安全链 37/37 等完整回归仍保留 2026-10-20 日期，本轮未全量重跑。

```bash
PYTHONDONTWRITEBYTECODE=1 python3 tests/test_project_consolidation.py
```

其他 ROS 测试先审查脚本并加载对应 workspace，使用隔离 domain/localhost。事实与日志见 [STATUS](docs/STATUS.md)、[整合报告](docs/consolidation_report.md)、[在线报告](docs/fast_livo2_online_report.md)。历史阶段报告作为追溯证据保留，不将其旧状态当当前操作指令。

后端入口（下列命令均为 dry-run）：

```bash
bash scripts/run_fast_livo2_backend.sh
bash scripts/run_fast_livo2_backend.sh --offline-bridge
```

阶段 B 交付见 [报告](docs/fast_livo2_stage_b_report.md) 和 [后端接口](docs/fast_livo2_backend_interfaces.md)。C1 见 [报告](docs/fast_livo2_stage_c1_report.md) 和 [接口/操作](docs/fast_livo2_localizer_interfaces.md)：`bash scripts/build_fast_livo2_localizer.sh`、`bash scripts/run_fast_livo2_localizer.sh` 默认 dry-run，显式 `--offline-bridge` 才接隔离公共桥接。不自动进入 C2、真机或高程阶段。

项目维护：[ujd66](https://github.com/ujd66) · [Ecust Robotics Lab](https://github.com/Ecust-Robotics-Lab)。
