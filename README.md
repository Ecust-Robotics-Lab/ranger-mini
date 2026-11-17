# Ranger Mini 导航系统

ROS2 Humble / Ubuntu22.04 / x86_64；实物Ranger Mini V2、Livox Mid-360S、海康USB3相机。更新：2026-11-10。

新上下文先读[HANDOFF](docs/HANDOFF.md)、[STATUS](docs/STATUS.md)、[AGENTS](AGENTS.md)，或复制[接手提示词](docs/NEXT_AGENT_PROMPT.md)。当前状态只在这些入口维护，历史报告保留当时条件。[完整索引](docs/README.md)。

## 当前能力

| 模块 | 当前结果与边界 |
| --- | --- |
| 生产路线 | 定位默认仍Lightning/wheel；导航源码入口已替换SE(2)/EXACT-MPPI，复用FAST公共输入，原安装资产保留 |
| FAST三融合/B建图 | 正式+100ms、重力对齐；390秒原速记录输入3900对、305KF/114接受回环及正式归档 |
| FAST定位 | 自有v2→KISS全6DoF→small_gicp→C1；静止强几何初始化/预处理复用已交付，同包原速FAST输出回放29.820→2.800秒，另包4.220秒，非新实机成绩 |
| wheel/TF | 同录包、工作外参、时间映射和同刻唯一公共TF已实现，有现场分项证据；物理同步/长时仍待验 |
| 当前重点 | 上方办公室—走廊端点掉头—返回的实车准备；窄通道参数已离线验证，门口/制动/模式及真实全程仍待验 |
| 其他限制 | 真实负样本、连续就绪、绝对精度、长时与D未全部验收；Tag默认关闭，无真实VALID表 |

源码/配置/证据已于2026-11-09上传，基线e8245fa，见[Git检查点](docs/git_checkpoint_initialization_upload_20261109.md)。2026-11-10交接文档整理及EXACT-MPPI实施均未提交；原bag、地图、标定、二进制和失败证据保留，见[清理记录](docs/handoff_cleanup_20261110.md)。历史设备状态不代表当前运行状态，不从文档自动开设备或执行控制。

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

原始图片/rosbag、地图、SDK、第三方完整工作空间、构建与.runtime按既有忽略规则留在本机。Git保存源码、配置、补丁、报告及可版本化证据，不能代替完整本机资产备份；组件来源/版本与许可证见交接指南及版本锁。

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
bash scripts/record_fast_livo2.sh --dry-run --seconds 60
bash scripts/map_fast_livo2.sh --dry-run
```

可修改 runtime 的 mapping_backend，或用 `--runtime-config` / `RANGER_RUNTIME_CONFIG` 选择 profile。profile 的 extends 与相对路径以工程根目录为准；地图通过 map_bundle 检查来源和后端兼容性。

上述其他入口实际执行须显式 `--execute`，online会打开相应实物，不能从默认dry-run推断获准测试。专用 `record_fast_livo2.sh` 按用户指令默认录制并自行启动传感器和只接收wheel反馈，默认使用FAST profile；查看计划须加 `--dry-run`。会接管控制的底盘/导航入口仍必须有该次用户授权及 `--control-authorized`，录包receive-only分支不接管控制；运动许可不跨重启，不自动恢复运动。完整流程见 [受管会话说明](docs/managed_sessions.md)。相机内参、外参和同步的入口/依赖见 [标定说明](calibration/README.md)。

FAST-LIVO2 前端与 B 不能设为先验地图定位后端；C1/C2 使用独立离线入口，未接管生产 runtime。FAST 局部世界系不等于 wheel odom。输入输出的参考点与时钟见 [前端接口](docs/fast_livo2_interfaces.md) 和 [C1 契约](docs/fast_livo2_localizer_interfaces.md)。

三传感器运动采集使用 `record_fast_livo2.sh`，默认执行、独立启动传感器并读取 `config/profiles/fast_livo2_frontend.yaml`；现默认同时录wheel，已有发布者则复用，否则启用CAN并自启不接管遥控器的receive-only驱动，`--no-wheel`关闭；`--attach`才接入已有预览，支持完整归档和定时/中断收尾。`run_fast_livo2_live.sh --visualization rviz|foxglove|both|none`选择显示，Foxglove经SSH隧道访问本机只读Bridge。命令和验证见[录包指南](docs/fast_livo2_recording.md)、[自动wheel报告](docs/fast_livo2_recording_wheel_report.md)。

正式FAST在线预建图使用 `map_fast_livo2.sh --execute`，在一个会话中运行三融合、B回环、优化地图显示、原始/去畸变数据录制及停止封存。`run_mapping.sh online` 选择FAST profile时也转到该入口；Lightning默认不变。单独录包入口现也保存去畸变云，但不启动回环后端。新入口默认dry-run，详细命令、时间契约与地图目录见[在线预建图](docs/fast_livo2_online_mapping.md)。

FAST在线先验地图定位/重定位使用 `localize_fast_livo2.sh`，默认dry-run，支持auto/manual与无设备原速重放；软件/离线首版及新282秒独立访问功能结果见[报告](docs/fast_livo2_online_localization_report.md)，命令见[指南](docs/fast_livo2_online_localization.md)。它使用同一C1/C2与独立安装，不接管生产Lightning或公共TF。

## 公共定位与 TF

```text
Lightning 适配器 / 外部全局定位后端
    → /localization/estimate + /wheel/odom
    → 公共桥接：map→odom→base_link、/navigation_ready、/localization_health
    → SE(2)全局路径/EXACT-MPPI → 仲裁/门禁/独立末端防撞 → /base/safe_cmd_vel
```

公共桥接用同一机体、同一原采样时刻的位姿计算修正，缓存默认 2 s，禁止外推；前端/参考后端不得同时抢发公共 TF。定位失效撤销就绪。工作外参属于运动估计；FAST导航使用既有会话时间映射与原采样wheel插值。详见 [定位契约](docs/localization_backend_contract.md)。

## 构建与验证

下列构建不打开硬件，默认不带 --execute 的专用脚本只显示计划：

```bash
bash scripts/build_camera_sync.sh --execute
bash scripts/build_calibration.sh --execute
bash scripts/build_fast_livo2.sh --execute
bash scripts/build_fast_livo2_backend.sh --execute
bash scripts/setup_exact_mppi.sh --execute
bash scripts/build_exact_navigation.sh --execute
```

这些是已有本机依赖下的构建入口，不是全新机器的一键安装程序。相机需要本地 Hik SDK；FAST 前端依赖在独立目录，来源与补丁见 [阶段 A](docs/fast_livo2_stage_a_report.md)。

新导航见[计划](docs/EXACT_MPPI_NAVIGATION_PLAN.md)、[报告](docs/exact_mppi_implementation_report_20261110.md)、[接口](docs/exact_mppi_interfaces.md)。最新检查及其适用范围统一见 [STATUS](docs/STATUS.md)；旧整合与Nav2报告保留各自验证日期，不能据此宣称本轮全量复跑或实车验收通过。

当前[实车准备](docs/EXACT_MPPI_FIELD_TEST.md)含可编辑路线图、standard/narrow参数、43话题录制与现场分项顺序；真实地图门内外两段仍未搜索通过，准备许可不构成设备或运动执行许可。

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

在线重定位及检索对照的本地保存见[Git检查点](docs/git_checkpoint_online_localization_20261106.md)；后续建议见[定位接入导航计划](docs/FAST_NAVIGATION_INTEGRATION_PLAN.md)。
