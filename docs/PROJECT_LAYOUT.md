# 当前工程结构

更新：2026-11-05。FAST-LIVO2 B/C1/C2与Tag增量已有软件/离线成果并已分组本地提交，真实Tag与D仍未验。FAST专用会话未运行，field及旁边工作区另有活动进程；最新会话、构建前缀、+100ms配置、原包/地图和清理状态见[HANDOFF](HANDOFF.md)。历史整合见[整合报告](consolidation_report.md)，定位入口见[C1](fast_livo2_localizer_interfaces.md)、[C2](fast_livo2_relocalizer_interfaces.md)。

新上下文先读 [HANDOFF](HANDOFF.md)，下一步见 [后端/重定位计划](FAST_LIVO2_BACKEND_PLAN.md)，当前与历史文档分类见 [文档索引](README.md)。

## 当前目录

```text
/home/lab307/ranger_nav/
├── AGENTS.md                    阶段、硬件与安全约束
├── config/
│   ├── runtime.yaml             统一默认；Lightning 仍为生产默认
│   ├── profiles/                Lightning / FAST-LIVO2 前端选择
│   ├── maps/                    地图、标定与源包关联
│   ├── fast_livo2/              本机参数、来源快照、版本锁、RViz
│   └── calibration_sources.json 标定/同步源码来源与哈希
├── calibration/                 从原 ~/calibration 迁入的正式目录
│   ├── README.md、workflow.py    流程与统一分派
│   ├── camera_intrinsics/       内参工具、原始图片与结果
│   ├── lidar_camera_extrinsics/  外参工具、配置、场景与结果
│   ├── bags/                    原始三传感器静态同步包
│   └── ws_calib/                旧同步工作空间，保留追溯
├── scripts/                     构建、运行、采集、会话管理
├── tests/                       配置/定位契约、时序、进程、安全验证
├── ws_camera/                   当前相机与同步构建部署
│   └── src/
│       ├── hik_camera_trigger_driver/ 新维护入口、本机 camera_info
│       ├── timeshare_writer/          v2 一致性协议
│       └── hik_camera_ros2_driver/     原驱动/SDK及用户修改，保留
├── ws_livox/                    Livox 驱动与 SDK
├── ws_base/                     Ranger V2 驱动与 ugv_sdk
├── ws_slam/                     Lightning-LM 生产路线
├── ws_fast_livo/                FAST 前端部署及 src/ 下 B backend、C1/C2 localizer 自有源码
├── ws_nav/src/                  定位消息、公共桥接、导航与安全
├── vendor/                      固定版本第三方与审查源码
├── patches/                     第三方实际修改
├── bags/                        既有运动包、前端派生结果
├── maps/                        地图、来源与验收
├── docs/                        操作/阶段报告、证据
└── .runtime/                    受管会话、台账与回退备份

/home/lab307/calibration -> /home/lab307/ranger_nav/calibration
```

兼容链接为历史绝对路径与旧构建保留。新入口不依赖它，临时移除链接的八项检查均退出 0。

## 入口职责

| 任务 | 入口 | 内容 |
| --- | --- | --- |
| 建图/前端 | `bash scripts/run_mapping.sh online`（或 offline） | 按 mapping_backend 分派既有 Lightning 或 FAST-LIVO2 入口 |
| 标定/同步 | `bash scripts/run_calibration.sh ACTION` | camera、capture、intrinsics、sync、extrinsics、solve；默认显示计划 |
| 现场定位与导航 | field_session.sh、run_navigation.sh | 既有定位契约、公共 TF 与安全链 |
| 算法专用入口 | run_fast_livo2_live/replay.sh、run_slam_online/offline.sh | 保留环境装载、专用参数与状态控制，不是重复算法 |
| 三传感器录包 | record_fast_livo2.sh | 默认dry-run；自动启动或接入预览，录包/配置/同步CSV归档与录后审查，见[指南](fast_livo2_recording.md) |

`mapping_backend` 与 `localization_backend` 分别选择前端/建图和全局定位。FAST 前端与 B 的连续局部里程计/回环建图不能作为先验地图定位后端；C1/C2 使用独立离线入口，不接管生产配置。配置解析器继续拒绝将局部前端冒充全局定位。`standard` 是既有外部标准定位输入接口。

## 资产保留边界

- 原始录包、正式基线、地图、失败证据、源码与回退备份保留；历史删除清单和迁移完整性记录见整合报告，不将旧磁盘容量当作当前容量。
- 2026-11-05按用户当次明确指令删除的3份过时试录、3空目录和6处缓存是本次例外，具体名单/哈希见[清理报告](handoff_cleanup_20261105.md)；未删除当前+100ms对照所需原包、任何地图或唯一失败证据，不继承为后续任意删除许可。
- vendor 中前端、B和可选C2的实际依赖按各自sources.lock固定；审查仓库也保留，不将“参考过”当作已集成，也不将不同提交误认为重复目录。
- 不整体清空 build/install：Lightning 从 `ws_slam/build/lightning/bin/` 运行，部分旧相机部署链接指向 build。新增标定构建使用 integrated 目录。

标定流程和同步源码已进入主仓库可跟踪范围，原始 bag/图片、SDK、第三方工作空间和编译产物继续由 Git 忽略。既有整合/标定源码与用户雷达证据已有本地提交；当前提交、工作树和本机资产状态统一见HANDOFF与实际Git。
