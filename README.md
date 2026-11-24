# Ranger Mini 导航系统

Ranger Mini V2 的导航工作台运行在 Ubuntu 22.04 / ROS 2 Humble 工控机上。在线链路使用 Livox Mid-360S 与海康外触发相机，运行 FAST-LIVO2 三融合、C1/C2 先验地图定位、SE(2) 全局规划、EXACT-MPPI 局部控制和独立安全链。

## 导航工作台

```bash
bash scripts/setup_navigation_workbench.sh --execute
bash scripts/run_navigation_workbench.sh --demo --execute
```

演示可查看预建图及历史记录。ROS 模式、远程浏览器访问和实车操作见[工作台说明](workbench/README.md)。当前配置、部署路径与现场问题见[接手指南](docs/HANDOFF.md)和[当前状态](docs/STATUS.md)。接手项目时可复制[接手提示词](docs/NEXT_AGENT_PROMPT.md)。

## 配套工具

| 用途 | 入口 | 操作说明 |
| --- | --- | --- |
| 预建图 | `bash scripts/map_fast_livo2.sh --dry-run` | [在线预建图](docs/fast_livo2_online_mapping.md) |
| 录包 | `bash scripts/record_fast_livo2.sh --dry-run --seconds 60` | [录包指南](docs/fast_livo2_recording.md) |
| 标定 | `bash scripts/run_calibration.sh sync` | [标定目录](calibration/README.md) |
| 独立定位、重定位或回放 | `bash scripts/localize_fast_livo2.sh --help` | [在线定位](docs/fast_livo2_online_localization.md) |

表中的命令只显示用法或计划。录包脚本省略 `--dry-run` 时会启动或复用设备。设备、底盘和实车测试按[工程约束](AGENTS.md)执行。

## 构建与维护

| 模块 | 构建或依赖入口 |
| --- | --- |
| 相机同步和标定 | `scripts/build_camera_sync.sh`、`scripts/build_calibration.sh` |
| FAST-LIVO2 前端和建图 | `scripts/build_fast_livo2.sh`、`scripts/build_fast_livo2_backend.sh` |
| FAST-LIVO2 定位和重定位 | `scripts/build_fast_livo2_localizer.sh`、`scripts/build_fast_livo2_relocalizer.sh` |
| EXACT-MPPI 和导航 | `scripts/setup_exact_mppi.sh`、`scripts/build_exact_navigation.sh` |

以上脚本依赖工控机已有的第三方源码、SDK 或 ROS 环境。安装前缀、来源锁与本地资产见[接手指南](docs/HANDOFF.md)。地图、原包、运行日志和安装资产在工控机维护。

接口、受管会话和历史验收见[文档索引](docs/README.md)。

## Lightning 对比基线

通用配置默认采用 FAST。以下命令显式选用 Lightning 并显示启动计划。

```bash
bash scripts/run_mapping.sh online --runtime-config config/profiles/lightning.yaml
RANGER_RUNTIME_CONFIG=config/profiles/lightning.yaml bash scripts/field_session.sh
```

Lightning 使用独立配置和历史地图包，来源校验与运行要求见[受管会话](docs/managed_sessions.md)。
