# 项目接手指南

工作区 `/home/lab307/ranger_nav`，分支 `refactor/pluggable-navigation`。先读 [工程约束](../AGENTS.md) 和 [当前状态](STATUS.md)。新对话可直接复制 [接手提示词](NEXT_AGENT_PROMPT.md)。

## 入口与资产

| 用途 | 路径 |
| --- | --- |
| 导航工作台与操作说明 | [workbench/README.md](../workbench/README.md) |
| 导航实际运行参数 | [exact_navigation.yaml](../ws_nav/src/ranger_navigation/config/exact_navigation.yaml) |
| 车体与工作外参 | [navigation_prep.yaml](../config/fast_livo2/navigation_prep.yaml) |
| FAST 前端及来源锁 | `ws_fast_livo/latency_frontend_install`，[active_frontend.lock.json](../config/fast_livo2/active_frontend.lock.json) |
| B 建图后端和 C1/C2 定位 | `ws_fast_livo/online_install`、`ws_fast_livo/online_localization_install` |
| 相机、底盘、导航及公共桥接安装 | `ws_camera/latency_install`、`ws_base/install_exact`、`ws_nav/install_exact` |
| 项目内 Python 环境 | `.runtime/exact_mppi_venv`、`.runtime/navigation_workbench_venv` |
| 工作台默认地图 | `outputs/fast-livo-demo-changed-scene-20261110/navigation_map_05`，UUID `160f8572a77a61856cf87e43514be769` |
| 同源定位档案 | `outputs/fast-livo-demo-changed-scene-20261110/mapping_replay_01/map` |
| 运行记录 | `.runtime/sessions/`、`.runtime/exact_navigation/`、`.runtime/navigation_workbench/records/` |

正式启动时从项目 YAML 读取参数。修改后需新建导航会话，运行中不热改。工作台的“重置规划控制”仅重建计算子进程并清理任务状态。工作台默认加载低延迟 FAST 前端，旧地图通过绑定 UUID、manifest 和新旧来源的 `frontend_timing_upgrade` 声明校验。

底盘的 `ranger_ros2` 和 `ugv_sdk` 是嵌套仓库。用 [EXACT-MPPI 来源锁](../config/exact_mppi/sources.lock.json) 指定的 HEAD、前置补丁与增量补丁重建。本地原包、地图、标定、安装目录、运行数据库、密钥及备份由工控机保存。

## 接手时先核对

1. 用 `git status -sb` 与 `git log -5 --oneline` 检查分支与工作树，核对项目 YAML 和实际 launch 参数。
2. 检查 `.runtime/sessions/` 的当前指针与状态。用 boot ID、PID 和启动计时核对 owner 与 guardian，确认进程归属。
3. 读取本次导航 worker 的 `status.json` 和工作台任务事件。对照 `/navigation/state`、`/navigation/execution`、`/base/execution_state` 和 wheel，分别确认控制候选、门禁放行、SDK 提交与实际运动。
4. 软件检查使用隔离 ROS domain 和 localhost。设备及实车操作按本次授权执行。

## 仍需现场核实的问题

- **整段回返**。以最近工作台记录 `live-41843270f2fb45fc81a3c41443de148f` 为参考。已有走廊方向的单点成功，下一步需按授权逐项完成门口、停转与完整回返。路线及验收分段见 [实车计划](EXACT_MPPI_FIELD_TEST.md)。
- **相机无首帧**。USB3 打开、Line0 抓图成功后若持续 `MV_E_NODATA`，检查外部同步源供电、雷达 PPS 输入和相机触发线。对照雷达 `time_type`、SDK 首帧、同步图像计数。具体线路故障尚未实测确认。 [相机排查](workbench_camera_startup_report_20261118.md)
- **近零速度请求未执行**。先算行驶段的 `abs(vx / wz)`。现有 V2 驱动模型的最小半径约 0.4764 m，源码尚无自动抬升半径的保护。若 SDK 已提交非零且轮速仍长期近零，再分模式测量起步死区。已知软件拒绝样本曾由超限转角触发，通用低速死区尚未标定。
- **连续定位与制动**。用现场原采样戳和日志复核短暂定位失效、恢复、较高速度下停车。估计位姿和模拟模型不足以给出绝对精度或真实制动距离。 [控制恢复](exact_mppi_control_recovery_20261117.md)

按模块查询 [导航接口](exact_mppi_interfaces.md)、[前端接口](fast_livo2_interfaces.md)、[B 后端](fast_livo2_backend_interfaces.md)、[C1](fast_livo2_localizer_interfaces.md)、[C2](fast_livo2_relocalizer_interfaces.md) 或 [文档索引](README.md)。
