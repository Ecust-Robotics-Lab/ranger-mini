# Ranger Mini + Livox Mid-360S 导航系统工程约束

## 授权范围与阶段
- 在当前机器上检查和实施工程，不能只提供泛泛的安装建议。
- 严格按用户指定阶段执行；不得自动跳到下一阶段。尚未收到阶段指令时，只做约束落盘与必要的项目初始化。
- 阶段结束更新 `docs/STATUS.md` 和对应阶段报告。

## 项目背景与路线
- 硬件为用户确认的 Ranger Mini V2 底盘与 Livox Mid-360S 雷达（2026-09-15 更正；历史记录中的 Mid-360 名称不作为实物型号依据）。前期讨论以 Ranger Mini 3 为目标，但启动底盘驱动前必须核对实物版本。
- 用户曾通过 `candump can0` 成功收到数据，优先查找并复用已有配置；不能将历史成功当成本机当前验证结果。
- 当前建图和定位明确先使用 `gaoxiang12/lightning-lm`。
- 目标是在有斜坡的巡检场景运行。第一版先完成通信、录包、建图、定位和平地导航，再增加坡道能力；此路线不构成自动进入后续阶段的授权。
- 不假设当前电脑就是其他项目使用的电脑，也不假设当前是原生 Ubuntu。

## 候选技术基线（须经实际检查确认）
- Ubuntu 22.04 + ROS 2 Humble。
- AgileX ranger_ros2 + ugv_sdk。
- Livox-SDK2 + livox_ros_driver2。
- Lightning-LM。
- 后续 Nav2 + Smac Hybrid-A* + MPPI。

官方源码：
- https://github.com/agilexrobotics/ranger_ros2
- https://github.com/agilexrobotics/ugv_sdk
- https://github.com/Livox-SDK/Livox-SDK2
- https://github.com/Livox-SDK/livox_ros_driver2
- https://github.com/gaoxiang12/lightning-lm

## 源码、环境与可复现性
- 优先读取实际源码、launch、参数声明和官方文档，不凭记忆编造接口。
- 所有仓库记录分支、完整 commit SHA 和本地修改，禁止用无法复现的“最新版”描述。游离 HEAD 要明确标记；未获取的仓库不得虚构版本。
- 不删除现有工作空间，不覆盖已有地图和 rosbag，不擅自重装 ROS、系统或 CUDA。
- 不修改无关项目，不把本项目混入已有 Isaac Sim、Conda 或其他 SLAM 工程。
- 安装脚本先审查再运行。网络和系统配置修改前记录原值，并提供回滚方法。
- 需要 sudo 时使用正常授权流程，不索取或记录密码，不绕过权限。

## 真机安全与测试
- 启动可能接管底盘的节点、发送 CAN 控制、发布非零速度或执行导航目标，必须先得到用户对该次真机测试的明确确认。
- 不能因为“没有发布非零 cmd_vel”就认为启动驱动绝对无副作用；启动前必须检查初始化过程是否切换控制模式。
- 自动运行的测试默认 dry-run。任何运动许可在进程重启后失效，必须重新取得该次真机测试确认。
- 真机不在线或缺少人工操作时，完成当前获授权阶段可完成的代码与离线测试，明确标记待验项，不伪造真机通过结果。

## 工程组织
- 项目根目录为 `/home/lab307/ranger_nav`；初始化前检查已有合适工程，优先合理复用。
- 基础目录：`docs/`、`scripts/`、`config/`、`patches/`、`bags/`、`maps/`，以及本文件。
- 按依赖实际需要建立独立工作空间：`ws_livox/`（雷达驱动）、`ws_base/`（底盘驱动）、`ws_slam/`（Lightning-LM）、`ws_nav/`（自有集成包与导航配置）。
- 不一次创建大量没有实现的空包；未到对应阶段不提前创建工作空间或安装依赖。

## 每阶段报告必填
- 本阶段实际完成了什么。
- 执行的关键命令与退出结果。
- 修改的文件。
- 测试证据与日志路径。
- `PASS` / `PARTIAL` / `BLOCKED`，明确其适用范围。
- 下一步需要用户做的最少人工操作。
