# 阶段 2：Ranger Mini V2 底盘通信、驱动和控制语义

2026-09-14，Asia/Shanghai。**阶段 PARTIAL；CAN 通信及已测 ROS 接口 PASS；停车安全未通过，自主导航不放行。**

> 最新更新：22:11、22:16 各获当次授权后执行 0.05 m/s、约 0.8 s 前进和主动发零，用户确认最后一轮实物运动。两轮驱动退出后均出现负向速度反馈，原因未明。详见 [02_ros_motion_test.md](02_ros_motion_test.md)。驱动已退出；以下 21:56 结果和源码审查保留为历史记录，已修改文件链接指向修改前快照。两个仓库现均有本地补丁。

> 22:32 后续结果：最新日志已完成约 5 秒的 0.05 m/s 指令，用户报告停车未观察到后退；退出后协议反馈仍有负向瞬态，实物与反馈关系未明。详见 ROS 测试报告最新章节，停车安全结论仍未通过。

## 2026-09-14 21:56 最新结果：CAN 反馈接收 PASS

用户提供启用后的 can0 状态，本助手随即在当前机器执行 `python3 scripts/base_readonly_diagnostics.py --execute --seconds 10`，退出 **0**。该脚本内四次 `ip` 查询均退出 0，`timeout … candump` 退出 **124**（正常到达 10 秒上限）；PASS 的依据是下述帧内容和统计，不是 124 本身。

- 前后均 UP/ERROR-ACTIVE，bitrate=500000、sample-point=0.875、restart-ms=100，映射仍为 gs_usb / USB 1-11:1.0。没有重新配置接口。
- 保存 **8,866 帧、25 个 ID，全部 RX**；没有错误标志帧或无法解析的输出行。0x211/0x221/0x241/0x271/0x281/0x291 等反馈持续出现；主要帧按“数量÷10 s”约 47.6–47.7 Hz，0x261–0x268 各约 9.5–9.6 Hz。这是平均值，原捕获没有逐帧时间戳，尚未验收抖动或最坏延迟。
- 网卡 RX packets 从 68,005 增至 76,890（+8,885），TX packets 始终为 0；RX/TX errors、dropped 及 CAN bus_error/error_warning/error_passive/bus_off/restarts 等计数无增长。计数窗口略大于 candump 窗口，19 帧差异不能直接当作丢包；本次没有单独测量接收 socket 的丢包统计。
- 0x211 原始样本 `00 00 01 F4 00 00 00 00` / `…01 F5…`：按 [agilex_msg_parser_v2.c:64](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/src/protocol_v2/agilex_msg_parser_v2.c:64) 和 [SystemState.msg:1](/home/lab307/ranger_nav/ws_base/src/ranger_ros2/ranger_msgs/msg/SystemState.msg:1) 解码，vehicle_state=0、control_mode=0（RC）、error_code=0、电压 50.0–50.1 V。
- 全部 476 条 0x221 的速度/转角字段为零，0x271/0x281 电机角度/速度反馈也全零；0x291 的 477 条均 `00 00`（模式 0、mode_changing=0）。解码依据：[agilex_msg_parser_v2.c:79](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/src/protocol_v2/agilex_msg_parser_v2.c:79)、[agilex_msg_parser_v2.c:193](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/src/protocol_v2/agilex_msg_parser_v2.c:193)、[ranger_interface.hpp:53](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/include/ugv_sdk/details/interface/ranger_interface.hpp:53)。这表示反馈数据静止，不等于人工观察车体停止或停车机制通过。
- 捕获到 0x312，但固定 SDK 没有对应解析分支，本轮只统计、不赋予语义；没有捕获到 0x361/0x362，BMS 话题数据仍未验证。

**结论范围：底盘 CAN 反馈接收通信通过；没有验收上位机发送控制、ROS 状态话题、遥控接管或运动/停车安全。** 本次未启动 ROS 底盘节点，未发送 CAN 控制或速度。当前生效 bitrate 已确认，不再以“缺少旧配置”阻塞只读接收；重启后配置是否恢复仍未验证。

证据：[commands.json](02_evidence/can-zlje17cv/commands.json)、[candump.log](02_evidence/can-zlje17cv/candump.log)、[before-json.log](02_evidence/can-zlje17cv/before-json.log)、[after-json.log](02_evidence/can-zlje17cv/after-json.log)、[analysis.json](02_evidence/can-zlje17cv/analysis.json)。离线分析脚本 [analyze.py](02_evidence/can-zlje17cv/analyze.py) 只读取日志，执行退出 0；实际诊断调用当前系统已有的 `/usr/bin/candump`（can-utils 2020.11.0-1），版本/hash 记录在 [analysis_execution.log](02_evidence/can-zlje17cv/analysis_execution.log)，本次没有安装软件。

本次新增上述唯一证据目录，更新本报告、STATUS.md、02_base_manual_tests.md；没有系统/CAN 配置变更，因此没有需要执行的配置回滚。下一步处理已审查的 SDK/驱动风险，再按当次许可安排 ROS 验收和真机测试。

## 21:54 前的实际完成与证据边界（历史记录）

- 用户在本阶段明确确认实物是 **Ranger Mini V2**，替代此前讨论中的 Mini V3 假设。这是用户提供信息，尚无本轮铭牌、完整 SN 或固件版本读取结果。
- 本机实测 USB `Bus 001 Device 005: ID 1d50:606f`，对应 `gs_usb` / `can0`；USB 识别 PASS。适配器序列号不是底盘 SN。证据：[usb_can_confirmation.log](02_evidence/usb_can_confirmation.log)。
- `can0` 为 DOWN/STOPPED，未显示已配置 bitrate，RX/TX 均为 0。限时 candump 返回 1、`read: Network is down`。**没有收到底盘反馈，不得称“底盘已通”。** 零错误计数同样不能证明通信健康。证据：[host_can_initial.log](02_evidence/host_can_initial.log)、[本轮诊断记录](02_evidence/can-mdxf3ols/commands.json)。
- 在当前 shell 历史及 systemd、udev、modules、netplan、已有相关目录中未找到旧成功 CAN 配置；搜索退出 1 表示本次范围无匹配，不能断言其他机器/历史终端没有配置。见 [source_and_can_search.log](02_evidence/source_and_can_search.log)。
- 官方 [bringup_can2usb.bash:4](/home/lab307/ranger_nav/ws_base/src/ranger_ros2/ranger_bringup/scripts/bringup_can2usb.bash:4) 使用 500000 bit/s。这是官方配置候选，不是找回的历史成功配置；本轮没有运行该脚本或盲改 bitrate。
- 没有配置或启用 CAN，没有启动底盘节点/SDK demo，没有发送控制帧、速度或导航目标。只运行了接收诊断、编译及无真机的检查。
- 实施项目启动配置、默认 dry-run 的诊断/构建脚本、V2 型号分支最小补丁和离线回归；准备了 [人工测试步骤](02_base_manual_tests.md)。

## 源码固定与兼容性

| 仓库 | 官方来源与选定分支 | 完整 commit SHA | 本地状态 |
|---|---|---|---|
| ranger_ros2 | https://github.com/agilexrobotics/ranger_ros2，明确检查并选择 humble | `b6ea21a275ca5e7168130cc6470e61474681d679` | 固定为 detached HEAD；仅 V2 分支补丁 |
| ugv_sdk | https://github.com/agilexrobotics/ugv_sdk，main | `f2704eacdc90357078cd93ec60aae08bb4baab35` | 固定为 detached HEAD；源码无修改 |

远端分支查询见 source_and_can_search.log；没有把默认分支名称当成 ROS 适配证明。官方 Humble 包依赖 `ugv_sdk`；本对版本在接口层均存在 `kRangerMiniV2` 与 `RangerMiniV2Base`（[ranger_messenger.cpp:29](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:29)、[ranger_robot.cpp:13](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/src/mobile_robot/ranger_robot.cpp:13)）。兼容性结论仅覆盖下述本机编译/链接与源码接口，不代表厂商认证的固件组合。

**构建 PASS（仅编译/链接）**：本机 Ubuntu 22.04.5 / ROS 2 Humble，G++ 11.4.0、CMake 3.22.1；执行 `bash scripts/build_base.sh --execute`，4 个包完成（18.9 s），`COLCON_EXIT=0`。完整日志：[build-0iEF8pvo.log](02_evidence/build-0iEF8pvo.log)，逐包命令及 stdout/stderr 在 `ws_base/log/`。环境包版本见 [build_environment.log](02_evidence/build_environment.log)。`BUILD_TESTING=OFF`，没有运行上游真机 demo 或测试，不能宣称上游测试套件通过。

SDK package.xml 仍带 catkin 元数据（[package.xml:17](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/package.xml:17)），本次 CMake 实际走独立构建分支（[CMakeLists.txt:16](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/CMakeLists.txt:16)），未安装 ROS 1/catkin。其 package.xml 版本为 0.1.6，而 CMake project 为 0.8.0；本项目用完整 SHA 固定，不依赖这些不一致的版本号。

构建保留警告：ranger_base 中有未使用变量/参数和 `steer_cmd may be used uninitialized` 编译器提示；SDK 的未运行 tracer demo 有空指针调用警告，另有未使用 CMake 参数提示。没有把“退出 0”描述为“无警告”或安全通过。安装后验证包可发现、消息定义与 Python/C 类型支持能加载、驱动 ELF 的共享库无 `not found`，均未启动节点；见 [postbuild_interfaces.log](02_evidence/postbuild_interfaces.log)。

固定导入清单：[base_sources.repos](../config/base_sources.repos)；实际 origin、分支/游离 HEAD、完整 SHA、本地 diff、补丁及二进制 SHA256：[source_versions.json](02_evidence/source_versions.json) 和 [source_versions.log](02_evidence/source_versions.log)。SDK 下载期间按同一 SHA 预审的 9 个文件已与最终本地 Git 工作树逐字节比对一致，见 [sdk_review_identity.log](02_evidence/sdk_review_identity.log)。

依赖采用当前 APT 索引中的固定 Ubuntu 包：`can-utils 2020.11.0-1 amd64`、`libasio-dev 1:1.18.1-1 all`。下载至 vendor/base_deps/downloads，以 `dpkg-deb -x` 解包至 vendor/base_deps/root，未执行维护脚本或系统安装；包元数据和 SHA256 见 [dependencies.log](02_evidence/dependencies.log)。Asio 仅通过本次构建的 CPLUS_INCLUDE_PATH 引入，未改 bashrc/ROS/其他工程。SDK 本身会编译 sample，但不运行。

## 实物型号、参数和 launch

- V2 的真实参数为 `robot_model: ranger_mini_v2`；轮距 0.364 m、轴距 0.494 m、原地旋转最大角速度 4.8 rad/s、转弯切换半径阈值 0.4764 m，来源 [ranger_params.hpp:49](/home/lab307/ranger_nav/ws_base/src/ranger_ros2/ranger_base/include/ranger_base/ranger_params.hpp:49)（49–65 行）。这些是源码常量，尚未量测本车，且不是现场测试安全限值。
- 上游 V2 launch 的确存在，[ranger_mini_v2.launch.py:31](/home/lab307/ranger_nav/ws_base/src/ranger_ros2/ranger_bringup/launch/ranger_mini_v2.launch.py:31) 声明 V2，但 [ranger_mini_v2.launch.py:38](/home/lab307/ranger_nav/ws_base/src/ranger_ros2/ranger_bringup/launch/ranger_mini_v2.launch.py:38)（38–46 行）没有把 `robot_model` 和 `update_rate` 传入节点；节点默认型号为 `ranger`（[ranger_messenger.cpp:64](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:64)）。两份 V2 launch 分别位于 ranger_bringup/launch 和 ranger_base/launch，均有此遗漏。**不能直接用未修的官方 V2 launch 验证本车。**
- 另一个独立问题是上游参数加载的 V2 `if` 后接独立 V3 `if/else`，导致 V2 先加载后被覆盖为 Ranger。已将 [ranger_messenger.cpp:116](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:116) 改为 `else if`，仅一行修改；补丁：[ranger_ros2_humble_v2_model.patch](../patches/ranger_ros2_humble_v2_model.patch)。未更改任何型号的几何常量。上游版本可用 `git show HEAD:ranger_base/src/ranger_messenger.cpp` 对照。
- [model_selection_test.log](02_evidence/model_selection_test.log) 对实际 LoadParameters 方法使用参数读取 shim 编译运行，复现原始 V2 错选，并验证补丁后四种型号均正确。没有创建 ROS 底盘对象或打开 CAN；此检查不替代整包构建/真机测试。
- SDK 的 V2 继承 RangerBase，但单独处理 BMS 电压比例（[ranger_base.hpp:104](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/include/ugv_sdk/details/robot_base/ranger_base.hpp:104)，104–127 行）；不是把 V1/V3 参数当成 V2。当前实物按用户确认的 V2 准备，型号变更时须重新检查。

## 初始化和控制权

1. 节点入口构造 messenger 并进入 Run（[ranger_base_node.cpp:33](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_base_node.cpp:33)）。构造函数在 Connect 成功后立即调用 `EnableCommandedMode()`，随后才创建话题（[ranger_messenger.cpp:39](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:39)，39–50 行）。
2. SDK 将该调用编码为 `CONTROL_MODE_CAN` 并发送（[agilex_base.hpp:77](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ugv_sdk/include/ugv_sdk/details/robot_base/agilex_base.hpp:77)，77–89 行；[agilex_msg_parser_v2.c:620](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/src/protocol_v2/agilex_msg_parser_v2.c:620)）；CAN 配置 ID 为 0x421（[agilex_protocol_v2.h:105](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/src/protocol_v2/agilex_protocol_v2.h:105)）。**即使没有任何 cmd_vel，启动也可能切换控制权。**
3. Connect 只是 SocketCAN 打开/绑定及启动接收线程，既没有底盘握手、型号验证，也没有确认收到反馈（[agilex_base.hpp:218](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ugv_sdk/include/ugv_sdk/details/robot_base/agilex_base.hpp:218)；[async_can.cpp:27](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ugv_sdk/src/async_port/async_can.cpp:27)，27–81 行）。不能把 Connect 返回成功当成底盘通信 PASS。
4. ROS 报告 control_mode：RC=0、CAN=1，以及 vehicle_state 正常/急停/异常（[SystemState.msg:1](/home/lab307/ranger_nav/ws_base/src/ranger_ros2/ranger_msgs/msg/SystemState.msg:1)）。SDK 缓存 RC 状态（[agilex_base.hpp:315](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ugv_sdk/include/ugv_sdk/details/robot_base/agilex_base.hpp:315)），但此 ROS 节点没有 RC 原始话题、没有 RC/CAN 仲裁状态机，也不根据 control_mode 拦截速度回调（[ranger_messenger.cpp:155](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:155)、[ranger_messenger.cpp:391](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:391)）。
5. **遥控与上位机优先级、开关位置、遥控接管后的恢复规则及急停电气效果未知**：所审查主机源码没有实现这些固件/硬件行为。必须用本车手册、固件信息及人工静止/低速测试确认，不能从枚举值推导“遥控一定优先”。
6. `simulated_robot` 仅在官方 launch 中出现，实际构造连接路径和参数加载没有仿真分支（[ranger_messenger.cpp:23](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:23)、[ranger_messenger.cpp:62](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:62)）。不得用 `simulated_robot:=true` 当作离线安全开关。

## ROS 状态接口与数据局限

后续 candump 应区分反馈和命令：协议定义 0x211 系统状态、0x221 运动状态、0x241 RC、0x251–0x258/0x261–0x268 执行器状态、0x271 电机角度、0x281 电机速度、0x291 运动模式、0x361 BMS；见 [agilex_protocol_v2.h:47](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/src/protocol_v2/agilex_protocol_v2.h:47)（47–99 行）。不是要求每种固件必须发布全部 ID，而是以本车协议交叉核对。21:54 前没有可统计的反馈样本；21:56 的接收计数和解码结果见本报告最新章节；接收脚本启用错误帧显示并保存前后统计，不发送查询帧。

话题声明见 [ranger_messenger.cpp:155](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:155)（155–171 行），真实输入是 **geometry_msgs/msg/Twist**，不是 TwistStamped。

| 接口 | 类型/用途 | 源码与限制 |
|---|---|---|
| /system_state | ranger_msgs/msg/SystemState；车辆状态、控制权、故障码、电压、运动模式 | [SystemState.msg:15](/home/lab307/ranger_nav/ws_base/src/ranger_ros2/ranger_msgs/msg/SystemState.msg:15)；[ranger_messenger.cpp:196](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:196)。反馈枚举不等于功能已验收 |
| /motion_state | ranger_msgs/msg/MotionState | [MotionState.msg:1](/home/lab307/ranger_nav/ws_base/src/ranger_ros2/ranger_msgs/msg/MotionState.msg:1)；[ranger_messenger.cpp:209](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:209)。只有 header 与 motion_mode，不含 vx/vy/wz |
| /actuator_state | ranger_msgs/msg/ActuatorStateArray | [ranger_messenger.cpp:233](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:233)。循环生成 8 项却始终解引用首个 hs/ls 元素，并重复 angle_5、speed_1；id 不同不代表数据正确 |
| /battery_state | sensor_msgs/msg/BatteryState | [ranger_messenger.cpp:264](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:264)；[ranger_base.hpp:104](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/include/ugv_sdk/details/robot_base/ranger_base.hpp:104)。V2 电压有单独 SDK 比例；ROS percentage 直接复制原始 SOC，而本机 [BatteryState.msg:41](/opt/ros/humble/share/sensor_msgs/msg/BatteryState.msg:41) 要求 0–1；当前未做归一化（[agilex_msg_parser_v2.c:260](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/src/protocol_v2/agilex_msg_parser_v2.c:260)），仍待修正/核对 |
| /wheel/odom | nav_msgs/msg/Odometry；底盘反馈积分里程计 | [ranger_messenger.cpp:184](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:184)、[ranger_messenger.cpp:289](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:289)。使用速度/转角和 ROS 时间积分，不是定位结果，也不是直接发布 CAN 编码器里程累计值 |
| /base/safe_cmd_vel | geometry_msgs/msg/Twist；项目约定的专用输入 | 原生绝对订阅 /cmd_vel 在 [ranger_messenger.cpp:168](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:168)；项目通过 ROS remap 实现 |

数据新鲜度：ROS 每轮用当前 ROS 时间打戳（[ranger_messenger.cpp:174](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:174)），读取 SDK 缓存并继续积分（[ranger_messenger.cpp:184](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:184)），没有检查 SDK 反馈时间戳或 CAN 断流；SDK 更新缓存的路径见 [agilex_base.hpp:287](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ugv_sdk/include/ugv_sdk/details/robot_base/agilex_base.hpp:287)。因此断 CAN 后仍有新 ROS 时间戳/里程计消息，不代表还在收到车体数据。初始反馈前也没有有效性门控，禁止将初始值作为实测状态。

运动模式先用上次 motion_mode_ 做里程计，再读取本轮模式（[ranger_messenger.cpp:187](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:187)、[ranger_messenger.cpp:211](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:211)）；切换时存在一周期模型不一致。双阿克曼姿态积分的线速度含 cos(phi)（[kinematics_model.hpp:44](/home/lab307/ranger_nav/ws_base/src/ranger_ros2/ranger_base/include/ranger_base/kinematics_model.hpp:44)），而 odom twist.linear.x 直接写 linear（[ranger_messenger.cpp:350](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:350)）；速度/转角含义需要实车核对。协方差未显式配置（[ranger_messenger.cpp:340–373](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:340)），后续融合不得把默认全零当成高置信测量。

## vx、vy、wz 的实际控制语义

下面描述**补丁后 V2 分支的主机实现**，不是经过实车校准的理想全向模型。输入单位按 Twist 约定为 m/s 与 rad/s，代码未处理 linear.z/angular.x/angular.y。

| 输入条件 | 自动选模式 | 发给 SDK 的值 |
|---|---|---|
| vy ≠ 0（优先于 wz） | Parallel=1 | 角度从 atan(vy/vx) 得到，按 vx 符号、上次非零 vx 和纯横移分支再修正；夹至 ±1.570；速度为带符号 sqrt(vx²+vy²)；**wz 被忽略**。[ranger_messenger.cpp:399](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:399)、[ranger_messenger.cpp:430](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:430) |
| vy=0 且 abs(wz)<1e-6 | Dual Ackerman=0 | 半径取无穷，转角 0，速度 vx。[ranger_messenger.cpp:501](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:501)、[ranger_messenger.cpp:420](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:420) |
| vy=0、abs(wz)≥1e-6 且 abs(vx)/abs(wz) ≥ 0.4764 | Dual Ackerman=0 | L=0.494，转角 sign(vx*wz) × atan((L/2)/radius)，先限 40°再限 ±0.6981；SDK 收到 vx、转角。[ranger_messenger.cpp:501](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:501)、[ranger_messenger.cpp:420](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:420) |
| vy=0 且半径 <0.4764 | Spinning=2 | SDK 收到 linear=0、steer=0、angular=clip(wz,±4.8)；**输入 vx 被舍弃**。[ranger_messenger.cpp:408](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:408)、[ranger_messenger.cpp:469](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:469) |
| 全零 Twist | Dual Ackerman=0 | 零速度、零转角，同时发送模式切换；不是保持原轮向的 Park 命令。[ranger_messenger.cpp:407](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:407)、[ranger_messenger.cpp:506](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:506) |

V2 的纯横移走 Parallel，V1 才走回调中的 SIDE_SLIP 分支（[ranger_messenger.cpp:400](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:400)）。例如 (vx=0,vy=0.05,wz=0.1) 并不会横移同时旋转；(vx=0.05,vy=0,wz=0.2) 因半径 0.25 m 会转成原地旋转。纯横移方向还依赖上次非零 vx，倒车后的横移符号必须专项验证，不能套用普通全向底盘语义（[ranger_messenger.cpp:433–466](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:433)）。

每次回调先发送 SetMotionMode 再发送运动值，没有等待模式切换完成或轮向到位的反馈（[ranger_messenger.cpp:395–491](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:395)；[agilex_base.hpp:157](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ugv_sdk/include/ugv_sdk/details/robot_base/agilex_base.hpp:157)）。反馈协议虽然包含 mode_changing（[agilex_protocol_v2.h:252](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/src/protocol_v2/agilex_protocol_v2.h:252)），本回调没有据此握手。小 vy 噪声也能触发 Parallel，没有模式迟滞；Dual/Parallel 路径没有对线速度做 max_linear_speed 限幅，也没有 NaN/Inf 输入拒绝（[ranger_messenger.cpp:420–467](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:420)）。

SDK 调用链：RangerRobot 转发（[ranger_robot.cpp:52](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/src/mobile_robot/ranger_robot.cpp:52)），V2 继承的 RangerBase 将 linear、angular、lateral=0、steering 交给通用发送函数（[ranger_base.hpp:28](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/include/ugv_sdk/details/robot_base/ranger_base.hpp:28)），再编码为 0x111 的四个 int16×1000 字段（[agilex_base.hpp:91](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ugv_sdk/include/ugv_sdk/details/robot_base/agilex_base.hpp:91)、[agilex_msg_parser_v2.c:382](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/src/protocol_v2/agilex_msg_parser_v2.c:382)、[agilex_protocol_v2.h:42](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/src/protocol_v2/agilex_protocol_v2.h:42)）。**该主机路径不计算/逐个下发四轮速度和四轮转角**，轮级分配由底盘侧实现；本仓库没有该固件实现，无法据此证明轮级约束/模式切换安全。

## 停车、超时与看门狗结论

| 情形 | 可从主机源码确认的行为 | 验收结果 |
|---|---|---|
| 上层持续发零 | 回调请求双阿克曼并发零速度/转角；不是硬件急停/Park | 待实测停止距离和转向变化；[ranger_messenger.cpp:407–427](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:407) |
| 上层停止发布 | Run 只发布状态并 spin；没有 cmd 时间戳、超时检查或主动补零；运动只在回调发送 | 停车未知；[ranger_messenger.cpp:53](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:53)、[ranger_messenger.cpp:391](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:391) |
| 驱动正常退出/SIGINT/崩溃 | SIGINT 直接 exit(EXIT_FAILURE)，没有最终零速或切 RC；SDK 析构仅关闭端口 | 停车未知；[ranger_base_node.cpp:18](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_base_node.cpp:18)、[ranger_robot.cpp:25](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/src/mobile_robot/ranger_robot.cpp:25)、[agilex_base.hpp:65](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ugv_sdk/include/ugv_sdk/details/robot_base/agilex_base.hpp:65)、[agilex_base.hpp:224](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ugv_sdk/include/ugv_sdk/details/robot_base/agilex_base.hpp:224) |
| CAN 断线/发送失败 | SDK 写失败仅日志，接收错误走 Close；没有车辆停止确认/自动恢复控制权 | 停车未知；[async_can.cpp:105](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ugv_sdk/src/async_port/async_can.cpp:105)、[async_can.cpp:126](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ugv_sdk/src/async_port/async_can.cpp:126) |
| 固件通信超时 | 主机发送函数注释要求 ≥50 Hz，但未实现周期重发或超时停车 | 超时阈值/动作未证实；[agilex_base.hpp:91–124](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ugv_sdk/include/ugv_sdk/details/robot_base/agilex_base.hpp:91) |
| 看门狗位置 | 所选 ROS 和 Ranger SDK 调用链均没有主机运动看门狗；若实车断流能停，需证明其底盘侧机制 | 不虚构 100/500 ms 阈值或默认保证 |

ROS 的 update_rate=50 只是 Run 循环频率（[ranger_messenger.cpp:53](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:53)、[ranger_messenger.cpp:68](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:68)），**不会自动把收到的 cmd_vel 变成 50 Hz CAN 心跳**。不要因为 ROS 节点存活就认为运动命令仍在刷新，也不要因为节点退出就认为底盘已经停止。

停车模式存在编码冲突：SDK 枚举 Park=3、SideSlip=4（[ranger_interface.hpp:53](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/include/ugv_sdk/details/interface/ranger_interface.hpp:53)）；ROS msg 却把 SIDE_SLIP 定义为 3（[SystemState.msg:8](/home/lab307/ranger_nav/ws_base/src/ranger_ros2/ranger_msgs/msg/SystemState.msg:8)、[MotionState.msg:1](/home/lab307/ranger_nav/ws_base/src/ranger_ros2/ranger_msgs/msg/MotionState.msg:1)）。parking_mode_ 只设为 false，没有 ROS 停车服务/订阅将其置 true（[ranger_messenger.cpp:151](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:151)、[ranger_messenger.cpp:155](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:155)、[ranger_messenger.cpp:397](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:397)）。因此 ROS 反馈值 3 不得直接解释为 V2 横移，不能宣称原生节点已经提供可用的 Park 机制。

## 需要在接管/运动前处理的源码风险

除已修复 V2 选型外，其余本轮只审查并记录，未改变上游控制/协议实现：

1. **SDK 异步发送缓冲区生命周期**：调用者创建栈上 can_frame（[agilex_base.hpp:84](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ugv_sdk/include/ugv_sdk/details/robot_base/agilex_base.hpp:84)、[agilex_base.hpp:120](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ugv_sdk/include/ugv_sdk/details/robot_base/agilex_base.hpp:120)、[agilex_base.hpp:164](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ugv_sdk/include/ugv_sdk/details/robot_base/agilex_base.hpp:164)），SendFrame 把其地址交给 async_write_some 并立即返回，没有复制或延长生命周期（[async_can.cpp:126](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ugv_sdk/src/async_port/async_can.cpp:126)）。本地 Asio 接口要求底层内存保持到 handler 执行（[basic_stream_descriptor.hpp:240](/home/lab307/ranger_nav/vendor/base_deps/root/usr/include/asio/posix/basic_stream_descriptor.hpp:240)）。源码存在缓冲区失效风险，不能仅凭构建成功放行发送；需修复/验证发送队列、消息所有权和完整帧写入，再进行控制权测试。本轮未用真机去探测此风险。
2. 接收出错时在 io 线程回调中调用 Close，而 Close 对 io_thread_ 执行 join（[async_can.cpp:78](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ugv_sdk/src/async_port/async_can.cpp:78)、[async_can.cpp:84](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ugv_sdk/src/async_port/async_can.cpp:84)、[async_can.cpp:110](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ugv_sdk/src/async_port/async_can.cpp:110)），存在自 join 异常路径风险；未做真机故障注入。
3. 无主机指令/反馈超时保护、速度输入缺少有限值与线速度限制、状态数据新鲜度缺失、执行器数组和 Park 枚举问题如上。后续安全输出发布者尚未实现，话题名称不能弥补这些问题。

以上是源码审查结论/风险，不冒充已复现的实车事故。当前不批准运动安全，不进入导航。

## 项目集成配置

[base_mini_v2.yaml](../config/base_mini_v2.yaml) 只使用实际 declare_parameter 的参数（[ranger_messenger.cpp:62–70](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:62)），[base_mini_v2.launch.py](../config/base_mini_v2.launch.py) 直接启动真实 ranger_base_node：

- 固定 robot_model=ranger_mini_v2、port_name=can0、update_rate=50。
- odom_topic_name=/wheel/odom；帧为 odom/base_link；publish_odom_tf=false。TF 条件分支在 [ranger_messenger.cpp:375](/home/lab307/ranger_nav/docs/02_evidence/initial_source_snapshot/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:375)，后续由定位集成决定唯一发布者。
- 将原生绝对 /cmd_vel remap 到 /base/safe_cmd_vel。没有编造 cmd_vel_topic/command_timeout 等原生参数。专用话题减少普通 /cmd_vel 的误驱动风险，**不构成权限隔离、限速或停车看门狗**。
- 默认 execute=false，不创建驱动。实际启动还要求当次 control_authorized=true、confirmed_model=ranger_mini_v2，并只读检查 can0 已 UP/ERROR-ACTIVE 且有 bitrate；不修改 CAN、不自动重启。
- 这些启动参数是本项目 launch 的门控参数，不是驱动原生参数，也不能替代用户的实际当次授权。

默认 dry-run、授权/型号缺失及 CAN DOWN 的拒绝路径已离线验证，正常路径只构造 action、未执行。见 [launch_guards_test.log](02_evidence/launch_guards_test.log) 和 [offline_checks.log](02_evidence/offline_checks.log)。

## 关键命令、退出码与文件

| 命令 | 退出结果/含义 | 证据 |
|---|---|---|
| git ls-remote --heads 两个官方仓库 | 各 0；明确存在 humble/main | source_and_can_search.log |
| 固定版本 apt download；dpkg-deb -x | 0；仅项目内解包 | dependencies.log |
| lsusb/udevadm/readlink/ip 查询 | 各 0；USB-CAN 枚举通过，CAN DOWN | usb_can_confirmation.log |
| timeout --signal=TERM --kill-after=2s 10s …/candump -e -x 'can0,0:0,#FFFFFFFF' | 1，Network is down | host_can_initial.log |
| python3 scripts/base_readonly_diagnostics.py --execute --seconds 10 | 2；诊断脚本报告 candump 1，未误判 PASS | offline_checks.log、can-mdxf3ols/ |
| python3 scripts/check_base_model_selection.py | 0；原始函数 V2 失败=1，修复后四型号通过=0 | model_selection_test.log |
| python3 scripts/check_base_launch_guards.py | 0；仅离线构造、没有启动 | launch_guards_test.log |
| ros2 launch …/config/base_mini_v2.launch.py（默认） | 0；仅 DRY-RUN 文本 | offline_checks.log |
| bash scripts/build_base.sh --execute | 0；4 packages finished，含已记录警告 | build-0iEF8pvo.log、ws_base/log/ |
| ros2 pkg/interface 查询、ldd、Python/C 类型支持加载 | 均通过；没有初始化 ROS 节点或访问 CAN | postbuild_interfaces.log |

按用户要求另分 Agent 独立只读复核，结论一致：USB 映射 PASS，CAN DOWN、限时接收失败，未找回历史成功配置。独立证据：[independent_can_review.md](02_evidence/independent_can_review.md)、`can-9h13lxmt/`。它也没有设置 CAN、运行节点或发送控制；这是 21:54 的历史结果；其后 21:56 的最新接收验证见本报告开头。

修改/新增：ws_base/src 两个固定仓库及构建输出；ranger_messenger.cpp 一行补丁；patches/ranger_ros2_humble_v2_model.patch；config/base_sources.repos、base_mini_v2.yaml、base_mini_v2.launch.py；scripts/build_base.sh、base_readonly_diagnostics.py、check_base_model_selection.py、check_base_launch_guards.py；vendor/base_deps；docs/02_evidence；本报告、02_base_manual_tests.md、STATUS.md，HANDOFF.md 增加最新阶段提示。无底盘固件、网络、CAN bitrate、系统服务或其他工程变更。

## 回滚与最少人工操作

系统/CAN 配置未变，无需网络回滚；不要运行雷达或官方 CAN setup 回滚。停止使用本阶段 overlay 即可恢复只用 /opt/ros/humble 的新 shell；不删除原有工作空间、地图或 bag。

撤销唯一上游修改前先在仓库运行 `git diff` 确认仅此补丁，再执行 `git apply --reverse --check /home/lab307/ranger_nav/patches/ranger_ros2_humble_v2_model.patch`，确认匹配后才手工 reverse apply；重新构建后才改变二进制。**撤销会重新引入 V2 型号错误，不能用作可运行的底盘配置。** 本轮没有自动执行回滚。

下一步：当前 CAN 接收配置已经现场复核，继续复用 500000 bit/s，不重复配置。记录本车铭牌/SN/固件并处理上述 SDK 发送等风险后，再安排需当次许可的 ROS/接管测试。CAN 接收、控制发送与停车分别验收；不自动推进后续阶段。
