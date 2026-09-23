# 阶段 2 待人工执行的测试

2026-09-14。**本文件不是运动授权。** CAN 通信及已测 ROS 接口通过；两次另获授权的 0.05 m/s、约 0.8 s 试验已执行，用户确认最后一次实物运动。两轮主动零速后反馈归零，但驱动退出后负速度异常复现，**停车安全未通过，自主导航不放行**。详见 [02_ros_motion_test.md](02_ros_motion_test.md)。实物为用户确认的 Ranger Mini V2，铭牌/SN/固件待记录。

## 执行前必须完成

1. 核对最新 `02_ros_motion_test.md`。SDK 发送/关闭风险已修复并离线验证，已完成多次各自获授权的启动；无主机运动看门狗、逐 ID 新鲜度及停车模式编码等问题仍需处理。旧许可不能用于重启。静止检查需要连续零速刷新，新版已复测，退出异常仍未解决。
2. 当前已实测 can0 为 500000 bit/s、sample-point=0.875、restart-ms=100，直接复用；**本助手没有执行任何 `ip link set`。** 每次测试先只读复核状态，重启后的状态另验；已正常运行的 CAN 不重新设 bitrate。
3. 本次限时只读收帧已完成，证据见 `02_evidence/can-zlje17cv/`。后续真机测试前，在人工确保底盘静止、急停可及、场地空旷且有人监护的情况下，复核收帧。检查多类反馈帧、频率、RX 增长和错误计数差值。TX 回环不能算底盘反馈。
4. 每次启动驱动前取得当次明确许可，记录测试编号、操作者、型号、固件、源代码 SHA/补丁、CAN 状态、停止条件。驱动或测试进程重启后重新确认；禁止自动重启或无人值守恢复运动。以下启动参数只用于人工执行，不代表本会话已经批准。

## 可执行的只读诊断与默认 dry-run

```bash
cd /home/lab307/ranger_nav
python3 scripts/base_readonly_diagnostics.py
python3 scripts/base_readonly_diagnostics.py --execute --seconds 10
source /opt/ros/humble/setup.bash
source ws_base/install/local_setup.bash
ros2 launch /home/lab307/ranger_nav/config/base_mini_v2.launch.py
```

前两条分别为诊断预览/接收诊断；第三条 launch 默认只打印配置，不创建底盘节点。日志每次写入独立 `docs/02_evidence/can-*`，不会覆盖已有记录。`candump` 的 124 只是到时退出，不能单独判 PASS。

## A. 静止状态读取（启动仍需当次接管许可）

完成上述前提且获得许可后，才人工执行：

```bash
source /opt/ros/humble/setup.bash
source /home/lab307/ranger_nav/ws_base/install/local_setup.bash
ros2 launch /home/lab307/ranger_nav/config/base_mini_v2.launch.py \
  execute:=true confirmed_model:=ranger_mini_v2 control_authorized:=true
```

该命令会发送控制模式配置，不是纯只读节点。先在硬件允许的急停/遥控静止条件下观察初始化；不假设急停状态下一定能切换 CAN 控制。参数文件固定 V2、`/wheel/odom`、`publish_odom_tf=false`，重映射 `/cmd_vel` 到 `/base/safe_cmd_vel`。

启动后先检查：

```bash
ros2 node info /ranger_base_node
ros2 param dump /ranger_base_node
ros2 topic info /base/safe_cmd_vel --verbose
timeout 10s ros2 topic echo /system_state
timeout 10s ros2 topic echo /motion_state
timeout 10s ros2 topic echo /wheel/odom
timeout 10s ros2 topic echo /actuator_state
timeout 10s ros2 topic echo /battery_state
```

核对订阅确为 `geometry_msgs/msg/Twist`、普通 `/cmd_vel` 没有本底盘订阅者、没有意外速度发布者；检查 TF 图，确认底盘没有发布 `odom → base_link`。不通过向普通 `/cmd_vel` 发非零速度来验证隔离。CAN 反馈与 ROS 内容交叉检查；ROS 消息更新时间不等于 CAN 数据新鲜。原 `/actuator_state` 重复首元素问题已修复，但八个执行器仍未逐个标定验收。

记录静止速度、轮向、故障码、电压、反馈频率；核对 `control_mode` 随人工 RC/CAN 开关变化的实际行为。若意外运动、切换模式不符或数据陈旧，立即由监护者使用硬件停止手段并终止测试。

## B. 短时低速前进与显式停车（另行运动许可）

用户请求自行运行时，可使用本轮实际脚本（**不要同时另启驱动**）：

```bash
cd /home/lab307/ranger_nav
source /opt/ros/humble/setup.bash
source ws_base/install/local_setup.bash
# 默认只预览，不操作底盘
python3 scripts/base_ros_motion_test.py
# 现场确认本次测试、有人监护且硬件急停可及后才执行
python3 scripts/base_ros_motion_test.py --execute --motion --authorized-this-run
```

前进阶段纵向反馈绝对值阈值在顶部 `MAX_FORWARD_FEEDBACK_SPEED_MPS = 0.12` 修改；这是异常中止阈值，发送速度仍 0.05 m/s。0.101 m/s 不再触发前进阶段的这一项检查，但静止与退出后的近零检查仍严格保留。新阈值仅离线通过，不能保证试验一定持续满 5 秒。

前进秒数在脚本顶部 `FORWARD_DURATION_SECONDS = 5.0` 修改，提示、截止时间与日志同步读取；参数必须是有限正数。脚本自行完成静止检查、0.05 m/s 前进所设时长（当前配置 5 s，仅预览验证；历史真机为约 0.8 s）、持续发零、退出和退出后观察。出现退出后异常时退出 2；每轮输出独立 LOG_DIR。5 秒是用户要求的配置变更，尚未真机验收；已知退出异常尚未排除，不构成 C/D/E 故障测试放行。终端 Ctrl+C 仅触发尽力清理，不替代硬件急停。

人工确认前进方向、车轮姿态、急停/遥控操作和停止观察窗口后，试验输入拟定为 `vx=0.05 m/s, vy=0, wz=0`，50 Hz，最长 1 s；之后连续发送全零 Twist 至少 1 s。数值只是待确认的试验上限，不是已验证的安全限值。操作者必须能够立即停发并使用硬件急停。

速度只能发布到 `/base/safe_cmd_vel`；该名字本身不提供限速或看门狗。这里不提供自动连续执行所有测试的脚本，避免重启后复用授权或把不同测试合并放行。测试发布器应使用单调时钟、有限时长、有限值检查及退出时尽力发零；退出回调不能作为崩溃/断线保障。

同时记录实际 CAN 运动命令和反馈、人工观察/视频、速度、停止时间、停止距离。全零 Twist 会请求双阿克曼模式，不是 SDK Park/硬件急停；观察转向是否变化。显式零速停车失败，禁止后续超时/故障注入测试。不得只用 `/wheel/odom` 的零速度宣称车体已停。

## C. 上层指令超时（另行许可）

先通过 B。短时同样低速输入后仅停止发布器，保持驱动存活，不主动补零，以测出底盘失去运动命令后的行为。此操作有保持旧速度风险，必须事先约定观察窗口和监护者介入时刻；尚无厂家/现场依据，不预填“500 ms 自动停车”。

记录最后一条 CAN 运动命令、最后运动反馈及实际完全停止时间。验证驱动没有意外重发，测出固件超时/停车行为；到达人工约定窗口仍未停立即硬件停止，结果 FAIL。恢复发布不自动获准，重新确认状态后再决定。

## D. 遥控接管与急停（逐项许可）

先静止切换 RC/CAN，确认开关位置、优先级、切回 CAN 是否需重新初始化；有持续上位机命令时的低速接管单独批准。记录 RC/CAN 控制码、反馈运动模式与车体实际反应，验证 RC 中位是否使车停，以及退出接管是否会执行旧指令。

急停先静止测试再另行批准低速测试。分别记录硬件急停动作、`vehicle_state`、轮速、实际停止距离；解除急停前停止发布器，确认不会自动恢复旧指令。不得把 ROS 的 `VEHICLE_STATE_ESTOP=1` 常量当成硬件急停已通过。

## E. 驱动退出与 CAN 中断（分开批准）

1. 先静止退出驱动，再跑限时 candump 并检查 CAN 状态、控制模式、反馈是否仍到达；不要假设退出会自动交还 RC。
2. 正常退出、SIGINT、进程异常退出的停止能力分别记录；只有在 B/C 已有可靠停止证据后，才考虑在低速条件下试验。`kill -9` 不会运行任何清理代码，不能作为停车命令。
3. CAN 中断是独立故障注入。人工确认后再操作，检查物理车体停止、错误计数、驱动状态以及恢复连接后的行为；不自动重连或重启驱动。此次未编写/执行会自动断 CAN 的脚本。

## 验收记录模板

每项记录：时间、当次许可、操作者、型号/SN/固件、源码 SHA/补丁、测试输入与真实发布频率、CAN 状态前后、原始反馈、实际停止时间/距离、急停介入、日志/视频路径、PASS/PARTIAL/FAIL 和适用范围。

**底盘通信 PASS**：正确 bitrate 下持续收到合理的底盘反馈，帧/频率/错误差值经过检查；ROS 静止状态接口若验收通过另行标注。

**运动安全 PASS**：不是通信 PASS 的推论。必须独立验证显式零速、上层停发、驱动退出/崩溃、CAN 中断、RC 接管、急停与解除后不自启，并有实际停距/时间满足场地要求的证据。未满足时保持“自主导航不放行”。本清单不是导航阶段授权。
