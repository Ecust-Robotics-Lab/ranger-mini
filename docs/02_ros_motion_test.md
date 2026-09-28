# 阶段 2：ROS 接口与短时运动测试记录

2026-09-14，Asia/Shanghai。阶段 **PARTIAL**：CAN 通信及已测 ROS 接口 PASS；最新日志完成约 5 秒前进指令，用户报告 5 秒测试停车未观察到后退。**驱动退出后仍有负速度反馈，反馈与实物关系待查；完整停车安全未通过，自主导航不放行。**

**【2026-09-28 结论更新】** 用户明确确认"阶段 2 停止安全测试已经通过"。停车安全据此改判 **通过（依据=用户现场确认，未新增自动化测试证据）**，本报告下文的"停车安全未通过"均为 2026-09-14 当时记录，作为历史原始记录保留不改写。上层停发超时、遥控接管与硬件急停在导航链路中的独立验收见阶段 6 验收文档；运动许可逐次授权约束不变。

## 5 秒测试现场观察与日志核对（2026-09-14T22:34:50+08:00）

用户报告：“观察到5S的时候停车没有往后退”。记录为现场观察，没有将其改写为“已发生倒车”。用户没有指定对应日志编号或是否涵盖退出观察窗口，本节对照当前最新完成的一轮 `ros-motion-ebju6tr2`（22:32:18–22:32:37），不声称已经完成逐时刻同步匹配。

- 204 条非零 CAN 指令全部为 `0032000000000000`，即 vx=0.05 m/s、其他分量零；首条非零至首条零约 **5.0015 秒**，ROS 发布记录约 5.0013 秒。最新轮确已完成所设脉冲，没有像 22:25 那次被提前中止。
- 主动零速后末状态为 vehicle=0/control=1、速度零；末段近零保持 1.332 秒，清理补零段速度保持零。前进段 vx 反馈 −0.002 至 0.080 m/s；静止检查阶段曾短暂为 −0.035 m/s，最终进入前进前检查为零。原始反馈不能直接视为已标定的实物速度。
- 首条主动零命令后约 **3.645–3.729 秒**（驱动已经退出），原始 0x221 又出现 5 帧负值：−1.910、−2.544、−1.433、−0.305、−0.049 m/s，最后归零。最低值原始负载 `f610000000000000`，解码依据 [agilex_msg_parser_v2.c:79](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/src/protocol_v2/agilex_msg_parser_v2.c:79)。**这是协议解码数值，不能据此断言车体实际以 −2.544 m/s 后退。** 用户观察与该反馈需分别保留，固件状态切换、反馈计算及物理动作的关系未查明。
- 驱动退出 0，原始 assessment 仍为退出后非零反馈异常；没有另存本轮终端退出码，不把 assessment 当成独立采集的 shell 结果。RX +18409、TX +1334，错误累计无增长；CAN 未重新配置。四个 ROS 状态流各 750 条，TF 边为空。
- 同时发现两轮较早的 5 秒配置日志：22:30 `ros-motion-2vuihjib` 主动零反馈稳定时间不足 0.5 秒；22:31 `ros-motion-30mbzbb0` 因 vx=0.144 超过 0.12 m/s 检查值提前中止。不能将最新一轮或现场“未后退”追溯为这些测试通过。

证据：[最新原始结果](02_evidence/ros-motion-ebju6tr2/result.json)、[观察与原始帧审查](02_evidence/ros-motion-ebju6tr2/observation_review.json)、[原始 CAN](02_evidence/ros-motion-ebju6tr2/can_frames.jsonl)、[发布命令](02_evidence/ros-motion-ebju6tr2/published_commands.jsonl)。本助手仅用 Python 读取日志、解码及统计，退出 0；没有启动 ROS、访问 CAN 或发送控制，也未修改测试脚本。修改：本报告、STATUS.md、阶段接口报告及上述审查记录。

结论：**5 秒前进指令完成 PASS（最新记录限定范围）；现场报告停车未见后退；反馈一致性及完整停车安全 PARTIAL。** 当前需要的是对应退出窗口的现场观察与反馈来源核对；上层停发、驱动崩溃、CAN 中断、遥控接管和急停仍待独立验收，不自动实施下一轮。

## 按用户要求放宽速度反馈检查（2026-09-14T22:29:28+08:00）

将前进阶段纵向速度绝对值中止阈值从 0.10 调整为 **0.12 m/s**，集中为 [base_ros_motion_test.py:25](/home/lab307/ranger_nav/scripts/base_ros_motion_test.py:25) 顶部 `MAX_FORWARD_FEEDBACK_SPEED_MPS`。这是本项目测试阈值，未获得厂家安全限值认证。指令仍为 0.05 m/s，目标时长仍 5 秒；静止纵向 0.005 m/s、角速度、模式、控制权、数据新鲜度和退出后异常检测不变。dry-run 提示和 result.json 记录本次阈值，非有限或非正参数在 ROS/CAN 之前拒绝。

`python3 scripts/tests/test_motion_interlocks.py`、`python3 scripts/tests/test_post_exit_feedback.py`、默认 `python3 scripts/base_ros_motion_test.py` 均退出 0。实际判断代码接受前进阶段 0.101 及 ±0.12，拒绝 ±0.121 和 −0.801，静止阶段仍拒绝 0.101；退出后异常与首次原因保留回归通过。无效阈值离线检查通过。证据：[feedback_limit_012.json](02_evidence/ros_motion_preflight/feedback_limit_012.json)。修改文件：测试脚本、状态检查测试、阶段报告、STATUS.md 和人工步骤。本助手没有运行底盘或发送控制。配置/离线检查 PASS，新阈值真机待验；原有速度与退出异常仍未解决，停车安全未通过。

## 用户自行执行 5 秒配置：速度检查提前中止（22:25:47–22:26:01）

用户在终端执行 `python3 scripts/base_ros_motion_test.py --execute --motion --authorized-this-run`，目录 `ros-motion-4mquwslq`。目标 5.0 秒已生效，但第约 **1.96 秒**反馈 `vx=0.101 m/s` 超过脚本前进阶段 `abs(v)>0.1` 的阈值，抛出 `Unexpected feedback speed` 并转入 cleanup_zero。阈值来源：[当前检查代码:153](/home/lab307/ranger_nav/scripts/base_ros_motion_test.py:153)（本节试验时阈值为 0.1，之后调整见上节），这是项目测试检查值，不能解释为底盘厂家限速。5 秒是无异常时的目标时长，异常可提前结束。

原始数据核对：78 条非零 ROS/CAN 指令，CAN payload 全为 `0032000000000000`（0.05 m/s、其他分量零）；首个非零到首个清理零 ROS 间隔 1.9683 秒、CAN 接收间隔 1.9585 秒。原始反馈 `0065000000000000` 即 0.101 m/s；不是时长参数未保存，也没有发出 0.101 m/s 指令。实际物理车速和时长尚未独立测量。

本轮前进检查通过前，静止阶段也曾出现最高 0.092 m/s 反馈；最终静止检查时为零。清理阶段最高 0.106 m/s，退出后最低 **−0.801 m/s**，最终又归零。均为反馈观测，不能据此确定固件动作或实物运动原因。没有执行器独立标定证据，不因只超出阈值 0.001 就提高阈值或取消检查。此轮 CAN TX +866，RX +13495，错误计数无增长。四个状态话题各 485 条，驱动退出 0；原 result.json 因后续退出异常覆盖了首先发生的速度错误，缺少原始 runner 退出码的独立终端记录，源码在此 assessment 下返回 2。

证据：[完整审查](02_evidence/ros-motion-4mquwslq/review.json)、[原始结果](02_evidence/ros-motion-4mquwslq/result.json)、[指令](02_evidence/ros-motion-4mquwslq/published_commands.jsonl)、[CAN](02_evidence/ros-motion-4mquwslq/can_frames.jsonl)、[驱动日志](02_evidence/ros-motion-4mquwslq/driver.log)。本助手本轮只做文件分析，没有再启驱动或发控制。

已修复测试结果记录：增加 `primary_error` 与 `abort_phase`，退出异常追加到 assessment，不再覆盖首次失败。`python3 scripts/tests/test_post_exit_feedback.py` 退出 0，验证原始负速度回放及首次错误保留；默认 dry-run 退出 0，仍显示 5 秒。修改文件：测试脚本、对应离线回归、报告和 STATUS.md；[修复版本记录](02_evidence/ros-motion-4mquwslq/logging_fix_record.json)。原始 result.json 保持不变。**5 秒持续前进未完成，停车安全仍未通过；阶段 PARTIAL，自主导航不放行。**

## 前进时长改为 5 秒（2026-09-14T22:25:23+08:00）

按用户指令将脚本顶部 `FORWARD_DURATION_SECONDS` 从 0.8 改为 **5.0 秒**，速度仍 0.05 m/s。`python3 scripts/base_ros_motion_test.py` 默认 dry-run 退出 0，输出 `vx=0.05 m/s for 5 s; then zeros`；未启动 ROS/底盘、未访问 CAN。配置修改和预览 PASS，5 秒真机测试待验，停车安全结论不变。修改文件：测试脚本、本报告、STATUS.md、人工步骤。以下 0.8 秒记录为历史实际测试/参数化记录。

## 脚本参数更新（2026-09-14T22:22:53+08:00）

用户要求将前进秒数集中为脚本可修改参数。现使用 [base_ros_motion_test.py:23](/home/lab307/ranger_nav/scripts/base_ros_motion_test.py:23) 顶部的 `FORWARD_DURATION_SECONDS = 0.8`；执行截止时间、dry-run 提示、运行提示及 result.json 的 `forward_duration_seconds`/范围描述均读取该值。速度保持 0.05 m/s，停车观察时间独立。默认仍 0.8 秒；修改参数不等于新的真机测试已经执行或通过。

配置必须是有限正数，0、负数、NaN、无穷及无效输入在接触 ROS/CAN 前拒绝。离线验证默认 0.8、自定义 2.5 的 dry-run，以及上述无效输入均通过；已有状态拒绝与退出后瞬态回放检查仍通过，命令均退出 0。证据：[duration_parameter_offline.log](02_evidence/ros_motion_preflight/duration_parameter_offline.log)。修改文件：测试脚本、本报告、人工步骤、STATUS.md；源码 SHA256 见 [duration_parameter_version.json](02_evidence/ros_motion_preflight/duration_parameter_version.json)。本次未启动驱动、未发控制，历史执行快照保持原样。结论：参数化及离线验证 PASS；停车安全结论仍未通过。

## 最新结果：22:11 与 22:16 两次重新授权测试

用户分别重新授权后执行，每轮仅启动一次，未自动重试。22:16 重复前已向用户说明前一轮负速度异常；用户明确要求再次执行。两轮均使用同一 V2 参数、补丁后驱动和 SDK、专用安全速度话题。遥控器此前由用户关闭。用户随后确认最后一轮“确实运行了”，但尚未明确确认停止过程、退出后是否回动或是否人工干预。

| 项目 | 22:11:57–22:12:12 | 22:16:37–22:16:52 |
|---|---|---|
| 日志目录 | ros-motion-mp69pbm2 | ros-motion-9yzy_0mk |
| 非零 ROS / CAN 指令数 | 34 / 34 | 33 / 33 |
| 所有非零指令 | vx=0.05 m/s，其他分量零 | 同左 |
| 首个非零到首个零 CAN 指令 | 0.8016 s | 0.8003 s |
| 主动零命令至反馈持续严格为零 | 约 0.494 s | 约 0.577 s |
| 主动发零观察末的近零保持时间 | 1.509 s | 1.402 s |
| 四个状态话题消息数（各） | 467 | 524 |
| 驱动退出码 | 0 | 0 |
| 测试脚本退出码 | 0，旧判断漏掉退出后的瞬态 | 2，新判断发现瞬态 |
| 退出后 vx 反馈最低值 | −0.425 m/s | −0.392 m/s |
| CAN TX 增量 | 832 | 946 |

时间为用户态接收时间，不能当作物理停车延迟。实际前进指令发布约 42 Hz（34 或 33 条/0.8 s），目标循环为约 50 Hz；这里没有宣称严格实时。前进阶段 vx 反馈最高均 0.056 m/s；发零减速期间最高 0.066 m/s，第二轮短暂最低 −0.009 m/s。指令限幅不等于实际速度绝无超调。

两轮的主动零速阶段均最终归零，清理补零阶段保持零；但驱动退出、命令刷新结束后，vehicle/control 回退为 2/0，随后出现连续 5 帧负向速度，最后再次归零。第二轮为 −0.184、−0.392、−0.177、−0.035、−0.007 m/s。第一轮同时观察到 0x281 轮速变化。没有任何反向运动指令；尚不能区分实物回动、轮速测量异常或固件内部动作，不归因于遥控器，也不把末帧归零当成可靠停车。

解码依据：[agilex_msg_parser_v2.c:79](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/src/protocol_v2/agilex_msg_parser_v2.c:79)（0x221）、[agilex_msg_parser_v2.c:349](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/src/protocol_v2/agilex_msg_parser_v2.c:349)（轮速）、[agilex_msg_parser_v2.c:382](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/src/protocol_v2/agilex_msg_parser_v2.c:382)（指令编码）。沿用下文系统状态解码，保留未解释字节，固件看门狗阈值及停车机制仍未确认。

两轮接口均保持 can0 UP/ERROR-ACTIVE、500000 bit/s；RX/TX errors、dropped 及 CAN 错误累计无增长。ROS 实测 V2 参数、/wheel/odom、禁用底盘 odom TF、geometry_msgs/msg/Twist 的 /base/safe_cmd_vel 订阅正确。没有收到 BMS。CAN 原始记录含本机 TX 回环，不能把所有帧当成底盘 RX。

证据：[第一轮结果](02_evidence/ros-motion-mp69pbm2/result.json)、[第一轮完整审查](02_evidence/ros-motion-mp69pbm2/review.json)、[重复轮结果](02_evidence/ros-motion-9yzy_0mk/result.json)、[重复轮完整审查](02_evidence/ros-motion-9yzy_0mk/review.json)。各目录保存 CAN/ROS/指令原始 JSONL、driver.log、CAN 前后统计、实际执行脚本快照及 execution_record.json。22:11 旧 result.json 保留原样，以 review.json 补充发现，不篡改原始结果。

实际命令（这段是记录及人工使用入口，不代表自动运行许可）：

```bash
cd /home/lab307/ranger_nav
source /opt/ros/humble/setup.bash
source ws_base/install/local_setup.bash
export ROS_LOG_DIR=/home/lab307/ranger_nav/docs/02_evidence/ros_motion_preflight/ros_logs
python3 scripts/base_ros_motion_test.py --execute --motion --authorized-this-run
```

脚本自行启动驱动，不要先另开驱动。去掉三个参数就是 dry-run，不接触 ROS/CAN。`--authorized-this-run` 表示操作者确认本次执行，不提供持续或跨进程许可。当前脚本遇到退出后非零反馈会退出 2；不会自动再试。测试人员须全程观察，硬件急停可及；异常时使用硬件停止手段，不能依赖脚本清理。没有执行移动时停发超时、遥控接管、急停、崩溃或断 CAN 注入。

### 本轮测试判断修复

原脚本只读退出观察期末状态，漏掉中途负速度。现保存退出后整段 0x221，任一速度分量超出近零阈值即标记 PARTIAL/ABORTED，接收失败也不得沿用成功结论，见 [base_ros_motion_test.py:300](/home/lab307/ranger_nav/scripts/base_ros_motion_test.py:300)。它只检测异常，不会在已退出的驱动上重新发送控制。

`python3 scripts/tests/test_post_exit_feedback.py` 退出 0：抽取实际判断代码，用第一轮原始反馈回放，验证最终为零仍能捕获全部 5 条异常，并检查静止及独立纵向/横向/角速度异常。`python3 scripts/tests/test_motion_interlocks.py`、默认 dry-run 均退出 0。该修复随后在 22:16 获授权重复轮实际捕获同类异常。源代码/构建版本沿用下文，仅测试脚本继续修改；两轮执行快照分别保存，不能混用。

当前没有运行底盘驱动；停止与退出异常仍需定位，后续自主导航 BLOCKED。用户已获取人工测试命令；其现场运行结果应另存独立日志并与实际观察对应。

## 以下为 22:05 首次尝试与修复的历史记录

## 本次授权与实际动作

用户确认现场平坦空旷、有人监护并能使用遥控或硬件急停，授权一次驱动启动和最高 0.05 m/s、最长 1 s 前进及主动零速停车。实际仅启动一次驱动（PID 23241），22:05:29–22:05:42 完成检查并正常退出（驱动退出 0）。测试脚本退出 2 表示按状态检查中止，不是驱动崩溃。

实际发送：1 条 0x421（请求 CAN 控制）；50 条 0x141（模式 0）；51 条 0x111（**全部 8 字节全零**，含正常退出时尽力补零）。ROS 发布 50 条全零 Twist。TX 增加 102、errors/dropped 和 CAN 错误累计没有增长。没有执行超时运动、遥控接管、硬件急停或 CAN 中断故障注入。

证据：[result.json](02_evidence/ros-motion-79ltuvud/result.json)、[review.json](02_evidence/ros-motion-79ltuvud/review.json)、[CAN 原始记录](02_evidence/ros-motion-79ltuvud/can_frames.jsonl)、[ROS 原始记录](02_evidence/ros-motion-79ltuvud/ros_messages.jsonl)、[速度发布记录](02_evidence/ros-motion-79ltuvud/published_commands.jsonl)、[驱动日志](02_evidence/ros-motion-79ltuvud/driver.log)。实际运行脚本的[快照](02_evidence/ros-motion-79ltuvud/base_ros_motion_test_executed.py) SHA256 与运行前版本记录一致。

## ROS 静止接口验证

- 实际参数为 ranger_mini_v2、can0、/wheel/odom、publish_odom_tf=false、update_rate=50。
- /base/safe_cmd_vel 唯一底盘订阅者为 ranger_base_node，类型 geometry_msgs/msg/Twist；底盘未订阅普通 /cmd_vel。
- /system_state、/motion_state、/actuator_state、/wheel/odom 各收到 **398 条**；odom/base_link 帧名正确，轮速反馈及里程计保持零；本次没有观察到 odom→base_link TF。
- 修复后的执行器消息按 id 区分电流/脉冲计数，相关数据不再重复首个元素。没有独立标定八个执行器；早期消息仍可能缺少部分低频反馈，不能把零值一概当成有效测量。
- 没有收到 0x361 BMS 帧或 /battery_state 消息，因此 BMS 接口尚未验收。

## 为什么没有前进

原始 CAN 时间线（相对采集首帧）：

| 时刻 | 反馈与动作 |
|---|---|
| 0 s | vehicle=0、control=0（RC）、速度零 |
| 约 2.064 s | 驱动请求控制后，vehicle=0、control=1（CAN） |
| 约 2.612 s | 没有持续运动命令，反馈变为 vehicle=2、control=0 |
| 约 9.0 s | ROS 静止检查完成；运动检查拒绝该状态，未执行非零脉冲 |
| 约 9.038–9.056 s | 清理阶段持续零速后先恢复 control=1，随后 vehicle=0 |
| 约 10.631 s | 驱动退出、零速刷新停止后，再变为 vehicle=2、control=0 |

依据 [SystemState.msg:1](/home/lab307/ranger_nav/ws_base/src/ranger_ros2/ranger_msgs/msg/SystemState.msg:1)，vehicle=2 是异常枚举，control=0 是 RC 枚举。解码来源 [agilex_msg_parser_v2.c:64](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/src/protocol_v2/agilex_msg_parser_v2.c:64)。error_code 始终为 0，但异常帧第 6 字节（从 0 计数）为 0x02；该字段在固定协议结构中名为 reserved0（[agilex_protocol_v2.h:181](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/src/protocol_v2/agilex_protocol_v2.h:181)），本轮不擅自把它解释成具体错误码。

**推断**：本车需要持续命令刷新来维持 CAN 控制，静止检查期间完全不发速度是本次测试流程的缺陷。约 0.55 s 后发生状态回退是观测值，不是已验证的固件超时规格；本次没有运动，不能据此证明失联后能可靠停车。持续零速恢复状态，也不能证明完整 RC/CAN 仲裁规则。

## 关闭遥控器后的只读复核

用户随后表示已关闭遥控器。本助手确认驱动和测试进程均不存在，再执行 5 秒接收诊断，没有重启驱动、没有发控制。保存 **4,427 帧，全为 RX**；TX 无增长，接口仍 ERROR-ACTIVE/500000 bit/s，错误计数无增长。

反馈仍为 vehicle=2、control=0、error_code=0，全部 238 条运动状态速度/转角为零，模式为 0。电压约 49.9–50.1 V。0x241 仍出现，不能仅凭该帧存在判定遥控器物理开关状态或射频连接状态。

因此关闭遥控器后没有观察到该状态自动恢复。不能由此断言“遥控器抢占”或“遥控器没关”；需要结合持续零速刷新后的下一次受控检查。证据：[commands.json](02_evidence/can-di358xjc/commands.json)、[review.json](02_evidence/can-di358xjc/review.json)、[candump.log](02_evidence/can-di358xjc/candump.log)。诊断脚本退出 0，内部 timeout/candump 退出 124，状态查询退出 0。

## 本次实施的修复与离线证据

固定 SHA 保持不变：ranger_ros2 b6ea21a275ca5e7168130cc6470e61474681d679，ugv_sdk f2704eacdc90357078cd93ec60aae08bb4baab35，均 detached HEAD，**两个仓库现在均有本地修改**。运行前修改清单和 SHA256 见 [versions.json](02_evidence/ros_motion_preflight/versions.json)。

- SDK：将发送改为加锁的立即非阻塞 send，栈帧不会在函数返回后留给异步写使用；没有用户态旧命令队列，失败打印错误。见 [async_can.cpp:138](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/src/async_port/async_can.cpp:138)。接收错误不再自 join，Close 正确释放 Asio 拥有的 fd 并退役取消回调（[async_can.cpp:86](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/src/async_port/async_can.cpp:86)）。缓存值初始化、出错后仍显式关闭（[agilex_base.hpp:200](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/include/ugv_sdk/details/robot_base/agilex_base.hpp:200)、[agilex_base.hpp:224](/home/lab307/ranger_nav/ws_base/src/ugv_sdk/include/ugv_sdk/details/robot_base/agilex_base.hpp:224)）。
- ROS：保留 rclcpp 正常信号清理，析构尽力发零（[ranger_base_node.cpp:37](/home/lab307/ranger_nav/ws_base/src/ranger_ros2/ranger_base/src/ranger_base_node.cpp:37)、[ranger_messenger.cpp:53](/home/lab307/ranger_nav/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:53)）；这不能覆盖 SIGKILL、崩溃或 CAN 中断。
- 状态：核心反馈组超过 250 ms 不重新打戳发布，并先更新模式再积分（[ranger_messenger.cpp:192](/home/lab307/ranger_nav/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:192)）。这是组级新鲜度，未解决每个 ID 独立丢流；**没有增加运动命令看门狗**。
- 按数组索引发布执行器，驱动电机的角度和转向电机的速度等不适用字段用 NaN；无 BMS 反馈不虚构电池值，SOC 按 0–1 转换（[ranger_messenger.cpp:246](/home/lab307/ranger_nav/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:246)、[ranger_messenger.cpp:285](/home/lab307/ranger_nav/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:285)）。
- 补丁：[ugv_sdk_stage2_transport.patch](../patches/ugv_sdk_stage2_transport.patch)、[ranger_ros2_stage2_test.patch](../patches/ranger_ros2_stage2_test.patch)。后者是相对固定上游的累计补丁，已包含之前的 V2 一行修复，不要再次叠加旧补丁。
- `g++ -std=c++14 -g -O1 -fsanitize=address,undefined -fno-omit-frame-pointer -pthread` 编译真实 async_can.cpp 与 [离线测试](../scripts/tests/async_can_offline.cpp)，随后 `timeout 15s /tmp/ranger_async_can_offline` 退出 0。覆盖帧所有权、顺序、拥塞不阻塞、fd 不重复关闭、接收错误退出；使用 AF_UNIX，无 SocketCAN。见 [async_can_offline.log](02_evidence/ros_motion_preflight/async_can_offline.log)，其中 errno=11 是故意制造的满缓冲区。
- `bash scripts/build_base.sh --execute`：退出 0，4 包完成（13.8 s），仍有已记录上游编译警告。日志：[build-JaMO1xCg.log](02_evidence/build-JaMO1xCg.log)。型号回归、实际测试脚本的 RC/异常/故障/过期/模式等离线拒绝检查均通过：[motion_interlocks.log](02_evidence/ros_motion_preflight/motion_interlocks.log)。

## 22:10 时的准备记录（随后已在上文两轮执行）

[base_ros_motion_test.py](../scripts/base_ros_motion_test.py) 默认 dry-run；新版在静止 ROS 检查期间以约 50 Hz 持续发送**零速**，前进阶段使用独立 0.8 s 截止时间和 0.05 m/s 上限，其后主动零速。状态异常、反馈过期或控制权不符时仍拒绝前进。新版零速刷新仅在测试脚本内实现，不是驱动原生能力或生产安全控制器；程序挂起/崩溃时不能保证停车。

这项修改仅做离线模拟：静止阶段有零速刷新，前进/退出等阶段不被后台零速逻辑覆盖，退出的驱动不继续刷新；故障检查仍通过。见 [zero_heartbeat_test.log](02_evidence/ros_motion_preflight/zero_heartbeat_test.log)、[next_test_prepared.json](02_evidence/ros_motion_preflight/next_test_prepared.json)。当时尚未重新运行；随后上文两轮均另获当次许可后执行。

当时待办已由上文两次授权执行推进；当前待办是退出后负速度异常定位及现场停止观察。依据 [AGENTS.md](/home/lab307/ranger_nav/AGENTS.md)：“任何运动许可在进程重启后失效，必须重新取得该次真机测试确认。” 超时运动、RC 接管、急停、断 CAN 和导航依然未放行。

## 文件与回滚

新增测试脚本/离线测试、两个补丁、原始源码快照及上述日志；修改 SDK 三个文件、ROS 三个文件，更新 STATUS.md 和阶段报告。没有更改 CAN bitrate、网络或底盘固件。

如需回滚，先确认驱动已经退出，保存当前 diff，分别用对应累计补丁 `git apply --reverse --check` 核对后再反向应用并重建；不要盲目叠加补丁或执行 git reset。回滚会重新引入已知 SDK/型号/状态问题，不能将回滚后的版本用于真机测试。之前源码审查的行号已指向修改前快照，防止与当前补丁后的行号混淆。
