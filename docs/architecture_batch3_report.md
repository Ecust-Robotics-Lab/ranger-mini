# 架构改造批次 3：采样时间对齐与会话管理

实施与全量测试：2026-10-20；中断恢复核验与收尾：2026-10-26。分支：`refactor/pluggable-navigation`。批次前提交：`b97cf91bbc44cd6b29e5e6a338fb3493169375e6`。

**PASS：本批实现、构建、数值测试与隔离离线启动/退出验证；真机导航验收仍为 PARTIAL。** 未启动物理底盘、雷达或相机，未发送真实 CAN 控制，没有安装新算法或更改系统/网络/CUDA。

## 1. 实际完成

- 实现整数纳秒索引的有界轮速历史，平移线性插值和最短弧四元数 SLERP；在定位采样时刻计算 map→odom，创新检查也使用同刻配对。
- 默认历史 2 s、最多 1000 条、最大插值间隙 0.1 s；不外推。重复心跳不重新评分/刷新采样年龄；原定位数据、后端心跳、轮速和已对齐修正共同参与就绪判定。
- 处理轮速/定位时间回退与 ROS 时钟回退，缓存不足或区间间隙过大时阻断，补充 time_aligned、alignment_reason、aligned_sample_time 诊断。
- Lightning 原生位姿改用私有 lightning_imu_pose；受管物理链统一为 base_link→imu_link→livox_frame，公共桥接拥有 map→odom 与 odom→base_link。安装参数保持原近似值。
- field/navigation/aux/base/lidar 入口统一为默认 dry-run、显式执行的前台受管会话。按数据/TF 新鲜度、发布者数量和真实 lifecycle GetState 结果调度，不再按进程名清场或靠固定等待启动。
- 使用资源租约、精确进程身份和双监督进程/subreaper 处理单实例、子孙进程、setsid、正常停止和监督进程异常退出。无自动重启；含底盘或导航的每次实际启动要求显式 --control-authorized，运动使能不由管理器发送。
- 日志与状态每次写入新的 `.runtime/sessions/<会话>/run-*`，不覆盖之前会话；该目录加入 Git 忽略。--stop/--status 不要求运行地图配置可解析。

接口、配置、操作和故障边界详见 [managed_sessions.md](managed_sessions.md)；消息接口仍为 LocalizationEstimate，原始采样时间语义不变。

## 2. 时间对齐验证

`tests/test_time_alignment.py` 包含：

- 平移中点、完整旋转、四元数跨 ±180° 最短弧及符号等价性。
- 缓存前后边界拒绝外推、轮速大间隙拒绝插值、重复/倒退时间戳、大 epoch 下整数纳秒插值与容量限制。
- 独立进程中的已知 6DoF 加速/转弯轨迹，定位延迟依次为 0.18/0.32/0.06 s。期望 map→odom 为已知的非零平移/旋转，检查实际 TF 数值，不仅检查是否存在。
- 轮速时间回退、定位时间回退、定位采样时间冻结和轮速断流均撤销就绪，恢复数据后可重新对齐。
- ROS 时钟回退清除轮速、定位和变换缓存。

最终日志记录数值误差；测试要求最大平移误差 <1 mm、最大 yaw 误差 <0.0001 rad。此为合成轨迹下的插值与时间语义验证，不是实际机器人定位精度。

阶段 5 的模拟定位也已改为按 header 的采样时刻生成位姿，避免“位姿是当前值、时间戳却写过去”的测试夹具掩盖错配；map→odom 的一致性门槛收紧至 5 mm。

## 3. 进程与真实入口验证

`tests/test_session_manager.py` 用纯合成进程验证：依赖等待、启动超时、接口占用预检、核心健康丢失停机、非核心退出降级、重复会话/共享资源拒绝、正常停止、setsid 子进程在父进程退出后的清理、任一监督进程被 SIGKILL、陈旧 PID 不误杀、全程保留无关进程，以及无授权标志时拒绝创建底盘/导航执行会话。

`tests/test_managed_session_ros.py` 使用合成 CustomMsg、IMU、轮速与标准定位消息，启动真实 field 和 navigation 入口。测试 profile 明确禁用底盘、雷达驱动及原生算法，仅启动探针、固定 TF 与公共桥接；随后仅在隔离 domain 中启动 Nav2：

- field 达到 RUNNING，标准桥接就绪且没有 base/lidar/localization 物理组件。
- Nav2 六个生命周期节点 ACTIVE，过滤点云/门禁心跳存在，最终速度输出保持零。
- 停止模拟输入后 field 进入 DEGRADED；navigation 关键健康超时后失败停机，不自动重启，owned 为空。
- 手动停止 field 后正常退出，owned 为空；日志和最终状态已复制到本批证据目录。

导航的 FAILED 在这个故障注入测试中是预期结果，代表数据丢失后的停机策略通过，不能将其解释为真机导航失败或成功。

## 4. 命令、退出结果与证据

证据目录：[`architecture_evidence/batch3_20261020/`](architecture_evidence/batch3_20261020/)。构建在 source Humble 和 ws_livox 后于 ws_nav 执行；ROS 测试另 source ws_nav。测试强制 ROS_LOCALHOST_ONLY=1，bridge/contract/alignment/managed/safety 分别使用 domain 179/181–182/183/184/180。进程测试不启动 ROS。

| 关键命令 | 结果 | 日志 |
| --- | --- | --- |
| `colcon build --packages-up-to ranger_navigation --executor sequential` | exit 0，三个包完成 | `build_complete.log` |
| `python3 tests/test_time_alignment.py` | exit 0，7/7 | `alignment_verified.log` |
| `python3 tests/test_session_manager.py` | exit 0，11/11 | `process_verified.log` |
| 恢复后复跑时间对齐、进程管理（2026-10-26） | 各 exit 0，7/7、11/11 | `alignment_resume_20261026.log`、`process_resume_20261026.log` |
| `python3 tests/test_architecture_contract.py` | exit 0，11/11 | `contract_verified.log` |
| `python3 tests/test_stage5_bridge.py` | exit 0，14/14 | `bridge_verified.log` |
| `python3 tests/test_stage6_gate_unit.py` | exit 0，12/12 | `gate_verified.log` |
| `EVIDENCE_DIR=.../safety python3 tests/test_stage6_safety.py` | exit 0，37/37 | `safety_verified.log`、`safety/` |
| `EVIDENCE_DIR=.../managed_complete python3 tests/test_managed_session_ros.py` | exit 0，真实入口与生命周期故障测试通过 | `managed_complete.log`、`managed_complete/` |
| `bash scripts/field_session.sh` / `bash scripts/run_navigation.sh` | exit 0，JSON dry-run，无节点启动 | `field_plan.json`、`navigation_plan.json` |
| shell 语法、Python AST、Git 差异、安装副本和保留资产检查 | exit 0 | `verification.json` |

具体命令和工具退出结果归档在 command_results.json。最终数值、参数与文件哈希以日志和 verification.json 为准；构建的禁用字节码提示保留，不视为构建失败。

恢复核验脚本为 `verify_resume.py`，仅做语法、哈希、版本及证据检查，不启动 ROS 节点。中断期间发现 `docs/03_mid360_validation.md` 的独立 IP 修改，保持原样并排除在本批提交之外；已有六个未跟踪雷达日志目录同样保留。

## 5. 失败尝试和测试修正

- `managed_attempt1.log`：ROS 探针字段 clients 与 rclpy 的只读属性冲突，启动退出；管理器停止会话、没有启动下游物理组件。改为 lifecycle_clients 后真实入口测试通过，原日志保留。
- `session_verified.log`：占用预检单元测试过度限定“必须已启动 probe”；预先提供的有效健康快照允许更早拒绝，实际没有启动被保护组件。改为检查禁止组件从未启动，保留更早拒绝行为。
- `contract_complete.log`：当时 unittest 行为断言通过，但 SIGINT 后适配器日志出现 Humble take_message 转换异常。收尾仅在 ROS 上下文已经关闭时容许此 RuntimeError；运行中仍抛错，并补充子进程退出码必须为 0 的断言。最终结果以 contract_verified.log 为准。
- `safety_attempt1.log` 与 `safety_attempt1/`：36/37，T13 有规划但未观测到达到原阈值的控制指令。原夹具未提供控制器声明消费的轮速输入；补齐与固定 TF 一致的静止 Odometry，并增加 t13_controller_trace.json 记录实际消息，单独复核 37/37。没有降低 0.02 m/s 的判定阈值；尚不据此认定这已排除随机 MPPI 在所有场景下的低速输出问题。

T13 是“规划/控制器产生指令、门禁保持零”的烟雾测试，不执行目标到达。固定模拟位姿不随指令移动，故日志中的 progress checker 超时不作为真机目标验收结果；最终指令轨迹记录了实际非零输出，而非以停机全零冒充链路可用。静态 TF 进程直接调用本机已核对的可执行文件，避免 ros2 run 包装器退出后留给兜底清理的子进程。

没有删改失败证据，没有通过放宽位姿新鲜度、修改生产 Nav2 参数、地图或运动学限制来通过测试。

## 6. 修改文件与待验项

- 公共桥接：新增 alignment.py，更新桥接、runtime 解析和 launch 参数；runtime.yaml 增加私有位姿帧、历史时长和最大间隙。
- 会话管理：新增 session_manager.py、robot_session.py、session_probe.py；将五个旧 shell 入口改为兼容包装；不修改底盘/雷达原生驱动、地图和 Nav2 生产参数。
- 测试：新增时间对齐、进程管理和受管入口 ROS 测试；更新阶段 5 的采样时刻夹具、契约覆盖及阶段 6 的静止轮速输入/指令记录。
- 文档与记录：更新计划、STATUS、接口文档和 Git 忽略/原始日志属性；新增操作说明、本报告及证据。具体文件和哈希见 verification.json。

**尚待真机/后续验证：** 实际硬件启动/正常退出与最坏延迟、精确安装外参、独立定位数据、原始传感器时间质量、真实停车/导航及坡道。插值假设源时间戳正确，不等于已解决硬件同步。

用户态双监督可处理任一监督进程被单独终止；同时 SIGKILL 两者、掉电或内核故障仍需更强的外部 systemd/cgroup 机制。只记录已归属进程，不以名字搜索外部旧会话，也不声称能在双监督都被强杀后找回所有未记录孤儿。资源锁只覆盖同一状态根目录，外部程序还需单独核查。

本批无需用户人工操作。后续为 FAST-LIVO2 具体实现、同步/标定及能力核验的准备批次；不自动安装、替换定位后端或开展新真机测试。
