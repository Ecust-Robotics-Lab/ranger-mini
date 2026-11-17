# EXACT-MPPI / Ranger Mini V2 实施与验收（2026-11-10）

本轮按[用户计划](EXACT_MPPI_NAVIGATION_PLAN.md)完成软件替换，最终核验结果在本文及同目录证据中记录。[接口/复现](exact_mppi_interfaces.md)。设备启动、底盘/CAN、实车导航和30分钟运行没有执行，全部真实验收仍PARTIAL；没有提交或推送。

随后用户要求准备上方办公室—走廊端点掉头—返回，并允许减少额外规划裕量；最新窄通道参数、反馈衔接/裕量一致性修复、111用例覆盖/标准9/窄档10/ROS19及路线5/7的准备结果见[实车准备](EXACT_MPPI_FIELD_TEST.md)。本报告下文的原始验收与身份保留历史条件；当前安装身份另存`exact_mppi_evidence/field_preparation_20261110/delivery_identity.json`。

## 授权、来源及保护

用户授权软件源码、依赖、构建和离线验证，后追加允许RTX5060驱动/CUDA。用户自行完成580.178.04-open安装、MOK和重启；SecureBoot保留。JAX0.6.2的cuda:0实际计算通过，CUDA12 pip运行库在项目`.runtime/exact_mppi_venv`，没有升级ROS/Eigen/PCL。驱动包及回退证据`.runtime/backups/exact_mppi_gpu_2dg9OIbf/`。没有证据证明595比本机580更稳定，本轮保留已实测工作的版本。

Git基线e8245fa80310a4e91b5d2eb303f33883bb965216，分支refactor/pluggable-navigation。本轮前已有AGENTS/README及交接文档修改，原内容/patch/status已保护于`.runtime/backups/exact_mppi_20261110/`，旧导航源码及专属测试也先备份再替换。原包、地图、标定、失败证据、旧安装及receive-only安装保留。最初GPU未加载等历史状态保存在[初始检查点](exact_mppi_evidence/20261110/initial_checkpoint.md)，不能当当前GPU状态。

上游固定bdc5456938a54267408f2b15870b00ff4e91c58f，`vendor/exact_mppi_review` detached，核心mppi_jax原样复制并附GPLv3全文与逐文件哈希。V2适配器链接该核心，许可及依赖清单见导航包THIRD_PARTY_NOTICES、config/exact_mppi/sources.lock.json和requirements.lock.txt。没有安装上游模拟器或把其V3模式切换器当V2实现。全局搜索是本项目SE(2)格点实现，使用SciPy距离场/KD树，不依赖旧Smac。

新底盘/SDK在ws_base/install_exact静态链接，新导航/消息在ws_nav/install_exact；原安装不覆盖。底盘与SDK新增补丁可在本轮保存的前像上重放并逐字节匹配现源码，见[补丁复现](exact_mppi_evidence/20261110/patch_reproduction.json)。前像SHA和既有补丁链单列锁定，不把新增patch冒充裸上游全量patch。

## 实现结果

| 阶段 | 实现及证据边界 |
| --- | --- |
| P0/P1 | 保存计划、冻结工作树/原值；上游锁与GPL、项目内JAX CUDA、GPU冷/热启动与内存；系统变更由用户完成 |
| P2 | 0.74×0.52m矩形与非零工作外参统一；净空独立配置；旋转扫掠，不再用0.10m规划半径 |
| P3 | FAST原戳云/里程计配对、wheel原戳插值、一次工作外参；高度/自身过滤、分布采样、完整点保留、遮挡/未知及短记忆 |
| P4 | DRIVE/PARALLEL/SPIN预测、反馈停稳/实际模式确认、超时；V2有符号速度与轮向转换，零速保持模式，receive-only不产生控制副作用 |
| P5 | 全轮廓SE(2)搜索，前后弧线、横移/斜行、原地旋转与目标朝向；模式/方向变更分段停稳，动态体素临时占据层及重规划 |
| P6 | 原EXACT优化器/代价/SDF/轨迹验证、模式内候选与最终序列约束、固定256×60×512输入、JIT预热；完整预测及停车尾段独立验证，100ms结果过期 |
| P7 | NavigateToPose反馈/取消/替换/结果，堵路等待及15秒失败、到点反馈停稳；遥控/RC模式与速度同时接管，异常撤销旧代际 |
| P8 | 正式源码入口替换；删除旧Smac/Nav2 MPPI配置/旧BT/旧门禁禁令/平滑器运行链；保留地图服务与定位/公共TF；软件及模拟结果见下 |
| P9 | 全部PARTIAL，未启动设备或实车运动；需按次授权进行方向、模式、制动、场地图与长时验收 |

关键实现路径：`ws_nav/src/ranger_navigation/ranger_navigation/{geometry,models,obstacles,planner,exact_controller,worker,task,coordinator,safety_core,safety_node}.py`；新增Observation/ControlCommand消息；底盘新增v2_command.hpp和显式模式回调/独立反馈时间戳/看门狗。

安全契约：协调器不阻塞等JAX，独立50Hz安全进程最终发布/base/safe_cmd_vel；输出租约100ms、输入默认250ms、模式切换停稳0.5秒及实际反馈，3秒失败。底盘还核对各原始CAN反馈的新鲜度，其他报文不能刷新模式时间。计算进程挂起/退出不阻塞停车。原基座/车体外参、FAST三融合+100ms、公共TF唯一发布者均不变。

## 本轮修复的关键问题

保留全部失败对照日志。早期搜索父节点可变导致路径不连续，改为不可变节点；平移段禁止每步任意改轮向；段间实际停车横向误差超过0.005m时从实际位置重新规划，避免固定转向轴无法消除上一段误差而擦向障碍；加权后序列再次按相同模型投影并验证，禁用验证后的通用平滑。修正V2车体vx与轮速cos转角关系、±90°相反轮向不能以π周期当作相同方向；接收路径无控制订阅/退出指令。

ROS Humble回调不提供所假设MessageInfo、Node.handle保留名、DiagnosticStatus.level需bytes、Action.Result为空等兼容项已修复；goal future按UUID隔离。输出保留整数纳秒原参考戳，实际模式健康读原始MotionState，避免心跳刷新旧模式。缺TF时只选择仍新鲜的同刻可对齐wheel样本，不用latest拼接。

碰撞与观察覆盖分开处理误差：真实点使用净空+体素不确定性+扫掠界限；观察格不重复加入点误差；正式静态图的规划与最终验证一致额外保留0.04m工程余量，避免擦边全局曲线无法被局部模型稳定跟踪。动态体素不再二次栅格化扩大，否则可能拒绝局部刚验证过的当前位置。观察格固定odom轴向/格界，避免随车转动反复重投影侵蚀。规划/优化用当前射线并预留视角变化余量，历史自由证据只用于短时独立末端覆盖；派生并集不刷新原证据有效期。对真实地面/高处回波，恢复其射线在0.10–0.80m高度带中的有效部分，只到高度交点；不能因为端点被过滤就丢弃整条射线。空云、遮挡和缺失扇区均不当自由。

## 验证和性能

最终命令、退出码、统计、日志见下列证据；历史失败日志不删除。单元测试与ROS模拟没有设备/CAN，ROS使用localhost/domain231；真实输入性能测试只读原bag，三融合竞争为有限帧离线直接回调重放/domain230，不发布实车控制。

| 检查 | 最终证据 | 结果 |
| --- | --- | --- |
| SDK/底盘与导航隔离构建 | [build_delivery.log](exact_mppi_evidence/20261110/build_delivery.log) | PASS，4底盘包+4导航包；保留原安装 |
| 几何/模式/障碍/任务/安全及项目相关回归 | [software_regressions_delivery.log](exact_mppi_evidence/20261110/software_regressions_delivery.log) | PASS，94项+25 subtests；CPU离线 |
| GPU全模式闭环 | [closed_loop_actual_mode.json](exact_mppi_evidence/20261110/closed_loop_actual_mode.json) | PASS，9/9；前后/横移/斜行/旋转/左右转/静态障碍/门口，无模拟车体碰撞 |
| ROS任务/故障/独立停车 | [ros_safety_actual_mode.log](exact_mppi_evidence/20261110/ros_safety_actual_mode.log)、[ros_safety.json](exact_mppi_evidence/20261110/ros_safety.json) | PASS，19/19；动态堵路、实际模式不确认、无路、输入断流、TF、定位撤销、急停/遥控/RC、worker挂起/退出 |
| V2转换及生命周期 | [v2_commands_final.log](exact_mppi_evidence/20261110/v2_commands_final.log)、[v2_lifecycle_final.log](exact_mppi_evidence/20261110/v2_lifecycle_final.log) | PASS，21个转换往返/边界及真实回调对mock SDK；没有CAN |
| receive-only/旧反馈回归 | [receive_only_final.log](exact_mppi_evidence/20261110/receive_only_final.log)、[feedback_sign_final.log](exact_mppi_evidence/20261110/feedback_sign_final.log) | PASS，无接管副作用，6组反馈方向 |
| 启动/受管入口dry-run、依赖一致性 | [launch_dry_run_verified.log](exact_mppi_evidence/20261110/launch_dry_run_verified.log)、[session_dry_run_verified.log](exact_mppi_evidence/20261110/session_dry_run_verified.log)、[dependency_consistency.log](exact_mppi_evidence/20261110/dependency_consistency.log) | PASS，未启动导航进程或设备；pip check通过 |
| 原包及三融合资源竞争 | [recorded_contention_height_rays/report.json](exact_mppi_evidence/20261110/recorded_contention_height_rays/report.json) | PASS输入/查询基准；1801严格同戳对中1799云完成处理，前2帧因缺wheel括号拒绝；119输入×3=357查询，150有效、207无有效轨迹而停车 |
| 实车及完整现场实时性 | 未执行 | PARTIAL；不能用上述回放/模拟替代 |

ROS测量从控制tick输入对齐/新云转换开始，包含路径检查、异步进程传递、JAX和独立最终验证：**P50 26.84ms、P95 38.70ms、P99 47.63ms**，满足本轮离线40/50ms目标；控制请求20Hz，100ms以上结果丢弃。此前仅从输入准备后开始的计时日志保留，不能混作同一测量。

GPU独立优化器热调用P95 10.21ms、P99 11.01ms；三模式冷JIT分别2.12/1.84/1.85秒，进程峰值RSS972384KiB。真实记录云与原三融合前端同时运行的优化+完整点/观察覆盖独立验证P95 22.65ms、P99 25.11ms；云过滤/观察计算单列P95 7.20ms、P99 7.93ms。这两个阶段的分位数不能直接相加当作真实ROS整链分位数。

资源竞争进程实际处理600对LiDAR/IMU+图像、600 VIO帧、3592视觉solver更新，7.103秒退出0；357次查询均在该进程存活期间执行。保持正式+100ms三融合，无设备输入。原包SHA、时钟映射、wheel括号、完整命令与前端summary在报告中。150是验证通过的静止状态查询数，**不是导航任务成功数或成功率**；207次拒绝不得记作成功。

紧绕障示例1.5m任务用了94.95模拟秒，反映停稳、换轴和实际位姿重规划的保守耗时，首版路径效率仍有限；不是实车行驶时长。闭环模拟和记录查询分开统计，未测APE/RPE或绝对停车精度。

最终命令及退出码见[command_ledger.json](exact_mppi_evidence/20261110/command_ledger.json)。旧算法删除清单为源码旧nav2_params/BT/motion_gate/obstacle_filter及其专属配置；新install_exact与build_exact中13个旧缓存/入口也已移入本轮备份，见[清单](exact_mppi_evidence/20261110/retired_generated_manifest.json)，原ws_nav/install与ws_base旧安装不动。

## 软件交付与真实验收的区别

软件与模拟覆盖前进、左右转、倒车、横移、斜行、旋转、窄门/静态绕障、临时堵路、目标替换/取消及故障停车。记录点云只证明过滤/时间契约和查询性能，查询中因障碍/覆盖不足被拒绝是停车，不是闭环导航成功。静态记录输入不能证明运动后的传感器观察或V2实际响应。当前障碍/射线为选定高度带的二维投影，未证明完整三维体积可见性；低矮物、细杆、动态遮挡和工作外参误差仍需现场专项验证。

当前真实验收最少待办：先按次授权验证V2实际模式/轮向符号/制动与停稳切换，再做正式地图门口与占用异常复核、低速单点/障碍/堵路/取消；独立补新版首次定位、局部恢复及连续就绪。最后≥30分钟重复任务，记录成功率、模式切换失败、超时、定位撤销和人工接管。无外部真值不声称绝对到点精度。工作外参仍运动估计，制动参数仍工程默认，不放宽定位/未知空间条件换取通过。

本轮文件与实际构建身份见`docs/exact_mppi_evidence/20261110/final_identity.json`；启动和安装默认dry-run。STATUS/HANDOFF/README已同步当前导航架构，原定位生产runtime未切换。


## 可视化补充（2026-11-10）

用户要求查看离线验收。新增[本机交互回放](../outputs/exact-mppi-visualization-20261110/index.html)、[俯视图总览](../outputs/exact-mppi-visualization-20261110/overview.png)及[静态绕障仿真动图](../outputs/exact-mppi-visualization-20261110/simulation_obstacle.gif)。页面完全本地、无网络/ROS/底盘控制接口，支持播放、暂停、时间轴、缩放及9种仿真场景切换。

实录页面使用原pwr5lx7u同录包验收时保存的119快照，先核对derived npz SHA与原extraction.json一致；重新执行同样的静止状态查询以保存原先没有落盘的预测轨迹，50通过/69拒绝。紫色为本次新增的未执行预测，不是原包车辆沿预测导航的证据；完整体素保留显示，白/灰为当前观察覆盖/未知。没有把wheel odom轨迹拼到map坐标冒充真实全局导航。

仿真页面通过新增可选--trace-output导出逐步位置、速度、模式、预测及重规划路径；9个场景全部通过，最终位姿和模拟完成时间与原closed_loop_actual_mode.json逐项一致。绿色为模型执行轨迹；初始路径与实际轨迹、以及原包数据明确分开。新增侧录不修改导航核心，历史验收JSON/原包均未覆盖。

复现命令（均无设备操作）：

```bash
JAX_PLATFORMS=cuda XLA_PYTHON_CLIENT_PREALLOCATE=false .runtime/exact_mppi_venv/bin/python3 scripts/validate_exact_mppi.py --output outputs/exact-mppi-visualization-20261110/simulation_report.json --trace-output outputs/exact-mppi-visualization-20261110/simulation_trace.json
JAX_PLATFORMS=cuda XLA_PYTHON_CLIENT_PREALLOCATE=false .runtime/exact_mppi_venv/bin/python3 scripts/visualize_exact_mppi.py --stage recorded --output outputs/exact-mppi-visualization-20261110
python3 scripts/visualize_exact_mppi.py --stage render --output outputs/exact-mppi-visualization-20261110
```

recorded阶段拒绝覆盖已有导出，复验时使用新输出目录。生成物身份见输出目录visualization_manifest.json；系统Matplotlib3.5.1/Pillow9.0.1用于绘图，项目内JAX仅用于重新查询。未升级依赖。首次GIF渲染用了较新Pillow枚举导致AttributeError，已改用本机兼容常量；失败日志render_final.log保留，最终render_delivery.log退出0。Firefox headless检查与桌面打开日志同目录。大型可视化生成物按.gitignore留本机，原验收源码/资产身份记录继续保留其历史版本。
