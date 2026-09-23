# 阶段 5：Lightning-LM 地图定位与 Nav2 接口适配

更新：2026-09-23。实物：Ranger Mini V2 + Mid-360S（车顶中间、距车尾约 0.25 m、离地约 0.60 m、装平，用户约值）。范围为定位与导航接口，**不启动自主导航**。

## 结论

**PARTIAL（管线与接口完成并经合成验证；真实冷启动定位测试待人工执行）。** 离线定位自测（同源建图包）通过：FP 初始化成功、73 次 NDT 匹配全部成功（另有一行配置回显曾被误计为 74）、置信度均值约 3.0（阈值 1.8）、终点位姿与建图轨迹终点一致（平移差 ≈0.41 m vs 建图闭合差 0.45–0.49 m、航向 −3.7° vs −4.3°/−4.7°）。桥接节点 `ranger_localization_bridge` 合成测试 v2 **11/11** 通过（经对抗性审查后重写，见第 4 节）。**冷启动（车实际在场、另一段数据）验证待用户执行，标记 PENDING。**

| 项 | 结果 | 证据 |
|---|---|---|
| Lightning 定位源码接口审计 | 完成（见第 1 节；无 /initialpose、无重定位服务、轮速未融合） | 本文第 1 节，源码行号 |
| 离线定位自测（同源包） | PASS（限定自测范围） | [loc-offline-6ggxi299](05_evidence/loc-offline-6ggxi299/) |
| 桥接节点合成测试 | PASS 11/11（v2；9 轮复跑 8 轮全绿、1 轮 10/11 复跑通过） | [test_stage5_bridge.py](../tests/test_stage5_bridge.py) |
| TF 发布者唯一性与语义 | 已实施（第 3 节清单） | 第 3 节 |
| map→odom 计算 | 同时刻同参考点公式，已实现并测试 | bridge_node.py |
| navigation_ready 异常状态 | 已实现（TF 新鲜度/状态/置信度/innov/odom 五类检查） | bridge_node.py |
| 地图 3D/2D 一致性 | 同坐标系构造 + 数值比对（中位 0.047 m） | 第 5 节 |
| **真实冷启动定位（在场）** | **静止部分 PASS（2026-09-23 现场实测）**；运动短距验证未做（可选强化） | 第 7 节 |
| **非建图包的独立数据验证** | **PENDING（需新录包）** | 第 7 节 |

## 1. Lightning-LM 定位实际接口审计（固定提交 1325fed8）

以下全部来自源码阅读，**不虚构任何接口**。

### 1.1 输入（在线，[loc_system.cc:39](/home/lab307/ranger_nav/ws_slam/src/lightning/src/core/system/loc_system.cc:39)）

| 话题 | 类型 | QoS | 备注 |
|---|---|---|---|
| `/livox/imu`（YAML `common.imu_topic`） | sensor_msgs/Imu | QoS(10) reliable/volatile | IMU 原始 g/rad/s |
| `/livox/lidar`（YAML `common.livox_lidar_topic`） | livox CustomMsg | 同上 | 主点云输入 |
| PointCloud2 话题（YAML `common.lidar_topic`） | PointCloud2 | 同上 | 我们指向 `/ranger/unused_pointcloud2` 防误接 |
| `/wheel/odom`（底盘） | — | — | **未接入**：`ProcessOdomMsg` 整段被注释（[localization.cpp:299](/home/lab307/ranger_nav/ws_slam/src/lightning/src/core/localization/localization.cpp:299)）。Lightning 未融合轮速；/wheel/odom 仅作为桥接的独立观测 |

### 1.2 输出

- **TF `map` → child**（[localization_result.cc:10](/home/lab307/ranger_nav/ws_slam/src/lightning/src/core/localization/localization_result.cc:10)）：`child_frame_id` 上游硬编码 `"base_link"`，**但位姿实际是传感器（IMU）位姿**（源码注释自认 Til=I 时把 Twl 当 Twi；地图分块即 IMU 系点云）。这是"雷达位姿改 frame_id 当底盘位姿"的上游缺陷，本阶段补丁已允许显式更名（我们用 `imu_link`）。
- **频率/时间戳**：PGO 高频输出在每次 IMU（约 200 Hz）与 LO/定位事件时触发（[pgo.cc:169,215](/home/lab307/ranger_nav/ws_slam/src/lightning/src/core/localization/pose_graph/pgo.cc:169)）；TF 时间戳 = 结果时间戳（传感器时间外推值）。**仅在 `valid_`（首次 PGO 求解成功）后发布**；初始化期间无 TF。
- **无话题输出**（原提交）：状态/置信度回调被注释。本阶段补丁新增 `/lightning/loc_status`（std_msgs/Int32，LocalizationStatus 枚举 0=IDLE/1=INITIALIZING/2=GOOD/3=FOLLOWING_DR/4=FAIL）与 `/lightning/loc_confidence`（std_msgs/Float32，NDT `getTransformationProbability()`）。
- **无协方差输出**；速度仅内部使用（UI NavState）。**禁止**对 TF 位姿差分取控制器速度。
- 平滑：PGO 输出经 PoseSmoother（[pgo.cc:110](/home/lab307/ranger_nav/ws_slam/src/lightning/src/core/localization/pose_graph/pgo.cc:110)），全局校正在图优化中吸收，输出位姿含小幅修正。

### 1.3 初始化机制（[lidar_loc.cc:479-538](/home/lab307/ranger_nav/ws_slam/src/lightning/src/core/localization/lidar_loc/lidar_loc.cc:479)）

| 机制 | 状态 |
|---|---|
| 依赖建图起点 | **是**。`init_with_fp=true` 遍历地图 `index.txt` 的功能点；我们的地图仅有一个 `start`（建图首帧位姿）。**无任意位置全局搜索**；YawSearch 代码存在但其调用被注释（[lidar_loc.cc:296](/home/lab307/ranger_nav/ws_slam/src/lightning/src/core/localization/lidar_loc/lidar_loc.cc:296)），起点朝向需大致一致 |
| 外部初始位姿 | 仅 C++ API `SetInitialPose`（一次性，[lidar_loc.cc:430](/home/lab307/ranger_nav/ws_slam/src/lightning/src/core/localization/lidar_loc/lidar_loc.cc:430)）；`run_loc_online` 原版固定单位阵（=建图起点同朝向）。本阶段补丁增加 gflags `--init_x/y/z --init_qx/qy/qz/qw` |
| `/initialpose`、重定位服务 | **不存在**，未编造 |
| 初始化质量门槛缺陷 | `Localize()` 成功判定两个分支均为 true（[lidar_loc.cc:853-857](/home/lab307/ranger_nav/ws_slam/src/lightning/src/core/localization/lidar_loc/lidar_loc.cc:853)），`min_init_confidence=1.8` 实际不拦截初始化。**补救**：桥接节点用 `/lightning/loc_confidence` 自设门槛 |
| 状态机 | `match_fail_count_` 因上述缺陷实际不会增长，FOLLOWING_DR/FAIL 基本不可达；PGO 有 `localization_unusual_tag_`（置信度 <0.5 连续 10 次）但原提交无人消费。**补救**：桥接自设置信度门槛 + innovation 检查 |

第一版方案（用户允许）：**人工确认的固定启动点** = 把车放到建图起点、朝向与建图起步一致，初始位姿取单位阵（默认）。

### 1.4 地图交互

- TiledMap 从 `index.txt` 载入分块与 FP（[tiled_map.cc:120](/home/lab307/ranger_nav/ws_slam/src/lightning/src/core/maps/tiled_map.cc:120)）；NDT 目标体素 1.0 m（粗配 5.0 m）；按位姿加载邻近分块（load 2/unload 4）。
- **回写风险**：`LidarLoc::Finish()` 在 PERSISTENT+`save_dyn_when_quit=true` 时会 `SaveToBin` 回写地图目录，且 `has_set_pose_` 永不置位（[lidar_loc.cc:983-994](/home/lab307/ranger_nav/ws_slam/src/lightning/src/core/localization/lidar_loc/lidar_loc.cc:983)）。**定位配置强制 `save_dyn_when_quit: false`，运行入口脚本校验**，保护已验收地图。
- 匹配输入：LIO 去畸变后 IMU 系点云（`GetProjCloud`，loc_on_kf=true 时每关键帧 0.5 m/10° 一次匹配）。

## 2. 底盘轮速里程计（ranger_ros2 实测源码）

`/wheel/odom`：nav_msgs/Odometry，`header.frame_id=odom`、`child_frame_id=base_link`，约 50 Hz，平面运动学积分位姿（z=0、四元数仅 yaw）+ 车体速度（来自底盘反馈，非位姿差分）；`publish_odom_tf` 默认 false（阶段 2 实测关闭）——因此 **odom→base_link TF 由桥接节点唯一发布**。见 [ranger_messenger.cpp:360-397](/home/lab307/ranger_nav/ws_base/src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:360)。

## 3. TF 发布者清单（唯一发布者，v1）

| TF | 发布者 | 频率 | 时间戳 | 语义 |
|---|---|---|---|---|
| `map` → `imu_link` | Lightning `run_loc_online`（补丁更名 child） | 随 IMU/定位事件（≤200 Hz） | 传感器时间（外推） | **原始 6DoF 传感器位姿**（含真实 roll/pitch，供地形处理） |
| `odom` → `base_link` | **ranger_localization_bridge（唯一）** | 50 Hz（随 /wheel/odom） | odom header 时间戳 | 连续局部里程计（平面） |
| `map` → `odom` | **ranger_localization_bridge（唯一）** | 50 Hz | 同一次 odom 时间戳 | 全局校正吸收项：`T_map_odom = T_map_base × inverse(T_odom_base)`，同一次回调内同车体参考点（base_link） |

- `T_map_base = T_map_imu × inverse(T_base_imu)`；`T_base_imu` 为**近似外参**（用户约值：x/y 未精测暂取 0、z=0.556 m 按 base_link 位于地面假设、rpy=0 装平），已在参数与日志中显式声明 APPROXIMATE，待实测后替换。
- **不使用 base_footprint**（v1）：如需需由 base_link 沿真实 roll/pitch 投影到地面计算，禁止固定写 z=0/俯仰零；当前坡道场景未到，暂不引入。
- 全局校正跳变进入 map→odom（odom→base_link 保持连续），控制器速度取 `/wheel/odom.twist`（独立观测），满足全部禁止条款。
- Lightning 与桥接各发布不同 child 的 `map` 分支（`map→imu_link` 与 `map→odom→base_link`），TF 树无冲突、无双重发布。

## 4. ranger_localization_bridge（[ws_nav/src/ranger_localization_bridge](../ws_nav/src/ranger_localization_bridge/ranger_localization_bridge/bridge_node.py)）

- 输入：TF（map→imu_link，取**最新**并检查其自身时间戳年龄 ≤`tf_max_age_s` 0.5 s——同刻精确查询会因定位管线 0.1–0.2 s 延迟大量失败；输出 TF 仍用 odom 时间戳，不掩盖延迟）、`/wheel/odom`、`/lightning/loc_status`、`/lightning/loc_confidence`。
- `navigation_ready`（std_msgs/Bool，另有 `/localization_health` DiagnosticArray 逐项）= 全部满足：loc TF 存在且新鲜、odom 在流且新鲜、status==GOOD 且 ≤1 s、confidence≥1.0 且 ≤1 s、innovation 通过。**节点活着本身不构成 ready**。
- **看门狗**（对抗审查后新增）：0.2 s 定时器独立重评估——轮速断流后 ready 必须翻转（否则最后一次 True 会永久滞留），诊断含 `odom_present`/`innovation_skipped` 键；轮速缺失=not ready（原因显式标注），不是"跳过检查的降级通过"。
- innovation：loc 与轮速在 ~0.5 s 间隔上的相对运动一致性，判据为位移差 ≤0.5 m **且** 平均速率差 ≤0.3 m/s **且** 航向差 ≤20°（yaw 判据为对抗审查后补上：原地旋转时位移差为 0，无 yaw 判据则航向发散不可见；常数航向偏置由 map→odom 合法吸收，不属失败）。连续 3 次失败才置 not-ready，单帧噪声不触发。
- 时间戳：全部用数据自身时间戳；未打"现在"戳。
- 合成测试 **11/11（v2 重写）**：SE3 数学往返、map→odom 恒等式、零四元数防护、ready 稳定置真、**TF 数值 vs 真值比对**（非仅存在性）、**纯 TF 停发→loc_tf 门禁归因成立**（status/conf 保持流）、**持续 +1 m/s 速度分歧→3 s 窗口内 ready 从未回真且 innovation 归因**、恢复、**轮速断流→看门狗翻转且 odom_present=0**。跨进程运行；每次相位切换前先沉淀到稳定 ready 基线，杜绝残留态误过。v1 的 8 项检查经对抗审查证实多项空转（jump 检查可被任意满足、TF 仅查存在性、stall 归因错误），已全部重写并用破坏性实验验证区分力。

## 5. 地图一致性（3D 分块 vs Nav2 2D 栅格）

- **同源同坐标系**：`map.pgm`/`map.yaml` 由 g2p5 在 SLAM 世界系内生成（同一实验目录 `lightning-offline-suej7qsz`），定位加载的 3D 分块与 Nav2 的 2D 图共享同一 `map` 原点与轴向（pgm origin(−5.15,−14.95,0)，无旋转）。
- **尺度/分辨率**：同为米制；pgm 0.05 m/格（map.yaml 实测值，阶段 4 补丁保证写实际分辨率）。
- **数值比对**（阶段 4 已做，此处复核引用）：占据格与 3D 地图按同障碍带（离地 0.10–0.80 m、地面 z=−0.556）投影的最近距离中位数 0.047 m（≈1 格）；优化轨迹 49 关键帧栅格全部自由。
- 结论：原点、方向、尺度、分辨率一致，**无需**额外配准变换。

## 6. 本地修改与新增

- [stage5_loc_interface.patch](../patches/stage5_loc_interface.patch)：① `run_loc_online` 增加 `--init_*`（固定启动位姿）、`--pub_tf`、`--tf_child`，**Init 失败即退出 1**（原版仅打日志继续，析构时空指针崩溃）；② LocSystem TF child 可更名；③ 新增 `/lightning/loc_status`、`/lightning/loc_confidence` 发布（启用被注释的回调）；④ `Localization::Finish()` 空指针保护（Init 失败路径不再崩溃）。编译通过（上游警告照旧），二进制与库已更新并验证含新符号。
- 新增：`config/slam/ranger_mid360s_loc.yaml`（定位配置；`save_dyn_when_quit: false`、地图指向已验收 `suej7qsz`）、`scripts/run_loc_offline.py/.sh`（入口校验+证据目录；首次运行缺 slam_env 退出 127 的失败证据 [loc-offline-mfrsk5zz](05_evidence/loc-offline-mfrsk5zz/) 保留）、`ws_nav/src/ranger_localization_bridge/`（首个 ws_nav 包，构建通过）、`tests/test_stage5_bridge.py`（11/11）。
- 失败与修正记录：桥接首版 `quat_rotate` 构造纯四元数用错分量（数学测试抓出，已修）；创新性判据从单帧绝对阈值改为 0.5 s 间隔位移+速率+yaw 三判据（对抗审查+合成测试驱动）；单进程 DDS 发现不可靠改跨进程测试；`/navigation_ready` 无看门狗致轮速断流滞留 True（对抗审查 major，已加 0.2 s 看门狗）；零四元数输入崩溃防护。v1 测试多项空转（对抗审查以破坏性实验证实），v2 全部重写。

## 7. 现场冷启动结果（2026-09-23 晚）与剩余待验

**真实冷启动定位（静止部分）实测 PASS**：

- 现场插曲（如实记录）：USB-CAN 适配器未连接导致 can0 不存在、底盘 launch 预检拒绝启动（护栏生效）；重插适配器 + 用户执行 500 kbps 配置后恢复。
- 车摆位与建图起点偏差约 0.43 m / −10°，**FP 初始化仍收敛**：`/lightning/loc_confidence`=3.04 @ 200 Hz（门槛 1.0）、`/lightning/loc_status`=2(GOOD)、TF `map→imu_link` 持续输出；`/wheel/odom` 50 Hz。
- 桥接启动后 **`/navigation_ready`=true**（全部门禁通过）；TF `map→base_link` z=−0.53 m 与外参换算一致（IMU 0.027 − 安装高 0.556，base_link 按地面假设），航向 −9.8° 与摆位一致。
- Foxglove 经 foxglove_bridge 查看二维地图（`scripts/publish_loc_map.py` 发布 latched `/map`）+ TF + 实时点云，用户确认可视化正常。
- 会话结束方式：雷达驱动 `--seconds 600` 到时自动退出→定位/底盘随后停止；**桥接新看门狗当场把 ready 翻转为 false**（此前版本会滞留 true）——看门狗获得现场验证。
- **未做**：低速遥控运动下的 TF 连续性与 ready 保持（可选强化项，运动许可按 AGENTS.md 现场另取）。

**剩余 PENDING**：

- **PENDING-2 独立数据验证**：录一段**新**的运动包（不同起点或不同路径），先 `run_loc_offline` 验证定位，再做在线。建图原包自测（loc-offline-6ggxi299）不作为独立验证依据。
- **待测参数**：`T_base_imu` 精测（当前近似值会引入 cm 级 base_link 偏移与未知 x/y 偏置；Nav2 footprint 需加余量或精测后收紧）。

**下次现场恢复流程**（终端 A/B/C 三条命令见第 8 节；桥接/地图发布/派生点云/静态 TF 由助手侧脚本常驻）。

## 8. 命令与退出记录

| 命令 | 退出 | 说明 |
|---|---|---|
| `build_lightning.sh --execute`（补丁后） | 0 | 有上游警告；新符号已验证在库与二进制中 |
| `run_loc_offline`（mfrsk5zz，缺环境） | 127 | libpango 缺 LD_LIBRARY_PATH；证据保留，已加 .sh 包装 |
| `run_loc_offline`（6ggxi299，同源自测） | 0 | FP init 成功、73 匹配全成功、conf≈3.0 |
| 现场冷启动（2026-09-23 晚，静止） | PASS | conf 3.04/GOOD/ready=true/TF 一致；运动部分未做；细节见第 7 节 |
| `tests/test_stage5_bridge.py`（v2） | 0（11/11；修复后多次复跑全绿） | 跨进程；数学、TF 数值、纯门禁归因、看门狗 |
| `colcon build ws_nav bridge` | 0 | 误建于根目录的 build/install/log 已删除并重建于 ws_nav |
