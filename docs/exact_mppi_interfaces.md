# EXACT-MPPI 导航接口与复现

本文描述当前导航接口。实际配置见 [项目 YAML](../ws_nav/src/ranger_navigation/config/exact_navigation.yaml)，验收状态见 [STATUS](STATUS.md)。启动示例所需的设备与运动许可按本次任务取得。

## 结构与安装

`ranger_navigation` 包实现 SE(2) 全局搜索、EXACT-MPPI、单点任务和独立安全进程；`ranger_navigation_msgs` 提供原采样时间及代际消息。JAX/全局搜索在独立子进程，协调器异步轮询，安全进程不导入 JAX。没有旧 Smac/Nav2 MPPI/行为树/速度平滑器运行分支。

```text
NavigateToPose → task → SE(2)模式分段路径 → EXACT-MPPI工作进程
                                             ↓ 原采样戳、session/generation
                         仲裁/门禁 → 独立末端扫掠检查 → /base/safe_cmd_vel
FAST同戳云/里程计 + wheel历史 + 唯一公共TF ────────────┘
```

项目内依赖环境 `.runtime/exact_mppi_venv`；隔离安装 `ws_nav/install_exact`、`ws_base/install_exact`。原安装及 `install_recording_rx` 保留。仅以下命令默认 dry-run，不启动设备：

```bash
bash scripts/setup_exact_mppi.sh
bash scripts/build_exact_navigation.sh
bash scripts/run_navigation.sh
source ws_base/install_exact/setup.bash
source ws_nav/install_exact/setup.bash
ros2 launch ranger_navigation navigation.launch.py
```

依赖/构建脚本加 `--execute` 才安装项目内 Python 包或构建软件。驱动脚本是另一个系统操作入口，本轮用户已自行完成安装和 MOK 登记，不需要再次执行。

正式导航启动由受管入口执行；需要正在运行且同地图的 FAST 公共定位、映射到传感器时钟的 wheel、地图及 TF，不自动开传感器或底盘寻找初值。直接 launch 还需 `execute:=true control_authorized:=true`；缺少本次设备/控制授权时不要执行。`start_map_server:=false` 默认复用地图；如显式开启只启动地图服务及其生命周期管理。默认 `use_sim_time:=true` 适配当前会话传感器时钟；不能把 host wheel 历元直接接到该接口。

## ROS 接口

| 名称 | 类型/含义 |
| --- | --- |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose`；目标 frame 必须 map，有限位置及有效四元数，behavior_tree 必须为空 |
| `/fast_livo/odometry`、`/fast_livo/deskewed_imu` | 严格整数纳秒同戳；fast_livo_local→fast_livo_imu 里程计、单区间 IMU 参考点机体系云 |
| `/fast_livo/navigation/wheel/odom` | 默认 wheel 输入，可通过 wheel_topic 参数指定；odom→base_link，同会话映射采样时间 |
| `/map` | 占据图，未知禁止通过；数组、分辨率、原点核对绑定地图资产 |
| `/localization/estimate`、`/navigation_ready` | 标准定位及就绪，原位姿采样时间有效；地图 UUID 必须与启动绑定值一致 |
| `/motion_state`、`/system_state` | 原始底盘模式/状态反馈，各自采样新鲜度独立；请求受理不能刷新模式反馈 |
| `/navigation/control` | `LeasedControlCommand`，原参考采样戳、会话、控制代际、request_id、固定steady到期时刻、明确模式与速度 |
| `/cmd_vel_nav` | 保留导航 Twist 观察接口；底盘不能绕过门禁订阅此话题 |
| `/base/safe_cmd_vel` | 独立安全进程的最终 Twist；后面没有通用阿克曼裁剪或平滑器 |
| `/base/motion_mode_request` | UInt8，0双阿克曼行驶、1固定转向平移、2原地旋转；请求不代表实际完成 |
| `/teleop/control` | 遥控 ControlCommand，速度和模式一起转交；旧 `/teleop/cmd_vel` 必须伴随新鲜 `/teleop/motion_mode` |
| `/ranger/motion_enable`、`/ranger/soft_estop` | Bool，启动默认未使能；独立门禁检查，不自动恢复跨会话运动许可 |
| `/plan`、`/navigation/prediction` | Path，map坐标全局路径与已验证局部预测，保留原参考采样戳 |
| `/navigation/observation` | Observation，odom障碍全量体素、观察网格在odom的位姿、180扇区距离、80×80观察网格 |
| `/navigation/state` | JSON String：状态、停车/失败原因、控制与历史代际、输入健康、worker状态、请求耗时、历史复用及执行链观察 |
| `/navigation/execution` | String JSON：请求、门禁放行、末端防撞后速度与测量速度；含请求编号、原采样戳及原steady截止时刻 |
| `/base/execution_state` | String JSON：驱动会话、机体请求、SDK提交三元组、转向/模式/反馈拒绝/看门狗原因及各原反馈年龄；SDK提交不等于CAN送达或实测速度 |
| `/navigation/minimum_distance` | Float32，已验证预测轨迹至实际矩形的最小点距离 |
| `/navigation/control_owner`、`/navigation/actual_mode` | 当前控制权和模式观察值；协调器新鲜度依据原始 MotionState，不依据心跳 |
| `/navigation/safety_ready`、`/ranger/motion_permitted`、`/diagnostics` | 独立输入健康、运动许可及末端拦截原因 |

接受目标后提供当前位姿/剩余距离反馈；Humble NavigateToPose.Result 为空，成功/取消/失败使用标准 action 状态，详细原因读 `/navigation/state`。新目标替换旧目标并撤销旧输出；取消不继续执行旧结果。任务含规划、停稳切换、执行、末端停稳、等待、堵塞及终态。持续堵路15秒无可行路线失败；没有盲目脱困行为。

## 几何、模式和控制契约

车体和工作外参读取 [navigation_prep.yaml](../config/fast_livo2/navigation_prep.yaml)。轮廓是 0.74 × 0.52 m 的矩形，`base_link` 为轴间中点的地面投影。

`clearance_profile:=standard|narrow` 从项目配置选择实时点障碍的物理净空、观察代价预留及航向容差。受管入口可用 `RANGER_CLEARANCE_PROFILE=narrow`。窄档优先固定转向平移，仍支持行驶、倒车与旋转。实际停稳位姿不符合下一行驶或平移段起点约束时，从实际位置重新规划。

静态占据图使用完整单元 SAT 检查及 0.015 m 的扫掠界限。两档总静态裕量由 `static_map_clearance` 加扫掠界限得到。动态障碍使用体素中心和矩形 SDF，裕量由物理净空、0.04 m 体素误差和 0.015 m 扫掠界限组成。实时观测覆盖由独立的 `coverage` 参数控制。

行驶模式只允许满足最小转弯半径的 vx/wz；平移模式允许倒车、纯横移、斜行，但每段转向轴固定，转向方向改变须停稳；旋转模式只有 wz。上一段停稳后若实际位置对下一平移段有超过0.005m横向偏差，先从实际位置重新规划，再选择固定转向轴，不能平移旧的已验证路径来补误差。候选、加权控制序列和最终验证都使用相同模式模型。优化时域末段按速度和加速度限制留出制动时间，发布命令来自已验证序列，不在优化后投影成另一运动。

模式切换先用反馈确认停稳连续0.5秒，再请求并等待实际模式反馈；3秒超时失败停车。V2驱动另核对模式/系统/运动各自反馈新鲜度及模式请求租约；平移转向先零速定位，反馈转向方向确认后才发非零速度。2π方向一致性避免把相反轮向当相同有符号速度。零速保持模式/转角；接收专用路径不订模式/速度、不请求控制、不在退出时发零速。

正式入口从项目 YAML 的 `motion` 读取限速和停稳参数。显式 `low_speed:=true` 时采用 `real_vehicle_first_test` 的限速。默认终点容差 0.15 m / 0.15 rad，并要求反馈连续停稳。行驶模式的 `min_radius` 应满足 V2 驱动的转角约束。实际限速见 `/navigation/state.control_limits`。

## 采样时间、障碍与失效

每帧云只应用一次 IMU→base 工作外参，再使用该云原戳的 wheel 历史插值变换到 odom。不用 latest TF 或缺口外推。map→odom查询使用原参考 wheel 采样时间；输出与预测沿用整数纳秒原戳，心跳不刷新命令样本。定位原采样和接收时限分别读取 `timing` 段，wheel、障碍和反馈各自检查。结果/命令租约为 0.10 秒；计算阻塞或进程退出时独立安全链撤销输出。

高度筛选0.10–0.80m、去自身、8m范围、0.05m体素；扇区/径向分布采样最多512点用于优化，全量保留点用于独立扫掠。障碍最多保留2秒，只有更远有效观测才能提前清除；空云/没有穿过高度带的有效射线不是自由空间；地面/高处回波的真实射线只观察到其与高度带的交点，不能越过该处清除。未观测扇区未知，邻扇区一致才可清除。自由观察证据按原始帧时间保留≤2秒，最多32帧，网格固定odom轴向并按0.05m平移，避免随车旋转重栅格化侵蚀自由区；历史单元按整数格位移合并，不用派生并集刷新记忆。局部规划和独立末端检查按配置复用有界的短时观测证据，遮挡和保留期仍参与检查。80×80、0.05m的局部观察网格覆盖采样基座附近±2m；实际射线原点使用配置的 LiDAR 安装位置。

全局未知格禁止穿越。局部路径前段、完整预测和停车尾段使用同一覆盖策略。实时总外扩读取 `coverage.total_margin`，边界和保护范围内部未知格分别使用 `unknown_cell_budget` 与 `protected_unknown_cell_budget`。开启 `allow_boundary_unknowns` 时，只有完整落入保护范围的格计入内部。关闭时，所有触及保护范围的格计入内部。已知障碍与越界直接拒绝，未知格保留原值。标准和窄档的观察代价预留分别为 0.10 m 与 0.04 m，与覆盖硬门槛分开。网页从 `/navigation/state.coverage_diagnostic` 读取原因和格数，worker 回报实际 `coverage_policy`。

前端/定位会话切换、定位跳变、wheel/ROS时间回退、遥控接管都撤销旧轨迹和工作进程代际。worker每个结果匹配进程nonce、任务代际、请求ID和steady deadline；跨目标迟到结果丢弃。公共TF仍只有既有桥接发布，本导航不发布map→odom或odom→base_link。


## 短暂失效恢复与执行历史

控制`generation`撤销过期/跨暂停结果；独立`history_generation`只在新目标、重新规划或硬失效时更新。暂停不直接销毁候选，但只有协调器最终几何验证确认的request_id能成为历史；新结果仍须属于当前控制代际。历史最多保留1秒，仅用作求解初值；推进不超过原采样时间差及实测位姿进展，驱动对轮角等待不推进，反馈缺失/过期、位姿偏离或换路线时不用历史。重新按当前反馈速度、当前运动段投影，再做全部约束/碰撞验证。

输入失效时撤销输出。恢复后，同一实际模式、在原路线及原转向轴上、速度符合模式的车辆可从实测余速重新求解，不强制先刹到零。模式/转向轴改变仍需停稳。3秒无进展先撤销旧结果并重验当前执行，不单凭此触发全局重搜；真实路径受阻仍重规划，15秒持续无进展仍失败。明确驱动对轮角等待单独累计有效等待，3秒超时失败，不冒充位移进展。

自主消息改用独立的`LeasedControlCommand`，保留原`ControlCommand`用于旧包解码及遥控接口。协调器、独立门禁和消息包必须一起重建；旧消息不能驱动新自主入口。非零指令的`valid_until_steady`固定为原请求提交时间+100ms，重复发布不续租；零速/模式请求可发布新的零速租约。steady来自同机本次boot，不是传感器时间；原采样戳仍单独检查。驱动会话更换后门禁锁定，必须先收到motion_enable=false、再true才重新使能，连续true心跳不会恢复运动。

执行观察数据不替代原wheel/定位/TF及安全检查。门禁报告和驱动报告保留各自时刻，SDK指令与request_id通过记录时间/速度链核对，不声称CAN逐包ACK或物理执行已确认。录包入口额外收录两条执行话题和消息定义。软件验收与真实边界见[恢复优化报告](exact_mppi_control_recovery_20261117.md)。


## 工作台与规划重置

`/navigation/route`（std_msgs/String JSON），在接受新全局计划时发布；不参与控制、许可或规划决策。schema_version=1，frame=map，携带 map_uuid、history_generation、原 sample_stamp_ns 和 segments（mode、reverse、poses[x,y,yaw]）。浏览器只有在路线历史代际与当前导航状态一致时按行驶/平移/旋转着色；旧包无该消息时使用原 `/plan`，不推断未记录的运动模式。录包入口追加该话题，旧话题保持兼容。

工作台通过原 NavigateToPose 和 motion_enable 进行自有任务管理；不提供速度或模式直通接口。连续定位由显式 `--continuous` 移除有限 recorder，保留原 CLI 默认有限采集行为。使用和验收见 [工作台说明](../workbench/README.md)、[工作台报告](navigation_workbench_report_20261117.md)。

`/navigation/reset`（std_srvs/Trigger）要求先取消活动 Action。服务清理旧目标、路线和历史，并重建独立计算子进程。工作台等待新 generation、worker nonce、READY 和 IDLE，定位、传感器及现有记录继续运行。受管健康检查允许 35 秒显式重置预热。任务和终态原因通过 `/navigation/state` 的 `failure_cause` 与 `planning_failure` 查询。新会话加载 YAML，重置沿用会话的配置。
