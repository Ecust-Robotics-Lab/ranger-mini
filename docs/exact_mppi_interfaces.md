# EXACT-MPPI 导航接口与复现

2026-11-17后续边界/速度更新：默认`coverage.allow_boundary_unknowns=true`，部分相交保护边界的未知格与外缘格共用unknown_cell_budget总额度，完整落入保护矩形的未知格仍拒绝；false恢复旧“任意保护相交必拒”。不改格值，不按前后半区豁免；2.5cm/点障碍检查保持。正式low_speed默认false，默认上限读取YAML motion.max_speed/max_yaw_rate，显式true用real_vehicle_first_test。`/navigation/state.control_limits`回报实际上限，网页显示。见[报告](workbench_boundary_speed_report_20261117.md)；本段覆盖下方旧边界规则及默认低速快照。

2026-11-17覆盖口径更新：静态地图总2.5cm保持；实时观测覆盖也统一总2.5cm（已含扫掠），不叠加body.clearance。`coverage={total_margin:0.025,protected_margin:0.015,unknown_cell_budget:8}`同时传至协调器/GPU/独立验证/末端门禁。完整车体+1.5cm必须观测，仅最外1cm带容忍≤8未知格，已知障碍/越界不豁免，不改格值。点障碍净空/体素误差单独保持。`/navigation/state.coverage_diagnostic`包含current/blocked/checked_distance_m及具体原因/格数/米单位外扩；worker状态回报coverage_policy。见[报告](workbench_coverage_policy_report_20261117.md)，覆盖相关历史“净空+1.5cm”被本段替代。

2026-11-17新增`/navigation/reset`（std_srvs/Trigger）：调用者须先取消自有活动Action；服务拒绝活动目标。接受后清除目标/路线/确认历史并重建独立计算子进程，发布RESETTING及reset_started_steady，完成后IDLE。工作台等待新generation/worker nonce且READY后确认；不改定位/观测，不使能运动，不自动重发目标。受管计算健康仅对35秒内显式重置保留初始化条件，其他健康/权限门限不变。见[报告](workbench_navigation_reset_report_20261117.md)。

更新：2026-11-10。软件实施范围、实测与剩余限制见[报告](exact_mppi_implementation_report_20261110.md)，设计基准见[计划](EXACT_MPPI_NAVIGATION_PLAN.md)。本文件中的启动示例不构成设备或运动许可。

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

唯一车体/工作外参来源 `config/fast_livo2/navigation_prep.yaml`，0.74×0.52m矩形，base_link轴间中点地面投影，保留现有非零安装外参。净空单独配置于导航包 `config/exact_navigation.yaml` 的 geometry.clearance（默认0.04m）；launch传给规划/控制/过滤/末端检查，不通过缩小车体制造通行能力。

实车准备新增同一算法的 `clearance_profile:=standard|narrow`，或受管入口 `RANGER_CLEARANCE_PROFILE=narrow`；实际launch从 `clearance_profiles` 统一取值，geometry中的标准值保留兼容离线旧入口。standard维持0.04m净空/0.04m静态图额外余量/0.10m观察空间预留；narrow取0.03/0.01/0.04m，静态图单侧总裕量含0.015m扫掠界限从0.095m降至0.055m。动态体素误差项仍0.04m；最终独立验证分别读取静态图余量与点云余量。优化器障碍代价及最终轨迹验证器现在也含0.015m扫掠项，防止候选在较小裕量下被独立检查反复撤销。

窄档在静态空间不足以宽松转向时提高行驶弧线代价，优先固定转向平移，行驶/倒车/旋转仍可搜索；不是对优化后速度做裁剪。窄档段间朝向判据0.03rad，标准仍0.06rad。行驶/平移段停稳后的朝向或必要起点偏差超过0.005约束时先从实际反馈重规划；旋转用当前真实旋转中心预测并独立验证。65cm通道的静态检查、完整闭环与现场门口的待确认项见[实车准备](EXACT_MPPI_FIELD_TEST.md)。这些参数不等于实车最窄通行宽度的保证。

占据图完整单元 SAT + 扫掠界限；动态层保留体素中心并做矩形SDF检查，避免再次栅格化产生重复误差。点障碍裕量为净空+0.04m体素误差+0.015m扫掠界限；观察单元只用净空+0.015m；正式静态占据图在规划与最终验证中另保留0.04m工程余量（总计净空+0.04m+0.015m），避免全局路径擦着局部可执行边界。独立 NumPy 检查与 JAX 代价分别实现。

行驶模式只允许满足最小转弯半径的 vx/wz；平移模式允许倒车、纯横移、斜行，但每段转向轴固定，转向方向改变须停稳；旋转模式只有 wz。上一段停稳后若实际位置对下一平移段有超过0.005m横向偏差，先从实际位置重新规划，再选择固定转向轴，不能平移旧的已验证路径来补误差。候选、加权控制序列和最终验证都使用相同模式模型。最后20步包含受约束制动，发布命令来自已验证序列，不在优化后投影成另一运动。

模式切换先用反馈确认停稳连续0.5秒，再请求并等待实际模式反馈；3秒超时失败停车。V2驱动另核对模式/系统/运动各自反馈新鲜度及模式请求租约；平移转向先零速定位，反馈转向方向确认后才发非零速度。2π方向一致性避免把相反轮向当相同有符号速度。零速保持模式/转角；接收专用路径不订模式/速度、不请求控制、不在退出时发零速。

默认离线最大平移0.30m/s、旋转0.50rad/s，加速度0.30m/s²、角加速度0.50rad/s²。正式launch默认低速0.10m/s、0.20rad/s；这是待实测工程限制，不是制动距离保证。最终到点误差≤0.15m/0.15rad且反馈停稳连续0.5秒；中间段采用更紧容差。估计位姿误差不能称为绝对停车精度。

## 采样时间、障碍与失效

每帧云只应用一次 IMU→base 工作外参，再使用该云原戳的 wheel 历史插值变换到 odom。不用 latest TF 或缺口外推。map→odom查询使用原参考 wheel 采样时间；输出与预测沿用整数纳秒原戳，心跳不刷新命令样本。采样与接收年龄都检查（默认0.25秒），结果/命令租约0.10秒；计算阻塞或进程退出时独立安全链撤销输出。

高度筛选0.10–0.80m、去自身、8m范围、0.05m体素；扇区/径向分布采样最多512点用于优化，全量保留点用于独立扫掠。障碍最多保留2秒，只有更远有效观测才能提前清除；空云/没有穿过高度带的有效射线不是自由空间；地面/高处回波的真实射线只观察到其与高度带的交点，不能越过该处清除。未观测扇区未知，邻扇区一致才可清除。自由观察证据按原始帧时间保留≤2秒，最多32帧，网格固定odom轴向并按0.05m平移，避免随车旋转重栅格化侵蚀自由区；历史单元按整数格位移合并，不用派生并集刷新记忆。规划与优化仅使用当前射线覆盖，避免进入将在制动时域内过期的遮挡区；独立末端检查保留短时真实观察证据。80×80、0.05m的局部观察网格覆盖采样基座附近±2m；实际射线原点使用配置的 LiDAR 安装位置。

全局未知格禁止穿越；当前路径初段也需局部观察覆盖，远端允许使用正式静态图。路径和局部代价另预留0.10m可见空间边界以应对视角变化（独立于物理净空）；局部完整预测与停车尾段必须在观察覆盖内；看不清就停车/重规划，不让点云为空放行。首版没有可靠行人速度预测。

上文0.10m为标准档，窄档的额外观察空间预留为0.04m；两者硬覆盖检查都保留“真实车体+所选净空+0.015m扫掠界限”。

前端/定位会话切换、定位跳变、wheel/ROS时间回退、遥控接管都撤销旧轨迹和工作进程代际。worker每个结果匹配进程nonce、任务代际、请求ID和steady deadline；跨目标迟到结果丢弃。公共TF仍只有既有桥接发布，本导航不发布map→odom或odom→base_link。


## 2026-11-17 短暂失效恢复与执行历史

控制`generation`撤销过期/跨暂停结果；独立`history_generation`只在新目标、重新规划或硬失效时更新。暂停不直接销毁候选，但只有协调器最终几何验证确认的request_id能成为历史；新结果仍须属于当前控制代际。历史最多保留1秒，仅用作求解初值；推进不超过原采样时间差及实测位姿进展，驱动对轮角等待不推进，反馈缺失/过期、位姿偏离或换路线时不用历史。重新按当前反馈速度、当前运动段投影，再做全部约束/碰撞验证。

输入失效仍撤销输出，不增加250ms门限。恢复后，同一实际模式、在原路线及原转向轴上、速度符合模式的车辆可从实测余速重新求解，不强制先刹到零。模式/转向轴改变仍需停稳。3秒无进展先撤销旧结果并重验当前执行，不单凭此触发全局重搜；真实路径受阻仍重规划，15秒持续无进展仍失败。明确驱动对轮角等待单独累计有效等待，3秒超时失败，不冒充位移进展。

自主消息改用独立的`LeasedControlCommand`，保留原`ControlCommand`用于旧包解码及遥控接口。协调器、独立门禁和消息包必须一起重建；旧消息不能驱动新自主入口。非零指令的`valid_until_steady`固定为原请求提交时间+100ms，重复发布不续租；零速/模式请求可发布新的零速租约。steady来自同机本次boot，不是传感器时间；原采样戳仍单独检查。驱动会话更换后门禁锁定，必须先收到motion_enable=false、再true才重新使能，连续true心跳不会恢复运动。

执行观察数据不替代原wheel/定位/TF及安全检查。门禁报告和驱动报告保留各自时刻，SDK指令与request_id通过记录时间/速度链核对，不声称CAN逐包ACK或物理执行已确认。录包入口额外收录两条执行话题和消息定义。软件验收与真实边界见[恢复优化报告](exact_mppi_control_recovery_20261117.md)。


## 2026-11-17 导航工作台只读分段路径

新增 `/navigation/route`（std_msgs/String JSON），在接受新全局计划时发布；不参与控制、许可或规划决策。schema_version=1，frame=map，携带 map_uuid、history_generation、原 sample_stamp_ns 和 segments（mode、reverse、poses[x,y,yaw]）。浏览器只有在路线历史代际与当前导航状态一致时按行驶/平移/旋转着色；旧包无该消息时使用原 `/plan`，不推断未记录的运动模式。录包入口追加该话题，旧话题保持兼容。

工作台通过原 NavigateToPose 和 motion_enable 进行自有任务管理；不提供速度或模式直通接口。连续定位由显式 `--continuous` 移除有限 recorder，保留原 CLI 默认有限采集行为。使用和验收见 [工作台说明](../workbench/README.md)、[工作台报告](navigation_workbench_report_20261117.md)。


## 2026-11-17 用户指定停稳阈值 0.05m/s

用户要求线速度停稳阈值改为0.05m/s。motion.stop_linear=0.05、stop_angular=0.025、stop_dwell=0.5由正式exact_navigation.yaml传给协调器/门禁和base_mini_v2.launch的nav_stop_*；Limits默认同步。底盘仍只凭新鲜实际反馈和连续停稳切换模式，不改变原始twist、命令租约、几何裕量或输入有效期。网页使用原始host-clock wheel反馈显示；历史无motion_limits字段记录保持其已有结果，不重写原记录。
# 2026-11-17 工作台续修参数

用户最新指定：定位原采样年龄400ms，接收断流250ms；wheel/障碍/指令独立期限保持。标准与窄通道的静态地图总裕量均为2.5cm（static_map_clearance=1cm+1.5cm扫掠界），与实时body.clearance独立。正式规划、预览、重锚定和最终轨迹验证显式传同一值。任务状态增加failure_cause/planning_failure以保留起终点分层拒绝原因。见[报告](workbench_goal_latency_frontend_report_20261117.md)；下方旧250ms/净空值为对应历史接口记录。
