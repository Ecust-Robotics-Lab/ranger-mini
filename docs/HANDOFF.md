# 新 Agent 接手指南

## 2026-11-17：分段本地Git保存（最新）

用户要求将当前更改分段保存；已按实验/定位、导航与V2模式、导航延迟、前端候选、证据及历史清理分组本地提交，最后补本次报告和交接入口。见[Git检查点](git_checkpoint_navigation_20261117.md)。没有推送/设备/运动。导航延迟优化已安装，前端候选仍独立、正式定位入口不切换；完整实车仍PARTIAL。底盘增量补丁已补齐并逐文件重建核对，源码保持现状。大包/地图/二进制仍本地保留，不把Git视为完整资产备份。下方“未提交推送”属于对应阶段历史记录，提交状态以本节及实际git log为准。


## 2026-11-17：输入延迟首轮软件优化（最新）

用户要求减少数据延迟，仅软件/离线工作恢复，实车仍暂停。见[延迟优化报告](exact_mppi_input_latency_optimization_20261117.md)。定位新有效结果相对FAST中位35/P95 52ms，云→导航观测中位34/P95118ms；前端sync_packages已确认多等下一包雷达的缺陷。独立候选修正后8400共同帧中5639帧可提前约100ms、0帧延后，录制输入释放间隔P95约201→101ms，三融合/原戳/+100ms保留；长段数值非完全一致，原版重复也有约4cm最大差，不能宣称精度不变。前端仅构建/安装在latency_frontend_install，默认mapping/localization仍用原online_frontend_install，来源兼容尚待处理，勿直接改旧地图身份。导航已安装优化：忙worker不重复准备输入、20Hz准备及反馈、固定网格整数切片合并；81真实帧全点/覆盖逐帧一致，点云更新P95约9.2→6.6ms，忙时tick约1.38→.048ms。107回归、最终相关20检查、C++sync及13原生检查、构建通过。新boot6c018ad9，无设备/CAN/运动或目标，未提交推送；完整实车仍PARTIAL。


## 2026-11-17：用户暂停实车，整理问题（最新）

用户已停车并要求先暂停；设备/运动不自动续用本轮许可。进程检查未发现底盘/导航/定位/FAST/目标客户端运行。见[坡前续测与问题盘点](exact_mppi_ramp_corridor_attempt_20261117.md)。本轮微小段暂停衔接已修，精朝向实机成功约89.8°；后续向(2.15,.60)微调只部分移动即失败，仍未到坡面/走廊。末尾位姿约(2.016,.487,1.570)仅历史估计，用户停车后不得沿用。主要已确认问题是250ms输入门限下的延迟/断流与高频代际重置（约28秒17→104）、低速和无进展重规划；不是已证实的坡道推力问题。末尾同戳回放odom/map两坐标锚定均通过，未证实覆盖坐标漂移根因，不改坐标和高度阈值。新增Task精目标容差默认.15/.15、现场.04/.04，旋转进展修正；完整出门、坡道与巡航仍PARTIAL。新封存包z7vdr1vj保留，当前阶段源码身份及日志见报告。


## 2026-11-16：复位续测及坡道信息（最新）

见[续测/坡道诊断](exact_mppi_door_retest_ramp_20261116.md)。用户复位后重新定位约(.844,.039,.017)，最初RC3、确认CAN1后，两段接近成功（估计0.669m/0.421m）；独立转向40秒取消、加强SPIN朝向代价后的复验仍no_progress失败。当前末尾估计约(1.87,.41,.625)，全设备停、指令/wheel零，尚未出门。SPIN GoalAngleCritic已12→60，其余模式12；87回归/10闭环/构建通过，但不能声称实机效率已改善。此前feedback_settling简述非唯一根因：曾恢复执行，长停发生在无进展重规划后STOPPING→短暂失效恢复OBSERVING，需修衔接。

用户新增确认：办公室到走廊为下坡，约6cm/50cm（约7°）。现有平地SE(2)/固定高度障碍没有坡道验收；仅离线诊断，未自动实现地形支持。两包俯仰中位约0.26°、未见接近7°倾斜；门前地面采样断裂，不能从现有记录确认坡面或把远处11–13cm低分位高度当坡高。新封存包snwbprb6/d_pha29a保留，坡道和原地软件缺陷需分别核对，不再盲目发全段目标。代码/日志/安装身份在office_door_retest_20261116；未提交推送。

## 2026-11-16：门口实机测试最新（覆盖离线准备）

后续用户已人工将车辆移回初始位置；上次结束约map(2.08,.72,.89)已不是当前位姿，任何新目标须用新会话定位重算。用户询问为何出不了门、能否全段运行；本轮只离线复核，未重启设备或发目标。近期观测前段约束的候选搜索对照已保存，尚未改默认/闭环验证，不能据此宣称能完整巡航。

用户本轮授权实机测试，当前结果PARTIAL：60cm门内接近点成功（估计前进48cm），门外目标仍失败或期限取消，尚未出门/巡航。已确认旧DRIVE低速vx/wz独立扰动导致请求转角左右换向120次，现改固定曲率分段与候选预测、真实转角确认后行驶；实机修正后行驶请求角换向/大跳角均0。再加平移/旋转方向约束后，旋转反向0，但45秒门外目标仅估计前移约19cm，取消后输出/wheel均零；静态路径与实时可执行覆盖、短段衔接仍待解决，不继续重发长目标挪动。87项GPU回归、10闭环、24静态模式检查通过，底盘/nav构建及receive-only/实际回调mock通过。最后feedback_settling分类改动已构建、未实车复验。全部设备已受管停止，当前约map(2.08,.72,.89)，不是下一会话可直接沿用的实测初值。见[本轮实机报告](exact_mppi_office_corridor_live_20261116.md)和证据office_corridor_live_20261116；新增u622df4n/fo3nvbdf/73q20pr8录包均已封存，早前“只有预建图包”清理快照已被新增采集更新。未提交/推送。

## 2026-11-16：办公室走廊巡航准备（最新）

用户要求开始准备办公室出发、走廊端点掉头返回；本轮仅离线准备，设备未启动、无目标/速度。新候选 `config/exact_mppi/office_corridor_preparation_20261116.yaml` 新增门外停稳转向后倒车/平移入室，8/8段、三初始模式24/24静态扫掠通过，约40.39m。车体及5.5cm静态裕量未减；入室搜索3.87–4.50秒且门口仍有较多短运动段，现场流畅性/持续0.30m/s/全程往返仍PARTIAL。起点是记录候选，不是当前位姿，需现场重新绑定与规划。见[准备报告](exact_mppi_office_corridor_preparation_20261116.md)、[交互路线](../outputs/exact-office-corridor-preparation-20261116/index.html)。14项相关回归通过；旧路线和失败证据保留，未提交/推送。下方6/7及7/7属于历史候选。

更新：2026-11-10。项目`/home/lab307/ranger_nav`。同一实施会话可继续已授权的软件工作；独立接手先核对源码、安装和当前测试状态。用户希望快速开发、复用现有成果，避免重复实现。


## 2026-11-16 最新清理（覆盖下方运行快照）

最终清理范围已按用户追加指令扩大：当前仅保留11月10日预建图原包bags/fast-livo-office_corridor_changed_20261110-s10xmu6g，SHA与预建图来源一致。第二阶段删除其他111个录包数据文件约700.16GiB，可用约791GiB；全项目数据扫描只剩该原包。地图、标定参数/结果、日志和备份保留。下方“11月5日及以后58个录包保留”为第一阶段历史，已被本次指令覆盖。所有受管设备已停止。详见录包清理报告最终范围节。


用户明确要求删除11月5日前录制的包，已执行：30个录包数据文件、约44.5GiB；11月5日及之后58个数据文件已复核保留，地图/标定/日志/备份不删。见[删除记录](recording_cleanup_before_20261105_20261116.md)。导航、底盘、定位和采集会话均已停止，无待发目标；不要沿用下方“当前正在采集/底盘RUNNING”快照。后续实车工作仍未完成，须核对当前状态。

## 2026-11-16 实车提速续测

最终状态（覆盖下方本日历史快照）：已确认CAN模式1并完成3次实际运动成功：60cm约17.6秒/零速25.7%；80cm约16.4秒/零速21.2%；瞬时失效暂停恢复修复后的60cm约8.6秒/零速18.3%、仅1次接受路线、指令峰值约0.165m/s。各次末尾输出/反馈零，未达到0.30m/s持续巡航或完成走廊往返。静止定位改为8帧/12000点，覆盖统一2秒；Task对短暂DEGRADED/输入缺口保留平移/旋转路线并撤销旧控制，恢复须重验，LOST/换会话/地图/跳时/遥控仍清空路线。89项回归通过，构建退出0；求解50→100次无改善，仍保留50次及原定位/几何/250ms门限。轮速/指令与定位有差异但无外部真值，未自动改比例。用户曾在两轮间自行移动车辆，零指令回程成功不计运动验收。设备已按受管顺序停止；细节见本日续测报告最终章节及static_refinement/final_identity.json。


21:19最新：静止重定位已改为8帧/12000点（.2m），同输入隔离回放约4.5秒有效、157不同有效采样，实车也确认成功，定位门限未放宽；无需反复人工移动。覆盖口径统一2秒，但两次0.6m目标仍在局部guidance_unobserved失败，输出全零。用户说明此前CAN控制未开；重新握手后协议仍control_mode3（遥控），当前只运行底盘节点、导航/传感器已停，等待用户切遥控器上位机/CAN档并停稳。全部证据与细节见本日续测报告最新章节/static_refinement；当前未完成提速，不发送完整走廊路线。


最新boot b62be450：CAN已恢复（被动3秒2661帧、正常握手RUNNING、CAN档/无故障）。底盘与定位采集正在运行，导航已停止、无待发目标/非零速度。定位候选重合约91%、RMSE约12.5cm，静止强几何未通过，正在等用户遥控小幅运动形成基线后回CAN停稳；不放宽定位门限。有限0.6m/0.30m/s客户端仅等待就绪，goal=null，尚未执行提速。详见本日续测报告boot_b62be450节；下方前一boot无CAN反馈是历史快照。


用户要求继续实车测试，并确认仍在原已建图区域。新boot下can0初始DOWN，按既有固定命令启用500kbit/s；被动监听0帧，底盘握手因无wheel/errno105超时后自动清理，复位队列后15秒仍无外部反馈。当前没有导航目标、非零速度或存活底盘节点；定位/传感器尚未启动。等待本日底盘上电、CAN档与接线核对。证据和本轮实际状态见[续测报告](exact_mppi_speed_retest_20261116.md)，历史11月10日许可/快照不代替当前状态。

## 当前实车阶段（覆盖下方历史准备快照）

最新提速阶段：用户要求更多覆盖记忆、大提速、降低零输出。源码改为1.5秒记忆，并可显式RANGER_NAV_MAX_SPEED=0.30（协调器/安全链一致，默认首测0.10不改）。已修无编号故障消息导致误淘汰定位会话、计算超时触发全局重搜、投影重复编译，以及末端微小SPIN导致平移被迫6cm精调。最近实车两个30秒任务均取消，计算恢复修正后零输出由68%降至26%，但实际速度仍慢；没有完成巡航提速。最终GoalCritic进展权重10→50的10项GPU/10场景闭环通过，末端/进展改动实车复验因C2长时未就绪未发目标。设备均已受管停止。当前仍PARTIAL，不能发送整条走廊路线。详见变化场景实车报告“1.5秒覆盖、提速与零输出原因排查”，身份speed_delivery_identity.json。


最新结果：多帧0.5秒覆盖记忆（原始时间戳过期、新障碍阴影立即撤销）、取消模式确认后重复等待、最终段恢复15cm/0.15rad及一次0.5秒停稳已实现。77项相关检查、10/10窄档闭环通过。新实车goal_camera_forward_continuity返回SUCCEEDED：30cm目标估计前进约26cm、位置误差约4cm、末尾零输出/零反馈；零输出占比约34%，耗时约26秒，速度仍偏慢，定位偶发撤销仍在。短目标单次PASS，整车往返仍PARTIAL。受管设备均已停止，详情见变化场景报告“多帧覆盖与停车流程优化”。


最新追加：已修复短目标多模式拆分、停稳小误差重搜、稀疏指导、跨重规划无进展期限及短时缺覆盖等待；76项相关回归、10项GPU检查、10/10合成闭环通过，沿路径恢复修正另52项通过。实车仍PARTIAL：30cm目标先前只移动约1.6cm；保留路线等观测后前进约18.3cm，再次继续至距目标约5.73cm，但两次均30秒期限取消，没有成功结果。实测非零指令中位数约0.0124m/s，覆盖/就绪中断及低速仍待解决。最新按受管顺序停止全部设备，不能将本轮称作已完成导航。证据见变化场景报告churn章节和churn_live_summary.json。


用户本次已授权实车测试。新变化场景包 `bags/fast-livo-office_corridor_changed_20261110-s10xmu6g` 完成；新图UUID `160f8572a77a61856cf87e43514be769`、归档 `outputs/fast-livo-demo-changed-scene-20261110/mapping_replay_01/map`，对应 `navigation_map_05`。生产runtime不改，旧资产保留。

用户确认相机镜头朝向为车头、雷达距同端约26cm。原工作base +X与相机相反，已统一修正工作外参为约[1°,21°,0.9°]、IMU XYZ=[.1031408,.0239459,.5552534]，驱动对SDK相反纵轴作指令/反馈转换；转向反馈同号配置+1另保留。73项相关Python检查、47协议往返及反馈/生命周期/receive-only检查通过。实车+0.05m/s×2秒，wheel/定位均估计前进约9.3cm、结束零速，用户明确“确认是正方向”。旧微动作标注使用当时SDK坐标，不能混为新物理方向验收。

车头修正后的新候选 `config/exact_mppi/office_corridor_camera_front_20261110.yaml`、复核 `outputs/exact-field-preparation-camera-front-20261110/` 为6/7静态通过，最终回办公室并恢复出发朝向三模式搜索超时。旧工作坐标下7/7不作为当前成绩。完整车体与5.5cm静态裕量保留。

用户允许覆盖算法更积极：精确检查栅格全部相交扇区，去掉邻扇区额外膨胀；全局指导不强求当前可见，局部完整预测/制动与独立输出继续受真实覆盖约束，未知全局格仍禁行。单纯地面阈值/多高度放宽无助的试验未进入代码。定位预处理复用、4线程/OMP及在线去除人工配对延迟已构建并通过backend/localizer两组CTest；future云排队、历史wheel同戳TF选择2项边界检查通过。250ms有效期未放宽，短目标先前因边界过期取消，最新安装正在受管实车复验。设备当前状态须查boot/PID/会话，整条走廊任务尚未执行。

详见[变化场景实车报告](exact_mppi_changed_scene_live_report_20261110.md)及 `exact_mppi_evidence/live_changed_scene_20261110/`。未提交/推送。


## EXACT-MPPI 当前入口

用户已授权[EXACT-MPPI计划](EXACT_MPPI_NAVIGATION_PLAN.md)的软件实施、构建、离线验证及RTX5060驱动/CUDA安装。新SE(2)/EXACT-MPPI、全模式V2接口、实时FAST障碍、NavigateToPose和独立安全链已实现；94项相关回归+25 subtests、9/9闭环及19/19 ROS任务/故障通过，离线整链P95 38.70ms/P99 47.63ms；源码导航入口已替换旧算法，原安装资产保留。最终结果和限制以[实施报告](exact_mppi_implementation_report_20261110.md)、[接口](exact_mppi_interfaces.md)及[STATUS](STATUS.md)为准，不回到“尚未确定算法/等待授权”的旧状态。

GPU580.178.04-open、JAX0.6.2/CUDA12已实际使用cuda:0，用户手动完成MOK/重启，SecureBoot保留。原未提交修改完整保护，新修改也未提交/推送。设备/底盘/CAN/实车运动没有启动；P9另需当次授权，用户此前重启不继承运动许可。

可视化入口：`outputs/exact-mppi-visualization-20261110/index.html`，包含实录119帧静止查询（重算50通过/69拒绝）及独立9场景闭环仿真；不是实车导航证据。生成脚本`visualize_exact_mppi.py`和`validate_exact_mppi.py --trace-output`，原验收文件不覆盖。

最新任务为实车准备：图上方办公室→走廊端点掉头→原办公室；用户允许适当减小额外裕量。见[实车准备](EXACT_MPPI_FIELD_TEST.md)。同一算法新增standard/narrow参数，窄档仍0.74×0.52m车体，净空0.03m/静态额外余量0.01m/观察额外预留0.04m；窄处优先固定转向平移，段间真实反馈重规划及优化器/独立验证裕量同步。门内外两段仍未找到路线，候选起点非实车位姿。`localize_fast_livo2.sh --public-tf --record-navigation`扩展43话题录制与实车RViz；默认dry-run，不启动导航或底盘。新证据在`exact_mppi_evidence/field_preparation_20261110`，原94/9/19及性能身份是修改前历史快照，不覆盖。设备与运动仍未授权执行。

准备阶段最终111独立用例覆盖通过（JAX环境109+25子项、2个launch由系统环境补齐）、标准闭环9/9、窄档10/10、窄档ROS19/19；窄档ROS P95 35.58/P99 40.71ms。当前源码/安装身份以新目录 `delivery_identity.json` 为准。最新交互路线页 `outputs/exact-field-preparation-narrow-final-20261110/index.html`；实机与门口仍PARTIAL，未启用CAN。

## 快速阅读

最新采集进展：用户确认现场变化并要重录，随后明确“我来运行命令”。由用户操作，Agent未启动录制；中断后确认无存活录制/设备进程。接线后网口192.168.1.50和海康USB已出现。录包入口复用latency_install相机/库、保留旧资产，11项单元检查通过；等待用户运行900秒office_corridor_changed_20261110采集，不能把dry-run当新包。后续不自动重启采集或底盘，按用户最新分工执行。

最新追加[静态1.5cm诊断](exact_mppi_static_margin_report_20261110.md)：用户要求全局1.5cm试验、局部维持5.5cm，并询问是否重录。8次搜索均超时；52cm内接圆的乐观中心空间仍断开，地图瓶颈约(1.3,-2.9)m。原305个3D关键帧哈希通过，128帧看到局部、17帧原点在1m内，最近两处占用有多帧低处回波支持。先核对现场净宽/物件，再利用现有数据复核投影；不凭超时要求重录。运行配置/安装身份仍不变，新脚本`diagnose_exact_static_map.py`和3项测试，图在`outputs/exact-field-preparation-static015-20261110/`。设备/采集/运动没有启动或取得新许可。

1. [AGENTS](../AGENTS.md)、[STATUS](STATUS.md)、[总计划](FAST_LIVO2_BACKEND_PLAN.md)、[AprilTag计划](FAST_LIVO2_APRILTAG_PLAN.md)。计划保留设计/验收要求，当前完成度以STATUS为准。
2. [最新首次定位优化](fast_livo2_initialization_optimization_report_20261109.md)：静止确认、预处理复用、原速对照与边界。
3. [最新实机结果](fast_livo2_field_latency_retest_report_20261109.md)、[相机/局部恢复实现](fast_livo2_failure_fix_report_20261109.md)。旧实机31秒发生于本次首次定位优化之前。
4. [规划控制前置计划](FAST_NAVIGATION_INTEGRATION_PLAN.md)、[导航输入实现](fast_livo2_navigation_prep_report.md)、[wheel/工作外参](fast_livo2_wheel_alignment_report_20261109.md)。后两份报告只说明当时状态，不能据其旧零旋转/未录wheel语句覆盖当前配置。
5. [保存上传检查点](git_checkpoint_initialization_upload_20261109.md)、[最新文档清理](handoff_cleanup_20261110.md)。需要追溯资产删除再读[20261105在线建图清理](handoff_cleanup_online_mapping_20261105.md)、[更早清理](handoff_cleanup_20261105.md)。这些名单不授权未来删除。
6. 涉及模块再读[前端](fast_livo2_interfaces.md)、[B](fast_livo2_backend_interfaces.md)、[C1](fast_livo2_localizer_interfaces.md)、[C2](fast_livo2_relocalizer_interfaces.md)接口。[目录](PROJECT_LAYOUT.md)、[索引](README.md)。

## 当前交付

- 在线预建图软件已打通：390秒真实包原速输入3900对，305关键帧、114接受回环。保持LiDAR–IMU–Camera、正式+100ms、重力对齐。
- FAST定位默认自有v2→KISS完整6DoF→small_gicp→同一C1。C2强几何静止确认与C1多初值预处理复用已实现。原速FAST输出回放同包首VALID墙钟29.820→2.800秒，另包4.220秒；40核心/21后端/9ROS通过。没有重跑传感器/前端，不能称为实机2.8秒。
- 相机分线程/两帧队列/迟到检查已实现；C2短时局部恢复2秒/.75m/.35rad、两新采样/.1秒，离线通过；实际现场未触发恢复验收。
- 最新实机co968xa3在首次定位优化前：90.044秒、921相机同步全OK、591个不同有效确认、无LOST、3次60–120ms撤销就绪。动态TF唯一及wheel/clock门控已有现场证据；连续就绪仍PARTIAL。
- 三融合＋wheel同录pwr5lx7u已核验；V2反馈符号、时间映射/同刻插值、导航图身份绑定、隔离公共桥接已实现。用户授权工作外参约RPY=[1°,21°,-179°]，精确XYZ/RPY读配置，非实测标定。
- 生产仍Lightning/wheel、terrain disabled；FAST隔离入口未接管生产。Tag默认关闭，无真实VALID地标表。

## Git与当前文档变更

分支`refactor/pluggable-navigation`。2026-11-09完整上传后的HEAD为`e8245fa80310a4e91b5d2eb303f33883bb965216`；2026-11-10只读ls-remote仍相同。源码/参数/补丁/可版本化证据在Git；原bag/地图/二进制与备份主要只在本机。

本轮用户要求精简过时交接信息，已重写活动入口，**这些2026-11-10文档修改尚未提交/推送，须保留**。原版本已备份`.runtime/backups/handoff_refresh_20261110/`，也可从e8245fa追溯。此后本轮新增EXACT-MPPI软件实施，未执行设备操作。

## 实际安装与哈希

| 组件 | 当前前缀 |
| --- | --- |
| FAST前端 | `ws_fast_livo/online_frontend_install/`，I/O派生源码`online_frontend_source/` |
| B/共用后端库 | `ws_fast_livo/online_install/` |
| 最新C1/C2 | `ws_fast_livo/online_localization_install/` |
| 在线定位相机 | `ws_camera/latency_install/`；原`ws_camera/install/`保留 |
| receive-only底盘 | `ws_base/install_recording_rx/`；原安装保留 |
| 新全模式底盘 | `ws_base/install_exact/`；新SDK静态链接，勿混旧ABI |
| 新导航/消息/公共桥接 | `ws_nav/install_exact/` |
| JAX/CUDA Python | `.runtime/exact_mppi_venv/`，固定依赖`config/exact_mppi/requirements.lock.txt` |

最新定位节点SHA `17ff9442653c37f7626c408558a628bacb43016e0961bf08d00cc2b23c3624dd`；新导航/底盘当前身份见`docs/exact_mppi_evidence/field_preparation_20261110/delivery_identity.json`，`20261110/final_identity.json`保留初版验收快照；定位原有完整节点/核心库/参数/地图哈希见`docs/fast_livo2_evidence/initialization_optimization_20261109/hashes.json`。相机节点/库、底盘等见`docs/fast_livo2_evidence/failure_fix_20261109/binary_hashes.json`，其中C1/C2哈希是优化前版本，不可当当前定位版本。旧定位/库回退副本在`.runtime/backups/initialization_20261109/`。

## 正式资产

| 资产 | 路径与意义 |
| --- | --- |
| 正式3D图 | `outputs/fast-livo-demo-online-mapping-20261105/full_replay_12/map`，UUID `253f573d7a7eabbc363a225ab192cc14`，manifest SHA `e81d4a05a8da6cf3c1badfc0a6171295020bec59e5615408796c92a85b49b247` |
| 候选导航图 | `outputs/fast-livo-demo-navigation-prep-20261106/navigation_map_final`，5cm/453×835，通行性未验 |
| 最新实机包 | `bags/fast-livo-localization-co968xa3/raw`；会话历史`run-ef5z0dya` |
| 独立访问包 | `bags/fast-livo-relocalization_visit01-trpozzyj/raw`，约282秒；优化复验仅前45秒 |
| 三融合＋wheel包 | `bags/fast-livo-wheel_alignment_visit01-pwr5lx7u/raw`，180秒 |
| 早期现场失败 | `bags/fast-livo-localization-um88vo7b`、`-jbwq4yhi`、`-88k5eclm`全部保留 |
| 最新优化证据 | `docs/fast_livo2_evidence/initialization_optimization_20261109/`，包括无效首轮测试入口的失败对照，勿删除 |

其他原包/地图/旧标定/失败证据按原路径保持；需要重放旧清理的replay_view时按历史清理记录重建，不把旧命令路径当现存路径。

## 下一步与首次核对

先读本轮报告的最终PASS/PARTIAL及失败对照，不用旧阶段日志覆盖当前结论。新入口复用FAST同戳云、映射wheel、公共定位/TF及绑定地图；启动默认未使能，不能在没有当次授权时发导航目标或打开设备。

基线HEAD仍e8245fa80310a4e91b5d2eb303f33883bb965216；本轮备份`.runtime/backups/exact_mppi_20261110/`、GPU安装回退`.runtime/backups/exact_mppi_gpu_2dg9OIbf/`。保留原包、地图、失败日志与旧安装，不reset/clean。当前boot/PID及实际源码/二进制须重新读取，不把20261110报告快照当存活状态。

下一真实验收按计划P9逐步授权：底盘模式/符号/制动→现场单点与异常→30分钟重复任务；连续定位与地图通行性仍独立PARTIAL。软件验证命令与参数见接口及报告，不默认恢复旧算法。

可复制提示词：[NEXT_AGENT_PROMPT](NEXT_AGENT_PROMPT.md)。
