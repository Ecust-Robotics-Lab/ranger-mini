# 新 Agent 接手指南

## 2026-11-23 分段保存与现场接手

用户要求结束本轮工作，按功能提交并上传现有代码。可复制的提示词见 [NEXT_AGENT_PROMPT](NEXT_AGENT_PROMPT.md)。分支为 `refactor/pluggable-navigation`，提交分为导航/V2 底盘、FAST 定位、工作台、验证证据和交接。上传前远端基点为 `e8245fa80310a4e91b5d2eb303f33883bb965216`。实际远端 HEAD 以 `git ls-remote origin refs/heads/refactor/pluggable-navigation` 为准。

本次只读核对发现导航和定位桥接安装的 Python 源码分别 17、9 个文件与工作树一致。正式 FAST 前端为 `ws_fast_livo/latency_frontend_install`，相机为 `ws_camera/latency_install`，定位为 `ws_fast_livo/online_localization_install`，导航与底盘为各自的 `install_exact`。正式地图 UUID `160f8572a77a61856cf87e43514be769`，占据图 `outputs/fast-livo-demo-changed-scene-20261110/navigation_map_05`，定位档案 `outputs/fast-livo-demo-changed-scene-20261110/mapping_replay_01/map`。原地图、录包、运行数据库、密钥及安装资产在工控机保留。

当前项目 YAML 为 `min_radius: 0.48`、速度 0.50m/s 和 0.80rad/s、停稳线速度 0.05m/s、定位采样/接收 400/250ms、实时覆盖总外扩/保护各 0.015m、边界/内部额度 16/6。运行参数以项目文件及新会话回报为准，安装目录的 YAML 可能是构建快照。最近 worker `run-xmm9byz_` 确认了 16/6。11月18日的 189 个约 0.007m/s 行驶样本对应半径 0.25m，SDK 提交为零。V2 转角上限对应理论最小半径约 0.4764m，本轮保存用户已设的 0.48，源码尚无自动抬升半径的保护。

最近现场记录 `.runtime/navigation_workbench/records/live-41843270f2fb45fc81a3c41443de148f/events.sqlite3` 有走廊方向的单点成功，也有失败和取消。完整办公室出门、走廊掉头回返仍为 PARTIAL。相机无首帧的排查见 [相机报告](workbench_camera_startup_report_20261118.md)。本机 boot `44806031-386f-449d-9e74-e406fa59ae59` 上未发现存活受管会话，最近 workbench/navigation/base/localization 均为 STOPPED。现场权限另取。

本轮跑了 49 项核心/配置、66 项工作台/前端身份、40 项 CUDA/导航相关测试，全部退出 0。两套底盘嵌套仓库按锁定 HEAD、前置补丁和 EXACT 增量核对，应用后各 3 个文件与实际源码逐字节一致，因此在主仓库保存补丁。当前工作树和 Git 远端用 `git status -sb`、`git log -5 --oneline`、`git ls-remote` 核验。旧报告里的“未提交/未推送”仅指写报告时的状态。

## 2026-11-18 保护范围未知格独立额度

用户要求独立计数。项目 YAML 新增 `coverage.protected_unknown_cell_budget: 0`，0–16整数，边界继续读 `unknown_cell_budget: 16`。内部额度0保留旧行为。若关闭 `allow_boundary_unknowns`，所有触及保护框的格归入保护计数。正式 nav 已构建并核对，工作台显示两组额度。新会话加载，GPU 重置不重新读取 YAML。软件 PASS，实车 PARTIAL。[报告](workbench_protected_coverage_report_20261118.md)

## 2026-11-18 相机外触发启动无帧

最新 run-0fxc91ct 的海康 USB3/Line0 成功开机抓图但 SDK 0 首帧，12次超时重连均未恢复出图。近期15会话中5个无图像均雷达 `time_type=0`，10个有图像均为2。先核对外部 PPS/时间源及 Line0 输入，证据未定位到具体线路。已增加工作台/独立定位诊断和回归，66项通过。原相机二进制、触发设置、三融合就绪门槛照常使用。定位/底盘已退出，用户工作台仍运行，下一次启动工作台加载新提示。现场恢复 PARTIAL。[报告](workbench_camera_startup_report_20261118.md)

## 2026-11-17：后角边界未知格与默认速度档（最新）

用户最近自行测试有2成功/2失败，记录live-3f7387…；后续仍被保护范围1～4未知格拒绝，8格额度原先只管最外1cm。新增`coverage.allow_boundary_unknowns=true`：部分接触保护边界的格共用8格额度，完整落入保护矩形的未知格仍拒绝；false恢复旧严格方式。总2.5cm不变，未固定豁免后方、未改格值，已知障碍/越界及全车预测/制动/点障碍检查保持。下方“保护范围任意相交必拒/仅外1cm额度”是旧严格模式。

同时发现正式launch默认low_speed=true将速度限0.10m/s，最新直线非零请求中位0.081m/s。默认已改false，直接读取项目YAML motion.max_speed=.30、max_yaw_rate=.50；显式低速档仍保留。网页新增实际导航上限，参数仍新会话加载/GPU reset不重读。见[报告](workbench_boundary_speed_report_20261117.md)。87核心/49GPU/后续25参数/65工作台、6项隔离实际协调器+GPU+门禁及5浏览器检查通过（有重叠）；模拟直线安全输出峰值约0.290m/s，断流/取消仍停，不是实车成绩。正式nav已构建，最后无存活受管会话，Agent未启设备/CAN/实车目标或速度，未提交推送。新实车高速度连续运动/停车/完整往返仍PARTIAL。

## 2026-11-17：实时覆盖总外扩2.5cm与边缘未知格（最新）

后续已开放项目YAML直接调节：`ws_nav/src/ranger_navigation/config/exact_navigation.yaml`的coverage.total_margin/protected_margin/unknown_cell_budget；默认2.5cm/1.5cm/8保持，启动前校验，同值传两节点/GPU。正式launch改为每次新会话直接读项目文件，改值无需构建；重置GPU不重读，需新导航会话。30相关检查与独立构建通过，正式share已备份后原子更新，见报告末节及coverage_yaml证据。用户随后要求停止，现查工作台/base/FAST前端定位/navigation均STOPPED且无存活归属进程及相关残留，没有代重启；此快照覆盖先前运行状态。

用户指出少量未知格过于保守，并要求总外扩2.5cm。此前只改静态图，实时覆盖仍4cm+1.5cm=5.5cm；保存输入被车体外侧4格拦在优化前。现在路径前段/JAX/独立全量扫掠制动/末端门禁统一CoveragePolicy：总2.5cm（已含扫掠），车体+1.5cm必须观测，仅外侧1cm带最多8未知格；已知障碍/越界不豁免、不改未知格。实时点障碍净空/体素项单独保持，不能称所有碰撞裕量均2.5cm。网页显示具体格数和25mm。

见[报告](workbench_coverage_policy_report_20261117.md)及[对照图](workbench_evidence/20261117/coverage_margin/coverage_comparison.png)。61核心、58 GPU/恢复相关（4个launch跳过另在系统ROS18项全跑）、20相关、65工作台、5项真实协调器/GPU/门禁隔离ROS及浏览器通过，部分重叠。保存实车输入反事实给出非零且全量障碍/制动复核通过的候选，不是实车移动。正式nav已构建，确认用户会话已停止后才安装；Agent未启设备/CAN/实车目标/速度。用户下次正常启动工作台和网页自有会话加载；新实车仍PARTIAL，无提交推送，原资产/未提交工作保护。

## 2026-11-17：失败后独立重置规划控制（最新）

用户指出单点失败后无法停止旧规划并重启规划控制。旧任务实际已FAILED、零输出；旧UI终态禁用取消且缺独立reset入口，不能据此称计算挂死。已新增“重置规划控制”与允许列表API：撤销使能、按Action协议取消自有目标、清除目标/路线/历史、非阻塞重建仅自有GPU子进程，确认新generation/nonce+READY/IDLE后等待新目标。定位/传感器/记录保留，失败/无有效定位也可请求重置，不自动恢复运动。活动外部目标拒绝，重置中禁止新目标，失败可重试、停止会话可中断。

同步修正受管会话1秒健康宽限会误杀JIT预热：只对35秒内显式RESETTING、无worker错误保留初始化条件，定位/输入/消息新鲜度等检查不变，运动许可独立。70工作台、67导航/恢复/门禁、2真实GPU重置、7ROS工作台、真实协调器+GPU+guardian与浏览器通过；正式导航已构建，未操作设备或实车目标。见[重置报告](workbench_navigation_reset_report_20261117.md)。0.4秒/静态2.5cm/正式低延迟前端保持，更新需用户重新启动工作台及自有会话加载。实车重置/再次导航仍PARTIAL，无提交推送，原资产/失败证据保护。

## 2026-11-17：正式低延迟前端、400ms定位与静态2.5cm（最新）

用户明确要求将此前优化直接替换旧版，并最终指定静态总裕量2.5cm。已将FAST预览/录制/建图/在线定位及工作台默认切到latency_frontend_install，实际二进制80450133…、新来源49d9cd5a…；active_frontend.lock.json逐文件锁定。旧地图UUID/manifest不改，精确父版本+相同传感器/时间的frontend_timing_upgrade侧车声明由C++独立核验，无声明拒绝，不用offline兼容绕过。正式三融合、+100ms、外参及旧资产保留。之前“前端候选未切正式”在本节之后已不适用。

定位原采样年龄统一400ms，接收/轮速/障碍期限与100ms租约保持；静态净空1cm+扫掠1.5cm=2.5cm，规划/预览/重锚定/最终验证同源，与实时净空独立。页面ROS批处理修复20话题负载下回调更新P95约234→17ms；具体起终点失败原因保留。原两目标均无路径/零速度，第二份请求证明起点旧9.5cm裕量过大、目标实时点进入车体。现在静态路线通过，但不能把目标实时障碍当作已解决。

见[报告](workbench_goal_latency_frontend_report_20261117.md)。109导航/恢复/安全等、65工作台/来源、46定位核心、5隔离ROS工作台、1隔离静止C2→C1、实际前端sync/13原生及浏览器通过；正式nav/localizer已构建，源/安装核对通过，真实地图迁移通过。四受管会话已STOPPED，Agent没有开设备/CAN/目标/速度；用户下次正常启动工作台并网页显式启动即加载全部新配置，不需额外开关。新的现场单点/连续定位/坡道/完整往返仍PARTIAL，无提交推送，原资产及未提交工作保护。

## 2026-11-17：工作台7/8显示与静止确认恢复（最新）

用户新会话已凑齐8帧并完成多次匹配，未就绪实际停在确认阶段；匹配期间旧7/8显示没有更新。已修首个弱候选/中途弱查询永久锁死静止确认的源码缺陷：从新强窗口1组重建，弱查询不计数、不污染强锚点；3组独立强窗口/1秒、位姿一致性与C1复验不放宽。新增8/8匹配阶段、确认计数/质量原因及逐样本支持/位姿差诊断。现场旧日志不含完整锚点字段，不能断言唯一根因。

见[报告](workbench_static_confirmation_report_20261117.md)。正式online_localization_install已构建，45定位核心、59工作台/CAN、4隔离ROS（含8帧静止C2→C1，3组/1.6秒采样跨度）、浏览器及Monitor落盘检查通过；是软件/合成结果，现场静止初始化仍PARTIAL。用户会话已STOPPED，Agent未开设备/操作CAN/发目标或速度，结束再次核对四受管会话均不存活。用户重新运行工作台再网页显式启动加载新版；不需要为了软件缺陷先移动。0.05停稳参数保持；无提交推送，原资产/失败记录及其他未提交工作保留。

## 2026-11-17：重复启动与用户指定0.05停稳阈值（最新）

用户报告重复启动navigation等待超时与静止时停稳显示，并明确要求线速度停稳阈值改为0.05m/s。日志证明导航节点根本未启动：localization=false，C2一直等8帧查询窗口。已修在线2帧coalescing误用于C2的饥饿：收集C2时保留所需8帧、跟踪仍2帧；8帧/1秒/12k点与几何/C1有效性保持。工作台显式等新定位并显示窗口进度，再起导航，超时明确归因；清理旧进程引用与任务观察，双循环启停模拟通过。

0.05m/s已统一配置并传至导航/worker/门禁/底盘/网页；角速度0.025及停稳0.5秒、原数据期限不变。不归零原轮速，网页优先原始host wheel避免依赖定位时钟。记录首次会话轮速P50 .007/P95 .021/max .023m/s，SDK非零快照0；其他失败会话轮速全0。58工作台/CAN、41定位核心、31恢复、69任务门禁、4ROS任务、3在线定位及浏览器/SDK mock/构建通过。见[报告](workbench_restart_stop_report_20261117.md)及workbench_evidence/20261117/restart_and_stop/。无实车复验，不自动重启/发目标，实际重复定位/停稳仍PARTIAL；已构建更新需用户重启工作台加载。无提交推送，原资产与未提交工作保留。

结束只读检查：用户已停止workbench run-0m24tul0，base/定位/导航会话均不存活，Agent没有代停或重启。现在可直接启动新版网页服务；新停稳参数与定位器构建已核对源码/安装及补丁SHA。


## 2026-11-17：电脑独立请求 CAN 控制，遥控器可保持关闭（最新）

用户希望不再每次开启遥控器/拨SWB。已核对 V2 官方协议：未连接遥控器时上电待机，电脑可发0x421模式请求进入CAN；遥控器开启时更高优先级。现有非receive-only驱动启动已调用EnableCommandedMode，不需要新增强制抢占或放宽V2FeedbackReady。此前将“必须开遥控器”作为普遍条件不准确，已更正。

工作台新增新鲜原始SystemState的CAN/健康确认，显式启动底盘后等control_mode=1再继续，目标接受也核对此反馈；遥控3/待机0/异常/过期不放行，不循环抢遥控控制权。网页新增底盘控制档，明确CAN/电脑、遥控优先、待机。48工作台+5CAN共53离线检查、4隔离ROS与独立HTTP/浏览器模拟通过。证据computer_control/，报告追加节；无真实CAN/设备/目标操作，无自动服务重启，遥控器关闭的实际固件接管仍PARTIAL。

只读现查：用户已停止原工作台run-lzokbvwj（STOPPED），base/定位/navigation均无存活受管会话。下方“工作台仍运行”是上一快照。软件更新需用户重新运行 `bash scripts/run_navigation_workbench.sh --execute --lan` 后显式点击启动；只开服务不启设备。普通SDK请求仍按实际反馈，不自动清故障，不改变急停/遥控接管优先级。无提交推送。


## 2026-11-17：工作台 CAN 前置初始化与失败反馈修复（最新）

用户在正式网页点击启动，两次请求到达任务所有者，但 base 启动失败并已受管退出；原日志明确 can0 must already be UP/ERROR-ACTIVE with known bitrate，实查 can0 flags 无 UP、state STOPPED、operstate DOWN。当前工作台 run-lzokbvwj 仍运行、会话 FAILED、task=null；传感器/定位/导航没有因此启动。用户随后询问遥控器/SWB：V2说明书确认遥控器开启后 SWB 最上方是 CAN 指令档；这是底盘控制权，与 Linux can0 行政启用分开。

已修软件：显式 session.start 才复用 wheel_capture 的外部驱动检查与固定 sudo -n CAN 500k 启用命令；保存 before/after/rollback 信息，已有 UP 参数异常拒绝重置。新增 CAN/底盘/定位/导航阶段及组件进度，嵌套 launch 真正错误透传到持久红色提示，修改请求后清快照缓存。43 工作台+5既有CAN检查共48通过，浏览器故障/进度模拟通过；没有执行真实 ip up、重启服务、设备重试或目标。证据见 workbench_evidence/20261117/startup_fix/，报告见 navigation_workbench_report_20261117.md 最后追加节。

当前运行进程没有热更新，需要用户原终端 Ctrl+C 后重新运行 `bash scripts/run_navigation_workbench.sh --execute --lan`，再在页面显式启动；不自动重启或代发请求。保留原有工作树、包/地图/失败证据，无提交推送。


## 2026-11-17：前端导航工作台 v1

用户要求实施已确认计划，软件已完成，见[工作台报告](navigation_workbench_report_20261117.md)、[使用说明](../workbench/README.md)、[计划](NAVIGATION_WORKBENCH_PLAN.md)。工控机本地/局域网浏览器，2D 正式地图、公共机体定位、单点目标/预设、静态扫掠预览、全局/局部/执行估计轨迹、模式/速度/年龄/原因、受管会话、轻量记录及真实包回放。浏览器断开继续当前目标；任务由独立工控机进程持有，独立 watchdog 在所有者故障时撤销使能。新增只读 /navigation/route；连续定位显式 --continuous 与原始录制分离，原 CLI 默认有限采集保留。37 工作台检查、4 隔离 DDS/Action（含关闭网页后目标只执行一次）、31 相关导航回归、4 包构建及浏览器通过；合成 GPU 加 3 个真实回放客户端 P95 增量约0.24ms，不是三融合整链成绩。现有 u622df4n 原包离线索引356541条消息，无队列丢帧。

启动：`bash scripts/run_navigation_workbench.sh --demo --execute` 只看真实地图/已有回放；正式 ROS 服务用 `--execute --lan`，本命令不启动设备，网页按钮才申请启动自有底盘/传感器/定位/导航。密钥在 `.runtime/navigation_workbench/access.key`，HTTP默认8080；默认不带execute仍dry-run。测试网页服务已停止；没有启动真实设备/CAN或发实车目标。实车仍暂停，完整会话/原始录包/远程控制器及三融合竞争/长时验收 PARTIAL，坡道和往返未通过状态不变。原未提交恢复优化、地图/原包/标定/证据保留，没有提交推送。


## 2026-11-17：恢复接续、历史复用与执行反馈（最新软件）

用户要求执行三项优化，已实施并离线验证，实车仍暂停。见[恢复优化报告](exact_mppi_control_recovery_20261117.md)。控制generation撤销输出、history_generation标识路线；只将最终验证确认的候选按实测进展对齐，1秒历史上限不延长输入期限。同模式、在原轴/路径上可从余速重验接续；3秒无进展先重验执行，真实堵路才重规划，15秒无进展仍失败，轮角等待单独3秒超时。新增LeasedControlCommand带固定steady截止时刻，旧ControlCommand保留解码/遥控；新增门禁/驱动执行观察，SDK提交不冒充实测运动。驱动换会话需新enable=false→true。148项回归、后续31相关GPU检查、38录包/生命周期检查、实际SDK mock及隔离ROS20故障项通过（部分重叠）。一次60ms周期失效的2m理想仿真：旧版30秒未完成，新版约10.08秒成功；不是实车成绩。base/nav已构建安装，补丁逐文件重建核对；源码基点bb41832，本轮未提交推送。低延迟前端仍独立，地图/定位门限不放宽，完整出门/坡道/往返仍PARTIAL。


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
