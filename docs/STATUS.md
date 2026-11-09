# 当前项目状态

更新：2026-11-09。**在线预建图已有原速联调；在线定位首版完成软件/离线交付，新282秒独立采集包原速无初值C2→C1功能通过，连续新鲜度泛化、真机、长时及D仍待验收。** 真实Tag辅助定位/回环和总计划未全面完成。快速接手见 [HANDOFF](HANDOFF.md)，阶段/参考架构见 [总计划](FAST_LIVO2_BACKEND_PLAN.md)，Tag细化见 [增量计划](FAST_LIVO2_APRILTAG_PLAN.md)。日期报告只描述其当时条件。

## 当前配置与工作树

2026-11-09用户已要求将首次定位优化及既有本地提交一并保存上传；本轮分组和资产范围见[Git检查点](git_checkpoint_initialization_upload_20261109.md)。最终上传结果以本轮Git核对为准，历史报告中的“未提交/推送”描述各自当时状态。

2026-11-09首次定位优化最新：用户明确授权实施，已完成C2强几何静止确认→C1内部交接，以及small_gicp/独立几何检查预处理复用；40核心＋21后端＋9ROS检查PASS。同co968xa3前45秒原速记录输入对照，旧版首VALID墙钟29.820秒→最终2.800秒，C1完整初始化约1秒→163ms；另一独立采集trpozzyj首VALID4.220秒。均为前端输出重放，非新实机测试，输入期间无LOST；真实负样本泛化与连续就绪仍PARTIAL。见[首次定位优化报告](fast_livo2_initialization_optimization_report_20261109.md)。C2 overlay默认启用静止分支，手动C1保留运动要求；全部本次测试进程已清理，未开硬件/提交/推送。

2026-11-09首次定位耗时澄清：此前v2/GICP优化仍生效（同282秒包51.494→22.315秒）。本轮co968xa3约31秒主要等待静止阶段运动基线，首次几何支持约首FAST后0.7秒即出现；前25秒位移仅毫米级，C2约28.8秒交接，再约2.2秒C1初始化/接续。见最新实机报告附录，不将整段31秒称为计算瓶颈。未改参数。

2026-11-09修复后实机最新：用户明确授权再短测，co968xa3正常录制90.044秒，exit0并已STOPPED/清理。新相机节点/库实际加载，921帧同步OK、零错配/队列丢帧，取帧间隔最大101ms；591个不同有效确认，首次距首FAST采样30.700秒，无LOST。仍有3次C1 not_converged导致60–120ms就绪撤销，整体连续性PARTIAL；局部恢复分支未触发，不追认实机恢复时延通过。见[新实机报告](fast_livo2_field_latency_retest_report_20261109.md)。未改软件/门限、未提交/推送，未自动继续开设备。

2026-11-09交接保存：用户要求开新窗口前分组本地提交，源码、配置、证据与报告已按本次范围保存，无推送；见[Git检查点](git_checkpoint_handoff_20261109.md)。新窗口直接使用[NEXT_AGENT_PROMPT](NEXT_AGENT_PROMPT.md)，先只读核对并汇报。最新软件修复仍未实机验收，不从本地保存自动续开设备。

2026-11-09故障修正最新：用户授权实施后完成相机取帧/发布分线程、两帧有限队列、设备tick/scan进度检查和分段计时，以及显式C2的2秒短时局部恢复。36项核心、2项新增ROS断流/静止恢复、3项在线及2项C2回归通过；原CSV拒绝540–543、544通过，原包两帧真实几何恢复通过。详见[实施报告](fast_livo2_failure_fix_report_20261109.md)。在线定位指向隔离latency_install相机，原安装/生产不变；未开设备，实车连续就绪/USB根因仍PARTIAL，后续全局召回及初始化缓存尚未改，工作树未提交。

2026-11-09网络检索与只读复核：见[两项故障原因及方案](fast_livo2_failure_research_20261109.md)。相机实际USB3，帧540设备只进一帧而scan时间前进约0.5秒，迟到旧图仍标OK；根本停顿位于前帧publish之后/后帧SDK返回之前，具体等待源待分段计时。33.579秒恢复中观察39轮无几何匹配，单轮中位63ms，205候选粗配准失败、10进入精配准均过几何门槛，不能归因small_gicp独占34秒。建议先修图像时间关联、增加C2短时局部恢复，再优化全局召回和C1预处理复用。仅研究/文档，未实施修复/开设备/提交，连续就绪仍PARTIAL。

2026-11-09第三次现场最新：用户授权的88k5eclm完成90.045秒，原bag897对FAST、58个不同有效确认。4200条派生wheel零领先已记录clock、60个运行图快照均唯一动态TF，两项修正已实测；整体就绪仍PARTIAL。相机取帧交付0.53秒停顿、4帧同步BACKWARD，FAST输出0.66秒间隔与C1过期相吻合；约34秒后自动恢复。详见[最新现场验收/阻塞报告](fast_livo2_field_gate_acceptance_report_20261109.md)。设备已清理，不再自动续测；下一项先用此包离线分解采集等待/同步积压，规划控制软件可并行进入，实车自主闭环尚未通过。未提交/推送。

2026-11-09第二次现场最新：获“重新启动”授权后完成jbwq4yhi的90.027秒采集，C2→C1得到486个不同有效确认，首次距首FAST42.400秒，稳定锚点已生效。公共TF有输出，但就绪反复切换，仍PARTIAL。已离线修正wheel抢先于clock的问题与底盘空TF端点，隔离链路/转发检查及rx重建通过；修正后未新开实物。详见[复测报告](fast_livo2_field_retest_report_20261109.md)。设备已清理，数据保留，生产/规划控制不变，未提交/推送。

2026-11-09现场最新：用户明确确认后已启动首次实物联调，记录见[现场失败与锚点修正报告](fast_livo2_field_localization_report_20261109.md)。首次在驱动加载器阶段因缺底盘消息库退出，补加载既有base环境后完成设备首次启动；真实包um88vo7b正常封存37.945秒，但首批雷达17帧/54ms携带约2.96秒头戳跨度，污染首帧锚点，C1零VALID/就绪全false，现场FAIL。已停止并清理全部本次设备进程，can0保留UP500k。已实现启动稳定窗口单一锚点，失败包与合成ROS复验通过；尚未重启实物复测，不追认现场通过。原包/失败记录保留，未提交/推送。

2026-11-09最新继续：在线时间转换→实际C1适配器→公共桥接的隔离ROS链路通过wheel断流/恢复、receipt断流/恢复和锚点异常锁定；1项集成＋3项摘要＋5项wheel准备检查通过。新增显式`--auto-wheel`复用原receive-only准备与清理，默认不自启；录后功能摘要与流程成功分开，公共模式无有效就绪则退出2。合成C1不是新真实C2试验，物理同步/实际接车仍未验；现场命令见[最新报告](fast_livo2_live_chain_report_20261109.md)。未开硬件/控制、未提交/推送。

2026-11-09再后续：在线每会话时间映射及显式公共TF软件入口已完成，见[本轮报告](fast_livo2_live_clock_report_20261109.md)。复用雷达receipt/steady锚点，原wheel采样戳转换到sensor-clock，原话题保留；跳时/偏差锁定、断流暂停。4项核心＋1项隔离ROS检查、180秒真实包接收时间代理审查及两种入口dry-run通过。在线`--public-tf --navigation-map`复用既有公共桥接，消费已有同域wheel，不自启底盘。首帧传输偏差仍未知，物理同步/真实接车未验；本轮无硬件、运动、提交或推送。

2026-11-09后续：时间映射与完整同录包公共TF离线联调已交付，见[完整链路报告](fast_livo2_offline_wheel_chain_report_20261109.md)。默认dry-run入口`replay_fast_livo2_wheel.sh`：最终1801对、1502有效定位、首次VALID源时间28.800秒，1494次TF公式一致性检查，wheel断流撤销/恢复及尾部过期通过；单一TF、零速度发布者，三个子进程退出0。旧wheel诊断重建＋本包限定时钟仅作离线功能验证；raw反例因20.6°航向创新拒绝通过桥接。物理同步、绝对精度与实车接入仍PARTIAL；硬件/生产/规划控制未动，未提交/推送。

2026-11-09最新：用户已于11月6日录得180秒FAST＋wheel包pwr5lx7u，8话题审查PASS、receive-only自启/收尾流程通过，源DB两次SHA一致。进一步确认Mini V2反馈角左负右正，已修本地反馈换算并构建隔离录包版本；原安装保留。用户授权约RPY=[1°,21°,-179°]先作为工作估计，现已统一FAST profile、C1/C2入口和公共适配器，XYZ按同一旋转重算；不是实测标定。时间仍有异历元，候选映射仅用于本包诊断，物理同步/真实公共TF仍PARTIAL。见[本轮完整报告](fast_livo2_wheel_alignment_report_20261109.md)。本轮未启硬件/控制、未提交/推送。

2026-11-06最新：用户要求录包自动启动底盘、仍遥控采集。现默认自动wheel，无发布者时启用CAN并启动隔离receive_only驱动，有发布者复用，`--no-wheel`关闭；不请求CAN控制模式/订阅速度/退出发零速，自启进程随录包结束清理。隔离构建、28项Python回归及实际源码生命周期shim/机型检查通过；现有真实base会话只读查验并保留，未开启新实物录制或重启底盘。源码和原安装保护，修改未提交。见[自动wheel报告](fast_livo2_recording_wheel_report.md)，真实同录/时间映射仍待验。

2026-11-06再后续：用户明确要求尝试底盘/wheel启动并自行完成CAN免密配置；Agent已启动domain175/localhost的受管base会话`run-qqesz8kq`，约50Hz wheel可用，检查结束时保留运行。没有启动传感器/录包/Nav2或发送速度。原始CAN另发现`vehicle_state=2`、待机模式0、完整故障`0x00000200`（上位机通信故障位），旧ROS仅16位error_code错误显示0；运动就绪未通过，不能自动放行。先读[启动检查](wheel_start_report_20261106.md)，下次仍需现查会话boot/PID，旧实时状态不是未来存活证明。

2026-11-06最新：用户决定后续大改规划控制，先完成导航前置接入。本轮已交付正式3D图的5cm占据候选图、隔离`bridge_fast_livo2_offline.sh`入口、地图UUID/SHA绑定、既有C1适配/同刻wheel公共TF及传感器静态TF；4项投影＋2项ROS＋2项既有回归通过。用户授权按Ranger Mini V2模型选择base_link原点，已采用轴间中点在地面的投影，估算`T_base_imu.xyz=[0.120,0.02329,0.55588]`，独立配置`config/fast_livo2/navigation_prep.yaml`，不是实测标定。没有当前FAST＋wheel同录包，真实时间/接车仍PARTIAL。详情见[前置接入报告](fast_livo2_navigation_prep_report.md)。本轮未开硬件/控制、未提交/推送；生产及规划控制参数保持。

当前推荐的FAST在线重定位默认组合为：自有检索v2＋KISS完整六自由度粗配准＋small_gicp精配准＋C1确认/跟踪。用户再次要求设为默认后，已核验实际入口参数及现用二进制与默认短包检查时SHA一致；配置原已生效，无需重复修改。选择依据及SC对照边界见[报告](fast_livo2_retrieval_comparison_report.md)。

- 2026-11-06后续按用户指令完成[自有检索v2/STD/Scan Context对照及GICP切换](fast_livo2_retrieval_comparison_report.md)：新282秒包v2/PCL首VALID24.394秒、2579有效扫描，v2/GICP22.315秒、2590有效扫描；后者10个不同扫描短暂未收敛、无LOST/ERROR。在线默认v2检索，C1/C2入口默认small_gicp，PCL/v1可显式选择。检索区域参考来自估计轨迹，不是真值；SC在后段共同几何通过数更高，不能宣称v2全面胜出。改动已分组本地保存，未新增推送，见[检查点](git_checkpoint_online_localization_20261106.md)。

- 2026-11-06新增独立采集查询包 `fast-livo-relocalization_visit01-trpozzyj`：282.10秒、73,353条消息、2821对前端位姿/云，停录正常、父进程后审查被中断；只读补验基本数据PASS，原结果/无完成标志保持，见[新包核验](fast_livo2_relocalization_visit01_capture_report.md)。用户随后明确授权在线重定位源码、隔离构建与离线开发，首版已交付，见[报告](fast_livo2_online_localization_report.md)；实物操作未授权。

- 正式FAST-LIVO2试验 `img_time_offset=+0.1 s`，由用户明确要求，原包/驱动header不改；曝光中心/剩余相位未实测。新390秒包已完成+100ms整包前端/B联调，原两90秒包固定+100ms对照及C1/C2评估仍待完成。上游配置、作者解释与两包检查见 [扫描相位报告](fast_livo2_timestamp_phase_report.md)。
- Tag默认关闭；独立诊断配置仍 `offset_ns=0, time_verified=false`，没有真实VALID地标表。启用前明确其原图映射，与前端保持采样语义一致且不重复补偿。
- 生产仍为Lightning建图/定位、wheel连续里程计、terrain disabled；Nav2/Smac/MPPI与安全链保留。C1/C2是独立离线路线。
- 2026-11-05按用户新指令简化专用录包：默认执行、独立启动传感器并读取FAST profile，`--dry-run`仅预演，`--attach`才接入已有预览；其他入口仍保持原默认。相关21项检查通过；此项源码/测试/文档已纳入2026-11-06分组保存，验证及命令见[录包默认行为报告](fast_livo2_recording_defaults_report.md)。Agent未开设备，用户随后自行录得约390秒的新包，见下一条。
- 新走廊/教室包 `fast-livo-corridor_classroom_map01-182fdzyj` 已正常停录，390.38秒、97,587条消息、原DB约32.8GB，归档正式+100ms。停录后审查曾被Ctrl+C中断，原result/完成标志缺失，另存基本审查PASS，见[录制核验](fast_livo2_corridor_classroom_capture_report.md)。随后完整三融合重放PASS，3899对输出、2333万点前端PCD及轨迹图已交付，见[展示报告](fast_livo2_corridor_classroom_mapping_report.md)。额外B诊断306KF/108接受回环，但一项图目标增加检查FAIL，快照仅作诊断；原两90s包固定+100ms对照及C1/C2评估仍待做。
- 用户随后授权统一在线预建图：[操作说明](fast_livo2_online_mapping.md)、[交付报告](fast_livo2_online_mapping_report.md)。隔离I/O前端+B+录包/显示/封存实现；最终390秒原速DDS会话退出0，3900对、305KF/114接受回环、919文件校验与C1共用读取器加载PASS。整包三路输入payload与视图一致，IMU实际记录最大到达间隔12.51ms；此前确有约1.11秒接收延迟，失败记录保留。连续新鲜度仍出现短暂LOST，实物/长时/D未验。新入口默认dry-run，单独录包已补存去畸变云但仍不启动B；本阶段改动已纳入2026-11-06分组保存。
- 分支 `refactor/pluggable-navigation`。2026-11-05的7组提交见[历史检查点](local_git_checkpoint_20261105.md)。2026-11-06用户授权合理保存并上传Git；保存前HEAD为 `4f4977992438dcca6e84c5d2a904d7dea94d1f70`，刷新远端后领先15提交、无分叉，本轮新增4组，目标为 `origin/refactor/pluggable-navigation`。分组与核对见[Git检查点](git_checkpoint_20261106.md)，最终HEAD和远端状态以Git现查为准。原包、地图和被忽略资产继续保留，后续未提交修改仍须保护。
- 早期交接/本地提交之后，按用户新指令完成录包简化、新包展示及上述在线建图的软件/离线工作，Agent没有启动实物设备。新建图验证会话已结束；历史field/旁边K工程的保存状态不能作为当前活进程依据，下一次操作先按boot/PID身份重新现查，未获授权不接管或停止。

- 新入口 `localize_fast_livo2.sh` 默认dry-run，auto/manual、同一C1/C2、有界最新扫描调度、原采样年龄门控及录包/停止已实现；[指南](fast_livo2_online_localization.md)、[计划](FAST_LIVO2_ONLINE_LOCALIZATION_PLAN.md)、[报告](fast_livo2_online_localization_report.md)。新282秒包约51.494秒首次VALID、2309个不同有效扫描，之后无DEGRADED/LOST/ERROR；同包390秒2978个不同有效扫描，但仍有新鲜度LOST及自动恢复，不称连续性能全面通过。阶段改动已分组本地保存，未新增推送。

## 能力与验收边界

2026-11-06冷启动后续调研已核对KISS、Scan Context、STD/BTC、3D-BBS和iBTC一手资料；建议优先检查采样/局部特征、候选覆盖和临时支持验证，见[在线定位计划调研附录](FAST_LIVO2_ONLINE_LOCALIZATION_PLAN.md#2026-11-06-冷启动改进调研与建议未实施)。该句对应早期仅调研阶段；后续已实现v2及GICP默认，实际结果见上方最新状态和对照报告。

| 项目 | 已有证据 | 当前限制 |
| --- | --- | --- |
| A 三融合前端 | 锁定源码/补丁、真实运动重放；新+100ms原速整包3900对，I/O隔离构建 | 原两90秒对照、物理同步、标定/真值精度待验 |
| B 独立回环PGO | 旧包PCL/GICP与新原速在线会话；新305KF/114接受回环及原子归档 | 多环境误环、真值、连续新鲜度、实物/长时未验 |
| C1 先验定位 | 六自由度初值确认、持续跟踪、健康/失效/会话与桥接检查 | 新计算优化仅隔离安装，真实跨场景与D未验 |
| C2 全局重定位 | 无人工初值候选/粗细配准/多帧确认/恢复，已有不同采集包→C1功能试验 | 完整C2-4、真实相似地点/地图外负样本、绝对位置精度未验 |
| Tag增量 | 官方检测、双解/像素/时间契约、地标保存/补录、C2/B候选及C1监视，首轮123项软件检查 | 所有已报真实C2成功均Tag关闭；真实VALID表、Tag恢复/回环未验 |
| D 系统实车 | 原Nav2/公共链保留 | 本轮未实施；真实wheel历元、T_base_imu、接车/导航另验 |

## 最近证据与正确解读

- **正式+100ms修正**：[扫描相位报告](fast_livo2_timestamp_phase_report.md)。两新包各900扫描，单帧约99.83～100.33ms；1843相机会话帧均读最新writer，未复现旧漏写。正式配置由0改0.1；11项既有回归、实际参数加载和录包dry-run通过，不等于新性能结果。
- **+110ms C1/GICP对照**：[C1修正报告](fast_livo2_c1_correction_report.md)。525扫描×4控制；原PCL单初值524帧复现。冻结原Tag常量、同516跨访问观测，回投RMS P95约17.324→6.114px。完整直行895提交/894处理/1取消、520有效、DEGRADED 1、无LOST/ERROR，首VALID37.400s。仅显式选既有small_gicp，门槛与生产默认保持，新增6项独立/3项相关检查通过。
- **两包/地图对照**：[两包报告](fast_livo2_tag_two_capture_report.md)。转向训练冻结+110ms，直行独立采集验证；新图31KF/10LiDAR回环。直行PCL C2→C1首VALID37.000s、525有效；C1等待帧计算和评估取消记录已修复，74项相关检查通过，仅隔离安装。
- **早期归因证据**：[运动几何](fast_livo2_tag_motion_geometry_report.md)、[120秒诊断](fast_livo2_tag140_sync_diag02_report.md)、[旧60秒writer漏收](fast_livo2_tag140_consistency_report.md)保留，各自偏移/源码/数据条件不可混用。
- **最新交接/清理**：[在线建图交接报告](handoff_cleanup_online_mapping_20261105.md)。删除可从原包重建的`replay_view`及Python缓存，约16.24GiB；原包、正式地图、失败证据、构建和备份保留。此前约14.1GiB早期试录清理另见[历史报告](handoff_cleanup_20261105.md)。

上述像素误差是整条定位/地图/标定/时间/Tag观测链的一致性，不能称Tag单独精度或绝对定位误差。初始3px RMS/6px max尚未完成误差预算标定，小幅残差本身不等于Tag不可用。各轮测试有重叠，123/74/9/11不相加为独立验收总数。

## 当前数据与运行版本

当前Tag为tag36h11:1、实测140mm黑色边界，Tag与相机/雷达安装在两新包内和之间固定，用户已确认，无需再问。旧155mm实例单独保留。

- 当前在线建图来源：`bags/fast-livo-corridor_classroom_map01-182fdzyj/raw`；原两90秒对照来源为`bags/fast-livo-tag140_turn_03-bk8_pjyy/`、`bags/fast-livo-tag140_straight_04-ryzykq2l/`。另保留120秒诊断、60秒漏写原证据、旧建图包及测试固定基线，具体用途/SHA见HANDOFF。
- 当前新局部图：`outputs/fast-livo-tag140-two-capture-20261104/new_local_map/snapshot`，UUID `4ec68a99c626e1c6e2652506f5ecd8d6`，由+110ms转向轨迹生成，有重力对齐，非整层/真值地图。
- 新+100ms走廊/教室B图：`outputs/fast-livo-demo-online-mapping-20261105/full_replay_12/map`，UUID `253f573d7a7eabbc363a225ab192cc14`；1829386个关键帧地图点，独立访问定位尚未验收。新受管建图使用`online_frontend_install`、`online_install`，不替换下面既有安装。
- 既有受管默认为 `ws_fast_livo/backend_install/`；C1优化版本为 `ws_fast_livo/install_c1_confirmation_20261104/`。二者不同，不能通过读取源码便声称已部署；实际节点和库SHA见HANDOFF/报告。
- 后续可复用 `independent_time_source_20261104/`、`build_independent_time_20261104/` 的原零gyro初始化隔离来源；旧mean-bias诊断副本不混用。当前+100ms参数不自动更新任何已运行旧会话。

## 下一步与用户待办

在线预建图、私有在线定位及自有检索对照已按用户指令分组本地保存。后续地图/桥接、当前同录包核验、V2反馈修复、用户授权工作外参及完整公共TF离线联调已交付，修改尚未提交；[导航接入计划](FAST_NAVIGATION_INTEGRATION_PLAN.md)已更新。下一步是经单独硬件授权后，以修复驱动进行短测、确认每会话时间关系及真实公共TF。规划控制将按用户另行设计大改，不从旧计划自动延伸Smac/MPPI；独立盲测/负样本、真实Tag和D分别验收。

当前已有用户180秒三传感器＋wheel同录包pwr5lx7u，不再按缺包阻塞软件。录包默认自动复用/启动receive-only wheel，行为见最新录包报告；入口默认执行不构成Agent自动开设备授权。未获独立真值时不报告APE/RPE或绝对精度。

旧阶段的完整命令、许可、失败和资产记录见 [文档索引](README.md) 及带日期报告。本次精简当前入口，历史报告仍有复核用途，不按日期一刀切删除。
