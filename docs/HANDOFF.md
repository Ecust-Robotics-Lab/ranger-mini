# 新 Agent 接手指南

更新：2026-11-09。项目 `/home/lab307/ranger_nav`。**在线预建图软件流程已打通，390秒真实包原速联调通过；真机、连续新鲜度、长时和独立访问定位尚未完成验收。** 总计划及真实Tag辅助定位/回环仍未全面验收，过程记录以带日期报告为准。

2026-11-06后续：用户录得独立访问282秒包，要求检查后立即开发在线重定位。首版软件/离线已完成，新包原速无初值C2→C1有2309个不同有效扫描，首次VALID约51.494秒，之后持续跟踪；390秒同包仍有LOST并能恢复。先读[在线定位报告](fast_livo2_online_localization_report.md)、[操作](fast_livo2_online_localization.md)、[实施计划](FAST_LIVO2_ONLINE_LOCALIZATION_PLAN.md)。新入口默认dry-run、隔离online_localization_install；本阶段后续已分组本地保存，见[检查点](git_checkpoint_online_localization_20261106.md)，未新增推送/未开设备。此更新优先于下方上一轮“停在交接”的历史范围，后续不自动开设备或扩展D。

2026-11-06再后续：自有v2与官方STD/Scan Context已做离线同条件比较；同一新282秒包v2/PCL首VALID24.394秒、v2/GICP22.315秒。在线入口默认自有v2，C1/C2入口默认small_gicp。先读[本轮对照及默认切换](fast_livo2_retrieval_comparison_report.md)；参考标签为估计轨迹，GICP有短暂拒绝，无LOST。原390秒限制未重新验收，改动已本地保存、未新增推送。

2026-11-06最新：用户要求先做好导航前置工作，并表示规划控制随后大改。本轮已实现3D正式归档→5cm占据候选图、隔离受管地图/既有C1适配/公共TF桥接、占据图与C1地图身份绑定及传感器静态TF，8项针对性检查通过；见[前置接入报告](fast_livo2_navigation_prep_report.md)。入口`bridge_fast_livo2_offline.sh`默认dry-run、domain219，正式候选`outputs/fast-livo-demo-navigation-prep-20261106/navigation_map_final/`。用户授权按Ranger Mini V2模型选取工程base_link，采用轴间中点的地面投影；当前估算`T_base_imu=[0.120,0.02329,0.55588]`m、RPY0，独立于生产runtime。用户确认没有当前FAST＋wheel同录包，愿意重录；已有`--record-wheel`可用，同录用于查明当前真实时间关系，不伪造映射。本轮未开硬件、未提交/推送，修改留在工作树。计划已更新，不自动进入旧Smac/MPPI方案或实物运动。

## 1. 本轮边界与快速阅读

2026-11-09交接保存：用户要求开新窗口前分组本地提交，源码、配置、证据与报告已按本次范围保存，无推送；见[Git检查点](git_checkpoint_handoff_20261109.md)。新窗口直接使用[NEXT_AGENT_PROMPT](NEXT_AGENT_PROMPT.md)，先只读核对并汇报。最新软件修复仍未实机验收，不从本地保存自动续开设备。

2026-11-09实施最新：先读[相机/局部恢复实施报告](fast_livo2_failure_fix_report_20261109.md)。用户已明确要求执行研究计划；相机取帧发布分线程/有限队列/迟到检查、C2短时局部恢复已实施并构建。36核心＋2新ROS＋3在线＋2C2回归通过，原CSV及原始点云局部恢复PASS。相机隔离latency_install已接在线定位dry-run，原相机安装保留。无新硬件操作，实际USB等待根因和实车连续性未验，全局召回/初始化缓存后续优化未实施；未提交/推送。

2026-11-09后续研究最新：用户要求网络搜索和解决方案，已完成[故障研究](fast_livo2_failure_research_20261109.md)及原包只读统计。新增事实：相机是USB3；540迟到旧图获得多前进约0.4秒的scan时间但标OK，随后四帧BACKWARD。恢复的39轮已记录失败搜索中位63ms，205粗配准候选失败；建议C2增加有旧地点先验的短时局部恢复，不能把34秒都归咎GICP算力。方案尚未实施，原包/未提交修改保留，无新设备启动。研究报告给出分段计时、时间关联、局部恢复与离线验收顺序。

2026-11-09第三次现场最新：先读[第三次90秒/剩余阻塞](fast_livo2_field_gate_acceptance_report_20261109.md)。用户“开始吧”授权录得88k5eclm，90.045秒正常结束；wheel不领先clock、TF唯一端点已实测通过，整体连续就绪仍PARTIAL。相机交付迟到约0.53秒，之后4帧同步BACKWARD，FAST间隔0.66秒导致C1过期，约34秒恢复；58个不同有效确认。全部设备已停止，不自动再测；先用现有raw/相机CSV/SDK日志离线分解，不能先归咎纯GICP或放宽新鲜度。规划控制软件可进入，实车自主闭环仍需验收。原包/失败/工作树保留，未提交。

2026-11-09第二次现场最新：用户明确“重新启动”后，jbwq4yhi完成90.027秒真实采集，稳定锚点已实测生效，486个不同有效C1采样、首次距首FAST42.400秒，公共TF已有输出。但就绪296次状态段切换，连续接入仍PARTIAL。读[复测报告](fast_livo2_field_retest_report_20261109.md)。已查出并离线修正wheel领先离散clock约20ms，以及底盘关闭TF仍构造空发布端点；有限队列等待后续clock、条件创建广播器已隔离验证/构建。新驱动df58f4…尚未启动，原安装保留。当前全部设备会话STOPPED，无残留，未自动重启；新原包/前次失败均保留，未提交。

2026-11-09现场最新：先读[现场联调失败/启动锚点修正](fast_livo2_field_localization_report_20261109.md)。用户已确认本次90秒实物测试；初次驱动加载失败后补base环境，在um88vo7b完成设备首次启动/37.945秒提前封包，但首批雷达积压污染第一帧时钟锚点，零VALID/零就绪，验收FAIL。全部本次设备进程已停止；CAN由DOWN启为UP500k并保留。现已改为在线定位启动稳定窗口＋monitor唯一锚点，失败包离线及合成ROS验证通过，未再启动实物。原始数据/失败记录保留，后续须明确授权新一轮设备启动，不把此前软件PASS或本次封包正常当实车定位通过。

2026-11-09最新继续：读[在线公共TF联调与短测准备](fast_livo2_live_chain_report_20261109.md)。实际时间转换＋C1适配器＋桥接的隔离ROS序列通过wheel/receipt断流恢复、锚点异常锁定；合成C1不作真实C2重跑。显式`--public-tf --auto-wheel --navigation-map`复用录包只接收驱动准备与base_control资源锁，已有源借用，自启源随会话清理；默认仍不自动启动。新增录后navigation_audit.json，公共模式未观察到就绪或发生时间/创新错误则退出2。现场90秒命令已准备，尚未启动硬件/运动；源码未提交。

2026-11-09再后续：先读[在线每会话时间映射报告](fast_livo2_live_clock_report_20261109.md)。新增live_clock/wheel_clock_node，复用现有雷达首帧header/steady锚点转换原wheel采样时间；新增显式`localize_fast_livo2.sh --public-tf --navigation-map`复用既有公共桥接/地图/静态TF，默认仍dry-run/私有定位，不自启底盘。4项核心＋1项隔离ROS检查、真实包记录时间代理审查通过。该偏移含未知接收延迟，不当作物理同步或真实公共TF验收；下一步为修复驱动后的实物短测，尚未执行。工作树未提交，原有改动保护。

2026-11-09后续：已完成本包时间映射封装与实际C2/C1→适配器→同刻wheel→公共TF的完整180秒隔离联调，先读[离线完整链路报告](fast_livo2_offline_wheel_chain_report_20261109.md)。最终1801对、1502有效、首次VALID源时间28.800秒；1494次TF代数一致性检查、断流撤销/恢复、尾部过期、唯一TF/零速度发布者通过，全部子进程退出0。新入口`replay_fast_livo2_wheel.sh`默认dry-run。使用本包绑定时钟及旧wheel诊断重建；raw负对照按20.6°创新锁定拒绝。检查器旧失败保留，物理同步/定位绝对精度/真实公共TF仍未验。没有开硬件、改规划控制、提交或推送。

2026-11-09最新：用户180秒同录包pwr5lx7u已核验，原包完整、8话题PASS、自启receive-only进程已清理。先读[wheel核验/转向修复/工作外参报告](fast_livo2_wheel_alignment_report_20261109.md)：V2反馈角符号修复已进入隔离录包构建；用户授权约[1°,21°,-179°]暂用，FAST profile、C1/C2和公共适配器已统一工作外参。旧零旋转只是早期假设，不再作为本轮FAST工作值；生产runtime/原底盘安装不变。候选时钟映射仍仅诊断，本轮未实测公共wheel/TF、未开硬件或导航，全部源码/报告尚未提交，须保护工作树。

2026-11-06最新录包行为：用户要求自动启动底盘并保持遥控采集，已实现默认auto-wheel，已有新鲜发布者复用、没有则检查跨域占用/启用can0/自启只接收驱动，`--no-wheel`关闭。驱动在`ws_base/install_recording_rx`，不请求模式、不订阅速度、退出不发零速；原安装/现有base会话保护，28项Python及生命周期shim/机型检查通过。先读[自动wheel报告](fast_livo2_recording_wheel_report.md)，本轮未开新的实物录制或重启底盘，源码尚未提交；旧文档“--record-wheel只接受已有发布者”不再是当前默认。

2026-11-06最新实物例外：用户随后明确授权尝试CAN/底盘wheel启动，并自行完成免密CAN配置。当前记录为base会话`run-qqesz8kq`、domain175/localhost、约50Hz wheel，检查结束时保留运行；Agent未开传感器/录包/导航或发速度。发现旧驱动V2故障解码截断及控制模式枚举不符，原始0x211显示异常2/待机0/故障`0x00000200`，不能按ROS error_code=0判运动就绪。读[本次启动检查](wheel_start_report_20261106.md)，再按boot/PID现查，不自动续开控制或清除保护。

2026-11-05后续已按用户指令完成统一在线预建图的软件与离线范围，见[操作说明](fast_livo2_online_mapping.md)、[交付报告](fast_livo2_online_mapping_report.md)。最终原速390秒会话3900对、305KF/114接受回环，录制/封存/文件与共用读取器加载PASS；实物、连续新鲜度、长时和D仍待验。此前交接清理与7组本地提交见[本地检查点](local_git_checkpoint_20261105.md)。2026-11-06用户另行要求合理保存并上传Git，本阶段源码、配置、证据及交接已分组保存，见[Git检查点](git_checkpoint_20261106.md)。Agent未开实物设备。下一Agent先核对实际状态和未提交改动，不从历史计划自动续跑或开设备。

依次阅读：

1. [AGENTS](../AGENTS.md)、[STATUS](STATUS.md)、[总计划](FAST_LIVO2_BACKEND_PLAN.md)。
2. [AprilTag 计划](FAST_LIVO2_APRILTAG_PLAN.md)、[前端契约](fast_livo2_interfaces.md)、[Tag 契约](fast_livo2_apriltag_interfaces.md)。
3. 当前重点：[在线预建图报告](fast_livo2_online_mapping_report.md)、[操作说明](fast_livo2_online_mapping.md)、[本次交接清理](handoff_cleanup_online_mapping_20261105.md)。原390秒包的录制中断审查见[录制核验](fast_livo2_corridor_classroom_capture_report.md)。
4. 涉及对应模块再读 [B](fast_livo2_backend_interfaces.md)、[C1](fast_livo2_localizer_interfaces.md)、[C2](fast_livo2_relocalizer_interfaces.md)；目录与入口见 [PROJECT_LAYOUT](PROJECT_LAYOUT.md)、[README](../README.md)。
5. 时间/Tag后续工作再读[扫描相位与正式 +100 ms](fast_livo2_timestamp_phase_report.md)、[C1修正/GICP对照](fast_livo2_c1_correction_report.md)、[两份新包/地图对照](fast_livo2_tag_two_capture_report.md)，不将其中+110ms结果追认为新配置结果。

可直接复制的开场提示词见 [NEXT_AGENT_PROMPT](NEXT_AGENT_PROMPT.md)。最新清理见[在线建图交接清理](handoff_cleanup_online_mapping_20261105.md)；此前删除早期试录的历史记录见[早期清理](handoff_cleanup_20261105.md)。新清理已移除约16.24GiB的可再生`replay_view`及缓存，原始390秒包、`full_replay_12`、其他地图和失败证据均保留；重放前按最新清理报告重建视图，勿把历史命令中的旧路径当作仍然存在。

用户后续已要求简化专用录包入口：`record_fast_livo2.sh` 默认执行、独立启动传感器并读取FAST profile；只看计划必须显式 `--dry-run`，接入已有预览必须 `--attach`。其他入口的dry-run默认不变。实现与隔离验证见[录包默认行为报告](fast_livo2_recording_defaults_report.md)；本轮没有启动真实设备。

正式在线预建图用`map_fast_livo2.sh --execute`，FAST profile的统一online入口也转到它。它默认dry-run，在单会话中启动前端/B/录包/显示，停止先封存再清理。实物建图使用固定雷达header/steady锚点的私有时钟，不等于wheel同步。单独录包现也补存去畸变云，但不启动B；旧走廊包缺该话题的事实不改。新数据面DDS共享段256MiB/队列8192，低带宽封存控制面仅回环UDP，均为会话内配置，不改系统网络。

## 2. 当前最重要的事实

- **正式前端试验参数现为 `time_offset.img_time_offset: 0.1`，即 +100 ms。** 用户已明确指定，勿退回 0 或改成拟合的 +110 ms。只改前端有效采样时间，原始 bag、相机/雷达驱动 header 不改；`exposure_time_init` 与 IMU/LiDAR 偏移仍为 0。
- LIV_handhold_2 锁定配置也为 +0.1；作者解释过雷达收完整帧才发布、共享值仍是扫描起始戳造成一帧差。新两包各900扫描跨度约100 ms，1843相机会话帧均读当时最新 writer；这支持固定扫描相位机制，不等于曝光中心/剩余相位已测定。
- **原两90秒包固定+100 ms完整对照及C2/C1评估仍待做。** 用户随后新录的390秒走廊/教室包已完成+100ms全量三融合和前端PCD/轨迹展示；额外B有一项图目标检查FAIL，见[展示报告](fast_livo2_corridor_classroom_mapping_report.md)。历史+110ms地图/轨迹/6.11px结果保留原条件，不追认为+100ms。
- 该B失败快照保留原FAIL。后续修复并新建的+100ms地图在`outputs/fast-livo-demo-online-mapping-20261105/full_replay_12/map`，UUID`253f573d7a7eabbc363a225ab192cc14`、graph419、305KF/114回环；原三路payload完整保真、305关键帧输入逐份核验。新图不追认旧结果，也没有完成独立访问定位或绝对精度验收。
- `config/fast_livo2/apriltag_camera.json` 仍为 `offset_ns=0, time_verified=false` 的既有诊断配置，Tag 功能默认关闭；不能把它当作与新前端匹配的已确认映射提交候选。以后原图分别进入前端与 Tag 时各自明确映射一次，不给已经修正的位姿戳再加一次。
- AprilTag 候选、双解、地标归档、旧图补录、C2/B 接入及监视的主体软件和合成/隔离检查已交付。**没有真实 VALID Tag 地标表；所有已报告真实包的 C2 成功均是 Tag 关闭时的纯 LiDAR 路径。**
- 既有 small_gicp 已在项目中实现，本轮仅通过离线 `--registration small_gicp` 选择它，影响 C1 粗/精与 C2 精配准；未换 FAST 前端、Tag PnP、KISS 全局粗配准或 B 建图默认，未部署新的生产版本。

用户已确认，不要重复询问：当前新部署为 **tag36h11、ID 01=1、打印后实测黑色检测边界140 mm**；Tag在两新包内及两包之间固定，传感器刚性安装未改变，组件在车上手动操控。新尺寸表 `config/fast_livo2/apriltag_sizes_capture_140mm.json`。旧155 mm实例及旧输出单独保留，不能按同一个ID合并。新包与早期建图包为同一场地；这不证明旧图完整覆盖或绝对精度。

## 3. 已验证能力及正确解释

| 内容 | 已有结果 | 不能据此推断 |
| --- | --- | --- |
| A/B/C1/C2 | 本机构建、合成/故障/接口、同包功能及部分不同采集包的全局搜索→C1跟踪 | 完整 C2-4、多地点负样本、真值、实车实时/长时、D 已验收 |
| Tag 软件 | 首轮123项软件/合成/隔离检查；后续有相关回归，数量有重叠 | 所有真实验收已完成或可相加成互不重复的总数 |
| +110 ms 新图直行/PCL | 895提交、894处理、1次合法超时取消，525有效，首VALID约37.000 s | 当前 +100 ms 的结果、实时吞吐、未来整层图恢复时延 |
| +110 ms 新图直行/GICP | 895提交、894处理、1取消、520有效、DEGRADED 1，无LOST/ERROR，首VALID约37.400 s | GICP启动更快、Tag辅助成功、位置绝对正确 |
| 冻结原Tag常量的跨访问对照 | 同516观测，像素RMS的P95约17.324→6.114 px，仅改查询使用的已有注册器 | Tag本身有6 px测量误差或机器人绝对定位误差已知 |

回投残差混合机器人定位、地图、Tag地图位姿、内外参、时间及角点误差。小残差非零正常；3 px RMS/6 px max是最初试验门槛，尚未完成整链误差预算标定，不能当普适物理标准，也不能为让当前数据通过而任意放宽。应验真实候选能否提高正确恢复、保持歧义并拒绝错误地点。

## 4. Git、源码和真实运行版本

当前分支 `refactor/pluggable-navigation`。2026-11-05的7组本地提交见[历史检查点](local_git_checkpoint_20261105.md)；2026-11-06保存前HEAD为 **`4f4977992438dcca6e84c5d2a904d7dea94d1f70`**，刷新远端后仍领先15提交、无分叉。本轮按用户新指令分4组保存119份既有改动及Git交接更新，上传目标为 `origin/refactor/pluggable-navigation`；分组与核对见[Git检查点](git_checkpoint_20261106.md)，最终HEAD及远端同步状态以Git现查为准。此前AprilTag、writer/录包、C1计算/评估、诊断和文档已归档提交；两份新增 `docs/03_evidence/driver-*` 单独成组。禁止reset/clean、覆盖后续未提交工作或把被忽略资产当垃圾；原包、地图及9份归档程序副本继续留在本机。

C1已提交基线 `1212b43`，C2 `2e0f06f`，重力 `4b49bb0`；此前B的三个提交已按历史指令推送。2026-11-06用户明确授权将此前未上传提交与本轮分组提交一并上传，不构成未来自动提交、推送或设备操作许可。

| 位置 | 当前用途/注意事项 |
| --- | --- |
| `vendor/fast_livo2_review/ros2_frontend` + `patches/fast_livo2_stage_a.patch` | 锁定ROS2前端源码；版本见 `config/fast_livo2/sources.lock.json` |
| `ws_fast_livo/src/ranger_fast_livo_backend` | B、Tag核心/检测/地标工具；正式依赖项目内前缀 |
| `ws_fast_livo/src/ranger_fast_livo_localizer` | 同一个C1/C2，含Tag候选/复核；不是新定位后端 |
| `ws_fast_livo/backend_install/` | 既有受管默认安装；不能当作最新隔离C1版本 |
| `ws_fast_livo/online_frontend_source/`、`online_frontend_install/`、`online_install/` | 本轮统一建图的I/O派生前端、严格图事务后端和C++收流计数；配置/来源/实际库见新报告，不替换原安装 |
| `ws_fast_livo/install_c1_confirmation_20261104/` | C1等待确认计算优化的隔离安装，未替换上行默认 |
| `ws_fast_livo/independent_time_source_20261104/`、`build_independent_time_20261104/` | 最新两包重放的锁定副本/程序，原零gyro初始化；可作固定+100 ms离线实验的来源 |
| `ws_fast_livo/motion_diag_source_20261104/` | 旧诊断含初始化零偏实验改动，不可混为正式锁定来源 |
| `ws_fast_livo/build_c1_correction_probe_20261104/` | 单扫描四条件配准控制实验程序，不是生产节点 |

实际隔离节点为 `ws_fast_livo/install_c1_confirmation_20261104/ranger_fast_livo_localizer/lib/ranger_fast_livo_localizer/localizer_node`，SHA **`99453aca95043aba1beb78f11fb1a08fb0805140c964e74ec7ad44a32d0bb619`**。动态库与构建证据见 `docs/fast_livo2_evidence/tag_two_capture_20261104/optimized_live_linkage.json`；不要只凭目录存在认定装载版本正确。

原安装前端/生成harness与锁定源码曾发现不一致；后续实验需记录源SHA、补丁、二进制、参数和动态库。新两包录制时前端SHA `dfcf5d4d54f7b81f573a000589527afa36f4dc71c5fe44df1c94f5bff9ae7b27`，不是“任意本地构建都相同”的证明。

关键已完成修改勿重做：writer RELIABLE/depth5、DDS诊断与v2共享一致性；录包 PREPARING/WAITING/RECORDING及输入缺失报错；C1等待帧减少重复13初值搜索，但首支持/可提交帧仍全检；`scripts/fast_livo2/localizer/validate_motion.py`明确区分超时取消与同戳处理确认，取消不算处理成功。

## 5. 当前保留的数据、地图和展示

| 资产 | 路径及作用 |
| --- | --- |
| 转向90s，下一轮训练/建图来源 | `bags/fast-livo-tag140_turn_03-bk8_pjyy/raw` |
| 新独立重定位访问282s | `bags/fast-livo-relocalization_visit01-trpozzyj/raw`；2821对位姿/云，七话题基本检查PASS，原审查中断现场保留，见[核验](fast_livo2_relocalization_visit01_capture_report.md)；地图覆盖/定位未验 |
| 新走廊/教室390s，用户随后自行录制 | `bags/fast-livo-corridor_classroom_map01-182fdzyj/raw`；归档+100ms，原DB SHA `b6f7902c1b5fbffbaea69bf241efff540d48c0a85358d1267a4f98d468496137`；停录后审查曾中断，另存基本审查PASS，见[录制核验](fast_livo2_corridor_classroom_capture_report.md) |
| 直行90s，独立采集查询 | `bags/fast-livo-tag140_straight_04-ryzykq2l/raw`；已经参与过方法评估，今后不能称全新盲测 |
| 120s同步诊断 | `bags/fast-livo-tag140_sync_diag_02-35f_m8nd/raw`；1200扫描，writer均覆盖 |
| 60s漏写证据 | `bags/fast-livo-tag140_check_01-dguhgx2l/raw`；7个漏写与重复共享戳对应，独立失败证据保留 |
| 早期136.6s三融合/旧图来源 | `bags/fast-livo-motion-w5phdosj/raw`；不是完整180s，也非真值 |
| +110 ms两包前端与新地图 | `outputs/fast-livo-tag140-two-capture-20261104/` |
| 新局部地图 | 上行 `new_local_map/snapshot`：UUID `4ec68a99c626e1c6e2652506f5ecd8d6`，graph41，31KF/10已接受LiDAR回环；**不是整层地图** |
| 旧B图 | `outputs/fast-livo-backend-validation-20261028-{pcl,gicp}/snapshot`；PCL UUID `6acd0a5763786389402bf72f397d9cbc`，49KF/13loop |
| 最新GICP控制与完整运行 | `outputs/fast-livo-c1-correction-20261104/`；证据 `docs/fast_livo2_evidence/c1_correction_20261104/` |
| 静态测试固定输入 | `bags/fast-livo2-baseline-126a5fAo`；现有ROS回归引用，不可删 |

转向/直行原DB SHA分别 **`23b3bdba0c4ac5497732eaae61f256bef74d57c52f0181efe58b7f5fe7a48e81`**、**`e6c7cd1b2d200b4629abfa7002458ace896e3c6a3b2268cc673145d21f400b6d`**。源包/配置、地图manifest与二进制完整性见两包及C1报告的最终核对。

重用实验脚本前先读参数与默认值，输出必须新目录：`docs/fast_livo2_evidence/tag_two_capture_20261104/replays.py`默认offset为0、只应显式传100000000；`validate_optimized_recovery.py`指向隔离新C1；`docs/fast_livo2_evidence/c1_correction_20261104/run_gicp_checks.sh`归档的是+110ms历史命令，带--execute且原输出已存在，**不要直接执行覆盖或当成新+100ms入口**。

可展示图片：`docs/fast_livo2_evidence/c1_correction_20261104/runtime_results.png`、`tag_reprojection_example.png`、`correction_controls.png`；地图/独立访问图在 `docs/fast_livo2_evidence/tag_two_capture_20261104/map_and_independent_visit.png`；中文图注与ZIP见 `outputs/fast-livo-apriltag-presentation-20261104/README.md`。黄角点是检测值，不是外部真值。

## 6. 时间、坐标和安全边界

- 保持三融合；Tag不注入ESIKF，不回灌前端状态/视觉地图，不新增在线Tag图因子。Tag给候选，原LiDAR配准/验证和合法提交决定是否接受。
- `T_A_B`将B点变到A；后端仅用同戳 `/fast_livo/odometry` 与 `/fast_livo/deskewed_imu`，VIO后IMU参考点、单区间机体系云，外参只应用一次。
- 本次前端重力对齐已开启；旧20261028 B图未对齐，新20261104局部图使用对齐后的+110ms轨迹。逐资产区别，不能一概称“所有地图未对齐”或默认锁roll/pitch。
- sensor虚拟UTC约2020、host/wheel约2026，不能直接拼接或重打now。离线用独立header-clock视图/sim_time；原包不改。
- 公共唯一桥接发布TF，按同采样时刻插值wheel，禁止外推/latest拼接。私有候选成功不绕过LOCKED，不删锁、不放宽运动/几何要求。
- Lightning/wheel/terrain disabled仍为生产默认；FAST前端/B不是先验地图定位器。硬件、底盘/CAN、导航与部署另按该次明确指令。

## 7. 下一步与首次只读核对

本轮停在交接，新Agent先汇报理解和建议，不自动开展实验。用户后续授权时按目标选择：

1. 在线预建图验收：先核对冻结配置、真实程序和会话归属；获得该次硬件许可后，再验证真机采集/前端/B/录包/封存。保留真实接收停顿证据，并查明整包中45个LOST后端心跳的原因，连续新鲜度尚不能标PASS。
2. 先验定位/重定位：绑定新地图UUID和实际查询来源，用独立访问分别验C1跟踪、C2全局初始化与丢失恢复。共用读取器加载PASS不代表完成这些验收。
3. 原两90秒包固定+100ms对照、Tag时间映射/真实地标表仍是单独待办；收到对应指令后复用已有数据与实现，不默认接续旧Tag计划，也不追认+110ms结果。

已有数据足够继续软件与离线验证，当前无需重录同类包。新独立访问、曝光测量、长时、真值及wheel/D测试如需采集，说明具体用途并遵循该次授权。

```bash
cd /home/lab307/ranger_nav
git status --short --untracked-files=all
git log -8 --oneline
git branch -vv
python3 scripts/runtime_config.py
bash scripts/map_fast_livo2.sh --status
bash scripts/run_fast_livo2_live.sh --status
bash scripts/record_fast_livo2.sh --status
bash scripts/run_fast_livo2_localizer.sh --status
bash scripts/run_fast_livo2_relocalizer.sh --status
```

2026-11-05接手实查：FAST live/record为STOPPED，B/C1/C2的历史故障测试状态为FAILED；五者均owned=[]且不存活，不能当线上故障自动重启。另一个受管field会话 `run-u0m5bw9a` 存活、状态DEGRADED，实际有Lightning、底盘驱动和公共桥接进程；旁边 `ranger_nav_k_integration` 工作区另有雷达时间归一化和Nav2相关进程，ROS_LOCALHOST_ONLY=0。没有据此验证车辆运动、就绪或新时间方案，也没有干预既有会话。新Agent仍须现查，不能把FAST会话停止写成整机停止。清理和交接前副本/完整diff在 `.runtime/backups/handoff_cleanup_20261105_171706/`；不是所有大文件备份，已删除原录包无法从这些元数据恢复。本次提交前清单和diff另存 `.runtime/backups/local_git_checkpoint_20261105_173931/`。
