# FAST-LIVO2 建图、先验定位与重定位总计划

更新：2026-10-29。总计划未全部完成；用户已于本日将后续工作转交新上下文，本轮结束实施，仅维护交接。本文件统一维护架构、参考项目、阶段任务、接口、数据和验收要求；实际完成情况以 [STATUS](STATUS.md) 与带日期报告为准。**计划不是后续阶段或硬件操作授权。** C1 已按后续明确指令实施离线范围，见 [C1 报告](fast_livo2_stage_c1_report.md)；后续授权的 C2 离线范围也已实现，见 [C2 报告](fast_livo2_stage_c2_report.md)。硬件不在本轮范围。

## 1. 当前基线、目标与顺序

目标：保留 FAST-LIVO2 LiDAR–IMU–Camera 前端，提供独立回环建图、地图归档、给定初值定位和无初值重定位，复用现有定位契约接入 Ranger 工程。

| 阶段 | 内容 | 截至本次更新的事实 |
| --- | --- | --- |
| A | 三融合前端迁移、输出契约与适配 | 本机构建、静态回放、真实在线预览完成；物理同步/外参精度仍 PARTIAL |
| B | 关键帧、回环、Pose3 iSAM2、传播和地图快照 | 已实现，单份三传感器运动包的独立后端离线闭环管线通过 |
| B 补充验证 | 多场景误环、重复结构、实时与长时运行 | 证据不足，不称为实车全面验收完成 |
| C1 | 归档加载、人工给定初值、持续先验地图定位 | 离线实现；合成、故障、同包功能及桥接结果见 C1 报告，独立会话/实车仍 PARTIAL |
| C2 | 全局地点检索、无初值粗配准、多帧确认与恢复 | 已完成离线源码/构建/合成与同包功能检查；独立会话、真实负样本、真值与实车仍 PARTIAL |
| D | 跨会话定位、系统联调和实车验收 | 未完成；硬件、底盘、导航按各次明确授权执行 |
| 可选离线精修 | HBA / Global-LVBA 等 | 预留归档转换，不作为 C1/C2 前置条件 |

B 的实际证据：PCL ICP 与 small_gicp 各处理 1362 对输入、49 个关键帧、13 条接受并优化的回环约束；16 项原验收命令和 3 项新增测试通过。详见 [运动报告](fast_livo2_stage_b_motion_report.md)、[B 初次交付](fast_livo2_stage_b_report.md) 和 [B 接口](fast_livo2_backend_interfaces.md)。这些不是独立真值精度、误环率或实时性认证。

生产默认继续为 Lightning 建图/定位、wheel 连续里程计、terrain disabled。阶段 B 后端不是先验地图定位器，不能仅修改 runtime 名称就作为全局定位后端。

推荐顺序：复核 B/C1/C2 证据与资产 → 补独立会话数据 → 分阶段 D 验收。B 多环境验证、C1 跨会话验证、同步与标定证据仍需补充；缺这些数据不妨碍可执行的离线开发，但限制实车结论。

## 2. 架构与不可变约束

```text
LiDAR + IMU + Camera → FAST-LIVO2 → 同刻 IMU 位姿与单区间 IMU 云
                                    ├─ 建图：B 回环/PGO → 不可变地图归档
                                    └─ 定位：C1 跟踪 ← C2 初始化/恢复候选
                                                   ↓ 私有结果
                                             标准定位适配器
                                                   ↓ LocalizationEstimate
wheel 历史 → 同刻插值 → 现有唯一公共桥接 → map→odom→base_link
```

- 建图与先验定位作为互斥运行模式；共享库可复用，不能同时向同一公共定位入口争抢输出。
- 保持三融合，不回灌 ESIKF、视觉局部地图或前端内部状态；在线前端恢复另立任务。
- `T_A_B` 将 B 中点变到 A。后端输入为 `/fast_livo/odometry` 与 `/fast_livo/deskewed_imu`，严格整数纳秒配对，不使用最新位姿或近似时间匹配。
- 输入是 VIO 后同刻 IMU 位姿及已应用一次 LiDAR–IMU 外参的区间云；VIO 未重算整段点级去畸变。同戳不证明曝光物理同步，见 [前端契约](fast_livo2_interfaces.md)。
- 新前端试验配置已设 `gravity_align_en=true`，IMU 初始化后将估计重力对齐世界 −Z（+Z 朝上），见 [重力对齐报告](fast_livo2_gravity_alignment_report.md)。既有 B/C1 地图由关闭对齐的会话生成，保留原坐标系；未经逐资产验证不可使用 XY/高度筛选或假设仅 yaw 不同。
- 当前沿用进程代际/ROS 时钟回退隔离。同进程前端坐标重置若将来引入，须新增明确事件，不能只靠 DDS GID 检测。
- ROS 时钟用于采样历元/新鲜度/回放；steady clock 用于耗时与任务超时。异步结果须携带地图、会话、请求代际和依赖，旧任务不得提交。
- 私有节点不发布公共 TF；仅显式离线联调接适配器和公共桥接，使用 localhost、隔离 domain、模拟时间。合成 wheel 只属于测试。

核心公式：

```text
U_i = T_local_imu(i), X_i = T_map_imu(i)
B 相邻因子：Z_ij = inverse(U_i) × U_j
B 历史子图：p_i = inverse(X_i) × X_k × p_k
B 非关键帧：X(t) = X_k_opt × inverse(U_k) × U(t)
C1 预测：X_pred(t) = X_a_accepted × inverse(U_a) × U(t)
适配：T_map_base(t) = T_map_imu(t) × inverse(T_base_imu)
桥接：T_map_odom(t) = T_map_base(t) × inverse(T_odom_base(t))
```

参考 a 必须为同一定位会话中不晚于 t 的已接受定位，不跨前端重置传播。心跳不刷新位姿采样时间；预测不能冒充新的地图几何确认。公共桥接继续禁止 wheel 外推、latest-TF 拼接以及拒绝后污染修正缓存。

## 3. 参考项目、借鉴范围与版本许可

以下是固定版本的历史审查与选型，不声称核对了上游最新版本。构建来源见 [sources.lock.json](../config/fast_livo2/sources.lock.json)，参考审查见 [B reference_audit](fast_livo2_evidence/stage_b_20261026/reference_audit.json)、[A 源码审查](fast_livo2_source_review.md)。参考审查项不全部属于构建依赖。

| 项目与来源 | 当前角色 | 具体借鉴与边界 |
| --- | --- | --- |
| [FAST-LIVO2 官方](https://github.com/hku-mars/FAST-LIVO2) / [ROS 2 前端](https://github.com/Robotic-Developer-Road/FAST-LIVO2) | 已部署前端 | ROS 2 humble 固定源码加本项目补丁，保持三融合，不替换成 FAST-LIO2 |
| [SimplePGO / FASTLIO2_ROS2](https://github.com/liangheming/FASTLIO2_ROS2) | B 已借鉴，C1 参考 | 关键帧、相邻因子、子图、iSAM2、逐帧地图重建；C1 参考 `icp_localizer.cpp` 粗细配准。原版配对、队列、状态、保存覆盖、时钟和 TF 不沿用 |
| [GTSAM](https://github.com/borglab/gtsam) | B 项目内链接依赖 | Pose3、Prior/BetweenFactor、iSAM2 和鲁棒核；固定 4.2 提交、系统 Eigen，不安装替换全局库 |
| [small_gicp](https://github.com/koide3/small_gicp) | B 已集成，C1 可复用 | GICP/VGICP，保留 PCL ICP 基线；统一总变换、指标和失败语义。Hessian 逆不直接当作已标定协方差 |
| [KISS-Matcher，含已审查的 SAM 相关代码](https://github.com/MIT-SPARK/KISS-Matcher) | C2 可选粗配准已集成 | 外部里程计接入、候选与配准分层、粗细配准和内点接口；不导入整套 SAM。配准器不自动完成全局地点检索 |
| [SC-A-LOAM / Scan Context](https://github.com/gisbi-kim/SC-A-LOAM) | C2 首个原型优先评估项，未集成 | 候选地点/航向及独立几何验证；已完成自有极坐标数学原型的合成对照；上游核心许可未解决，不复制；当前采用三维距离分布原型，不因 Mid-360S 型号直接排除 |
| [STD](https://github.com/hku-mars/STD) | C2 候选接口参考，未集成 | 几何描述子和搜索；描述符不变性不保证所有环境可靠定位，必须另做几何验证 |
| [iBTC](https://github.com/hku-mars/iBTC) | C2 备选参考，未集成 | 候选与数据结构；不同时引入多套检测器，不预设性能排名 |
| [HoloAgent](https://github.com/HorizonRobotics/HoloAgent) | 已审查，C1/C2 架构参考 | 多帧初始化、增量预测、持续定位和失败管理；不导入框架/CUDA，不沿用 now 戳和 map→base TF |
| [OpenFleX mapping_localization_layer](https://github.com/OpenFleX-Wheeled-Humanoid/mapping_localization_layer) | 已审查的对照参考 | SC→配准调用链、地图组织与状态缺陷；不沿用 latest-TF 回退、默认平面化和 TF 所有权 |
| [SC_FAST_LIVO2](https://github.com/jxk6575/SC_FAST_LIVO2) | 已审查，未集成 | 仅理解外部优化接口；其滤波器/局部地图反馈不属于本项目路线 |
| [HBA](https://github.com/hku-mars/HBA) | 后续独立离线精修候选 | 不在 C1/C2 集成；独立仓库版本和许可须另核对，不用参考后端 hba 子目录审查替代 |
| [Global-LVBA](https://github.com/xuankuzcr/Global-LVBA) | 已有格式审查，未集成 | 离线 LiDAR/视觉精修；保留可转换信息和原 bag 图像，不声称已导出完整 LVBA 数据包 |
| GLIM | 后续架构对照候选 | 暂不选用或替换前端；具体仓库、版本、许可与适用模块在专项审查时确认 |

已记录的完整提交：

| 来源 | 固定提交 |
| --- | --- |
| FAST-LIVO2 官方 | `0d2c0346107b75b59934975adec9a6eeeb913c64` |
| ROS 2 前端 humble | `837b7bbc1431cb04cf936528e52c83c835efba8e`，另加 `patches/fast_livo2_stage_a.patch` |
| SimplePGO / FASTLIO2_ROS2 | `7baa59f7339ba88ce3b646fe35b23f61031c5d3f` |
| GTSAM | `4f66a491ffc83cf092d0d818b11dc35135521612` |
| small_gicp | `fa0cfc983417c044b6036f71dfa448038bb9c820` |
| KISS-Matcher | `8b58b118e00c7a5a22448741f8d956d638a44ae2` |
| SC-A-LOAM | `321cb4b07b7815c114237ce5d87014d2e46b972c` |
| STD | `6e5903cfa2aaed5ca3c9e7d21424eebcca7655bb` |
| iBTC | `9f3bf49e26ff254d28043e8e5d404f3348da8ef2` |
| HoloAgent | `ef14d3152ca6246d8ae64920694c6c74581d246c` |
| OpenFleX mapping_localization_layer | `0e891ff05dc562868d84716933bb68fdc74476f4` |
| SC_FAST_LIVO2 | `f39e3d0008af7cbd838b05ccfbad794c66161596` |
| Global-LVBA | `10493a2b0f695e32db83dde49dad843a4826e090` |

许可与依赖要求：

C2 可选构建锁另见 `config/fast_livo2/relocalization_sources.lock.json`：KISS→ROBIN→PMC 包含 GPLv3+，不是纯 MIT 依赖链；固定 SHA、生成的本地依赖投影和保留许可详见 C2 报告。

- SimplePGO 对应包 MIT、small_gicp MIT、GTSAM BSD 及其单独许可组件；KISS-Matcher 根 MIT 仅为已审查范围的观察。保留 [THIRD_PARTY_NOTICES](../ws_fast_livo/src/ranger_fast_livo_backend/THIRD_PARTY_NOTICES.md) 和必要授权文本。
- SC-A-LOAM 已审查树中 package.xml 写 BSD，但未发现根 LICENSE；复用具体文件前必须完成组件核对，不宣称整个仓库许可已解决。
- STD README 含 GPLv2、个人/学术及商业联系措辞；iBTC README 写 GPLv2、package.xml TODO。检查实际文件及限制，不以元数据替代许可审查。
- HoloAgent 根 Apache-2.0 不覆盖 GPL 等子组件；OpenFleX 根 CC BY-NC-SA 4.0 与子包许可不同。优先参考架构/接口，不整体复制。
- 前端 GPLv2、vikit 逐文件/GPLv3 等组合分发问题未全部解决，本机测试不构成完整分发授权结论。
- 新增依赖先做最小 C++17/Humble 构建和实际链接核对，再锁定 SHA、许可、补丁及项目内前缀；HBA/GLIM 未完成专项审查，不编造版本。

GTSAM 固定提交同时对应 4.2/4.2.0 标签，数字版本 4.2.0、显示串含 a9，保留事实。SYSTEM_EIGEN=ON、MARCH_NATIVE=OFF、UNSTABLE/Python/MATLAB 等关闭；核对真实头路径、链接库、Eigen/SIMD/对齐。关闭 native 本身不能保证 ABI。

## 4. C1：给定初值的先验地图定位

### C1-0 工程结构与最小构建

本节保留 C1 任务与验收要求。自有 `ranger_fast_livo_localizer` 已落地，复用 B 导出的核心库；实现接口见 [C1 契约](fast_livo2_localizer_interfaces.md)，实际证据见 [C1 报告](fast_livo2_stage_c1_report.md)。vendor 原版 localizer 不等于本项目实现。

包位于 `ws_fast_livo/src/ranger_fast_livo_localizer`，负责只读地图、初始化、跟踪和私有诊断。复用 B 的输入检查、配准和数学约定；B 核心库补齐可链接导出，不复制整套实现，也不让 Lightning 依赖 FAST 专用消息才能启动。

拟实现模块：`MapArchiveReader`、有界 `MapSubmapProvider`、初始化器、跟踪器和会话状态管理。现有 `Registration::align(target, source, initial)` 可复用；现有 `CandidateSearch::find(Snapshot, query_id)` 面向 B 图内查询，C2 还需外部查询与只读地图索引接口，不能声称它已支持跨会话检索。

交付：包/库边界、私有入口、拟新增消息服务与默认 dry-run 构建脚本，先完成无硬件构建。

### C1-1 地图加载与兼容

直接消费 B 的 `schema_version=1 / kind=stage_b_graph_archive`，不另造一套无转换关系的格式。新进程加载必须检查：

1. `COMMITTED.manifest_sha256`、manifest 与全部文件 SHA；拒绝 partial、FAILED、未知版本/种类和缺文件。
2. 引用路径位于归档根内；拒绝路径越界/符号链接越界、重复 ID、非法位姿、非有限点、点数/字段异常；限制解析大小、点数和内存。
3. map UUID、冻结 graph version、IMU 云系、`U_local_imu`、`X_map_imu`、整数 `stamp_ns` 和变换方向一致。地图逐帧只变换一次，不能把世界云再次当 body 云处理。
4. 检查 provenance 中实际存在的源码、时间与标定身份。当前来源可能缺省或仅声明配置，不能当作物理校验通过。
5. 区分地图生成所用 LiDAR–IMU/同步配置与输出使用的当前 T_base_imu；变更要判断兼容性或拒绝，不能用当前配置伪造历史元数据。
6. 原 schema 缺必要信息时，新增带来源的兼容说明或派生索引并版本化，不改原档。证据不足的旧包只用于明确标记的离线试验。

读取 body 关键帧云及各自优化位姿，建立 3D 空间索引，按需生成 map 系目标子图并使用有界 LRU 缓存，不长期载入所有全分辨率云。派生索引绑定源 manifest SHA、算法版本与参数哈希，写新目录原子提交。

加载失败不破坏最后有效地图。切图时停止旧有效输出并提升请求代际，新图完整提交后重新初始化；“加载完成/服务受理”不等于“定位成功”。

交付：B 导出→退出→新进程加载的往返测试，以及损坏、版本、标定和资源失败测试。

### C1-2 初值接口与几何确认

- 拟提供私有初始化服务：map UUID、明确参考体（base_link 或 IMU）的 PoseWithCovarianceStamped、请求 ID；字段与服务名在实施时确认，不重设计公共协议。
- 初值 frame 必须匹配地图；时间须能关联当前前端会话。stamp=0 若允许，须显式定义为“绑定下一对有效输入”，不静默解释为 now。
- base 初值转 IMU：`T_map_imu = T_map_base × T_base_imu`；反向输出使用逆变换。非零 XYZ/RPY 测试必须覆盖两方向。
- 初值与扫描不同时，用同会话前端历史传播到扫描采样时刻；缺历史则等待/拒绝，不能用最新位姿拼接。
- 检查初值不确定度；如支持邻域多假设，明确位置/姿态范围与预算，不宣称 ICP 有任意米/角度的保证收敛范围。
- 粗细两级体素/配准参考 SimplePGO；source 为当前 IMU 云、target 为 map 子图，初值和最终总变换均为 `T_map_imu`。若目标在历史 i 系，则最终 `T_map_imu = X_i × T_i_query`。
- 验收包括收敛、对应数、覆盖/重叠、残差、退化、多解歧义及必要多帧支持。仅最低 fitness 不足以接受。

交付：请求受理→几何验证→首个有效位姿的完整状态链。错误初值保持未初始化，不误报成功。

### C1-3 持续定位与失效

用最后接受的地图定位和前端相对运动预测，更新附近子图并精配准。地图只读，不把当前扫描写入目标制造自匹配。

分开记录最后确认匹配、预测、最新扫描和健康时间；拒绝候选不能更新已接受修正，连续拒绝/过期触发 DEGRADED/LOST。前端预测不能冒充地图匹配成功或无限延续 VALID。

记录对应数、重叠定义、RMSE、退化、初值到最终偏差、歧义、目标版本及耗时。相同指标复用 B，定位门槛单独配置，B 默认值不直接当作定位已标定阈值。六自由度定位，不把 roll/pitch 锁为 0；可选重力约束默认关闭、另做对照。

LOST 后由操作员重新给定初值并完成确认，本阶段不暗中增加 C2 搜索。队列、子图、缓存、处理频率和单次预算明确上限；超时不发布过期 VALID。若库内求解不可硬中断，报告限制并用点数/迭代数控制预算。

交付：有地图匹配、无匹配、连续拒绝、失效再初始化和资源上限的测试。

### C1-4 私有状态、适配与公共桥接

复用公共 INITIALIZING、VALID、DEGRADED、LOST、ERROR；内部 map_loading / waiting_initial_pose / verifying / tracking 等用 reason 表达，不随意扩展公共枚举。

私有结果原子包含原采样戳、pose_valid、map UUID/归档 graph version、定位 session、初始化请求代际、参考扫描和质量诊断。不为复用 B 消息伪造 loop_event；如需新私有消息，在现有适配框架增加独立输入转换。

显式离线联调才启动适配器/公共桥接，覆盖外参方向、wheel 同刻插值、禁止外推、断流、心跳、首次创新失败锁定。分别记录私有定位确认和桥接接纳；桥接拒绝不迫使定位器丢弃正确结果，也不放宽门槛。

### C1-5 验收矩阵和完成条件

| 验证组 | 必须覆盖 | 证据 |
| --- | --- | --- |
| 地图往返 | B 导出→退出→新进程加载、各帧一次变换、版本一致 | manifest/SHA、逐点和位姿数值核查 |
| 加载故障 | 截断/缺文件/坏 SHA/越界、版本/字段/标定不兼容、超限 | 拒绝原因、原档及最后有效地图不受损 |
| 合成几何 | 三轴旋转/平移、外参、参考时刻、初值偏差网格、错误初值 | 已知真值误差、接受/拒绝表、耗时 |
| 跟踪 | 低重叠、走廊、相似地点、失效再给初值 | 状态转移、拒绝不污染缓存、无伪 VALID |
| 生命周期 | 重启、重复请求、切图、回退、旧任务、多发布者和缺对 | 会话隔离、资源上限、旧任务不提交 |
| 公共链 | 私有无 TF、桥接唯一、同刻 wheel、断流和大跳变 | 隔离 domain 日志和相关回归 |
| 数据泛化 | 同包功能检查与独立采集定位分别测试 | 分开报告，无真值不报 APE/RPE |

交付源码、锁/许可证、参数、加载/初值/状态契约、构建与离线脚本、原始日志和 C1 报告。数值/接口/故障实际通过可记 PASS；缺独立运动定位包不阻止最小实现，但跨会话、物理同步及真值性能保持 PARTIAL。

## 5. C2：无人工初值全局重定位

本节源码、构建、合成与同包功能检查已交付，实际代码/命令/证据见 [C2 报告](fast_livo2_stage_c2_report.md) 和 [契约](fast_livo2_relocalizer_interfaces.md)。这不表示 C2-0 至 C2-4 全部验收完成：C2-4 跨会话、真实负样本及真实条件验证仍 PARTIAL，不以同包功能代替。

前置：C1 可在新进程加载地图、验证给定初值并持续跟踪，私有状态和失效处理稳定。C2 向 C1 提交确认后的初始化，不再复制一套跟踪后端。

### C2-0 最小选型实验

冻结调参集、独立测试集和资源预算，优先评估一个 Scan Context 原型，STD/iBTC 为备选。依据许可、重力条件、实际召回、错误候选和资源确定采用项，不能预设赢家；首选不适用时记录原因再替换。

KISS-Matcher 通过 `CoarseRegistration` 风格接口先做最小构建与已知变换验证，核对输入输出方向、总变换、内点、退化和失败。可行才项目内集成，不引入 SAM。检索和配准各自独立，不强制同时集成 KISS/STD/iBTC。

### C2-1 地图索引和查询

- 派生索引绑定 map UUID、manifest SHA、关键帧/扫描 ID、描述子坐标系、重力约定、算法和分辨率，失败不留下成功标志。
- 定义独立 `PlaceRetriever` 职责：查询云/描述子和只读索引 → 多个地点、分数、航向/姿态假设及来源；与 B 候选适配。全局候选不能再被原 3 m 位置半径全部过滤。
- Mid-360S 不简单视为窄视场，不因型号排除 SC；当前地图不保证水平，须验证地图/查询的重力约定或选能处理相应姿态差的方法，不默认 yaw-only 即可。
- 多帧查询按前端相对运动变到明确参考 IMU 帧，保存去重 scan ID 集合，限制时间/运动跨度。静止重复扫描不计作独立支持。

### C2-2 粗配准、精配准和歧义验证

1. 检索多个历史地点并建立局部目标子图；描述子分数不直接决定定位成功。
2. 若初值在候选 i 系，构造完整 `T_i_query`，再取 `X_i × T_i_query` 得到 map 初值；不能绕 map 原点只旋 yaw 而漏掉候选平移。
3. 无可靠初值时用已验证粗配准，再进入 C1 精配准；各层返回总变换，防止初值漏乘/重复乘。
4. 独立检查覆盖、残差、对应数、退化和刚体合法性；比较空间分离且质量接近的地点，无法消歧则拒绝。
5. 大修正不直接判错；需要具有足够时间/运动基线的独立几何支持和前端相对运动自洽。连续静止 N 帧不足以确认。
6. 提交前复查地图版本、查询会话、请求代际和依赖；切图或前端重置后丢弃旧结果。

### C2-3 状态、恢复和资源

内部 searching / verifying / tracking / lost / retry_backoff 映射到既有五态和 reason；首次搜索与 C1 丢失后的搜索共享链路、日志区分。

仅确认后向 C1 提交初始化，再验证持续跟踪；服务受理/首个候选不是成功。私有重定位成功与公共桥接创新锁定分别管理，不自动解除锁定或放宽阈值，恢复公共联调需显式重启会话。

候选数、查询点数、总墙钟预算、取消及重试间隔配置有界。预算耗尽返回未确认，不接受“剩下最好的”候选凑成功。

### C2-4 验收和交付

当前：合成/接口/故障/同包功能检查 PASS；独立实采会话、真实相似地点与地图外负样本、动态/遮挡、真值及实车性能未完成。后续按用户新指令推进，本计划不是自动续跑授权。

- 合成已知真值：未知位置、不同 yaw/roll/pitch、变换方向、多个相似候选、地图外查询、低重叠、退化、错误粗配准、大修正正确/错误候选。
- 不同采集会话建图/定位，加入相似地点负样本、动态干扰、遮挡和不同起点；同包回放只作功能检查。
- 分层统计检索 Recall@K（有地点标签才计算）、粗配准通过率、最终成功/总次数、错误接受、拒绝/超时、初始化耗时 p50/p95/max 和峰值内存。列分母、标签方法、参数和条件；零错误样本不证明风险为零。
- 覆盖丢失→搜索→多帧确认→C1 跟踪，以及切图、回退、取消、延迟任务和重启。
- 公共 TF 唯一、原采样时间、wheel 不外推、锁定策略必须保留，不为接入放宽安全门槛。

交付选型与许可实验、最终采用的检索实现/可选粗配准插件、派生索引工具、参数、状态、数据清单和 C2 报告。完整 STD/iBTC、PCM、前端恢复及离线 BA 均不是强制项。

## 6. 数据、物理条件与 D 验收

已有 `bags/fast-livo-motion-w5phdosj/raw` 约 136.6 秒，可只读复用。同源地图/查询不证明跨会话定位。该包未录真实 wheel，不得把合成 wheel 联调当实时接车验收。原包、标定、地图、失败记录和已有输出全部保留。

获得采集授权后，分别采地图构建会话、同环境不同起点定位会话、相似地点/地图外负样本会话；包含静止、转弯、回访、遮挡和正常速度变化。记录手持或上车等条件，涉及底盘控制/导航单独授权。按 [录包指南](fast_livo2_recording.md) 确认完整三传感器输出，不以关闭图像绕过问题。

| 缺口 | 所需证据 | 不能替代它的方法 |
| --- | --- | --- |
| 曝光/扫描相位 | 触发源、时间基准与动态同步实验，保留原戳 | 同戳复制、无证据 ±100 ms 补偿 |
| T_base_imu | 方向、实际测量/标定及验证 | 相机–雷达外参当车体外参 |
| 实时 wheel 历元 | sensor/host/wheel 时钟来源与同刻可插值记录 | 重打 now、合成 wheel 冒充实车 |
| B 泛化 | 多环境有环/无环/重复结构/错误候选标签 | 单包 13 条已接受约束当普遍正确率 |
| 实时/长时 | 固定资源、原速输入、积压/丢帧、CPU/内存/磁盘与持续时长 | acknowledgment 节流离线耗时当实时吞吐 |
| 定位精度 | 独立真值或带不确定度的外部基准 | 首尾距离、地图 NN 残差直接当 APE/RPE |

D 顺序：离线跨会话 C1/C2 → 获授权后只读真机定位/公共桥接 → 获明确运动授权后导航/安全链。每步单独验收，前一步通过不自动授权后一步。高程、坡道导航、在线前端恢复另立范围。

## 7. 文档维护与阶段交接

- 本文件维护整体计划与参考项目；[STATUS](STATUS.md) 写当前能力；[HANDOFF](HANDOFF.md) 写接手入口；[AGENTS](../AGENTS.md) 只保留有效约束，临时会话与历史过程放带日期报告。
- 每阶段报告列实际修改、版本/许可、构建和执行命令、退出码、参数、数据身份、原始日志，区分 PASS/PARTIAL/未实施，不能只写“测试通过”。
- 清理前检查引用；修正过时事实，保留独立历史证据，仅删除已被替代且无独立价值的重复说明。不删除原包、标定、地图、备份、失败证据或用途不明源码。
- C1 离线实现已按明确指令执行，第 5 节 C2 已按后续指令完成离线范围，D 须另授权；独立会话数据与 D 另行验收。新上下文先核对本计划、工作树和 B/C1 证据，复用实现；缺独立定位包不阻止后续离线开发，但真实验收保持 PARTIAL。
- 各阶段完成后停止在授权边界，不自动实施 D、BA、硬件或导航。提交/推送按实际用户指令执行，计划不代替授权。
