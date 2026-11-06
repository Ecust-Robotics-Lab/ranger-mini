# FAST-LIVO2 与 AprilTag 增量增强实施计划

版本：2026-10-29。适用基线：`acffa879cc8d52330276089cd3df83ad6a43f112`，分支 `refactor/pluggable-navigation`，Ubuntu 22.04 / ROS 2 Humble / x86_64。本文细化用户提供的 T0–T7 执行指令，并以实际代码为接入依据。用户已明确选择“计划落盘后继续离线编码与测试”；本次范围包括源码、项目内依赖、编译、合成与离线联调及文档，不包括设备、采集、底盘、导航、Git 提交或推送。

实施结果见 [本轮报告](fast_livo2_apriltag_report.md)，接口/开关/地标操作见 [契约](fast_livo2_apriltag_interfaces.md)。用户后续确认现有包内 tag36h11:1、0.155 m，真实抽样检测已完成；不因此升级物理时间、真实地标或独立访问验收状态。

## 0 当前完成度（2026-11-05）

**主体软件已交付，原计划尚未全部验收。** 后续用户明确要求继续离线排查；硬件、部署、提交/推送仍不在自动执行范围。

2026-11-05本轮停在交接和清理，没有继续算法实验。[HANDOFF](HANDOFF.md)集中维护当前构建/数据/边界；下次收到继续指令后先用已有两包固定+100ms另存完整对照，再依序完成真实地标、Tag C2/B及独立负样本验收。当前3px/6px像素门槛属于初始试验值，应按整链误差预算和独立证据标定，不把所有非零回投残差当Tag自身失败。

2026-11-04 后续时间核查及用户新指令：[扫描相位报告](fast_livo2_timestamp_phase_report.md)。正式前端试验 `img_time_offset` 现为 +0.1 s（上游锁定配置/作者说明支持），历史0/+110ms实验不变。两包1800扫描与1843相机会话帧审查、11项既有回归通过，未启动设备。Tag独立时间仍未确认/关闭；先用已有数据固定+100ms对照并记录剩余相位不确定性，再推进原地标/Tag C2与B验收。下文“正式时偏不改/零偏移”均指相应历史阶段，不再描述本次修改后的默认。

| 阶段 | 已完成 | 仍待验收 |
| --- | --- | --- |
| T0 审查/基线 | 原 B/C1/C2 接入核对、依赖锁定、基线及回归 | 实际传感器时间/动态标定质量 |
| T1/T2 观测/坐标/时间 | 官方检测、双解、像素模型、插值与会话隔离、独立几何测试 | 真实有效时间映射与不确定性、动态跨帧一致性 |
| T3 地标工具 | PGO 与旧图 C1 补录、不可变版本保存/加载/禁用 | 新 140 mm 部署的真实 VALID 表；旧 155 mm 实例不合并 |
| T4/T5 C2/B | 原配准/验证/提交链集成、歧义仲裁、合成/隔离 ROS 联调 | 独立访问 Tag 恢复、真实回环收益和错误地点拒绝 |
| T6/T7 监视/可靠性/验收 | C1 监视、有限资源/代际核心、原回归与合成检查 | 洪泛极限、实测尾延迟/长时、真实负样本与完整性能对照 |

最新[C1 修正归因与 GICP 对照](fast_livo2_c1_correction_report.md)：原PCL路径可精确复现，四条件同扫描实验及完整两包运行支持已有GICP可降低额外地图修正不一致。冻结原Tag、同516个跨访问观测P95从17.324降到6.114px；直行原C2→C1有520有效、1次短暂DEGRADED，无LOST/ERROR，启动仍约37.4s。未更换注册器实现，仅离线显式选择已存在的small_gicp；正式默认、门槛、标定、时间与部署保持。软件基础具备，真实Tag地标/恢复/回环仍未验收。

首轮软件交付 123 项检查通过；不把该数量等同于计划所有真实场景已通过。另已完成一次 Tag 关闭的不同采集包 C2 功能试验；它不依赖真实 Tag 表，不能代替 Tag 辅助验收。

最新[两份新包与地图对照](fast_livo2_tag_two_capture_report.md)：两份分离动作包已经完成训练冻结和独立验证，+110 ms 获支持但三融合残差未全部改善。转向包新建局部图后，独立直行原C2→C1有条件功能通过（895提交/894处理/1取消，37.000 s首VALID，525有效）；旧图同查询仍无VALID。相同160转向留出帧旧/新图C1 RMS P95为16.206/6.231 px，说明地图/配准影响不能忽略；冻结Tag跨访问P95仍17.286 px，不可登记真实VALID。C1等待帧计算优化和离线取消记录已修改，74项相关测试通过，构建仅在隔离前缀。正式时偏/标定不改，`time_verified=false`、无真实VALID表。

后续顺序：C1目标子图/空间支持/注册器控制归因已完成，优先独立核验物理时间与剩余动态尾部；必要时提出针对性测量，不要求反复重录同类包。通过后才登记匹配图版本的Tag表、冻结并使用未参与本轮方法选择的独立访问做Tag C2与负样本，再覆盖真实Tag回访回环；随后扩展正式覆盖地图。当前小局部图不代替整层图，纯C2/GICP功能通过不代替Tag、精度、实时或D验收。

## 1 评审结论与实施原则

方案方向合理：把 Tag 当作带歧义和误差的候选来源，交给已存在的 LiDAR 验证与恢复链，而不把一次检测直接当成地图定位。本轮优先完成 Tag→C2→C1 闭环，再增加 B 回环和 C1 监视。已有 B/C1/C2 的实现与离线证据可复用，跨会话、物理同步和精度验收仍为 PARTIAL，不能改称“全部实测完成”。

结合源码，实施时必须明确以下调整：

1. **C2 不存在通用外部候选总线。** `PlaceCandidate` 只有关键帧 ID/距离；`Relocalizer::process` 内部串行执行检索、KISS、精配准和多帧确认。增加小型带初值与复核器的候选接口，复用同一确认/提交路径，不另建 Tag 定位器。
2. **C2 的 source 是多帧聚合查询。** 所有帧经 `U_latest.inverse()*U_frame` 变到最新 IMU 帧；Tag 初值和注册结果必须绑定该帧及其 scan 集合。确认支持仍要求互不重叠的 scan 集合及运动/时间基线。
3. **B 的候选目前不携带初值。** `CandidateSearch::find` 可扩展，但 `Backend::detect_loops` 当前总用 `Xi.inverse()*Xj`；需添加候选初值与结果复核，而不是只插入一个 ID。不同假设可能落在同一历史 ID，不能提前按 ID 去重。
4. **旧 B schema 严格枚举文件。** `Archive::load` 拒绝额外文件；第一版采用独立、不可变、绑定源 manifest SHA 的地标扩展包，与现有 C2 派生索引方式一致。原 B 包不变。扩展有自己的 manifest/COMMITTED、revision 和源图身份；不把旁路文件偷偷塞进旧归档。
5. **图像和 CameraInfo 时间域不同。** `/front/image` 与 `/front/camera_info` 使用主机 stamp，`/front/image_sync` 复制 timeshare 的雷达 stamp。第一版显式绑定已审查的静态相机标定及分辨率，保留 CameraInfo 来源，不伪造同戳同步 CameraInfo。
6. **本机暂无已安装 AprilTag 包。** OpenCV C++/Python 均为 4.5.4。优先使用官方 libapriltag 的薄观察器，以满足原始畸变图、双解、元数据及无 TF 的契约；不同时维护 apriltag_ros 第二条生产检测路径。
7. **未知时间语义必须可降级。** 当前 image_sync 只是扫描戳复制，曝光相位未证实。真实旧包可检测/记录；没有显式时间映射证据时不能产生可提交定位候选。合成夹具提供独立、已知的曝光时间与轨迹。
8. **视觉复核不等于独立确认。** 同一 Tag 既生成初值又作回投，仅能检查一致性；必须保留原 LiDAR 绝对门槛和独立运动支持。Tag 错贴在几何重复地点仍可能无法判别，纳入负样本边界。

## 2 固定边界

- 前端保持 LiDAR–IMU–Camera 三融合；不向 ESIKF/视觉局部图注入 Tag，不回灌后端，不新写 IMU 积分。
- 不新增 Tag 图节点、Tag 因子、Tag 绝对 Pose3 先验或视觉–LiDAR 联合优化；B 仅增加原类型的 LiDAR 回环边。
- 复用当前 PCL ICP / small_gicp、GTSAM、C2 检索与可选 KISS；不更换已验证算法或系统库。
- C1 默认人工初始化/LOST 后人工重启，显式 C2 才允许原有自动恢复。Tag 不增加公共桥接解锁入口。
- 公共 TF 唯一、wheel 同刻插值、禁止外推、原采样戳、创新拒绝缓存隔离均保持；私有 Tag 节点不发公共 TF/速度。
- Tag 关闭、断流、未知 ID、损坏地标或观察器退出时保留原 LiDAR 路径。候选队列和资源预算必须隔离，Tag 不得饿死 LiDAR。
- 原 bag、旧图、标定、失败证据和备份保留；所有派生数据另存。新配置默认被动，入口默认 dry-run。
- Tag 只能帮助选择地点/初值，不能补足点云本身缺失的可观信息。不能区分的走廊/对称场景允许拒绝。

## 3 T0 源码接入点与基线

以下路径相对于仓库根；模块名以实际代码为准。

| 实际位置与接口 | 现状 | 最小接入修改 |
| --- | --- | --- |
| `ws_fast_livo/src/ranger_fast_livo_backend/src/backend.cpp`，`process/is_keyframe/detect_loops/submap/task_current` | 单 owner，IMU 区间云；关键帧平移/旋转/时间门槛；目标在历史 IMU 帧；任务记录依赖位姿 | 附加 Tag 历史观测，候选可带总初值/复核器，保留候选间隔、图依赖、独立支持及加边事务 |
| 同包 `include/.../types.hpp`，`Candidate/MatchTask` | 候选 ID/距离/来源；任务含 session/version/target scans/依赖位姿 | 可选初值、证据、结果复核；未提供时原逻辑不变 |
| 同包 `registration.hpp/.cpp` | `Registration::align(target,source,initial)` 返回总 `T_target_source` | 直接复用，不把总变换当增量 |
| 同包 `graph.cpp` | Pose3 相邻/回环因子、重复边/优化检查 | 保持原加边入口，禁止 Tag 直接构造绝对先验 |
| `ranger_fast_livo_localizer/src/archive_reader.cpp` | schema 1、SHA/路径/文件枚举、只读图、离线兼容声明 | 地标另包绑定；不放宽原完整性检查 |
| 同包 `src/relocalizer.cpp` | 多帧 query→PlaceIndex→KISS→fine→歧义→独立运动确认 | 可选外部总初值，源/目标坐标明确；通过统一确认返回 `SearchResult.confirmed` |
| 同包 `src/node.cpp`，`process_search` | worker/generation；成功后 `core_->initialize` 再由 C1 确认 | Tag 消息有界接收，worker 消费，原代际守卫；监视不改 C1 位姿 |
| 同包 `src/localizer.cpp` | `history_pose`、六自由度初值/跟踪、VALID/DEGRADED/LOST | 复用插值原则，Tag 缓存独立；初值协方差仅为搜索不确定性，不宣称 Tag 精度 |
| `ws_nav/src/ranger_localization_bridge/.../bridge_node.py` | `_innovation_latched`，显式 alignment session 重开 | 无修改目标，只做回归；不引入不存在的 NORMAL/LOCKED 枚举 |
| `ws_camera/src/hik_camera_trigger_driver/src/hik_camera_node.cpp` | rgb8、实际 step，原图/CameraInfo 主机 stamp，sync 图复制雷达 stamp | 不改驱动；观察器使用原始图模型和显式标定/时间来源 |
| `vendor/fast_livo2_review/ros2_frontend/src/vio.cpp` | `Rli=extRᵀ`，`Pli=-extRᵀ*extT`，`Rci=Rcl*Rli` | 据实际代码导出 `T_C_I=T_C_L*inverse(T_I_L)`；不依变量名猜方向 |
| `scripts/run_fast_livo2_{localizer,relocalizer}.sh` 与受管 session | 默认 dry-run、localhost、清理 owned 进程 | 扩展参数/独立 Tag 入口，不启动设备或改变生产 runtime |

当前相机配置：1280×1024、`front_optical_frame`、plumb_bob 五参数，曝光配置 20000 μs；这是配置值，不是曝光时刻测量。当前 /fast_livo/odometry 约 10 Hz，原包 1367 条；IMU 约 200 Hz 不提供可直接替代的 200 Hz 状态轨迹。新前端已重力对齐，旧 B 图未对齐，地标绑定具体源图。

本轮证据目录为 `docs/fast_livo2_evidence/apriltag_20261029/`；大输出放 `outputs/fast-livo-apriltag-20261029-*`。命令记录 argv、环境、退出码、测试数、耗时和日志。现有 C++/C2 ROS/C1 ROS/桥接相关检查在修改前与修改后分别执行；环境配置失败单独保留，不能记为算法失败或 PASS。

## 4 T1 观察器与像素契约

### 4.1 依赖与组织

在现有后端包内增加共享 Tag 核心，供 B/C1/C2 使用；数学、时间、地标核心与检测器构建解耦。官方检测器固定版本放项目内前缀，锁 commit、源地址、许可证和实际链接；不 sudo 安装，不升级 OpenCV。观察器一个生产路径；同一核心可用于 ROS 输入及离线图像工具。

优先锁定官方 apriltag v3.4.5（远端 tag 指向 `94be783968e5091bcc9972c72c84fd63efce2935`），拉取后复核完整 SHA/许可证/头文件，再最小构建。未安装 apriltag_ros/apriltag_msgs 不是假设已有接口的理由。OpenCV 使用本机 4.5.4。

### 4.2 观测契约

观测必须保存：schema、observation/image/capture/source ID、family+id、尺寸及来源、原始/有效 int64 ns、optical frame、模型/像素域/尺寸/ROI/binning、解码语义角点、质量字段可用性、1–2 个有效位姿假设及各自像素残差、标定/时间映射/前端会话/历元、接收/处理耗时和拒绝原因。原 bag/帧引用可替代长期保存整图。

未知尺寸只保留 ID/角点，不生成公制姿态。默认可用 tag36h11、hamming=0，family 可配置；不将其解释为零误检。尺寸是检测四角之间的物理边长。decision margin、面积、距离、边缘等阈值必须注明初始试验值。缺失字段用缺失/valid 标志，不填零冒充高质量。

同帧同 family+id 多实例、持续空间多峰、移动标签均不能进入唯一锚点集。整个部署域要求 family+id 唯一物理身份；配置中的 instance ID 不是解决重复部署的魔法。

### 4.3 像素与双解

支持并显式区别：原始 K/D 像素；校正有效 K/P 与零畸变像素；归一化射线与单位内参。第一版生产输入采用实际 raw plumb_bob；其他已支持模型需测试，不支持模型明确拒绝。ROI/binning/裁剪缩放必须正确调整或拒绝，不静默套旧 K。

`solvePnPGeneric(...SOLVEPNP_IPPE_SQUARE)` 的物点顺序固定为 `(-s/2,+s/2,0),(+s/2,+s/2,0),(+s/2,-s/2,0),(-s/2,-s/2,0)`。保留解码方向的 corner 索引映射，使用 0/90/180/270 度实渲染测试证明；不能按图像左上重新排序。声明 canonical Tag frame 及到 AprilTag 原生 frame 的常量关系。

每个返回解检查有限性、SO(3)、所有角点正深度、像素 RMS/最大误差、重复解；按实际数量输出 0–2 支。归一化求解后必须投回实际观测像素域计算误差。AprilTag object-space error 不作为像素 RMS。畸变后检测偏差保留为限制，可另做全图 remap 对照，不做无证据性能承诺。

重力只作有来源的辅助先验，不假设 Tag 竖直，不使用瞬时加速度替代重力，不用 LOST 旧位姿唯一消歧。保留无法区分的双解。

## 5 T2 坐标与时间契约

统一 `p_A=T_A_B*p_B`。M 为绑定地图；O 为同会话连续前端局部系；I 为实际 IMU 状态参考系；C 为 optical；F 为 canonical Tag。`X(t)=T_O_I(t)`，`E=T_I_C`。

```text
T_C_I = T_C_L * inverse(T_I_L)       E = inverse(T_C_I)
T_M_I(tc) = T_M_F * inverse(T_C_F(tc)) * T_C_I
Z_k = inverse(X(tk)) * X(tc) * E * T_C_F(tc)
T_I(i)_I(j)_init = Z_i * inverse(Z_j)
T_M_I(ts)_init = T_M_I(tc) * inverse(X(tc)) * X(ts)
T_M_S_init = T_M_I(ts)_init * T_I_S
T_Q_S_init = inverse(T_M_Q) * T_M_S_init
T_M_I(ts)_reg = T_M_Q * T_Q_S_reg * inverse(T_I_S)
T_M_I(tc)_reg = T_M_I(ts)_reg * inverse(X(ts)) * X(tc)
T_C_F_pred = inverse(T_M_I(tc)_reg * E) * T_M_F
```

当前 C2 source 是最新 I 帧聚合云，target 是 map 子图，故 `T_I_S=I,T_M_Q=I`；B source 为 j 的 body 云，target 为 i 系子图，结果为总 `T_i_j`。云若已在 O/M 系，不能把上述 I 简化照搬。

连续局部位姿缓存按 int64 ns 关联。用前后包围样本平移插值+四元数 SLERP，禁止矩阵逐元素插值、最近帧、latest TF 或外推。精确采样点可直接使用；无后样本允许有界等待，超时拒绝。最大间隔、挂载时间差、速度/角速度及观测年龄显式有界。

时间映射记录 `effective=header+offset` 的符号、来源、版本、不确定性和应用次数。当前复制 stamp 不是曝光证明，不硬编码历史 ±100 ms。合成测试注入已知 100 ms 检查正负号和重复补偿。仿真时钟暂停时使用 steady clock 超时。乱序、重复、时钟回退、前端重启、标定/映射切换清除受影响缓存和支持集。

前端无效或时间来源不可靠时只记录观测。端点之间的 10 Hz 插值也包含运动模型误差，不能声称无损或物理同步已验证。

## 6 T3 地标建立、补录和持久化

两条路径统一输出地标扩展包：

- 新建图：记录原始观测和 `Z_k`；冻结图后逐观测使用自己的 `X_k_optimized`，`T_M_F=X_k_optimized*Z_k`。跨非刚体图修正必须重算所有受影响锚点。
- 旧图补录：固定旧 map UUID/manifest/graph version，用 C1 已几何确认且时间连续的段，`T_M_F=T_M_I(ts)_C1*inverse(X(ts))*X(tc)*E*T_C_F(tc)`。C1 跳变、失效、时间缺口或前端会话变化时拒绝该段。不要求旧图可编辑 PGO。

只读旧地图、观察记录、连续轨迹和已确认定位段必须分别有身份。无 Tag 图像、尺寸、时间/标定来源时工具可完成合成验收，但不能生成真实 VALID 表。

聚合先按身份、姿态模式及独立访问分组。每个独立观测最多为一个物理解投票；两分支不是两票。同一静止连拍不增加独立支持。先检查固定性/模式，再做 SE(3) 稳健估计；多峰或持续矛盾保留 AMBIGUOUS/SUSPECT，不平均。VALID 要求明确的独立运动/视角/访问支持，门槛必须在配置与报告中列出。

扩展包至少包含 `schema_version/kind/revision/source_map_uuid/source_manifest_sha256/graph_version/map_frame/units`、family/id/size/frame、完整 T_map_tag、状态/原因、来源方法、观测引用与独立支持、历史标定/时间映射、像素误差/位姿离散度。未知 covariance 不写零矩阵；可不提供 covariance。新相机标定允许复用固定地标，旧观测始终按自己的历史标定解释。

原子写入新目录：有界校验→临时目录→文件 SHA→manifest→COMMITTED→fsync→不覆盖 rename。失败保留失败证据、不能留下成功目标。绑定源图 SHA；加载损坏扩展时禁用 Tag 并诊断，源图仍按原完整性契约加载。地标 revision 改变使旧候选失效。无扩展时纯 LiDAR 正常运行。

## 7 T4 C2 接入与仲裁

候选携带来源、observation/hypothesis、地标 revision、地图身份、scan 及参考时刻、总初值、前端/标定/时间/请求代际和复核依据。已知 Tag 产生 map→IMU 初值，经时间传播后进入已有 fine 配准；LiDAR 检索仍使用原 KISS 路径。两个来源在同一 attempt 内统一验证/仲裁，保留各自预算和至少一份 LiDAR 工作配额。

每支必须独立通过原收敛、对应数、重叠、残差、退化门槛；低残差不能自动获胜。通过的结果回投到图像时刻，检查全部用于本候选的可用观测；多个已登记 Tag 冲突不得选择性忽略。回投只说明一致性。

同一物理解聚类去重；多处通过且无法消歧则 AMBIGUOUS；全部失败拒绝；超时/未评估分支不能被当成失败从而挑剩余解。可疑 Tag 不应让原 LiDAR 永久失效：单独报告 Tag 尝试失败，仍允许没有使用该 Tag 的原路径按自身契约判断，但不得把一个尚未完整评估的 Tag 候选伪装成已消歧。

复用已有 correction、独立 scan 集、时间/运动支持，然后由 `SearchResult.confirmed`→`process_search`→`Localizer::initialize`→C1 独立验证。不能使用一次 Tag 直接调用公共定位发布。候选生成、验证、C2 handoff、C1 VALID、公共桥接接纳分别记录。桥接锁定保持锁定，私有成功不是解锁权限。

## 8 T5 B 回环增强

当前图内同 family+id 检索历史关键帧，可越过位置搜索半径；仍执行原时间/ID 间隔、近邻排除、最大候选和固定性检查。Z_i 与 Z_j 各最多两支，最多四组合，经有界去重进入原注册器。不同机器人位置应得到非单位相对位姿。

目标子图排除当前原扫描，自匹配和重复边继续由原机制拒绝。候选复核用注册后相对变换与两端图像的运动补偿作角点一致性检查。相同历史 ID 的不同解也须纳入物理解歧义比较，不能只比较目标关键帧位置。

大修正复用原独立运动支持，最终只有原 LiDAR Factor 进入 `Graph::add_loops`。pending 任务也保留 Tag 证据版本并在提交时复查。新增无关关键帧不能无条件取消任务；按现有依赖位姿检查。跨前端会话不复用 Z，不扩展跨会话 PGO。

## 9 T6 C1 监视与资源

已有效 C1 位姿可计算观测投影残差，输出 OK/CONFLICT/UNAVAILABLE 等诊断；一次冲突不改位姿、不移动地标。持续冲突形成待修订记录，不写运行中表。恢复仍走既有状态机。

图像输入、观测、位姿、候选、工作队列和日志条目都有上限；同图去重、同 ID 冷却、过时丢弃。检测在独立观察器，注册在原 worker，不阻塞前端回调。共享每次查询预处理与已有子图缓存；不为每个双解重新建全图。

全链路 token 包含 map/landmark revision、frontend session/time epoch、calibration/time mapping、attempt/generation、scan/KF 和目标依赖。节点重置/切图/取消时清空相关支持；消费幂等。库内不可中断求解以点数/迭代/线程限制并在返回后检查预算，不把软时限宣称硬实时。

独立开关：观察器、重定位、回环、监视、补录模式。默认生产 runtime 不启用，离线配置按测试需求启用；关闭后无需相机或 Tag 进程即可保留原能力。

## 10 T7 验收矩阵与证据

| 组 | 必须覆盖 | 合格证据 |
| --- | --- | --- |
| 原基线 | B/C1/C2、桥接、地图加载；Tag 关闭/退出/断流 | 修改前后实际命令/退出码/测试数，无旧失败掩盖 |
| 检测 | 官方渲染标签 4 个编码方向、真实 detector、stride/encoding/非连续/过期/重复 ID | 角点到物点的独立对应检查；不是只调用同一公式自证 |
| 像素/姿态 | raw/rectified/normalized，三轴旋转与杆臂，双解/重复/负深度/NaN/错尺寸/模型 | 独立投影夹具已知真值；错误配置拒绝 |
| 时间 | tc≠tk≠ts 的转弯轨迹、包围/缺后样本/间隔/禁止外推、偏移/回退/重启/切换 | 原始/effective ns、插值端点和拒绝原因 |
| 地图 | 两种构建路径、逐帧优化、双峰/移动/连拍支持、保存加载、损坏/目标存在/失败 | SHA、不覆盖、旧源图不变、状态与 provenance |
| C2 | 正确/未知/错贴/错尺寸、多解仲裁、多 Tag 冲突、低重叠/退化、低残差但回投冲突 | 候选→注册→回投→独立支持→C1 VALID，迟到旧代际拒绝 |
| B | 非单位回环、越半径、低重叠/自匹配/重复边/静止/双解/移动 Tag | 原类型 factor、后端/公共桥接分别记录 |
| 生命周期 | 切图/取消/重启/标定映射改变/图依赖改变、候选洪泛与线程预算 | 旧结果不提交，缓存有界，LiDAR 路径不饥饿 |
| ROS | localhost、隔离 domain、sim time、TF 唯一/无 Tag TF、wheel 同刻和创新锁 | 原安全链回归，私有有效不解锁 |
| 真实性能 | 独立访问纯 LiDAR vs Tag 辅助 C2/B | 数据/样本数/标签/真值来源、错误接受/拒绝/歧义、p50/p95/资源；缺数据 PARTIAL |

新合成夹具的登记集与查询集必须不同姿态/图像/采样；几何场景不能仅把 query 作为 map 直接自匹配。Tag 方形平面不能作为唯一 LiDAR 环境，否则原退化检查合理拒绝。地图外与几何重复地点分别测试。检测零误报须带帧数/场景，不能写普适保证。

已有运动包仅确定包含图像和前端位姿，不预设含可用 Tag；原包没有真实 wheel。只读抽查/检测结果属于数据审查，不能替代独立动态验收。

## 11 顺序、参数与完成条件

顺序：T0 基线/锁定→T1 观察器与独立像素测试→T2 时间核心→T3 地标工具→T4 C2 闭环→T5 B→T6 监视/生命周期→T7 回归和交付。每阶段完成后继续授权内下一项，不逐步等待人工确认；真实数据不足只阻止相应真实验收。

初始参数记录表必须包含：支持 families/size 表、hamming、margin/面积/边界、pixel RMS/max、时间 offset/uncertainty/gap/age/wait、速度角速度上限、聚类/独立支持阈值、各队列/点数/候选/线程/时限。几何安全检查为硬条件，经验阈值标成未实测标定；禁止以增大既有安全阈值使测试变绿。

交付：实现/配置/构建锁与许可、接入审查、坐标像素时间契约、地标 schema/工具与旧图兼容、候选日志、基线/新测试/回归证据、PASS/PARTIAL/FAIL/未实施分列、启用与关闭说明、后续独立采集需求。最终更新 STATUS 与本轮报告；本文保持计划语义，报告记录实际完成范围。

软件完成条件：可开关的观测、地标、C2、B、监视形成实际调用链；双解/时间/坐标/版本/安全规则得到独立测试；无 Tag 保持旧行为。真实动态精度、跨访问召回与错误接受、曝光同步、实时/长时和接车保持独立验收，不因软件通过而升级状态。

## 12 官方核查入口

以下为 API/模型依据，不代表使用了浮动 master 构建；实际依赖以锁文件为准。

- [AprilRobotics apriltag](https://github.com/AprilRobotics/apriltag)：检测核心、家族与物理尺寸。
- [pose API](https://github.com/AprilRobotics/apriltag/blob/master/apriltag_pose.h)、[检测结构](https://github.com/AprilRobotics/apriltag/blob/master/apriltag.h)：双解与误差单位、角点语义。
- [apriltag_ros 实现](https://github.com/christianrauch/apriltag_ros/blob/master/src/AprilTagNode.cpp)、[消息](https://github.com/christianrauch/apriltag_msgs/blob/master/msg/AprilTagDetection.msg)：只作包装接口核查；不假设 publish_tf 参数。
- [OpenCV 4.5.4 calib3d](https://docs.opencv.org/4.5.4/d9/d0c/group__calib3d.html)、[IPPE](https://github.com/tobycollins/IPPE)：物点顺序、PnP 与歧义。
- [Humble CameraInfo](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/CameraInfo.msg)：像素模型与 optical 坐标。
- [small_gicp](https://github.com/koide3/small_gicp)、[REP-105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)：总变换接口与连续局部坐标/全局定位职责；本项目的具体契约仍以固定源码为准。
