# 两份新包的时间、地图与跨会话定位对照

日期：2026-11-04。工作树基线 `acffa879cc8d52330276089cd3df83ad6a43f112`，分支 `refactor/pluggable-navigation`。本轮按用户后续授权继续软件与离线分析；原包、旧图、标定、生产配置与已安装前端均保留，没有启动设备、底盘、导航、提交或推送。

本报告承接 [运动几何诊断](fast_livo2_tag_motion_geometry_report.md)。用户指出早期地图可能质量不足，因此增加“新转向包建局部地图、独立直行包查询”的对照；不把旧图匹配后的波动直接归因于新传感器数据。

**本轮结论：新局部图上的独立直行包已完成纯 LiDAR 候选 C2→C1 功能验证；Tag 跨访问一致性仍不通过，尚不能登记真实 VALID 地标。** 新图同帧内部残差比旧图显著降低，支持继续核验地图与配准影响；没有独立真值，不能据此认定新图或全局定位的绝对精度。

后续更新：C1额外动态不一致已完成[C1 修正归因与 GICP 对照](fast_livo2_c1_correction_report.md)。保持原Tag常量不变、同516观测，已有GICP完整查询的跨访问P95约6.114px；本报告17.286px为此前PCL结果，历史证据保留。正式时间/地标状态仍未升级，详见后续报告。

## 1 数据身份与确认条件

| 项目 | 转向包 | 前后直行包 |
| --- | --- | --- |
| 输入目录 | `bags/fast-livo-tag140_turn_03-bk8_pjyy` | `bags/fast-livo-tag140_straight_04-ryzykq2l` |
| 原 DB 字节数 | 7,571,578,880 | 7,563,784,192 |
| 原 DB SHA256 | `23b3bdba0c4ac5497732eaae61f256bef74d57c52f0181efe58b7f5fe7a48e81` | `e6c7cd1b2d200b4629abfa7002458ace896e3c6a3b2268cc673145d21f400b6d` |
| 实际入包 LiDAR / IMU | 900 / 18001 | 900 / 18002 |
| 同步图像 / 在线 odometry | 900 / 900 | 899 / 900 |
| Tag 1 检出 | 897 / 900 | 899 / 899 |
| 全部检出的双解 | 保留 2 支 | 保留 2 支 |

标签为 tag36h11:1、打印后实测黑边界边长 0.140 m。用户明确确认两包内及两包之间 Tag 位置、相机与雷达安装均未改变，声明在证据 `setup_declaration.json`。旧 0.155 m 实例和本次实例不合并。

每份采集归档的 67 个来源文件均核验 SHA，两包前端二进制、相机配置和标定配置一致。在线前端 SHA 为 `dfcf5d4d54f7b81f573a000589527afa36f4dc71c5fe44df1c94f5bff9ae7b27`。终端计数包含录包等待窗口，以上采用数据库实际计数。

两包各 900 个已录扫描均可找到 writer 记录；DDS 来源序号没有缺口、丢失计数为 0。相机会话的 924 / 919 帧全部消费到达时最新 writer 序号，设备时间单调、帧号连续。本轮未复现旧包的 writer 漏扫描或 BACKWARD；这仍不能证明曝光时刻与扫描时间的物理对应。详见 `delivery_summary.json`、`turn_delivery.json`、`straight_delivery.json`。

## 2 预先冻结的时间检验

`protocol.json` 规定：固定 K/D、外参、尺寸和 LiDAR–IMU 诊断轨迹，只用转向包训练帧选择时偏；5 秒时间块留出、边界各剔除 0.5 秒，训练帧至少相隔 0.5 秒。前后包不参与时偏搜索。每个假设只拟合一个固定 Tag 位姿，最多 4 个初值，选择依据为训练 Huber 代价，不能用留出帧挑解。

转向包在 −50～+250 ms、5 ms 步长的 61 点网格中选出 **+110 ms**，训练代价约 0.5145；+105 / +115 ms 约 0.5937 / 0.5717。选择后先写 `frozen_selection.json`，再评估直行包。这与先前 120 秒包提出的约 +110～115 ms 区域一致。

所有时间仍为原始 int64 纳秒。实验定义为 `t_effective=t_header+110000000 ns`：前端派生配置与 Tag 的时间关联各自使用该映射一次，没有改原消息或原 bag，也没有在同一关联链重复补偿。

隔离前端副本为 `ws_fast_livo/independent_time_source_20261104/`，构建在 `build_independent_time_20261104/`。初始化 gyro bias 保持原锁定源码的零值，没有借用上一轮的均值零偏实验改动。两包分别执行 LiDAR–IMU 诊断、零时偏三融合、+110 ms 三融合，共 6 次重放，均退出 0；各三融合输出 895 对 VIO 后同戳位姿/云，初始重力对齐各执行一次。

| 同一组 240 留出帧上的固定 Tag 检验 | 零时偏 RMS P95 / px | +110 ms RMS P95 / px |
| --- | ---: | ---: |
| 转向，LiDAR–IMU 诊断 | 46.051 | 2.746 |
| 前后，LiDAR–IMU 诊断 | 3.096 | 2.893 |
| 转向，完整三融合 | 14.105 | 4.325 |
| 前后，完整三融合 | 6.483 | 7.050 |

每行的两个配置使用相同观测集合与相同留出帧。各行完整共同集合为 892、895、891、892 帧。LiDAR–IMU 仅作诊断，不替代三融合验收；直行包仍独立拟合自己局部坐标中的 Tag 常量，这是时间验证，尚不是跨地图地标复用。

结论：**+110 ms 获得新的独立采集证据支持，但没有解决全部三融合一致性问题，也不是已测得的物理曝光偏差。** 直行三融合 P95 没有改善，不能只展示转向改善曲线。原始 gyro 的短时旋转检查还受零偏及平面 PnP 双解影响，未作为物理时间验收。正式 `apriltag_camera.json` 仍是 `time_verified=false, offset_ns=0`。

## 3 旧地图对照及其局限

旧图来自 **`bags/fast-livo-motion-w5phdosj`**，归档为 `outputs/fast-livo-backend-validation-20261028-pcl/snapshot`，49 个关键帧、13 条已接受回环约束，UUID `6acd0a5763786389402bf72f397d9cbc`，graph version 62；manifest SHA `a890afe331f16ad5d65e601f390a00c7f270cd489346884a41b0b4274b9a4f58`。旧图没有 Tag 不影响纯 LiDAR 候选 C2 的运行，但不能直接提供 Tag 地标表。

| 原 C2、无人工初值、Tag 关闭 | 输入对数 | 有效采样 | 结果 |
| --- | ---: | ---: | --- |
| 转向查询，原 5 候选 | 895 | 588 | 首次 VALID 在查询传感器时间 +30.700 s，无 LOST/ERROR |
| 前后查询，原 5 候选 | 895 | 0 | 有几何支持但未累计足够独立运动证据 |
| 前后查询，仅改为 10 候选的离线对照 | 895 | 0 | 没有解决初始化失败 |

直行包最长连续支持的局部平移约 0.137 m；后续粗配准拒绝使支持重置，未达到原 C2 的 0.15 m / 0.08 rad 门槛。没有降低门槛、删除锁或伪造静止/健康状态。

在转向包 584 个健康 C1 支持观测中，用完全相同的 160 个留出图像比较：局部三融合轨迹的 Tag RMS P95 为 **4.852 px**，匹配旧图后的 C1 为 **16.206 px**。这表明在该条件下，旧图匹配轨迹引入了更多 Tag 不一致；不能单凭这项内部残差证明旧图的绝对精度差，地图覆盖、历史标定、几何歧义及配准偏差都可能参与。直行旧图没有健康 C1 支持，不能为它补录真实地标。

## 4 新局部地图与 C1 重复计算优化

用户提出旧图质量疑问后，只用本次**转向包**的 +110 ms 三融合输出，经现有 B 建立新局部地图；独立直行包不参与建图，Tag 也不进入前端或图优化。

新图：`outputs/fast-livo-tag140-two-capture-20261104/new_local_map/snapshot`，UUID `4ec68a99c626e1c6e2652506f5ecd8d6`，graph version 41，31 个关键帧、10 条已接受并优化的 LiDAR 回环、184315 点，97 个文件 SHA 校验通过。逐关键帧重建最大数值误差约 2.38e−7 m；这仅验证坐标与归档实现，不是地图实测精度。该图覆盖本次短程动作附近，**不是整层地图**。

后端测试脚本的独立副本仅将历史硬编码的 `frontend_gravity_aligned=false` 元数据改为本次实际的 true，并保存 diff；原 B 算法、6D 配准和回环门槛均未变。新图不重用旧图 UUID，也未覆盖旧归档。

原节点的新图直行试验已完成 C2 全局候选确认，随后 C1 初始化超时。源码与单帧日志表明：C1 等待时间/运动基线期间，每帧重复 13 个初值的粗、精配准，约需 1.5 秒。在逐输入等待确认的离线节流下，10 秒墙钟超时先于足够传感器运动到达；这与 C2 未找到地点是不同失败环节。两次转向同包给定初值在静止/低运动段的尝试也保留为失败证据。

最小源码修改仅涉及 `localizer.hpp`、`localizer.cpp` 和数值测试：

- 首个支持帧、每个可能转为 VALID 的帧，仍执行完整 13 初值歧义检查。
- 未达到确认帧数、时间或运动条件的等待帧，只进行一个初值的粗、精配准及原几何复核；它们不能发布有效定位。
- 任一次拒绝仍清空初始化支持，下次从完整歧义检查开始。跟踪、退化、代际、超时、采样戳及公共桥接规则保持。

原 C1 的 0.05 m / 0.03 rad、0.2 秒运动确认和 10 秒初始化墙钟超时，以及 C2 的全部门槛均保留。一个初值复核失败时仍会拒绝，可能保守地丢失能从其他初值恢复的中间帧；没有新增保证收敛的承诺。

隔离安装在 `ws_fast_livo/install_c1_confirmation_20261104/`；没有替换 `backend_install/`。数值测试 32 项、Tag 集成 7 项、C1/C2/Tag ROS 9/4/5 项均通过；两项受管进程清理测试仍测试原有受管入口，其余直接节点测试指定隔离新二进制。新二进制 SHA `99453aca95043aba1beb78f11fb1a08fb0805140c964e74ec7ad44a32d0bb619`，测试明细见 `optimized_ros_regressions.json`。

新图同包 C1 已用记录轨迹自动选取的首个明确转动区间请求初值（index 267，选择规则和先前失败 index 在 `c1_motion_start_selection.json`），895 对完整处理、625 个有效采样、无 LOST/ERROR。初值来自本包 B 图，明确只作登记访问和功能检查，不是独立访问定位成功证据。

新旧地图进一步取**完全相同的 584 个观测、55 个训练帧和 160 个留出帧**比较，分别在各自坐标中只拟合一个固定 Tag 常量：局部三融合 RMS P95 为 4.852 px、旧图 C1 为 16.206 px、**新图 C1 为 6.231 px**。相同跟踪算法下新图匹配结果的内部一致性明显改善；地图/初始对齐条件不同，不能把这项对照等同于地图绝对精度测量。三个完整拟合与身份绑定见 `same_cohort_map_comparison/`。

直行新图的优化后首次尝试仍在 323 对已记录输入后超时，没有 VALID，原失败保留在 `straight/c2_new_map_optimized/`。原评估脚本只等待“当前扫描已处理”的确认，而节点初始化超时会合法清空队列并取消当时扫描，随后继续发布保留前一扫描戳的无效状态。因此脚本会停在等待已取消扫描，而没有喂入后续数据供原 C2 自动重试。

本轮为 `validate_motion.py --global-search --allow-recovery` 增加明确的取消记录：只有同地图、同会话、同请求的新初始化超时、且无有效位姿时，才把该输入记作 `cancelled_initial_verification_timeout` 并继续送下一帧。原节点自行按已有 C2 状态机重新搜索；脚本不调用初始化/解锁、不刷新采样时间、不重发旧观测。输出分别计 submitted / processed / cancelled，保留 `reported_latest_scan_stamp_ns`，取消不冒充处理成功。3 项新的失败模式测试覆盖有效/陈旧心跳、未知状态、切图/会话/attempt；原评估与图像检查合计 8 项通过。该修改只修正离线评估流程，没有修改节点的失败恢复规则。

## 5 新图独立直行包与地标复用结果

修正取消记录后，仍使用隔离新 C1 节点、原 C2、原参数、Tag 关闭和无人工初值，`straight/c2_new_map_recovery/` 完整试验退出 0，节点退出 0：

| 项目 | 实际结果 |
| --- | --- |
| 提交输入 / 获处理确认 / 明确取消 | 895 / 894 / 1 |
| 私有有效采样 | 525，index 370～894 |
| 首次 VALID | 原查询流传感器时间 +36.999920 s |
| 状态计数 | INITIALIZING 370，VALID 525，LOST/ERROR 0 |
| 初始化过程 | +27.4 s C2 首次确认；+32.3 s 一次 C1 超时取消；原 C2 自动重试，+36.0 s 再确认，+37.0 s C1 VALID |
| 首个 VALID 的歧义检查 | 13 个初值，13 个几何通过，原门槛不变 |
| 原采样时间与身份 | 有效采样的 last_confirmed 与输入整数纳秒一致，地图/会话/请求绑定保留 |
| 公共 TF / 速度发布者 | `/tf`、`/tf_static`、`/cmd_vel`、`/base/safe_cmd_vel` 全为 0 |
| 确认求解耗时 P50 / P95 / max | 90.0 / 100.1 / 1434.3 ms，max 含初始化完整歧义探测 |
| 节点峰值 RSS | 546456 KiB，约 533.6 MiB |
| 整次离线墙钟 / 源数据时间 | 302.78 / 89.40 s；逐输入确认节流，**不是实时吞吐或实车恢复时延验收** |

37 秒包含本包等待实际运动、一次初始化失败和重试，不是整层地图定位的固定耗时承诺。本次证明一组独立采集的输入能经过原门槛完成 C2→C1；实际位置正确性、绝对误差、多起点、相似地点、地图外负样本、跟踪后真实丢失恢复及实时性仍待验。

地图对照与跨访问图见 [实际数据图](fast_livo2_evidence/tag_two_capture_20261104/map_and_independent_visit.png)，原始汇总在 `final_summary.json`。候选级示例 `candidate_examples.json` 分开保留两次 C2 确认、一次取消、首次 C1 VALID；公共桥接均为 NOT_STARTED，不能把私有成功写成公共接受。

固定地标检查使用 621 个健康转向观测拟合，按时间块留出 181 帧 P95 **5.986 px**；冻结这个 Tag 位姿后，不再对直行包重拟合，523 个健康直行观测的 RMS 中位数 **5.474 px**、P95 **17.286 px**、最大 **24.957 px**。初始试验门槛 3 px RMS / 6 px 最大角点仅有 147/523 帧通过。这不是可登记 VALID 地标的证据。

另做明确隔离的次级诊断：直行包单独拟合自己的固定 Tag 常量，160 留出帧 P95 仍有 **15.618 px**；相同帧的局部三融合轨迹自拟合为 **9.055 px**。两次访问各自估计的 Tag 常量相差约 **1.97 cm / 2.04°**，只是内部估计差，不是实测定位误差。重新拟合一个常量也没消除动态误差，说明不能只修改地图里 Tag 的位置来解决它。次级结果不用于回头修改冻结地标、时偏或独立访问主结果，见 `secondary_straight_self_fit.json`。

因此，早期地图/匹配条件确实影响此前结论；新局部图也尚未消除所有 C1 配准与局部三融合的动态不一致。现有数据不能把剩余误差唯一归因于时间、外参、地图形变或检测。没有对它们联合拟合再宣称找到物理解，也没有放宽门槛登记地标。

## 6 证据与复现入口

证据目录：`docs/fast_livo2_evidence/tag_two_capture_20261104/`；大输出：`outputs/fast-livo-tag140-two-capture-20261104/`。完整来源与参数在各 `manifest.json`、`result.json` 和独立协议中；失败目录保留，不覆盖原包。

主要数据复现脚本默认为 dry-run（批处理 `.sh` 与 ROS 测试脚本明确为已授权执行记录）：`delivery.py`、`replays.py`、`evaluate.py`、`build_new_local_map.py`、`validate_optimized_node.py`、`validate_optimized_recovery.py`、`backfill_assessment.py`、`compare_map_cohort.py`、`secondary_straight_diagnostic.py`。数据目录使用新位置并拒绝覆盖；摘要/汇图脚本只更新本轮派生输出，不改源数据。`run_new_map_optimized.sh` 保留首轮两项验证完整命令，`run_new_map_recovery.sh` 保留修正评估后的独立直行试验，`commands.json` 汇总实际 node/replay argv 和退出情况。

最初直接运行 C++ 测试因缺少项目内 GTSAM 动态库路径退出 127，原日志保留；补齐 `LD_LIBRARY_PATH` 后 39 项通过。首个批处理在 ROS setup 的 nounset 环境检查中退出 1，未启动节点；移除 nounset 后重试。两项均为环境准备失败，不能算算法失败或隐藏成通过，见 `setup_failures.json`。

相关检查如下，原始首轮 123 项验收不重复累加成新的独立测试数。

| 实际命令或执行记录 | 退出码 / 数量 | 日志（证据目录内） |
| --- | --- | --- |
| `colcon ... --packages-select ranger_fast_livo_localizer --build-base ws_fast_livo/build_c1_confirmation_20261104 --install-base ws_fast_livo/install_c1_confirmation_20261104 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DRANGER_ENABLE_KISS=ON -DBUILD_TESTING=ON` | 0 / 1 包 | `build_c1_confirmation.log`、构建目录 `colcon_build.rc` |
| 隔离构建 `localizer_tests --gtest_output=xml:.../localizer_tests_retry.xml` | 0 / 32 | `localizer_tests_retry.log/.xml` |
| 隔离构建 `tag_integration_tests --gtest_output=xml:.../tag_integration_tests_retry.xml` | 0 / 7 | `tag_integration_tests_retry.log/.xml` |
| `python3 .../run_optimized_ros_tests.py`，隔离 domain 196/localhost | 0 / 9+4+5 | `optimized_ros_regressions.log/.json`、逐测试目录 |
| `python3 tests/test_tag_imu_timing.py` | 0 / 6 | `gyro_tests.log` |
| `python3 tests/test_tag_motion_geometry.py` | 0 / 3 | `geometry_tests.log` |
| `python3 tests/test_fast_livo_global_evaluation.py` | 0 / 8 | `evaluator_tests.log` |
| `python3 .../compare_map_cohort.py --execute` | 0 / 三种轨迹同帧比较 | `same_cohort_map_comparison.log` |
| `bash .../run_new_map_optimized.sh` | 1 / 直行超时；转向 C1 退出 0 | `new_map_optimized_exits.json`、对应逐项日志 |
| `bash .../run_new_map_recovery.sh` | 0 / 独立直行功能通过 | `new_map_recovery_exit.json`、`straight_c2_new_map_recovery.log` |
| `python3 .../backfill_assessment.py --execute --archive .../new_local_map/snapshot --turn-c2 c1_new_map_optimized --straight-c2 c2_new_map_recovery --output NEW_DIR` | 0 / 工具完成，真实地标不通过 | `backfill_new_map_recovery.log` |
| `python3 .../verify_final_integrity.py --execute` | 0 / 两原 DB、两图及正式配置 | `final_integrity.log/.json` |

新构建复用现有锁定的 PCL、GTSAM、small_gicp、KISS 和官方 AprilTag，无新增或升级依赖。版本、commit 与许可证继续沿用原 AprilTag/C2 报告及 sources lock；实际进程动态库路径和 SHA 另存 `optimized_live_linkage.json`，确认载入隔离的新 C1 core 与原有 backend core。

本轮相关测试合计 **74 项通过**（32+7+9+4+5+6+3+8），其中新增 2 项 C++、3 项评估失败模式测试；没有把数据分析命令退出 0 等同于真实验收通过。原包 SHA/尺寸/mtime、两份来源归档各 67 文件、旧图 151 与新图 97 个文件、正式 trial/相机 Tag 配置及前端程序 SHA 均复核通过；HEAD 未变。录包会话 STOPPED、live=false、owned=[]。

## 7 状态与后续边界

| 分类 | 本轮结论 |
| --- | --- |
| PASS | 两新包交付检查与全帧检测；冻结时偏流程；六次重放；新局部图归档/坐标核对；C1 优化与 74 项相关测试；新图独立直行 C2→C1 功能及后续跟踪；原始资产完整性 |
| PARTIAL / 未验证 | +110 ms 为有效时间假设，不是物理曝光标定；新图质量/覆盖、位置正确性、绝对精度、真实误接受、实时/长时与完整 C2-4 验收 |
| FAIL / 不满足目标 | 旧图直行原 5/10 候选均无 VALID；新图原节点及首次优化试验初始化超时（均保留）；冻结 Tag 的独立直行 P95 17.286 px，不满足真实地标登记条件 |
| 未实施 | 真实 VALID 地标导出、实测 Tag 辅助恢复/回环收益、正式时偏部署、替换原安装目录、公共桥接接车、硬件/导航、Git 提交/推送 |

下一步按证据收敛，不要求用户立即重新录相同动作包：

1. 以本轮新图为**条件性局部基线**，定位直行包中 C1 地图修正相对连续局部轨迹的变化，结合点云对应的空间分布、遮挡和目标子图变化检查配准偏差；同图同帧与跨访问分别核验，不能只平滑输出掩盖错误。
2. 继续分开验证曝光/扫描物理时间和动态标定：+110 ms 保持派生假设，保留不确定性。需要硬件时间证据或新标定时，先提出具体测量及采集需求，不自动启动设备，不把拟合出的联合参数当作已辨识的物理解。
3. 时间与几何证据足够后，再登记绑定地图版本的真实 Tag 表，冻结后用独立访问测试 Tag C2；随后扩展更大、覆盖充分的正式地图及真实负样本。整层地图和实车启用另按明确授权推进。

目前可继续做离线定位开发与小范围功能测试；**尚不能将本次成果称作可靠 Tag 辅助全局定位的实车交付**。
