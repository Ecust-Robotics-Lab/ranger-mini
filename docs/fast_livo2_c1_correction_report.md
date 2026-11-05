# C1 地图修正动态偏差归因与既有 GICP 对照

日期：2026-11-04。承接 [两份新包与地图对照](fast_livo2_tag_two_capture_report.md)。用户授权继续核验 C1 地图修正带来的动态不一致。本轮只读原包、使用已建的新局部图；没有启动硬件、回灌前端、修改公共桥接、降低门槛、部署、提交或推送。

**结论：当前数据上，配准方式是额外不一致的重要来源。使用仓库已有的 small_gicp 后，保持原先拟合的 Tag 位姿不变，直行包跨访问回投 RMS P95 从 17.324 降到 6.114 px。** 新方法的完整 C2→C1 功能试验也通过，但仍有一次初始化超时和一次短暂 DEGRADED；没有 LOST/ERROR。Tag 物理时间仍未验证，没有真实 VALID 地标，真实 Tag 恢复和 Tag 回环尚未验收。

本轮新增诊断工具与 6 项独立几何测试，未改 C1/C2 或注册器实现。已有 PCL、GICP 两条实现均保留；正式配置继续原值，GICP 仅通过离线命令显式选择。

## 1 数据与比较边界

- 原转向包 `bags/fast-livo-tag140_turn_03-bk8_pjyy`；原直行包 `bags/fast-livo-tag140_straight_04-ryzykq2l`。用户已确认 Tag 和传感器安装在两包内及两包之间未变；标签为 tag36h11:1、打印后实测 0.140 m。
- 两包均复用上一轮原锁定源码、原零 gyro 初始化、+110 ms 派生假设的三融合重放，各 895 对 VIO 后位姿/区间云。没有再次调整时间、标定或尺寸。
- 地图固定为转向包经原 B 建立的 `outputs/fast-livo-tag140-two-capture-20261104/new_local_map/snapshot`，UUID `4ec68a99c626e1c6e2652506f5ecd8d6`、graph version 41、31 KF / 10 LiDAR 回环；manifest SHA `14658c3b156a05c3f792b9a3a1d90f2c79032f4a379c399715ce338c97faf9ef`。
- 直行是地图之外的独立采集访问；转向是建图访问本身。直行包已参与本轮方法选择，不能再作为未调参的盲测集；转向回归不能替代新的跨访问泛化验收。
- 软件基线 HEAD `acffa879cc8d52330276089cd3df83ad6a43f112`、分支 `refactor/pluggable-navigation`，原未提交成果保留。新增证据 `docs/fast_livo2_evidence/c1_correction_20261104/`，大输出 `outputs/fast-livo-c1-correction-20261104/`；备份路径在 `baseline.json`。

## 2 怎样量化“动态修正”

统一 `T_A_B` 将 B 中点变到 A。`X_k=T_map_imu(k)` 为 C1 私有结果，`U_k=T_local_imu(k)` 为同会话连续前端轨迹；这里的 local 不等同于 wheel odom，也不发布公共 map→odom。

```text
A_k = X_k * inverse(U_k)
X_pred(k) = X_(k-1) * inverse(U_(k-1)) * U_k
额外平移修正 = translation(X_k) - translation(X_pred(k))
额外旋转修正 = angle(R_pred(k)^T * R_k)
```

另外将第一次有效对齐冻结为 `A_0`，比较机器人原点 `translation(X_k)` 与 `translation(A_0*U_k)`；本报告的平移均在前端 IMU 状态参考点统计。这样不会把绕地图原点的旋转误读成机器人本身的平移。所有时刻保留 int64 ns；只接受同地图、同版本、同会话/attempt、原扫描戳已确认的有效帧。

原 PCL 直行 525 个有效帧的统计：

| 项目 | 实际结果 |
| --- | --- |
| 相对首次对齐的机器人位置差 P50 / P95 / max | 0.892 / 1.817 / 2.327 cm |
| 相对首次对齐的朝向差 P95 / max | 0.311 / 0.395° |
| 不同目标关键帧集合 | 47 组 |
| 相邻有效帧目标集合切换 | 208 / 524 次 |
| 集合不变时，逐帧额外平移修正 P50 / P95 | 0.517 / 1.055 cm，316 次 |
| 集合切换时，逐帧额外平移修正 P50 / P95 | 0.519 / 1.341 cm，208 次 |

这是两条估计轨迹之间的差异，不是实测绝对定位误差。切换时尾部略大，但不切换时也有明显变化，不能把原因只归到子图切换。

## 3 同扫描、同初值的注册器控制实验

预先保存 `protocol.json`，对原 525 个有效扫描逐一导出精确 XYZ/I，并固定每个扫描原本的预测初值。诊断程序直接链接现有 backend 与隔离 C1 core，调用原 `SubmapProvider`、`make_registration`、旋转合法性检查和独立几何复核。所有扫描、地图、标定与时间不变，Tag 不参与点云配准。

四个预定条件：

1. 原 PCL、动态选取的原 8 KF 子图。
2. 同一 PCL、冻结首个 VALID 帧选出的 8 KF；同包检查仍排除同扫描自匹配。
3. 同一 PCL、原动态子图合并后按已有 0.15 m 分辨率再降采样一次。
4. 仓库已有 `small_gicp`，原动态子图、原分辨率及独立几何门槛。

原始分片单帧已降采样，C1 当前代码合并多帧后直接给 PCL；因此第 3 项专门检查合并密度/离散化的影响。没有新写 ICP、调高迭代数或放宽收敛标准。

**复现检查：524 个原本单初值跟踪帧，原 PCL 对照与保存位姿的平移差为 0，旋转差最大约 1.69e−16 rad（数值舍入量级）。** 首个初始化帧原来使用 13 初值，不列入单初值位姿等同性断言。这确认实验复用了原路径，不是在另一套近似配准上推断。

原几何、500 ms 跟踪预算和 0.75 m / 0.35 rad 创新门槛保持。GICP 有 1/525 帧未收敛，被拒绝；其余条件 525/525 通过。这一失败及其受影响的图像支持均保留。用所有条件均有合法三帧支持的 **521 个共同观测、46 个训练帧、160 个留出帧**比较，每种轨迹仅在训练段拟合一个固定 Tag 常量，其他参数不动：

| 条件 | 训练 Huber 代价 | 同一 160 留出帧 RMS P95 / px |
| --- | ---: | ---: |
| 原 PCL 动态子图 | 2.892 | 15.626 |
| PCL 冻结子图 | 3.008 | 13.618 |
| PCL 合并后降采样 | 2.648 | 14.579 |
| 已有 GICP 动态子图 | **1.103** | **6.136** |
| 连续局部三融合轨迹 | 1.194 | 9.068 |

单帧条件不构成完整定位成功。方法候选按四个预定条件的训练代价选择，先写 `frozen_gicp_trial.json`，再运行完整流程；留出结果没有被当成第三方盲测。原 PCL 与 GICP 的独立点云 RMSE 中位数均约 5 cm，而 Tag 一致性明显不同，说明类似的点云总残差不足以辨别这类厘米级位姿差异。

GICP 未收敛时原结果中的默认 RMSE=0 不作为有效分数。原始注册器返回值原样保存并带失败状态；后续报告统计只使用几何通过结果，没有把 0 当作完美配准。

空间检查另取 5 个均匀扫描和 5 个额外修正最大扫描，检查**完整原始源云**的最近邻分布：10 帧的 0.5 m 门槛内匹配比例均不少于 98.83%，单个目标 KF 最大占 32.74%，内点 XYZ 协方差最小特征值不少于 0.994 m²；原 C1 平面 Hessian 比值也不接近退化门槛。这没有发现严重低重叠或全部支持挤在一个 KF 的特征，但它不是离散错地点排除或真实精度证明。完整源云统计与生产降采样/法向过滤分数分开标识。

## 4 完整 C2→C1 与转向回归

节点仍为上一轮 `install_c1_confirmation_20261104/` 的隔离构建；无需新改或部署运行节点。本轮只显式选择 `--registration small_gicp`，KISS 全局粗配准、C2 独立运动确认、C1 初始化/跟踪门槛和所有失效规则保持。这个选项影响既有 C2 精配准及 C1 的既有配准入口；Tag 仍未用于定位。

| 项目 | 独立直行 C2→C1 | 转向同包给定初值 C1 |
| --- | ---: | ---: |
| 提交 / 处理 / 取消 | 895 / 894 / 1 | 895 / 895 / 0 |
| 有效采样 | 520 | 622 |
| 首次 VALID（源数据时间） | 37.399760 s | 27.300408 s，index 267 请求初值 |
| 状态 | INITIALIZING 374、VALID 520、DEGRADED 1 | INITIALIZING 273、VALID 622 |
| LOST / ERROR | 0 / 0 | 0 / 0 |
| 公共 TF / 速度发布者 | 全为 0 | 全为 0 |
| 评估 / 节点退出码 | 0 / 0 | 0 / 0 |

直行保留一次初始化超时取消，原 C2 自动重试；初始化期间也有不收敛、初值历史窗口不足的拒绝。跟踪中一次 `not_converged` 产生 DEGRADED、位姿无效，随后恢复，没有伪造有效值或越过原状态机。单帧控制实验里的失败同样没有强制改成收敛。

PCL 同包对照首次 VALID 为 37.000 s，因此本轮**没有证明启动更快**。GICP 直行整次墙钟约 246.80 s，仍是逐扫描确认的离线节流；求解 P50 / P95 约 79.46 / 81.14 ms，峰值 RSS 550968 KiB。不能把整次墙钟、单帧时间或首次有效的源数据时间互相替代，也没有实时/长时验收结论。

对完整运行结果重新取共同图像、用相同训练/留出分组：

| 数据 | 共同观测 / 留出数 | 原 PCL RMS P95 / px | 已有 GICP RMS P95 / px |
| --- | ---: | ---: | ---: |
| 转向建图访问 C1 | 618 / 178 | 6.026 | **4.328** |
| 独立直行 C2→C1 | 516 / 160 | 15.185 | **6.553** |

直行相同有效扫描上，相对各自首次对齐的机器人位置差 P95 从约 1.536 cm 降到 0.775 cm；目标集合不变时逐帧额外平移修正 P95 从约 1.056 cm 降到 0.705 cm。它们仍是内部轨迹差异，不是物理测量误差。

## 5 冻结旧 Tag 后的跨访问检验

最关键的对照保留原先 PCL 转向访问拟合的 Tag 常量，SHA 绑定原 `backfill_new_map_recovery/turn_fit.json`，**没有在直行包中重新登记或调整它**。仅让查询使用新的 GICP 运行轨迹，在相同 516 个可验证观测上：

| 冻结地标与查询 | RMS 中位 / P95 / max，px |
| --- | --- |
| 原 Tag 常量 → 原 PCL 直行轨迹 | 5.315 / **17.324** / 24.957 |
| 同一原 Tag 常量 → GICP 直行轨迹 | 2.547 / **6.114** / 14.433 |
| 只用 GICP 转向访问重新拟合常量 → GICP 直行轨迹 | 2.556 / **6.117** / 14.442 |

原先全 523 个 PCL 健康观测的 17.286 px 与本表的 17.324 px 分母不同；本表为配对比较统一到 516，不能混用。改善主要来自查询配准轨迹变化，重新拟合 Tag 常量几乎没有进一步改善。

以原初始试验门槛 3 px RMS / 6 px 最大角点统计，新查询 321/516 帧通过，最坏 RMS 仍约 14.43 px。该阈值没有在本轮重新标定，也不是普适物理精度标准；本轮没有放宽它或导出真实 VALID 表。+110 ms 仍为有效时间假设，正式 `time_verified=false, offset_ns=0`。

因此：软件链路具备继续做 Tag 辅助离线联调的基础，一致性比前轮明显改善；**可靠 Tag 登记、实际 Tag 辅助恢复/回环、未调参的独立访问、物理时间与真值精度尚未通过**。当前所有真实恢复结果仍是 Tag 关闭的纯 LiDAR 候选 C2。

## 6 新文件、命令与证据

新增源码：`scripts/fast_livo2/tags/diagnose_c1_corrections.py`，默认 dry-run，输出单段轨迹诊断；`tests/test_c1_correction_diagnostics.py` 的 6 个独立夹具验证非单位六自由度对齐、已知 2 cm 交替偏差、地图坐标变换不变性、目标 ID 顺序、未确认帧排除以及会话/时间/缺样失败模式。没有给低影响的文档修改另写实现镜像测试。

诊断 C++ 程序仅在证据 `probe/`，构建到 `ws_fast_livo/build_c1_correction_probe_20261104/`，不安装至原前缀。`prepare_probe.py` 验证原 PointCloud2 frame/encoding/step，逐点导出 XYZ/I 并做读回等值检查，原 CDR 和 bag 不变。`analyze_probe.py`、`analyze_runtime.py` 分别处理单扫描与完整运行结果，不能互相冒充。

| 实际命令/执行记录 | 退出码与数量 | 日志（本轮证据目录） |
| --- | --- | --- |
| `python3 tests/test_c1_correction_diagnostics.py` | 0，6 项 | `diagnostic_tests.log` |
| `python3 tests/test_tag_motion_geometry.py` | 0，3 项相关回归 | `motion_geometry_regression.log` |
| `python3 scripts/fast_livo2/tags/diagnose_c1_corrections.py --execute --samples .../straight/c2_new_map_recovery/samples.jsonl --native-db .../straight/livo_110000000/result/native_frames/native_frames_0.db3 --output NEW_DIR` | 0，525 有效帧 | `straight_diagnostics.log` |
| `cmake -S .../probe -B ws_fast_livo/build_c1_correction_probe_20261104 -DCMAKE_BUILD_TYPE=Release`；`cmake --build ... -j2` | 0，诊断目标 | `probe_configure_retry.log`、`probe_build_self_exclusion.log` |
| `python3 .../prepare_probe.py --execute --capture straight --output NEW_DIR` | 0，525 云逐点等值 | `prepare_straight.log` |
| `.../c1_correction_probe .../straight_inputs/job.json NEW_OUTPUT_DIR` | 0，525×4 注册结果 | `straight_probe.log` |
| `python3 .../analyze_probe.py --execute --capture straight --output NEW_DIR` | 0，共同集合比较 | `straight_comparison.log` |
| `python3 .../inspect_correspondences.py --execute` | 0，10 个空间检查扫描 | `correspondence_inspection.log` |
| `bash .../run_gicp_checks.sh` | 0，两完整运行 | `runtime_exits.json`、`straight_gicp_runtime.log`、`turn_gicp_runtime.log` |
| `python3 .../analyze_runtime.py --execute` | 0，两包同帧/跨访问比较 | `runtime_comparison.log` |

首次 CMake 配置因未将已有 backend 安装前缀加入 CMAKE_PREFIX_PATH 退出 1，原 `probe_configure.log` 保留；补齐项目内前缀后成功，没有安装系统依赖。注册器、GTSAM、small_gicp 等均沿用既有锁定版本及许可，没有升级或替换第三方代码。

原 74 项相关验收是上一轮结果，不累加成新的测试数量。本轮修改运行诊断工具，复用未改变的已验节点/库；实际两包 ROS 流程重新执行并记录全部失败、短暂无效状态和恢复。源码、库链接、输入与原始资产 SHA 的收尾核对见本轮 `delivery_checks.json`。

图与原图证据：

- [控制实验和修正曲线](fast_livo2_evidence/c1_correction_20261104/correction_controls.png)
- [实际 Tag 图像的回投示例](fast_livo2_evidence/c1_correction_20261104/tag_reprojection_example.png)：按原 PCL 留出误差最大帧选出的单例，仅作说明；完整统计在上表，角点为图像检测值，不是人工真值。

## 7 当前状态与下一步

| 分类 | 结论 |
| --- | --- |
| PASS | 新几何诊断及 6 项独立测试；原 PCL 跟踪逐帧精确复现；四条件控制实验；既有 GICP 完整两包运行；同集合与冻结旧 Tag 的跨访问改善；原始资产保护 |
| PARTIAL | 主要来源支持配准方式/点云离散化，而不是唯一物理根因的鉴定；GICP 只在当前数据上得到支持；真实位置正确性、绝对精度、物理时间、实时/长时、负样本及未调参独立访问仍待验 |
| FAIL / 未满足 | 单帧 GICP 1 次不收敛；完整直行一次初始化超时取消和一次跟踪 DEGRADED，均拒绝后按原规则继续；真实地标仍未满足本轮登记条件 |
| 未实施 | 更改正式默认、部署、真实 VALID 地标写入、真实 Tag C2/B 性能验收、硬件或导航、Git 提交/推送 |

下一步优先验证曝光/扫描物理时间映射与剩余尾部误差；只有获得足够来源证据，才登记地图版本匹配的真实地标，冻结后用新的独立访问检验 Tag 辅助 C2，再用实际回访覆盖 Tag 辅助 B。回环源码和合成基础已具备，但本轮 C1/GICP 改善不自动转成真实 B 回环验收。需要补充采集时给出具体目标，不要求用户立即重录相同诊断动作。
