# FAST-LIVO2 阶段 B 后端契约

更新：2026-10-26。实现位于 `ws_fast_livo/src/ranger_fast_livo_backend`。
本页描述已实现的 B 接口；真实三传感器运动回环、定位精度仍未验收。

## 输入、时刻与坐标

统一记 `T_A_B` 将 B 中的点变到 A：`p_A = T_A_B p_B`。

- `/fast_livo/odometry`：`nav_msgs/Odometry`，父 `fast_livo_local`，子 `fast_livo_imu`，`U_i=T_local_imu(i)`。
- `/fast_livo/deskewed_imu`：`sensor_msgs/PointCloud2`，`fast_livo_imu`，不是世界云或累计云。
- 后端以 `sec*1_000_000_000+nanosec` 的 int64 完全相等配对，不使用 ApproximateTime，不转 double 建索引。
- 前端 `sync_packages` 在图像时刻（加试验 exposure offset）切分扫描，先 LIO，再在同一时刻做 VIO。`ImuProcess::UndistortPcl` 将该区间点补偿到区间末端的 LiDAR 系；`processImu` 保留这份云。`handleVIO` 在视觉更新后调用 `publishBackendFrame`，使用当前 `_state`，并以 `extR*p+extT` 应用一次 `T_imu_lidar` 得到 IMU 系云。
- 去畸变依赖前端预测运动，未用 VIO 后的整段轨迹重新去畸变。云与位姿具有同一参考时刻/机体，不能解释为它们共同经历了一次 VIO 后的点级重算，也不证明曝光与激光物理同步。
- 前端只调用一次 `sec2Stamp(last_lio_update_time)`，将结果复制给云。原始 header 已在前端内部经过 double，存在亚微秒舍入；后端严格配对保证这两个输出之间的整数一致性，不能恢复原始传感器纳秒精度。
- 后端不再应用 LiDAR–IMU 外参，不按 curvature 再去畸变。x/y/z/intensity 为 FLOAT32；允许带正常布局的其他字段，完整消息以 CDR 保留。支持行填充和未对齐字段，拒绝大端、坏类型/偏移/重叠/步长、非有限使用字段及非法四元数。

阶段 A 的 `Reset()` 在 IMU 初始化路径调用；当前正常估计循环没有获授权的运行中局部坐标重置入口。两话题本身没有 session 字段。B 用两条输入的 DDS publisher GID 识别进程代际，发现身份建立/变化、消失或 ROS 时钟回退时创建新 session，清空缓存/队列/图。多发布者和终端处理错误不自动恢复，需重启受管会话。若以后增加同进程前端 reset，必须同时增加明确 reset/session 信号；不能只靠 GID 检测。

## 图、候选与配准

地图保留 FAST 局部原点、轴和 yaw，首帧先验为 `X_0=U_0`，不把首 IMU 姿态强制变成水平。当前配置 `gravity_align_en=false`，地图重力方向未验证。默认使用 3D 距离检索；XY/独立高度模式还要求 gravity 标志和 metadata 中的重力对齐证据。重力因子未实施、默认无此额外信息。

```text
相邻观测 Z_ij = inverse(U_i) × U_j
历史子图中第 k 帧点：p_i = inverse(X_i) × X_k × p_k
配准初值：inverse(X_i) × X_j
配准最终输出：TOTAL T_imu(i)_imu(j)，已包含初值
BetweenFactor<Pose3>(i, j, Z_ij, noise)
非关键帧：X(t) = X_k_optimized × inverse(U_k) × U(t)
```

PCL ICP 与 small_gicp/GICP/VGICP 共用 `Registration` 接口。二者直接接收自己的 body source 与历史 i 系 target，返回总变换，不再左乘一次初值。未来的 `CoarseRegistration` 无可靠初值接口仅预留，不包含 KISS-Matcher 求解器。

`CandidateSearch` 与几何验证分开。当前 `PositionSearch` 对优化位置做有界线性空间检索，按距离/ID 排序、时间和 ID 排除、邻近历史窗口去重，再限制候选数。未来描述子实现返回的候选不再经过位置半径过滤。位置先验超出搜索范围会漏环，尚无描述子大漂移恢复能力。

历史子图只含排除窗口外、与当前 source 不同的 scan ID，逐帧变到 i 系；本版不聚合额外非关键帧扫描。位姿关键帧记录与匹配云缓存分开，full/match 分片关联同一 scan ID；后续聚合必须维护去重后的扫描集合。

所有方法共用独立门控：收敛/刚体有限性、有效对应数、source 重叠率、米制 RMSE、归一化点到平面 Hessian 的最小/最大特征值比。这个退化指标是工程启发式，非可靠性证明。small_gicp 自身的内点、error、H 特征值只作为诊断，未求 H 逆充当协方差。质量接近且历史地点相距超过可配置的 ambiguity_separation（默认 1 m）的多解会拒绝；该门槛独立于搜索半径。

超过大修正门槛不直接判错：先 pending，等待不同关键帧、足够时间且有平移或旋转基线的独立观测；前端运动基线与几何配准基线都须成立，两个 map 左乘修正需一致，才把两份因子共同提交。没有独立证据就不提交。本版不是完整 PCM，重复结构风险仍需运动数据验证。

图由一个 worker 持有；配准同步读取冻结快照，ROS 线程只入队。`MatchTask` 检查 session、版本和源/子图依赖位姿。旧 generation 的处理结果不得进入新 session 输出。iSAM2 失败后从已接受因子重建，公开记录和版本不前移；只有有限且目标函数未明显增大的完整结果才提交。日志的优化前/后 error 使用同一组因子。

Pose3 噪声顺序为 **旋转 xyz、平移 xyz**，分别配置标准差，非 ROS covariance 顺序。回环核支持 huber/cauchy/none，默认 Huber；鲁棒核不替代几何拒绝。

## 缓存、关键帧与预算

实验初值全部在 `config/fast_livo2/backend.yaml`，未以实车调优。

| 项目 | 默认 |
| --- | --- |
| 选帧 | 平移 ≥0.5 m 或旋转 ≥10°；最小关键帧间隔 0.1 s |
| 时间选帧 | ≥2 s 且（平移 ≥0.05 m 或旋转 ≥2°）；静止不重复生成 |
| 配对 | 32 项、64 MiB、0.05 s 乱序等待、0.25 s 缺对超时 |
| 工作队列 | 16 对、64 MiB；按可用槽位限量从配对缓存移交 |
| 匹配缓存 | LRU 128 MiB；原始完整云不长期驻留 |
| 子图/图上限 | 300000 匹配点、100000 关键帧；超限明确报错 |
| 位置候选 | 3 m、排除最近 20 关键帧和 30 s、最多 5 个、子图 ±3 帧 |
| 几何门控 | 对应距离 0.5 m、至少 30 对、source overlap ≥0.45、RMSE ≤0.2 m、特征值比 ≥1e-4 |
| 处理预算 | 50 次迭代、单线程配准、候选批预算 1000 ms |
| 大修正确认 | >1 m 或 >30°；独立证据 ≥1 s 且 ≥0.5 m 或 ≥5°；修正相差 ≤0.3 m/10° |
| 因子标准差 | odom 平移 0.05 m/旋转 0.02 rad；loop 平移 0.1 m/旋转 0.05 rad |

预算计时使用 steady clock；采样新鲜度、epoch 与回放回退使用节点 ROS clock。候选超时在求解器返回后检查，不能硬中断库内一次计算；迭代数和点数共同限制工作量。长时间 CPU/内存实测仍待验。

缺对时只删除过期项；容量不足优先删除最老未配对项，全部完整时拒绝新增并计数，不清空后续完整数据。byte budget 拒绝不覆盖已有内容。原始完整关键帧云采用同步、检查完成后的不可变分片落盘，因此没有异步写盘队列；代价是磁盘延迟可能让处理落后，届时新鲜度撤销有效输出。

## 私有输出、适配与公共 TF

`/fast_livo/backend/estimate` 使用 `BackendEstimate`，原子包含原采样 stamp、map 中 IMU 位姿、pose_valid、五态健康、session、graph_version、scan_id、reference_keyframe_id、loop_event_id、回环接受和优化提交信息。scan_id 是本 session 的前端区间云接收序号，**不是**雷达硬件帧号；前端区间可能跨越原雷达扫描边界。

另外发布 raw_path、optimized_path 与 private health；不创建 TF broadcaster。路径最多约 2000 个显示点；完整轨迹在快照中。

适配器 `fast_livo_adapter` 计算 `T_map_base=T_map_imu×inverse(T_base_imu)`，使用既有 runtime 中的 xyz/rpy 和公共 `LocalizationEstimate` 五态。不做 latest-TF 查询；心跳保留位姿 stamp，错误、过期、断流和无效 pose 不产生 VALID。

正常受管入口仅启动 monitor 和后端。显式 `--offline-bridge` 才增加适配器与公共桥接，强制模拟时间和 localhost；没有传感器/底盘/导航/速度节点。仍需调用者在同一隔离域提供合成 wheel。公共 TF 的唯一拥有者是原桥接。

桥接创新失败后不更新接受缓存或接受历史；同戳失败心跳不重复累加。默认生产策略继续使用原 fail_limit。B 联调显式启用 `latch_on_innovation_failure=true`：首次失败就停止新的 map 修正、撤销就绪，时钟回退也不清除锁定；须重新开启联调进程。wheel 插值和禁止外推继续使用既有 `PoseHistory`。

分别观察私有消息/候选日志中的回环与优化提交，以及公共 `/localization_health` 中的 innovation_unlocked、backend_reason（含 session/graph/loop event）与 innovation_last_error。桥接拒绝不删除私有回环。

## 一致性导出与后续接口

服务 `/fast_livo/backend/export_snapshot`：请求 `destination` 为不存在的绝对目录，父目录须存在；响应 success/reason/session_id/graph_version/manifest_path。

先冻结图与因子、分片清单、候选日志字节前缀；释放图锁后进行长时间导出。复制并校验不可变 full.pcd/match.pcd/scan.cdr，流式逐帧用各自 `X_i` 构建 map.pcd，每点只变换一次。内存中只装一份完整分片，不积累完整全局云。

最终目录包含 manifest.json、COMMITTED、original.tum、optimized.tum、map.pcd、keyframes/、candidates.jsonl。manifest 包含 schema/map UUID/session/graph version、整数 ns、位姿方向、全部因子与唯一 ID、字段定义、分片 SHA256、参数及提供的时间/标定/代码来源。TUM 只是兼容展示，权威时间仍为整数 ns。

scan.cdr 是完整 **后端输入 PointCloud2**，不是原始雷达包。原始三传感器 bag、图像和标定仍须保留。受管入口记录声明的前端配置/外参/源码哈希，可用 `--source-id` 绑定录包身份；它不声称已经证明正在运行的外部前端与这份声明一致。独立直接执行节点时，缺少来源会明确记录为未提供。

同文件系统 `.partial-UUID` 完整写入、校验与 fsync 后，以 Linux renameat2(RENAME_NOREPLACE) 提交，拒绝覆盖和目标竞争；失败去掉成功标记，保留 FAILED.json/partial 证据。文件大小限制注入、不可写路径、损坏分片均有测试；未实际填满磁盘或模拟突然断电。

这是 B 几何图归档，不是 C1 已实现的加载格式。几何、字段、整数时刻及来源信息为后续 HBA/LVBA 转换保留；LVBA 所需原始图像还需从绑定的原始 bag 提取。C1 应先定义加载校验、标定兼容与定位入口；C2 在独立 CandidateSearch/CoarseRegistration 接口增加检索和无初值几何验证。均未在本阶段实现。
