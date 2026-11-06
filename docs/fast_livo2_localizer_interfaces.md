# FAST-LIVO2 C1 离线先验地图定位契约

更新：2026-11-04。实现为 `ws_fast_livo/src/ranger_fast_livo_localizer`；原始验收见 [C1 报告](fast_livo2_stage_c1_report.md)，初始化计算优化与隔离新构建见 [两份新包对照](fast_livo2_tag_two_capture_report.md)。本页描述默认 C1 人工初值模式；显式启用的 [C2](fast_livo2_relocalizer_interfaces.md) 在同一包提供全局确认初值和丢失恢复，仍由 C1 跟踪。人工 Initialize 会暂停可选自动搜索，需显式重新允许。生产 runtime 仍使用 Lightning。

## 模式与地图

输入仍为 `/fast_livo/odometry` 和 `/fast_livo/deskewed_imu`，沿用 [B 输入契约](fast_livo2_backend_interfaces.md)：VIO 后 IMU 位姿、单区间 IMU 云、严格整数纳秒配对。云中的 LiDAR–IMU 外参不重复应用。建图和先验定位为互斥模式；受管入口与 B 共用 `fast_livo_backend` 资源锁。私有节点没有 TF broadcaster，也不反馈前端。

C1 直接读取 `schema_version=1 / kind=stage_b_graph_archive`。读取器检查 COMMITTED 与 manifest SHA、所有清单文件 SHA、必需文件、版本/字段/变换方向、非负整数 ID/时间、唯一关键帧及扫描 ID、严格递增时间、合法位姿和有限 PCD。路径必须位于归档根内，拒绝 `..`、非规范路径、符号链接、partial 和 FAILED。JSON 重复键、过深嵌套、截断和额外 PCD 数据均拒绝。PCD 为 B 输出的 binary XYZI FLOAT32，CDR 只校验和保留，不反序列化或作为定位目标。

默认加载上限：manifest 16 MiB、单文件 512 MiB、文件总量 8 GiB、100000 关键帧、单分片 200 万点、全局地图 1 亿点；逐分片及流式检查全局 PCD，不整体载入全分辨率地图。位姿平移绝对值限制在 1e6 m 内，供 PCL 浮点索引使用。地图坐标不保证重力对齐。

完整来源身份模式要求归档 provenance 与当前显式期望的 `sensor_configuration_sha256`、`time_contract_sha256`、`frontend_sources_sha256` 三项一致。当前 B 运动归档缺这些完整绑定，默认不能静默接受。可提供独立 `compatibility_path`：版本 1、`offline_experiment`、源 manifest SHA、证据文件的绝对路径/SHA、非空 limitations，且同步/绝对精度标志必须为 false。它不能覆盖已经指定的期望身份，不会补写原档或把当前配置冒充历史配置。

2026-11-05统一在线建图的新归档提供这三项身份及`source_identity.json`中的构成。C1/C2受管入口与`archive_tool`新增`--identity-file ABS`以传入期望身份，与旧图`--compatibility`互斥。身份应来自当前查询前端实际配置和版本；匹配只证明配置/来源兼容，不证明物理同步或绝对精度。旧归档缺身份的处理规则保持。

`archive_tool --archive ABS --compatibility ABS` 在新进程中执行上述加载。可加 `--index-output NEW_ABS_DIR`，以不可覆盖的原子提交写出派生索引身份、优化位姿、参数及哈希。内存三维 KD-tree 从归档优化位姿建立，运行不依赖派生索引文件。每个 map 实例拥有独立 LRU，切图不会按重复 ID 复用旧缓存。

## 私有服务与结果

`/fast_livo/localizer/load_map`：`LoadMap` 请求包含 request_id、绝对 archive_path、compatibility_path，响应 accepted/reason/map_generation。受理后停止旧有效输出，提升地图代际和定位会话；成功后等待新初值。失败保留最后可读地图，但不恢复旧跟踪，需要给该地图的新初值。重复同 ID/同内容返回原受理结果；同 ID 不同内容拒绝。地图加载和定位确认均不由 accepted=true 证明。

`/fast_livo/localizer/initialize`：`Initialize` 请求必须匹配最近私有结果的 map_uuid、map_generation、session_id；包含 request_id、reference_body 和 `PoseWithCovarianceStamped`。frame_id 必须为 map，reference_body 为 base_link 或 fast_livo_imu。

- `stamp=0` 明确定义为绑定服务受理后的下一对有效输入；不解释为 now。
- 非零 stamp 使用同前端会话的历史插值，将初值传播到扫描原采样时刻。历史不足则等待或拒绝，不外推或拼接最新值。
- `T_map_imu=T_map_base*T_base_imu`。base 初值协方差按 map 切空间位置/小角度、XYZ/旋转 XYZ 顺序解释，并通过外参杆臂的 Jacobian 传播。
- 协方差必须有限、对称、半正定；转换后的各位置方差不超过 4 m²、旋转方差不超过 0.5625 rad²。这是有界实现范围，不保证该范围内 ICP 收敛。
- 新请求提升 request_generation，取消旧任务；每会话最多保留 128 个初始化请求 ID。无有效初值或 LOST 时不进行无初值搜索。

私有 `/fast_livo/localizer/estimate` 使用 `LocalizerEstimate`。原子包含 map 中 IMU 位姿、原采样 stamp、pose_valid、既有五态、reason、map UUID/manifest SHA/冻结 graph version、地图代际、定位 session、初始化请求代际/ID、scan_id、最新扫描和最后确认时间以及 quality_json。scan_id 是当前前端会话的输入对序号。

只有几何确认后的结果可为 VALID。拒绝候选不改已接受位姿/时间；DEGRADED/LOST 不输出有效新位姿，心跳保持最后确认时间。预测仅用于下一次配准初值，不作为地图匹配成功输出。ROS 时钟回退、输入发布者代际变化或消失撤销初值；多输入发布者、多私有输出发布者属于终端 ERROR，重开受管会话后才能恢复。地图、会话或请求变化后，worker 的旧结果不得提交。

## 几何确认与预算

子图按 map XYZ 的三维距离选择，默认半径 8 m、最多 8 帧、100000 匹配点、LRU 128 MiB。每个机体系关键帧只用自己的 X_map_imu 变换一次。默认排除与查询同整数采样戳的关键帧，作为同包功能检查的直接自匹配防护；这不构成数据独立性。

初值使用粗/细两级配准，返回总 T_map_imu。局部 13 个种子包括初值、三个平移轴的正负扰动（由协方差限定为 0.3～0.6 m）和三个旋转轴的正负 0.15 rad 扰动。质量接近但相隔超过 0.5 m 或 0.25 rad 的解拒绝。该有界局部探测不是全局多解排除保证。

默认至少两帧，间隔至少 0.2 s 且有 0.05 m 平移或 0.03 rad 旋转的前端基线，并检查两帧地图修正的一致性。静止重复扫描不能补足运动支持。跟踪按 `X_pred=X_accepted*inverse(U_accepted)*U_current` 预测，再做精配准；相对预测超过 0.75 m/0.35 rad 的结果拒绝。

质量使用独立对应数、source overlap、米制 RMSE、归一化点到平面 Hessian 特征值比和刚体合法性；定义复用 B，阈值由独立 `localizer.yaml` 配置，属于试验值。Hessian 逆不作为已标定协方差。quality_json 记录总变换、预测、目标帧、点数、缓存、耗时、配对和队列计数。

PCL float 求解矩阵先通过既有刚体合法性检查，再将微小旋转数值误差投影到 SO(3)，并对实际接受的变换重新计算几何门控，避免连续预测累积非正交应变；非法原始结果仍拒绝。

2026-11-04 源码优化：首个初始化支持帧，以及达到帧数/时间/运动条件、可能提交 VALID 的每一帧，保留完整 13 初值歧义检查。未满足提交条件的等待帧只运行一个初值的粗/精配准和原几何、修正一致性检查，保持无效；任何拒绝仍清空支持。诊断增加 `native_diagnostics.initial_ambiguity_probe`，与 `local_hypotheses` 一同记录。没有放宽几何、运动或超时门槛；目前仅安装于独立 `install_c1_confirmation_20261104/`，原 `backend_install/` 尚未替换。

默认初值单次预算 2000 ms、跟踪 500 ms、50 次迭代、单线程；库内求解不能硬中断，返回后检查预算，超时结果不提交。初始化总墙钟超时 10 s，确认健康时间从几何确认完成计；位姿采样时间独立检查。连续 3 次拒绝或确认超过 1 s 触发 LOST，重新给初值才能恢复。

输入配对 32 项/64 MiB，缺对超时 0.25 s、乱序等待 0.02 s；worker 队列 8 项/32 MiB；ROS 输入 depth=16。前端历史最多 5 s/1000 项、插值最大间隔 0.15 s。默认位姿年龄 0.5 s、流超时 2 s。以上上限不能代替原速及长期压力验收。

## 离线入口与公共适配

2026-11-04 [C1 修正归因与 GICP 对照](fast_livo2_c1_correction_report.md)通过现有 `--registration small_gicp` 显式选项运行完整两包；这同时选择C2精配准与C1既有实现，没有改默认 `pcl_icp`。实际验收使用 `install_c1_confirmation_20261104/` 隔离节点；受管入口的原 `backend_install/` 尚未替换，不能把默认入口视作相同构建版本。Tag、物理时间与真实回环仍单独验收。

```bash
bash scripts/build_fast_livo2_localizer.sh                 # dry-run
bash scripts/build_fast_livo2_localizer.sh --execute
bash scripts/run_fast_livo2_localizer.sh                   # dry-run
bash scripts/run_fast_livo2_localizer.sh --status
# 指定地图和兼容说明后显式执行；不启动传感器、wheel、底盘或导航
bash scripts/run_fast_livo2_localizer.sh --execute \
  --archive /ABS/B_ARCHIVE --compatibility /ABS/OFFLINE_DECLARATION.json
```

入口固定 localhost，默认隔离 domain 189 和模拟时间。初始化由上述私有服务提供；结果可观察 `/fast_livo/localizer/estimate`。正常入口仅启动监视器和 localizer；加载错误可观察并重试，终端错误或子进程死亡由 guardian 清理所属进程，不自动重启。

只有显式 `--offline-bridge` 才启动独立 C1 适配器和现有公共桥接。适配器计算 `T_map_base=T_map_imu*inverse(T_base_imu)`，保留原戳，检查确认身份、代际、已退休 session、采样和 steady 心跳新鲜度。公共消息保持既有 `LocalizationEstimate`，没有伪造 B loop_event。

公共桥接继续唯一发布 map→odom、odom→base_link，复用 wheel 原时刻历史插值、禁止外推和首次创新失败锁定。调用者在占用检查完成后提供同域合成 wheel；私有定位成功、桥接接纳分别记录，不能自动解除锁定。配置内 T_base_imu 仍是近似值，合成外参通过不等于实物标定通过。
