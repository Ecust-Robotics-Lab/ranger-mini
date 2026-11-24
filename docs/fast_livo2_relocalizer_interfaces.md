# C2 全局重定位接口

C2 在 `ranger_fast_livo_localizer` 内向 C1 提供确认后的初值。C1 负责独立确认与持续定位。[在线定位入口](fast_livo2_online_localization.md)和工作台使用同一链路。手动 C1 配置关闭自动全局搜索。

## 数据与地图

输入沿用 [C1 契约](fast_livo2_localizer_interfaces.md)：同一前端会话、严格整数同戳的 `/fast_livo/odometry` 和 `/fast_livo/deskewed_imu`。云为单区间 IMU 系去畸变云，外参已经应用。多帧查询使用 `inverse(U_current) * U_previous` 变到当前参考 IMU，不用累计世界云。

新前端试验配置开启初始重力对齐，但既有 B 地图不因此变为水平；C2 不使用 yaw-only 或重力约束。描述子在关键帧 IMU 云上计算，为自有三维径向/点对距离分布，分数仅供排序。`PlaceRetriever` 返回多个关键帧 ID，与 B 图内位置搜索分开；没有原位置 3 m 筛选。

`CoarseRegistration` 可选实现使用 KISS-Matcher 的完整六自由度总变换，`use_quatro=false`。目标子图先变到历史候选 i 系，得到 `T_i_query` 后，精配准初值为完整 `X_i * T_i_query`。精配准复用 B 注册接口和独立几何门控，包括对应数、单向 source 覆盖、RMSE、退化与刚体合法性。质量接近且地图位姿分离的已检索候选视为歧义，拒绝该次尝试。

确认必须同时满足：多次几何通过、查询 scan ID 集合互不重叠、原戳时间基线、前端运动基线、地图几何匹配运动基线，以及 `X_query * inverse(U_query)` 的跨查询一致性。静止重复云即使伴随里程计漂移，也不能供给地图运动基线。大地图修正本身不是拒绝理由。

## 派生索引

`PlaceIndex` 只读源 B 归档，绑定 map UUID、源 manifest SHA、冻结 graph version、关键帧/scan ID、`keyframe_imu` 坐标系、无重力假设、算法版本和描述子参数 SHA。目录包含 `entries.json`、`parameters.json`、`manifest.json`、`COMMITTED`，全部检查 SHA/版本/点数和大小界限；不允许索引文件符号链接或已有目录覆盖。

写入新 `.partial-*` 目录，使用原子 no-replace rename 提交；失败保留 FAILED/UNCOMMITTED 证据，不能留下成功标志。切图、参数或来源身份变化须另建索引，不能改写旧归档。

```bash
# 这些命令只操作离线文件，必须加载本地工作空间环境。
ws_fast_livo/backend_install/ranger_fast_livo_localizer/lib/ranger_fast_livo_localizer/place_index_tool \
  --build ABS_ARCHIVE ABS_COMPATIBILITY ABS_NEW_INDEX
# 新进程核对，不重建：
ws_fast_livo/backend_install/ranger_fast_livo_localizer/lib/ranger_fast_livo_localizer/place_index_tool \
  --check ABS_ARCHIVE ABS_COMPATIBILITY ABS_INDEX
```

入口可用 `--index ABS_INDEX` 指定已绑定索引；省略时在加载地图阶段构建有界内存索引。兼容说明沿用 C1，缺完整物理身份的旧地图须显式绑定离线限制。

## 启停、请求与状态

```bash
bash scripts/build_fast_livo2_relocalizer.sh    # 默认 dry-run
bash scripts/run_fast_livo2_relocalizer.sh      # 默认 dry-run，domain 190 / localhost
bash scripts/run_fast_livo2_relocalizer.sh --status
# 仅离线私有定位节点；不启动驱动、底盘或导航：
bash scripts/run_fast_livo2_relocalizer.sh --execute \
  --archive ABS_ARCHIVE --compatibility ABS_COMPATIBILITY
```

可选 KISS 构建要显式 `--execute`，版本锁在 `config/fast_livo2/relocalization_sources.lock.json`，安装前缀 `ws_fast_livo/relocalization_deps`；不安装替换系统库。C1 构建脚本显式关闭 `RANGER_ENABLE_KISS`；C2 构建脚本开启。未编译插件却启用全局搜索时，节点启动失败，不能静默回退为伪全局定位。依赖包含 GPLv3+ PMC，详见包内 `THIRD_PARTY_NOTICES.md`。

私有服务 `/fast_livo/localizer/relocalize`，类型 `Relocalize`：

- 请求：`request_id`、当前 `map_uuid/map_generation/session_id`、`enable`。
- 响应：`accepted`、`reason`、`request_generation`。受理不等于定位成功；重复相同 ID/载荷幂等，变更载荷拒绝，历史上限 128。
- `enable=true` 撤销当前有效输出，清除旧初始化/支持并开启新搜索；`false` 取消并撤销该初始化/跟踪链，等待显式重开或人工初值。
- 人工 Initialize 服务优先，暂停自动搜索；要重新允许 C2，显式请求 `enable=true`。默认 C2 会在首次启动及 C1 失效后搜索。

同一个 worker 负责 C1/C2。切图、时钟回退、前端发布者变化、取消及人工初值都提升共享任务代际；异步旧结果不能提交。服务检查地图/会话身份。重启不保留已确认支持。

结果仍为 `/fast_livo/localizer/estimate`，沿用五态；reason 区分 `map_loaded_searching`、`searching_waiting_query_window`、`verifying_*`、`retry_backoff`、`global_no_geometric_match`、`global_ambiguous_places`、`global_inconsistent_motion_support`、`global_budget_exhausted` 等。质量 JSON 包含候选分层指标、查询 ID/原戳、地图身份、资源与耗时；`global_search_enabled/allowed` 显示是否编译入口已启用及当前允许搜索。

确认后用原采样戳向 C1 初始化，请求 ID 为 `global-*`，提升初始化请求代际，先继续几何/运动验证，再由 C1 发布 VALID。候选或服务受理不会直接产生 VALID。LOST 搜索期间保持无效；心跳不刷新最后确认戳，失败不改已接受位姿缓存。

在线定位用 `--public-tf` 接入公共适配，离线复现用 `--offline-bridge`。两者沿用既有 TF 所有权及同刻 wheel 插值，外推不允许。若公共桥接因创新拒绝而锁定，C2 成功后仍须重开对应会话。

## 默认资源与限制

显式 C2 配置 [relocalizer.yaml](../config/fast_livo2/relocalizer.yaml) 使用 8 帧和最多 12000 个查询点，时间窗口至多 1 秒。运动确认与强几何静止确认各有独立判据，具体门槛由对应配置与下文契约说明。

索引最多 10000 关键帧（可配置且有硬上限），目标子图沿用 C1 的最多 8 帧/100000 点。C1 与 C2 各自有 128 MiB LRU，并非进程总内存上限；另有索引、查询、工作队列、配准器临时内存。求解器内部不能硬中断；每层返回后重新检查预算/代际，超时拒绝整次尝试，不能选取此前最好候选凑成功。

跨会话泛化、真值精度及持续现场定位验收见 [STATUS](STATUS.md)。离线评估的 `--query-provenance` 绑定独立输入与地图身份，`--allow-recovery` 只调整评估报告如何处理实际丢失后的恢复。

在线使用自有`radial_pair_multiscale_v2`（195维、平滑多尺度距离分布与Hellinger嵌入），`global_descriptor`控制索引绑定与比较；在线入口默认v2，旧v1索引/离线配置兼容。官方STD/Scan Context仅进入独立离线benchmark，不是在线插件。C1/C2入口默认注册器已按用户指令改为small_gicp，KISS全局粗配准保持。对照与限制见报告。

### 静止确认

显式C2 overlay默认`global_stationary_initialization: true`（核心SearchConfig默认false，手动C1接口不变）。启用时每轮检查最多16检索候选，保持原全局地点歧义拒绝；静止分支额外要求重叠≥max(0.65,min_overlap+0.1)、RMSE≤min(0.12m,0.65×max_rmse)、归一化几何特征值比≥max(1e-4,3×degeneracy_ratio)，不同地点竞争解使用双倍歧义余量。至少3个互不重叠查询窗口、跨度≥1秒、FAST与地图运动均≤3cm/0.03rad，修正一致性≤5cm/0.03rad；任一支持窗口质量不足即不能用该组静止证据。原运动确认路径保留。

通过后仅内部InitialPose带`stationary_verified`，C1仍做自己的多初值几何歧义检查和至少2帧/0.2秒确认，此内部证据只免除运动基线。ROS手动Initialize请求不能设置该标记。取消、切图、前端重置仍清除支持；Tag参与的候选不走此纯LiDAR静止分支。质量新增stationary_strong_geometry/evaluated_candidate_limit/confirmation_mode及global_stationary_initialization。top16并非穷举全图，同视角多帧也非独立视角证明，真实负样本与泛化仍须验收。

静止确认时，首个新强窗口重建支持，旧弱证据不计数。已有强锚点遇到弱窗口时不计数，后续仍须满足与原强锚点的一致性及独立scan ID。阈值、3组/1秒和C1复验不变。新增`searching_geometric_registration`中间遥测，以及`verifying_stationary_support_restarted`、`verifying_wait_strong_stationary_geometry`原因；质量增加query_window_*、support_queries/elapsed_s/anchor_strong、odom/map/correction差和best_rmse_m/stationary_max_rmse_m。这些阶段均不产生有效控制位姿。见报告。
