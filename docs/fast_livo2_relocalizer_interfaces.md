# C2 离线全局重定位接口

2026-11-06增补：显式[在线定位入口](fast_livo2_online_localization.md)已复用本C2确认与C1跟踪链，实现原速输入的有界调度和过期结果抑制，私有服务/消息不变。生产默认及本页旧离线入口保持，实际新包功能结果和未验范围见[首轮报告](fast_livo2_online_localization_report.md)。

更新：2026-10-29。C2 在既有 `ranger_fast_livo_localizer` 内向 C1 提供确认后的初值；C1 继续负责精确初始化和持续定位。默认 C1 配置关闭全局搜索，生产 runtime 继续 Lightning。本阶段没有硬件、前端恢复或导航授权。

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

公共适配仍须显式 `--offline-bridge`，不变更已有 TF 所有权、同刻 wheel 插值、禁止外推和首次创新失败锁定。C2 成功不解除公共锁定，须显式重开公共联调会话。

## 默认资源与限制

默认最多 5 个候选；合并 2 帧，最多 4000 查询点；窗口跨度不超过 1 s/3 m/1 rad。每次尝试墙钟预算 10 s，失败完成后退避 0.5 s。需要 2 个独立支持查询、至少 0.25 s 及 0.15 m 或 0.08 rad 的前端和地图运动基线。参数在 `config/fast_livo2/relocalizer.yaml`。

索引最多 10000 关键帧（可配置且有硬上限），目标子图沿用 C1 的最多 8 帧/100000 点。C1 与 C2 各自有 128 MiB LRU，并非进程总内存上限；另有索引、查询、工作队列、配准器临时内存。求解器内部不能硬中断；每层返回后重新检查预算/代际，超时拒绝整次尝试，不能选取此前最好候选凑成功。

这是受检索数量和几何可辨识性限制的离线实现，不保证所有环境召回或消歧。实际证据与未完成项见 [C2 报告](fast_livo2_stage_c2_report.md)，跨会话、真值精度与实时接车仍 PARTIAL。

2026-11-04 新增一次不同采集包的 [无初值功能试验](fast_livo2_cross_session_global_report.md)：1195对查询、936个VALID采样，无LOST/ERROR；场地覆盖与绝对位置未确认。离线评估工具支持 `--query-provenance` 绑定地图/native DB SHA，独立输入禁止同图推导初值；`--allow-recovery` 只调整试验结果如何容纳一次真实丢失后恢复，不改变节点门槛/锁。新日志额外保存 `T_map_imu` 和请求代际。真实Tag尚无VALID表不阻止纯LiDAR C2测试。

2026-11-06检索增补：新增自有`radial_pair_multiscale_v2`（195维、平滑多尺度距离分布与Hellinger嵌入），`global_descriptor`控制索引绑定与比较；在线入口默认v2，旧v1索引/离线配置兼容。官方STD/Scan Context仅进入独立离线benchmark，不是在线插件。C1/C2入口默认注册器已按用户指令改为small_gicp，KISS全局粗配准保持。对照与限制见[报告](fast_livo2_retrieval_comparison_report.md)。
