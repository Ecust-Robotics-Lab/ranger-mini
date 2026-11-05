# AprilTag 观测、地标与候选接口

更新：2026-10-29。实现属于既有 B/C1/C2 的可选离线增强；计划见 [实施计划](FAST_LIVO2_APRILTAG_PLAN.md)，实际验收见 [本轮报告](fast_livo2_apriltag_report.md)。生产仍为 Lightning/wheel，所有 Tag 功能默认关闭，观察器不启动相机或任何数据源。

## 1 代码与构建

- `ranger_fast_livo_backend/include/.../tags.hpp`、`src/tag_geometry.cpp`：像素模型、IPPE 双解、时间缓存、观测契约。
- `src/tag_store.cpp`：地标扩展包、稳健聚合、C2/回环初值和回投复核。
- `tag_detector.hpp/.cpp`：唯一的官方 libapriltag 检测路径；`tag_observer_node` 为 ROS 薄节点，`tag_image_tool` 用于离线图像。
- `ranger_fast_livo_localizer/src/tag_map_tool.cpp`：固定 PGO 图导出、C1 旧图补录和禁用修订。
- 原 `Relocalizer::process` 增加可选 `tags::Prior`，原 `Candidate` 增加可选总初值、证据和结果复核器。未提供时沿用原接口行为。

```bash
bash scripts/build_fast_livo2_tags.sh              # dry-run
bash scripts/build_fast_livo2_tags.sh --execute    # 仅软件构建
```

依赖锁为 `config/fast_livo2/apriltag_sources.lock.json`。官方 apriltag v3.4.5 在项目内构建，BSD-2-Clause 原文保留；OpenCV 使用本机 4.5.4，不升级系统库。`RANGER_ENABLE_APRILTAG` 控制检测器目标，默认 OFF。B/C1/C2 核心消费观测但不链接 libapriltag；开启检测器不等于开启候选功能。C2 原可选 KISS/PMC 的 GPLv3+ 依赖保持原许可边界。

公共 build/install 目录会随构建更新，已安装观察器可能留存；不能以文件存在判断当前构建开关，检查 CMakeCache、链接和实际启动参数。

## 2 参数与默认行为

| 入口 | 参数 | 默认/职责 |
| --- | --- | --- |
| B | `enable_tag_loop`、`tag_config_path` | false；显式开启才产生 Tag 回环候选，要求 sim time |
| C1/C2 | `enable_tag_relocalization` | false；只允许在显式 C2 模式启用 |
| C1/C2 | `enable_tag_monitor` | false；只输出投影残差，不改变位姿 |
| C1/C2 | `landmark_recording_mode`、`tag_record_path` | off；c1 模式记录连续可信段，输出必须是新文件 |
| C1/C2 | `tag_landmarks_path` | 空；加载失败只禁用 Tag 分支 |
| 观察器 | `tag_config_path`、`family`、`tag_sizes_json` | 静态显式模型；tag36h11；JSON 字符串尺寸表，单位 m |
| 观察器 | `image_topic`、`threads`、`processing_max_age` | /front/image_sync；1（允许 1–4）；0.5 s |
| 观察器 | `record_path` | 空；可选原始观测 JSONL，新文件，64 MiB 后停止记录 |

受管观察器入口默认读取 `config/fast_livo2/apriltag_sizes.json`。它保留旧包确认的 **tag36h11、ID 1、0.155 m**；此文件只登记尺寸来源，不登记地图位姿或宣称固定性。切换其他 family 时默认不沿用这个尺寸表。

用户本次新采集使用 **tag36h11、ID 01（整数 1）、Black-border width 140 mm = 0.140 m**，另存 `config/fast_livo2/apriltag_sizes_capture_140mm.json`。新包提取时显式传 `--sizes config/fast_livo2/apriltag_sizes_capture_140mm.json`；若用观察器，则显式传 `--sizes-json '{"1":0.140}'`。旧默认文件不改，避免重新处理旧包时悄悄换尺寸。新打印标签不继承旧物理标签的地图位姿或固定性证明，同域部署不得同时复用该 ID 的不同物理实例。尺寸应对应黑色外边界四角之间的边长，不含外侧白边或整张纸；新表记录用户声明，尚无独立尺量证据。

本机模型为 `config/fast_livo2/apriltag_camera.json`，由正式 CameraInfo 和实际前端外参推导 `T_imu_camera`，保留来源 SHA。它默认 `time_verified=false`：复制 timeshare 扫描戳还不能证明曝光相位。该状态可以检测并求相机坐标下的位姿，但**时间适配器只记录、不产生可提交地图候选**。真实时间来源应在独立新配置中说明证据，不能为了启用而直接改成 true。

`fixed_tags` 是 B 回环使用的显式固定部署声明，例如：

```json
{"fixed_tags":[{"family":"tag36h11","id":1,"fixed":true,"source":"实际固定安装记录及验证引用"}]}
```

空表仍可记录建图观测，但不从未知物理标记提交 Tag 回环。固定声明不代替点云几何验证。C2 只使用扩展包内 VALID 的地标；未知 ID 不生成候选。

初始试验阈值为 pixel RMS 3 px、最大残差 6 px、margin≥0、面积≥16 px²、hamming=0；并非现场标定结果。历史 5 s/256 位姿、最大插值间隔 0.15 s、观测年龄 0.3 s、steady 等待 0.5 s、速度 10 m/s/角速度 6 rad/s 上限。地标模式聚类 0.15 m/0.15 rad，独立视角基线 0.15 m 或 0.08 rad，至少 3 个独立支持。均可通过配置覆盖并记录；不修改原 LiDAR 的门槛。

## 3 私有传输与元数据

复用 `std_msgs/msg/String` 传递严格版本化 JSON，避免增加第二套检测消息包。主题均为私有数据，不发布 `/tf`、`/tf_static`、定位公共结果或速度。

- `/fast_livo/tags/context`：当前启用 Tag 的 B 或 C1/C2 节点发布，transient-local depth 1。包含 schema=1、context token、calibration_id、time_mapping_id；C1/C2 还包含 map UUID。建图/定位运行模式互斥。
- `/fast_livo/tags/observations`：观察器 best-effort depth 4。顶层为 schema=1、context、observations、processing_ms。消费者每条最多 256 KiB、最多 8 个输入批次；每批最多 32 个观测。

观察器回显消费节点给出的 context。B 使用当前后端 session；C1/C2 使用 session+任务 generation，切图、取消/重新初始化、时钟回退或前端来源变化会更新 token。未关联消费节点时使用 passive token，只可用于检测记录。消费者不把收到时间当采样时间，不重新绑定旧观测。

每个 observation 至少包含：

```json
{
  "schema_version":1,
  "observation_id":"唯一帧和检测身份",
  "image_id":"同图去重身份",
  "source_id":"official_apriltag_3.4.5",
  "context":"消费者提供的会话代际",
  "family":"tag36h11", "id":1, "size_m":0.155,
  "canonical_frame":"apriltag-native-xy-down-z-in-v1",
  "image_stamp_ns":1590195801300123320,
  "observation_stamp_ns":1590195801300123320,
  "calibration_id":"绑定模型和外参的身份",
  "time_mapping_id":"显式时间映射身份",
  "hamming":0, "decision_margin":41.98,
  "corners":[[0,0],[1,0],[1,1],[0,1]],
  "hypotheses":[],
  "provenance":{}, "quality":{}
}
```

上例角点为结构示意，不是有效输入。实际 `provenance` 必须有 size_source、image_reference、完整 camera_model，观察器另写 time_source/time_verified/offset/uncertainty/application_count。quality 包括面积、最短边、距边界和姿态状态；没有使用未赋值的 goodness。每支 hypothesis 含 hypothesis_id、完整 `T_camera_tag`、重新计算的像素 RMS/最大误差。缺尺寸时 size=0、hypotheses=[]，只记录 ID。

解析器检查字段、有限性、模型身份、角点凸性/图内范围、尺寸和整数 ns、偏移只应用一次、各支正深度与重投影。相同图像中同 family+id 多实例全部拒绝，不静默合并。跨批次同 image+family+id 去重。

## 4 像素、坐标与时间

统一 `T_A_B` 将 B 点变到 A。原始畸变图使用 K/D；rectified 使用有效 K、零 D，并保存 rectification R；normalized 使用单位 K/零 D，同时必须提供 `pixel_error_K`，把射线残差换算到明确的虚拟校正像素域。该残差不声称是原始畸变像素误差。当前 ROI/binning 非默认或没有显式有效模型时拒绝，不猜测内参调整。实际静态 CameraInfo 模型在启动时绑定，换标定必须换配置身份并重开会话。

IPPE 物点顺序：`(-s/2,+s/2,0),(+s/2,+s/2,0),(+s/2,-s/2,0),(-s/2,-s/2,0)`。官方 v3.4.5 的解码角点按同一语义直接对应；0/90/180/270 度官方图案渲染测试验证了索引，不按图像左上重排，不添加肉眼判断的翻转。PnP 返回 0–2 个有效且去重的总位姿；所有点正深度，误差统一回到对应像素域。

令 `X(t)=T_O_I(t)`、`E=T_I_C`、M 为绑定地图、F 为 Tag：

```text
T_C_I = T_C_L * inverse(T_I_L)
T_M_I(tc) = T_M_F * inverse(T_C_F(tc)) * inverse(E)
A_s = inverse(X(ts)) * X(tc) * E
Z_k = inverse(X(tk)) * X(tc) * E * T_C_F(tc)
T_i_j_init = Z_i * inverse(Z_j)
T_M_I(ts)_init = T_M_F * inverse(T_C_F(tc)) * inverse(A_s)
T_C_F_pred = inverse(T_M_I(ts)_reg * A_s) * T_M_F
```

当前 C2 查询云已折算到最新 I(ts)，目标为 map 子图；注册输出是总 `T_M_I(ts)`。B 查询云在 I(j)，目标在历史 I(i)，输出为总 `T_i_j`。不重复乘初值或 LiDAR–IMU 外参；不同表达系的一般公式见实施计划。

局部位姿仅取同会话连续前端轨迹。精确样本或前后包围插值（平移+SLERP），检查间隔与运动上限；不外推、不取 latest TF、不插值跳变后的 map 轨迹。图像时间晚于当前扫描时有界等待，超龄/超时/缺包围样本则拒绝。所有原始时间保留 int64 ns，offset 符号为 `effective=header+offset`。

本机图像约 10 Hz，不能用 IMU 200 Hz 推断有可靠的 200 Hz 位姿。旧包 image_sync 为 2020 历元，原图/CameraInfo 为 2026；后者不能直接与前者严格同戳配对。相机 20000 μs 是曝光配置，不是曝光中点时间测量。

## 5 地标扩展包

不修改 B schema 1，不向旧图追加未列出的文件。独立新目录由三项组成：

```text
landmarks.json   地标与来源、独立支持、观测/标定/时间引用
manifest.json    schema=1、kind=apriltag_landmark_extension、源图身份、文件 SHA
COMMITTED        manifest_sha256
```

源绑定必须同时匹配 map_uuid、source_manifest_sha256、graph_version，地图坐标为 map，单位 m。revision 为扩展 manifest SHA。地标包括 family/id、尺寸定义、canonical frame、完整 T_map_tag、VALID/PROVISIONAL/AMBIGUOUS/SUSPECT/DISABLED、原因及支持统计。没有未知协方差零矩阵；离散度只表示观测一致性。

先区分空间/姿态模式，再以独立视角支持判断固定性。重复图像/静止连拍不能增加支持，稳健估计只使用独立视角代表，双解两票不算独立观测。两处持续模式不平均；不一致观测使地标 SUSPECT。换相机可复用固定物理地标，旧观测仍按旧标定解释；加载不强制历史标定 hash 与新相机相等。

导出要求新的绝对目录，临时写入、文件 SHA、manifest/COMMITTED、fsync、rename-no-replace；失败不覆盖目标。文件解析/字节/数量有界。无扩展的旧图保持纯 LiDAR；坏扩展只禁用 Tag，原图完整性检查不放宽。

### PGO 建图登记

启用 B 后，候选日志的 `tag_attachment` 保存原始 observation、keyframe/scan/stamp 与 `T_keyframe_camera`。冻结图快照后，对每条记录使用其自己的 `X_map_imu(k)`，而不是最新单一全局校正。B 导出的 candidates.jsonl 在 manifest 内校验；读取其他日志时另记录其 SHA。

### C1 给旧图补录

启用 `landmark_recording_mode=c1` 后，仅正常 map_match_confirmed 且连续三次修正一致的段生成记录；包含局部历史、原戳、地图/session/generation 和 segment_support。工具再校验段内时间/会话/修正、末尾位姿与修正一致、图像时刻包围插值，然后计算：

```text
T_M_F = T_M_I(ts)_C1 * inverse(X(ts)) * X(tc) * E * T_C_F(tc)
```

工具为 `tag_map_tool --execute JOB.json`，没有 --execute 时仅说明操作。JOB 必填：

```json
{
  "archive_path":"/absolute/source/snapshot",
  "compatibility_path":"/absolute/bound_offline_compatibility.json",
  "config_path":"/absolute/verified_camera_time_config.json",
  "method":"c1",
  "records_path":"/absolute/new_confirmed_observations.jsonl",
  "destination":"/absolute/new_landmark_revision"
}
```

PGO 路径 method=pgo，records_path 指向 Tag attachment 日志。禁用时 method=disable，另提供 landmarks_path、disable（如 `["tag36h11:1"]`）、reason，destination 仍为新目录。补录/重算产生新扩展版本；运行中的节点不热改表，重开离线会话加载新版本。原归档兼容声明仍按 C1 契约显式提供。

## 6 候选仲裁与回退

C2 先保证原 LiDAR 候选获得预算，再尝试最多 8 个 Tag 初值。各支通过原绝对几何门槛及所有参与的已知 Tag 回投；收敛到同一物理解视为一解，空间分离且都通过则拒绝歧义。没有完整评估所有 Tag 分支时，本次不提交幸存者；随后 Tag backoff 至少 1 s，给原 LiDAR 单独尝试机会。

确认沿用原 scan 集去重、时间与运动基线，增加同一 Tag image 不得重复作支持。成功仍是 `SearchResult.confirmed`→原 `process_search`→同一 C1 初始化→C1 几何确认。Tag 支持的 handoff 在首次 C1 VALID 时再回投验证；正常跟踪监视只输出残差，不直接重置。

B 只有显式固定身份可用于回环；同一身份两边双解最多四组合，目标排除当前扫描，保留原时间/ID 排除、大修正独立支持和重复边检查。最终仅通过原 `Graph::add_loops` 添加 LiDAR 因子。Tag 任务未完整评估时不提交幸存者，并给后续关键帧保留纯 LiDAR 机会。图任务仍按依赖位姿检查，不因无关新增关键帧自动失效。

公共桥接始终独立。私有候选通过、C2 handoff、C1 VALID、公共桥接接受分别观察；Tag 恢复不能清除创新锁或改变 wheel 同刻约束。相关离线联调用合成 wheel，不是实车验收。

## 7 启用与关闭

不启动 ROS 图的全包只读提取入口为 `scripts/fast_livo2/tags/extract_bag.py`，默认 dry-run；分析入口为 `analyze_fullbag.py`，复用现有 Detector 的有界流式单图工具。实际命令与输出见 [全包报告](fast_livo2_apriltag_fullbag_report.md)。这两个入口不会加载/提交真实地图候选。

以下先查看 dry-run，只有离线软件会话可在当前授权下添加 --execute。所有进程使用同一个明确的隔离 domain 和 localhost；输入由已存在的离线数据/测试发布，不由这些入口启动传感器。

```bash
bash scripts/run_fast_livo2_tags.sh --domain 189
bash scripts/run_fast_livo2_relocalizer.sh --domain 189 \
  --archive /absolute/snapshot --compatibility /absolute/compatibility.json \
  --tag-config /absolute/verified_camera_time_config.json \
  --tag-landmarks /absolute/landmark_extension --tag-relocalization --tag-monitor
bash scripts/run_fast_livo2_localizer.sh --domain 189 \
  --archive /absolute/snapshot --compatibility /absolute/compatibility.json \
  --tag-config /absolute/verified_camera_time_config.json --tag-record /absolute/new_records.jsonl
bash scripts/run_fast_livo2_backend.sh --domain 189 --tag-loop \
  --tag-config /absolute/verified_camera_time_and_fixed_deployment.json
```

B 与 C1/C2 是互斥入口。以上路径是参数占位，不是现有真实 VALID 地标。默认本机 time_verified=false、fixed_tags 空，不能据此执行真实定位。

关闭观察器用 `bash scripts/run_fast_livo2_tags.sh --stop`，对应 --status 查看 owned 清理。关闭候选则停止相应受管离线会话，按原参数重开、不带 Tag 开关；不改生产 runtime 或旧地图。观察器退出、断图和缺表时原 LiDAR 路径仍工作。C1 记录写盘失败只停用记录，不撤销正常 LiDAR 跟踪。

```bash
source /opt/ros/humble/setup.bash
source ws_fast_livo/backend_install/setup.bash
source ws_nav/install/setup.bash
python3 scripts/fast_livo2/tags/run_checks.py --execute
```

真实动态性能仍需：有时间证据的独立建图/定位访问、固定部署记录、未知/重复/移动标签与地图外负样本、遮挡/动态场景、独立真值或明确参考。当前真实包尺寸可用于检测/PnP检查，不填补这些验收缺口。
