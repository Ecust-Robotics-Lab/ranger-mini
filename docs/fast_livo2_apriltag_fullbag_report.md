# AprilTag 现有运动包全量提取与时间关联报告

日期：2026-10-29。用户在确认无需重新采集后，授权继续第 1 项：全量读取既有运动包，提取 `tag36h11:1`、0.155 m 的观测并检查与前端位姿的时间关联。本轮只做这项离线工作；没有登记真实 VALID 地标、提交重定位/回环、启动 ROS 图、硬件或采集，也没有 Git 提交/推送。

## 1 结果

**1367/1367 张同步图像全部处理成功，检测到 ID 1 共 14 次，每次保留 2 支通过当前像素门槛的位姿假设，共 28 支。** 原包、配置和前端位姿均只读；11,493,613,568 字节原数据库的处理前后 SHA-256 完全一致：

```text
aed4dedacc5bdc4713b2be5acbfe09bc777fa9e6d67fc49b5565ec162e7b2116
```

| 项目 | 实际结果 | 含义 |
| --- | --- | --- |
| 图像处理 | 1367，检测器退出 0，无漏处理/检测异常 | 全量提取通过 |
| 检测身份 | 仅 tag36h11:1，共 14 次 | 不是人工标注召回率或零误检结论 |
| 位姿假设 | 每次 2 支，共 28 支均满足当前像素门槛 | 保留歧义，未强选最小误差支 |
| 原图复制身份 | 1367/1367 内容 SHA 唯一匹配 | 同步图像对应原图，不按跨历元最近时间猜测 |
| CameraInfo | 1367/1367 唯一关联且模型匹配，1 个模型版本 | 1280×1024、RGB8、plumb_bob、step=3840 |
| 原始在线位姿 | 1365/1367 可关联：1361 插值、4 精确 | 尾部 1365/1366 两帧无后侧样本，拒绝外推 |
| 既有回放位姿 | 1360/1367 可关联：1356 插值、4 精确 | 头部 0–4 和尾部 1365/1366 无包围样本 |
| 14 次 Tag 观测 | 原始/回放两套位姿均可关联 | 两套局部坐标独立保存，没有混合 |
| 同步 CSV | 1367/1367 唯一匹配，v2 sequence 关系全部通过 | 软件协议与数据来源一致，不证明物理曝光同步 |

## 2 检测分布、质量与双解

从首张同步图像 header 起算，检测发生在 **72.900232–74.599927 s**。连续检测段为图像索引 729–731、733–740、744–746（索引从 0 开始），合计 14 帧。整个检测窗口约 1.700 s，属于一次短暂经过，不能当成 14 个独立访问。

窗口内部索引 **732、741、742、743** 没有检测输出。保存了 6 张原始图像用于复核；抽查 732、742 可见标签图案且有模糊。本报告不将模糊断定为唯一原因，也没有为消除间隙而调整 hamming、解码或图像门槛。

14/1367≈1.024% 是检测输出在全包帧中的占比。全包没有逐帧可见性/身份人工真值，这个比值不是召回率。

| 质量统计 | 最小 | 中位数 | P95 | 最大 |
| --- | --- | --- | --- | --- |
| decision margin | 38.951 | 40.610 | 42.307 | 42.317 |
| 每帧 hypothesis 0 原始像素 RMS | 0.0123 px | 0.2228 px | 0.7570 px | 1.3027 px |
| 每帧 hypothesis 1 原始像素 RMS | 2.1018 px | 2.2980 px | 2.3905 px | 2.3990 px |
| 相机到 Tag 平移范数，28 支 | 1.8458 m | 1.8530 m | 1.8640 m | 1.8655 m |

所有检测 hamming=0，14 次都通过现有质量检查；两支均低于当前试验 RMS 3 px 和最大残差 6 px 门槛。`hypothesis_id` 仅是单帧求解器索引，不是跨帧连续分支身份。28 支原始位姿、角点、误差及全部相邻组合均保存，没有用每帧最小误差建立真实地标。

若直接要求 decision_margin>50，这 14 次都会被丢弃；这说明固定经验阈值需要数据标定，并不授权在实车上降低其他安全门槛。0.155 m 由用户提供，相机距离是由该尺寸和当前模型推算的结果，不是独立测距真值。

## 3 时间与来源核查

同步图像 header 仍在 2020 雷达历元，原图和 CameraInfo header 在 2026 主机历元。本轮先对图像像素内容做 SHA 匹配，确认它们是驱动复制的同一帧，再用原图 header 精确关联 CameraInfo；没有拿两个历元的最近时间作配对。

对 `/fast_livo/odometry` 和既有 `native_frames` 的位姿分别建立历史：检查参考体/坐标、有限性、单位四元数与严格递增时间，按 int64 纳秒查找。仅允许精确样本或前后包围的平移插值+SLERP，最大间隔 0.15 s，检查运动上限；不使用 latest、最近帧或外推。绝对历元先作整数差再计算浮点比例，保留 ns 精度。

原始在线轨迹和此前三融合回放轨迹来自不同运行，局部坐标和估计值不能拼接。本轮分别记录关联结果，条件性 `T_raw_local_tag` 只用原始轨迹计算，未与旧 PGO 图或新的重力对齐轨迹混用。

对归档 `sensor_assets/camera_sync.csv` 与 `timeshare_writer.csv`，所有同步图像的原始 stamp 都唯一匹配；`camera.ts_seq=2×writer.seq` 的 v2 关系 1367/1367 通过。

| 软件时序统计 | 中位数 | P95 | 最大 |
| --- | --- | --- | --- |
| 相机帧到达 − 对应 writer 到达 | 22.035 ms | 23.347 ms | 23.978 ms |
| 同步图发布 − 相机帧到达 | 4.225 ms | 8.555 ms | 14.717 ms |
| rosbag 记录时间 − 对应原图主机 header | 23.649 ms | 328.082 ms | 439.935 ms |

这些是回调/发布/记录的软件时间，不是触发沿或曝光中点测量。不能据约 22 ms 直接写入时偏，也不能把 bag 记录延迟当采样时刻。本轮没有硬编码 ±100 ms 或其他补偿，`time_verified=false` 保持。

14 次观测的条件性局部 Tag 变换和 13 次相邻观测的全部假设组合已保存。组合中最小平移变化的中位数约 0.0110 m，最小旋转变化中位数约 0.00881 rad；这里分别取组合最小值，仅用于内部一致性描述，没有选择分支，也不证明固定性、外参精度或物理同步。

## 4 实施、命令与证据

基线 HEAD 仍为 `acffa879cc8d52330276089cd3df83ad6a43f112`。保留了上轮所有未提交修改，本轮操作前备份在 `.runtime/backups/apriltag_fullbag_20261029_194129/`，位置也记录于证据 `baseline.json`。

最小源码修改：为现有 `tag_image_tool` 增加有界流式输入模式，复用同一个官方 Detector；没有增加第二套检测器或修改 B/C1/C2。每帧使用受限 JSON 头+原始图像字节，复用一个检测进程，逐帧确认响应，避免保存全部大图或队列丢帧。

新增脚本：`scripts/fast_livo2/tags/extract_bag.py` 和 `analyze_fullbag.py`，均默认 dry-run。源数据库使用 SQLite `mode=ro` 与 `PRAGMA query_only=ON`，没有 ROS init/播放或传感器节点。

实际命令如下，均退出 **0**：

```bash
source /opt/ros/humble/setup.bash
source ws_fast_livo/backend_install/setup.bash
export LD_LIBRARY_PATH="$PWD/ws_fast_livo/apriltag_deps/lib:$PWD/ws_fast_livo/backend_deps/lib:${LD_LIBRARY_PATH:-}"

cmake --build ws_fast_livo/backend_build/ros/ranger_fast_livo_backend --target tag_image_tool -j2
cmake --install ws_fast_livo/backend_build/ros/ranger_fast_livo_backend
PYTHONDONTWRITEBYTECODE=1 python3 tests/test_fast_livo_tag_fullbag.py

PYTHONDONTWRITEBYTECODE=1 ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=208 /usr/bin/time -v \
  python3 -u scripts/fast_livo2/tags/extract_bag.py --execute \
  --output outputs/fast-livo-apriltag-fullbag-20261029

python3 scripts/fast_livo2/tags/analyze_fullbag.py --execute \
  --extraction outputs/fast-livo-apriltag-fullbag-20261029 \
  --output outputs/fast-livo-apriltag-fullbag-analysis-20261029
```

4 项专项测试覆盖：流式多帧及旧单图接口完全一致、坏 stride 后继续处理、截断/超大输入拒绝、大历元纳秒/非零平移/SLERP、缺包围/大间隔/重复回退/过大运动拒绝。上轮 123 项保留为上轮验收；本轮没有将其冒充全部重跑。

证据根：`docs/fast_livo2_evidence/apriltag_fullbag_20261029/`。保留编译/安装、专项测试、提取和分析日志、输入前 SHA、输出摘要与校验；完整派生数据在上述两个 outputs 新目录。原包 SHA 前后完全一致，生成数据不覆盖原包、旧地图或标定。

提取阶段从开始逐帧处理到末尾校验约 70.61 s；整条命令含前置读取/哈希约 80.14 s，最大子进程 RSS 129564 KiB。Detector 内部处理时延 P50/P95/最大为 **40.68/45.32/58.47 ms**，不含全部 I/O 与颜色转换；这是离线检测资源观测，不是三融合+后端+导航的实时性认证。

## 5 输出入口

- [完整提取摘要](../outputs/fast-livo-apriltag-fullbag-20261029/summary.json)：输入/工具身份、计数、关联、完整性及限制。
- [逐帧记录](../outputs/fast-livo-apriltag-fullbag-20261029/frames.jsonl)：全部 1367 帧、图像/模型来源、时间关联及检测输出。
- [观测与双解](../outputs/fast-livo-apriltag-fullbag-20261029/observations.jsonl)：14 次观测的角点、两支位姿、质量和条件性局部变换。
- [相邻假设组合](../outputs/fast-livo-apriltag-fullbag-20261029/hypothesis_transitions.jsonl)：保留全部分支组合，不预选分支。
- [图表](../outputs/fast-livo-apriltag-fullbag-analysis-20261029/overview.png)、[分析摘要](../outputs/fast-livo-apriltag-fullbag-analysis-20261029/analysis.json)、[CSV 质量表](../outputs/fast-livo-apriltag-fullbag-analysis-20261029/hypothesis_quality.csv)。
- [专项测试日志](fast_livo2_evidence/apriltag_fullbag_20261029/test_fullbag.log)。

## 6 验收与后续

**PASS**：全量处理与输入 SHA 不变；1367 帧内容/模型/同步日志来源核对；14 次真实检测及双解保留；原戳下可用的位姿关联；4 项专项检查；新输出与原资产隔离。

**PARTIAL / 未验证**：未检测帧的全面可见性标注和召回/误检率；曝光物理同步与实际偏移；标签固定部署；真实地图登记；独立访问重定位/回环；绝对精度、实时/长时验证。

**未实施**：生成真实 VALID Tag 地图、运行 C2/B 接受这些真实观测、修改生产开关、设备采集/运动、提交/推送。

第 1 项已完成。下一步若进行真实地标登记，应以本轮观测和来源记录为输入，先核实固定安装及时间证据；不能因为 14 次检测或内部一致性较好就自动启用真实定位。

后续更新 2026-10-29：用户确认采集期间 Tag 固定未移动，[旧图补录评估](fast_livo2_apriltag_landmark_assessment_report.md)已完成；跨帧一致性和时间问题仍不足以支持 VALID 登记。本文上方“固定部署未验证”是全包提取结束时快照。
