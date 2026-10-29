# 新上下文接手指南

更新：2026-10-29。项目 `/home/lab307/ranger_nav`。**总计划未全部完成。** A/B/C1/C2 的本机实现、合成和单份运动包功能检查已有证据；C1/C2 跨会话、C2-4 完整验收、物理精度与 D 实车验收仍未完成。

用户最新指令：后续工作交由新上下文 Agent，本上下文只更新交接，不再继续实施。新 Agent 先阅读、核对实际工作树，再按用户的新指令确定任务；不要从历史计划或旧授权恢复自动实现、采集、重启设备、提交或推送。

## 1. 阅读顺序与事实来源

1. [AGENTS](../AGENTS.md)：有效约束和授权；[STATUS](STATUS.md)：当前事实和未验证能力。
2. [总计划](FAST_LIVO2_BACKEND_PLAN.md)：完整架构、C1/C2/D 步骤与验收条件；计划不是硬件许可。
3. [C2 报告](fast_livo2_stage_c2_report.md)、[C2 契约](fast_livo2_relocalizer_interfaces.md)、[重力对齐报告](fast_livo2_gravity_alignment_report.md)：本轮最新实现、源码/许可、命令、测试与限制。
4. [C1 报告](fast_livo2_stage_c1_report.md)、[C1 契约](fast_livo2_localizer_interfaces.md)、[B 运动报告](fast_livo2_stage_b_motion_report.md)、[B 首次报告](fast_livo2_stage_b_report.md)：复用基础，避免重复实现。
5. [前端契约](fast_livo2_interfaces.md)、[B 契约](fast_livo2_backend_interfaces.md)、[公共定位契约](localization_backend_contract.md)。
6. [工程目录](PROJECT_LAYOUT.md)、[根 README](../README.md)、[受管会话](managed_sessions.md)；涉及采集/标定时读 [录包指南](fast_livo2_recording.md)、[正式标定入口](../calibration/README.md)。

当前事实以实际代码、工作树和带日期证据为准。历史报告的“未提交”“下一步”“尚未安装”是当时快照，不是待恢复的当前指令。

## 2. Git 提交与本机资产：先保护，再操作

分支 `refactor/pluggable-navigation`；本轮提交前基线为 `d5a38311bd2ee6fab5127a8bd7593b34eefc3667`。用户随后明确要求提交净化后的状态，按下表追加重力、C2和文档三组本地提交；实际最新HEAD以 `git log -1` 为准。相对本地upstream `46c2efad12c6cd065739a747f6edfa6a06ee52c1` 共领先8个提交，本次未推送或联网刷新远端。

| 本地已有提交 | 内容 |
| --- | --- |
| `0758ac7` | 用户雷达文档修正与六份 driver 证据，独立保留 |
| `0ced46c` | 相机/同步、正式标定整合、受管采集与既有证据 |
| `a13fee4` | B 补充构建/故障证据 |
| `1212b43` | C1 离线定位源码、测试与证据 |
| `d5a3831` | C1 提交收尾、文档与 C2 开工状态 |
| `4b49bb0` | 初始重力对齐、离线诊断和证据 |
| `2e0f06f` | C2源码、可选依赖/许可、测试与离线证据 |
| 本次文档提交（HEAD实查） | 过时信息清理、完整交接及提交状态同步 |

此前 B 的 `9e7c97e`、`b299078`、`46c2efa` 已按用户指令推送。原5个提交及本轮3个提交均只在本地，**C2、重力对齐、清理文档和可跟踪文本证据已提交；原包、点云/CDR、大输出、SDK、第三方树与备份仍为本机资产。** 不因Git提交完成就删除被忽略文件，也不把远端当作最新本地副本。接手仍检查 `git status --short --untracked-files=all`。

本轮已提交成果主要分组：

- 重力：`config/fast_livo2/ranger_mid360s_hik_trial.yaml`、`scripts/fast_livo2/replay.cpp`、重力报告/证据与当前文档。
- C2：既有 `ws_fast_livo/src/ranger_fast_livo_localizer/` 内新增检索、索引、粗配准、重定位、工具、Relocalize 服务、测试、许可；并修改 C1 node/CMake 和极少量核心接口。不是另一个跟踪后端。
- C2 入口：`scripts/build_fast_livo2_relocalizer.sh`、`scripts/run_fast_livo2_relocalizer.sh`、`scripts/fast_livo2/relocalization/build_kiss.py`、`config/fast_livo2/relocalizer.yaml`、`relocalization_sources.lock.json`、ROS 测试与 C1 受管/回放脚本扩展。
- C2/重力报告和 `docs/fast_livo2_evidence/stage_c2_20261028/`、`gravity_alignment_20261028/`；目录沿用10月28日开工日期，部分最终检查在10月29日完成。

禁止 reset/clean、覆盖或批量删除未跟踪目录；本次已获本地提交许可并完成，未获推送许可。原始包、地图、标定、SDK、失败证据、备份和用户 driver 证据全部保留。忽略的大文件仍在本机，不能从 Git 缺失推断可删或不存在。

本次交接前备份在 `.runtime/backups/handoff_context_20261029_121955/`：四份交接文档原版、完整 tracked diff、Git 状态/提交、提交前316份未跟踪文件 SHA 清单和会话快照（可跟踪源码/证据现已纳入本轮提交）。它不是所有本机大文件的内容备份；同机新上下文应直接保留现有工程。提交前快照另见 `.runtime/backups/pre_commit_clean_20261029_123335/`。

## 3. 实际实现与验收边界

| 阶段 | 已有事实 | 仍不能报告通过的事项 |
| --- | --- | --- |
| A 前端 | 固定 ROS2 源码/补丁、本机构建、静态829对、真实三融合预览；预览已停止 | 曝光/扫描物理同步、外参真值、全场景运动精度 |
| B 建图后端 | 严格配对、关键帧、PCL/GICP/VGICP、Pose3 iSAM2、传播、只读归档/导出、隔离桥接；同包PCL/GICP各1362对、49帧、13条回环约束 | 多环境误环/重复结构、独立真值、实时吞吐与长时稳定 |
| C1 定位 | 严格加载B地图、显式来源兼容、人工初值、六自由度确认/跟踪、失效/切图/请求代际、适配与受管入口 | 跨会话泛化及实车/精度验收 |
| C2 重定位 | 检索/索引、可选KISS、独立运动支持、向同一个C1交接、取消/恢复；最终C++30/30（C1 18+C2 12）、C2 ROS4/4、C1 ROS9/9 | **C2-4完整验收仍PARTIAL**：独立实采、真实相似地点/地图外负样本、动态干扰、真实遮挡和真值 |
| D 系统/实车 | 未实施本轮真机接车和导航 | 实际T_base_imu、真实wheel历元/同刻桥接、导航安全链、持续运行 |

C1 历史同包功能：PCL/GICP各1362对，第366对给初值，994/988有效，GICP另有6次不收敛拒绝，无LOST/ERROR。C2 最终同包功能：无人工初值，1362对全部处理，**979有效**，首次第384对，无LOST/ERROR/公共TF或速度，峰值RSS165840 KiB。输入确认节流不是实时性基准；同包自洽不是跨会话或真值验收。

最新证据入口：

- `docs/fast_livo2_evidence/stage_c2_20261028/verification_summary.json`、`acceptance_final_results.json`、`cpp_accepted.xml`、`regression_results.json`。
- `motion_final_result.json`、`motion_layer_summary.json`、`asset_postcheck.json`、`reference_audit_final.json`、`final_sessions.json`。
- 重力：`docs/fast_livo2_evidence/gravity_alignment_20261028/verification.json`。
- 早期 C2 中间版本980有效的输出保留；**最终以979有效版本为准**。逐输入快照不是完整求解事件日志，不能把候选/粗配准通过数当定位成功次数。

## 4. 代码、构建与运行的具体位置

- 前端：`vendor/fast_livo2_review/ros2_frontend`，humble固定 `837b7bbc1431cb04cf936528e52c83c835efba8e` 加 `patches/fast_livo2_stage_a.patch`；锁为 `config/fast_livo2/sources.lock.json`。参考过的项目不都已集成。
- B：`ws_fast_livo/src/ranger_fast_livo_backend`；依赖 `backend_deps`，构建 `backend_build`，安装 `backend_install`（均在 ws_fast_livo 下）。
- C1/C2：`ws_fast_livo/src/ranger_fast_livo_localizer`，复用导出的 B 核心与同一个 C1 跟踪器；安装到上述 backend_install。
- C2依赖：`ws_fast_livo/relocalization_deps`、`relocalization_build`；锁为 `config/fast_livo2/relocalization_sources.lock.json`，构建只用固定本地树和生成的 ROBIN 投影，不升级系统库。
- **当前公共构建缓存 `backend_build/ros/ranger_fast_livo_localizer` 为 `RANGER_ENABLE_KISS=ON`；C1默认构建脚本显式设OFF，C2脚本设ON，都会更新同一安装目录。** 独立 `ws_fast_livo/c1_default_build` 是关闭KISS的最小构建证据，不是当前安装入口。启用C2前核对实际二进制，不因目录存在认定插件已编入。
- `config/fast_livo2/localizer.yaml` 默认关闭全局搜索；显式 C2 overlay `relocalizer.yaml` 才开启。私有 `/fast_livo/localizer/relocalize` 可启用/取消搜索；人工 Initialize 会暂停自动搜索，需显式重新允许。默认 C1 LOST 后仍需人工初值。
- KISS自身根MIT，但传递依赖 **PMC为GPLv3+**，可选链接二进制不能宣称纯MIT；保留锁、THIRD_PARTY_NOTICES与third_party许可。SC-A-LOAM核心授权不明，没有复制；当前检索为自有三维距离分布原型，合成对照不是官方检测器排名。
- 生产默认 **Lightning建图/定位、wheel、terrain disabled**；保留Nav2/Smac/MPPI与安全链。C1/C2独立离线入口没有接管production runtime；FAST前端/B不能配置成先验地图定位器。
- 正式标定在 `calibration/`，`~/calibration`只是兼容链接；相机触发/同步包在 `ws_camera/src/`，原本地Hik SDK与用户修改保留，不随本轮Git分发。

## 5. 重力、时间、参考体与TF

新前端试验配置已设 `uav.gravity_align_en=true`，初始化后将估计重力旋转到世界−Z，+Z向上；完整三融合回放1362对、7943次视觉更新，对齐执行一次，初始重力约 `[0,0,-9.81]`。终态偏轴0.06436°是估计内部一致性，不是外部竖直精度。下一次新前端启动使用此配置，本次没有重启设备。

旧B/C1地图和旧native_frames保留原世界系，**不能追认重力对齐，也没有重建新的B地图**。新对齐回放另存下表。诊断harness在隔离源码投影构建中验证，原源码Log目录全部SHA保留；普通已安装前端本来就支持对齐开关，但若需要新增summary诊断字段须按构建脚本重建对应harness，不能把隔离投影误当常规部署。

输入 `/fast_livo/odometry` 是VIO后IMU位姿，`/fast_livo/deskewed_imu`是严格同戳、单区间IMU云。外参只用一次，不用累计世界云。统一 `T_A_B` 将B点变到A，完整三维，不锁roll/pitch；PGO/定位不反馈前端滤波或视觉地图。

`T_map_base=T_map_imu*inverse(T_base_imu)`；唯一公共桥接按原采样时刻插值wheel后求 `T_map_odom=T_map_base*inverse(T_odom_base)`。禁止外推、latest-TF或两个最新到达值相减。旧sensor历元为2020，host/wheel为2026，不能直接拼接或重打now；离线用header-clock视图和sim_time。心跳不刷新位姿采样时间。私有节点不发公共TF；公共创新锁定不会因C2成功自动解除。

## 6. 本机数据与证据资产

| 资产 | 路径 |
| --- | --- |
| 原始三传感器运动包 | `bags/fast-livo-motion-w5phdosj/raw`，约136.6秒；末尾发布者变化错误有记录，不称完整180秒 |
| 旧前端派生输入 | `outputs/fast-livo-demo-20261028-fHJTbGCU/frontend/native_frames`，1362对，DB SHA见C1/C2报告 |
| B地图 | `outputs/fast-livo-backend-validation-20261028-{pcl,gicp}/snapshot`，两图最终各151文件SHA通过 |
| C1最终输出/来源兼容 | `outputs/fast-livo-c1-final-20261028-{pcl,gicp}/`；`docs/fast_livo2_evidence/stage_c1_20261028/{pcl,gicp}_compatibility.json` |
| C2最终输出 | `outputs/fast-livo-c2-final-20261028-pcl/`，result.json、samples.jsonl和node日志 |
| C2派生索引 | `outputs/fast-livo-c2-index-20261028/`，绑定PCL源图49帧；不适用于另一图或不同描述子参数 |
| C2合成选择实验 | `outputs/fast-livo-c2-selection-20261028/`，固定种子/标签/参数，仅合成结论 |
| 新重力对齐输入 | `outputs/fast-livo-gravity-20261028/aligned/native_frames`，另有关闭对照、summary和独立build/source/Log；旧图不改写 |
| 展示视频 | 旧demo下1080p/720p视频，B的PCL输出下 `comparison/comparison_1080p.mp4`；见演示/B报告 |
| 静态基线 | `bags/fast-livo2-baseline-126a5fAo`、`bags/fast-livo2-dds-smoke-final`，不能代替运动/跨会话验收 |

先前明确删除/回收操作不是后续删除许可：第二份180秒 `ez09stbr` 曾按请求移入回收站；四份更早试录曾另行清空回收站，不能再称可恢复。清单在 `.runtime/capture_cleanup/20261028_122508_441848/manifest.json`、`20261027_231026_079043/manifest.json`。失败启动 `58obp75g`、失败/中间C1/C2输出及所有旧备份保留。

## 7. 新Agent首次操作与接手边界

2026-10-29 后续按用户要求清理当前入口的过时说明：删除“C1/C2未实现”“当前重力关闭”“当前无图像阻塞”和旧提交/后续目标描述，合并重复补充段落；旧底盘/Lightning采集清单标明历史用途。未找到可整份删除且没有独立价值的重复文档，历史原文、原始资产与失败证据保留。清理前副本、候选判断与核对清单在 `.runtime/backups/docs_cleanup_20261029_122711/`，未修改算法或运行设备。

本次交接于2026-10-29 12:19（Asia/Shanghai）只读核对：live预览STOPPED；C1/C2受管状态为故障注入测试留下的预期FAILED，均live=false、owned=[]、清理完成。不是待自动重启的线上故障。新上下文必须重新现查，历史状态不作设备占用锁。

以下均为只读或默认dry-run，不加 `--execute`：

```bash
cd /home/lab307/ranger_nav
git status --short --untracked-files=all
git log -7 --oneline
git branch -vv
python3 scripts/runtime_config.py
bash scripts/run_fast_livo2_live.sh --status
bash scripts/run_fast_livo2_localizer.sh --status
bash scripts/run_fast_livo2_relocalizer.sh --status
bash scripts/build_fast_livo2_relocalizer.sh
bash scripts/run_fast_livo2_relocalizer.sh
```

本次是文档交接，不为接手重复长时间回放、重新选型/实现C1/C2或重启设备。后续顺序与缺口看总计划：先补独立会话/真实负样本与物理证据，再分别验收C1/C2与D；原包没有真实wheel，不把合成wheel当接车。采集、设备启动、接管底盘、CAN/速度/导航以及提交/推送按用户后续明确范围处理。
