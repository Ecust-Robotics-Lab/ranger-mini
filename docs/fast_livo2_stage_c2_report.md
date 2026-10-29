# C2 离线全局重定位交付

> 提交状态更新（2026-10-29）：用户后续明确要求提交净化后的状态，C2源码与证据已本地提交为 `2e0f06f`，重力为 `4b49bb0`，文档清理与交接同批提交。下文“未提交/未获指令”保留交付时事实；最新状态见 [HANDOFF](HANDOFF.md)，未推送。

日期：2026-10-28 至 2026-10-29；证据目录沿用开工日期 `stage_c2_20261028`。本轮在既有成果分组本地提交后执行 C2-0 至 C2-4；另按用户要求开启并验证前端初始重力对齐。没有启动硬件、底盘或导航，没有推送或提交本轮新修改。

## 交付与边界

PASS：离线源码、项目内构建、严格派生索引、无初值候选检索、完整六自由度粗细配准、独立运动支持、向 C1 交接、取消/代际隔离、失效恢复和受管入口。独立会话、真实地图外负样本、物理同步、真值精度、实时接车和长时运行保持 PARTIAL。

实现位于 `ws_fast_livo/src/ranger_fast_livo_localizer`，新增 `PlaceRetriever/PlaceIndex`、`CoarseRegistration` 的可选 KISS 实现、`Relocalizer`、索引工具和私有 Relocalize 服务。C2 与 C1 共用 worker/地图/请求代际，确认后只提供原采样时刻初值；C1 独立再确认并持续跟踪。默认 C1 和生产 runtime 不启用 C2。

地图保持只读；派生索引绑定 UUID、manifest SHA、graph version、关键帧/scan ID、IMU 云系、无重力假设和算法/参数哈希。新目录原子 no-replace 提交，损坏、错图、错参数和超限拒绝。源地图两份各 151 个文件最终 SHA 全部通过，开工基线文件无缺失。

多帧查询变到当前 IMU 系，候选没有原位置 3 m 筛选；粗配准总变换按 `X_i * T_i_query` 进入复用的精配准/独立门控。空间分离且质量接近的候选拒绝。确认须有互不重叠 scan ID、时间基线、前端与地图匹配双方的运动基线及全局修正一致性；重复静止云伴随漂移里程计也不能确认。

每次尝试检查总墙钟预算，不能在后续候选超时后接受已有最好候选。取消、切图、前端身份/时钟变化、重启和人工初值使旧任务失效；C2 不解除公共桥接创新锁定，不增加 TF 或速度发布者。操作、参数和限制见 [C2 契约](fast_livo2_relocalizer_interfaces.md)。

## C2-0：选型与许可

先核对 SC-A-LOAM 固定树，未发现根或核心 Scancontext 文件的明确许可文本，仅 package BSD 元数据；不复制其源码。独立实现了极坐标最大高度数学原型，参数为 20 环、60 扇区、80 m 半径、2 m 高度偏移，用于重力条件实验，不等于官方 SC-A-LOAM 实现的验收。

最终离线检索原型采用自有 `radial_pair_distribution_v1`：径向/点对距离直方图与协方差形状，完整三维旋转和平移不变，分数仅供候选排序。既有地图与查询不能假设共享重力方向；新开启的前端重力对齐也不追认旧地图。选择是当前边界下的最小原型，不声称真实环境检测器排名。

冻结开发种子 47、留出种子 104729，各 10 个合成地点；每组 20 次查询，包含完整云和均匀丢弃 25% 点，标签为生成的源地点 ID。使用实际 C++ 检索实现，未根据留出结果改参数：

| 留出条件 | 自有三维原型 Hit@1 / Hit@5 | 自有极坐标原型 Hit@1 / Hit@5 | 分母 |
| --- | --- | --- | --- |
| yaw 旋转 | 20 / 20 | 14 / 20 | 各20次 |
| yaw、roll、pitch 旋转 | 20 / 20 | 6 / 18 | 各20次 |

这些是有标签合成形状的 Recall 计数，不是独立实采、真实遮挡、动态干扰或官方检测器性能排名。实验源码 `test/selection_experiment.cpp`，生成资产在 `outputs/fast-livo-c2-selection-20261028/`，结果见证据 `selection_result.json`。

KISS 最小 C++17 构建、真实已知六自由度总变换和内点检查通过后，作为显式可选插件集成；没有导入 SAM、CUDA 或另一套 ROS 后端。版本锁在 `config/fast_livo2/relocalization_sources.lock.json`：

| 组件 | 完整 SHA | 许可/角色 |
| --- | --- | --- |
| KISS-Matcher | `8b58b118e00c7a5a22448741f8d956d638a44ae2` | 根 MIT，可选粗配准核心 |
| ROBIN | `5623f02f62c63711052bbb8d6096a410ddca0ee4` | BSD-2-Clause |
| PMC | `4bbd40ababd8e925c4e1845c509173afe766b443` | GPL-3.0-or-later，静态传递依赖 |
| xenium | `1c449ae953ce2a440b0d16c5ed1181d2754860ab` | MIT |
| SC-A-LOAM | `321cb4b07b7815c114237ce5d87014d2e46b972c` | 许可未解决，仅审查 |

原 vendor 树保持 DETACHED、干净、固定 SHA。生成的 ROBIN 投影替换上游 xenium 的 moving-main 下载为上述本地固定树；只安装到 `ws_fast_livo/relocalization_deps`。复用系统 Eigen/TBB/FLANN/LZ4/OpenMP，不升级系统库。KISS 根 MIT 不覆盖 PMC；可选链接二进制不能标成纯 MIT。保留根许可、GPL-3 全文、逐文件通知和包内 THIRD_PARTY_NOTICES；完整组合分发合规仍不是本机测试的通过结论。

## 构建与验收

构建命令 `bash scripts/build_fast_livo2_relocalizer.sh --execute` 退出 0；包含固定依赖、B/C1/C2 包和既有 ws_nav 适配器。随后索引大小边界调整用 `cmake --build ws_fast_livo/backend_build/ros/ranger_fast_livo_localizer -j2`、对应 `cmake --install` 完成，日志在 `build_index_final.log`。C1 关闭 KISS 的独立最小构建也退出 0。入口默认 dry-run。

加载 ROS、backend_install 和 ws_nav 环境、设置 localhost/隔离 domain 后执行：

```bash
ws_fast_livo/backend_build/ros/ranger_fast_livo_localizer/localizer_tests
python3 tests/test_fast_livo_relocalizer_ros.py
python3 tests/test_fast_livo_localizer_ros.py
```

最终三条命令退出码均 0，证据 `acceptance_final_results.json`：C++ **30/30**（C1 18、C2 12）、C2 ROS **4/4**、C1 ROS **9/9**。覆盖已知三轴变换、错误粗配准、合成低对应、退化、多解、正确大修正、不一致假地点、重复/静止/漂移里程计、好候选后耗尽预算、取消、切图、旧请求、失效恢复、受管子进程故障清理。部分故障注入使用可控粗配准替身，真实 KISS 已知变换与 ROS 全链测试另行区分。

B 数值、公共桥接锁定、旧桥接、时间对齐、适配契约、架构和配置回归均退出 0，见 `regression_results.json`。49 关键帧真实 B 归档的索引新建、退出、新进程检查通过，命令与身份在 `index_build.json/index_check.json`。C2 ROS 受管故障测试结束为预期 FAILED，owned=[]、guardian_cleanup_complete=true；没有自动重启。最终所有相关会话 live=false，见 `final_sessions.json`。

## 最终同包功能检查

命令如下；原戳不变、按输入确认节流，不是实时吞吐测试，也没有人工初值调用：

```bash
python3 scripts/fast_livo2/localizer/validate_motion.py --execute \
  --global-search --domain 197 \
  --archive outputs/fast-livo-backend-validation-20261028-pcl/snapshot \
  --compatibility docs/fast_livo2_evidence/stage_c1_20261028/pcl_compatibility.json \
  --native outputs/fast-livo-demo-20261028-fHJTbGCU/frontend/native_frames \
  --output outputs/fast-livo-c2-final-20261028-pcl
```

| 项目 | 最终结果 |
| --- | --- |
| 输入/处理 | 1362 / 1362 对 |
| 有效定位 | 979 个采样；首次为第384对（零基索引383） |
| 状态 | INITIALIZING 383、VALID 979；LOST/ERROR 均0 |
| 定位子进程 | 退出0，验收结果 passed=true |
| 公共输出 | `/tf`、`/tf_static`、`/cmd_vel`、`/base/safe_cmd_vel` 发布者均0 |
| 进程峰值 RSS | 165840 KiB |
| 总墙钟 | 216.906 s，包含静止/运动基线等待与输入节流 |
| 输入确认延时 p50/p95/max | 140.47 / 260.80 / 3428.00 ms |
| 有效匹配耗时 p50/p95/max | 95.13 / 115.50 / 1803.87 ms |

逐输入快照记录了136个带候选的搜索结果、680次粗配准结果，其中165个粗假设通过并进入精配准，165个精配准几何通过；几何通过不等于初始化成功，静止运动基线仍会拒绝确认。记录到的搜索耗时 p50/p95/max 为200.66/460.43/535.57 ms。统计分母来自逐输入确认快照，不是完整求解事件日志；交接瞬态不保证被该采样器完整捕获。不能将这些比例当跨会话成功率。

源 native DB SHA `2eecd2c75e0c33eac80cda41f043f4c8ebe6bd5c089da06e6018b40deeaf62b4`；源地图 manifest SHA `a890afe331f16ad5d65e601f390a00c7f270cd489346884a41b0b4274b9a4f58`。完整结果/逐输入质量在输出目录，小型入口 `motion_final_result.json`、`motion_layer_summary.json`、`asset_postcheck.json`。此前980有效的中间版本回放保留在 `outputs/fast-livo-c2-functional-20261028-pcl/`，最终以979有效的新门槛版本为准。

## 失败证据、重力小任务与待办

最初临时 KISS 投影配置重复导出 xenium，已修正后由检查返回码的构建脚本验证；第一次索引 CLI 未加载工作空间环境，退出127，source 后新建/检查均通过。保留初始证据，不混作最终成功。最终收紧了地图运动基线、失败完成后退避和索引序列化大小界限，随后重建并重跑上述最终验收与完整运动包。

重力小任务已完成，见 [重力对齐报告](fast_livo2_gravity_alignment_report.md)：新前端试验配置开启初始对齐，世界 +Z 朝上；完整三融合回放1362对，初始化重力约 `[0,0,-9.81]`。旧地图、旧前端日志和原包保留，不追溯旋转旧资产。本轮没有硬件启动。

最少人工待办是另行授权并采集独立建图/定位会话、相似地点/地图外负样本，补物理同步、实际 T_base_imu、真实 wheel 时钟与外部真值；随后另立 D 验收。当前不实施在线前端恢复、BA、高程/坡道、接车或导航。本轮源码和文档留在工作树，C1及既有成果的先前本地提交保持；未获新提交/推送指令。
