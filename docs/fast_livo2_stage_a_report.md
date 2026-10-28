# FAST-LIVO2 阶段 A：工程审查与前端基线

> 后续整合说明：本报告保留当次证据；相机/标定现已迁入工程，新入口与路径见 [整合报告](consolidation_report.md)、[标定 README](../calibration/README.md)。

后续用户已另行授权在线传感器与 RViz，完成后又要求停止；最新状态见 [在线追加报告](fast_livo2_online_report.md)。下文保留本轮较早的离线基线快照。

日期：2026-10-26。分支 `refactor/pluggable-navigation`，起始及当前主工程 HEAD `a76d83a767d5691ca40351f3a57f65b398572a52`，本轮修改留在工作区供审阅，未提交/推送。

**总结果 PARTIAL：三传感器前端已构建，并完成真实静态同步包的算法回放与 DDS 接口验证；物理同步、标定验收与运动场景基线未通过。在线回环、优化地图保存和先验地图定位尚未实现，未进入 B–D，也没有放行 Nav2。**

本轮属于交接中的第四批/新任务阶段 A。用户再次确认实物 **Ranger Mini V2**，且目前**没有三传感器运动录包**。不使用任务开头的 3.0 字样改写实物版本，不把无图像运动包作为三传感器验证。

## 1. 本机检查与保留内容

- 实际系统 Ubuntu 22.04.5、x86_64、内核 6.8.0-40、ROS 2 Humble；未假设其他机器环境。系统依赖精确版本在 `fast_livo2_evidence/stage_a_20261026/dependency_versions.txt`。
- 原算法是 Lightning，`ws_slam/src/lightning` 保留；此前无 FAST-LIVO2 工作空间。公共消息、Lightning 适配器、轮速插值桥接与受管会话均已存在，继续复用其边界。
- `ws_camera/src/hik_camera_ros2_driver` 原有两处修改保留。`docs/03_mid360_validation.md` 原有 IP 文档修改与六个未跟踪雷达日志目录保留，未收入本轮算法变更。
- 找到独立 `~/calibration/ws_calib` 的外触发相机驱动、共享时间 writer、真实五参数内参和三场景联合外参。不是将原 `ws_camera` 的主机写戳实现认定为同步实现。
- 唯一发现含 LiDAR/IMU/同步图像的可用包为 `~/calibration/bags/stageB_sync_run2`，约 83.45 s。标定 scene/validation 包只有点云和图像，缺 IMU；原导航运动闭环包缺图像，均不能替代它完成本阶段 LIVO 回放。
- 内参状态仍 PARTIAL；外参 `geometric_calibration_accepted=false`。本轮使用本机试算参数做离线试验，保存原文件快照和 SHA256，没有改写外部标定工程或提升验收状态。
- 未启动真实传感器驱动、底盘、导航、CAN 控制或运动命令；没有 sudo/系统依赖安装、网络配置修改或 CUDA 改动。测试只读录包，在 localhost 的独立 ROS domain 运行。

初始及完成时的版本/修改清单见 `fast_livo2_evidence/stage_a_20261026/{initial_inventory,completion_inventory}.json`；中间的 final_inventory 也保留。引用仓库均完整克隆，13 个仓库的 branch/full SHA/许可事实见 [源码审查](fast_livo2_source_review.md) 和 [锁文件](../config/fast_livo2/sources.lock.json)。

## 2. 实际实现

1. 选定 ROS 2 Humble 候选 `837b7bbc1431cb04cf936528e52c83c835efba8e`，保留 LIO、IMU、视觉光度更新和视觉地图。与 HKU 官方关键文件的差异已归档。
2. 新增本机 Mid-360S PointCloud2 解析：字段/stride/长度/字节序验证，按**绝对纳秒**转算法所需相对毫秒，时间排序、无效点/时间/tag 过滤。没有伪造 ring 或改成只有 LiDAR–IMU。
3. 使用同节点真实五参数相机模型，修复 k3 漏传，移除远程黑板/CLI 参数读取依赖。原 Rcl 的六位小数数值舍入只作微小 SO(3) 投影，大误差拒绝；方向仍 T_camera_lidar。
4. 修复 main 在节点未创建前构造 ImageTransport、缺失 tf2_geometry_msgs 声明、硬编码 vikit 安装库路径及无实际使用的 pcl_ros 依赖；选择当前源码需要的模板版 Sophus，安装在本项目目录。
5. 修复上游 x86 `-march=native` 与预装 PCL 的对齐兼容问题：GDB 定位 PCL 云释放段错误，切换与系统库相容的基础 x86 编译后，短/完整回放和 DDS 正常退出。保留崩溃与修复证据，不将初次失败写成通过。
6. 专用 `/fast_livo/deskewed_imu` 与 `/fast_livo/odometry` 在 **同刻 VIO 后**成对发布，点云在 IMU 机体系，里程计为 T_local_imu。修复 VIO 步清空临时云导致的导出遗漏；不使用上游累积彩色世界云作后端输入。
7. 默认关闭前端 TF；`require_livo=true` 防止关闭图像/IMU静默降级。增加原采样戳重复/回退、>0.5 s 数据断层和内部队列上限保护，异常会话失败退出，不自动重启或沿用旧状态。
8. 提供无驱动的原 bag 直接回调回放、轨迹/原生云导出、逐帧视觉求解和状态改变量记录；另外提供消息字节不变的 header-clock 回放视图及真正 DDS 回放测试。

所有第三方修改保存于 [前端补丁](../patches/fast_livo2_stage_a.patch)，离线工具与测试源码在 `scripts/fast_livo2/`；专用构建目录为 `ws_fast_livo/`。未修改现有生产 runtime、地图清单、Nav2 或安全链。

## 3. 时间、坐标与 TF 审查结论

完整契约、外参方向、QoS 和时钟说明见 [前端接口文档](fast_livo2_interfaces.md)。关键结论：

- 图像与点云 834/834 header 精确相等；16,679,808 点的 FLOAT64 timestamp 是绝对纳秒，偏移范围 0–100.324096 ms，无负偏移和包内回退。原始 IMU 16,688 条。
- 共享戳相等只证明写戳/配对，不证明曝光与扫描物理同步。扫描首点→组帧完成→共享文件更新→图像读取的相位仍待测；writer 还存在先 O_TRUNC 再 flock、非完整 seqlock 的竞态风险。**本轮没有默认加减 100 ms，所有时间偏移为零试验假设。**
- 存储时间约 2026 年，传感器 header 约 2020 年；直接以原包 `--clock` 驱动后端新鲜度会造成历元错误。新回放视图仅调整新 bag 的存储调度时间，并独立验证所有序列化消息内容不变。
- 原上游 odometry 早于视觉更新且用了 now，原 cloud_registered 的参考系/累积/FOV 不适合直接接 PGO。新增专用接口明确 reference time/body，不靠话题重映射掩盖问题。
- FAST-LIVO2 的局部原点不是 wheel odom。以后后端输出同刻 T_map_imu，经 T_base_imu 转为标准 LocalizationEstimate，公共桥接使用原采样时刻轮速插值生成 map→odom；桥接继续独占 map→odom、odom→base_link。前端此阶段不发布公共 TF，也不发送 VALID 定位结果。
- T_imu_lidar 沿用本机原 Mid-360S 名义几何；T_base_imu 仍是现有近似安装值。相机–雷达试算不等于车体安装标定。

## 4. 验证与实测结果

| 检查 | 最终结果 | 证据 |
| --- | --- | --- |
| 隔离依赖与前端构建 | **PASS，退出 0**，生成 frontend/replay/checks 三个程序 | `stage_a_20261026/build_final.log`、仅空白/说明清理后的 `build_final_whitespace.log` |
| 前端定向检查 | **PASS，13/13，退出 0** | `stage_a_20261026/checks_final.log` |
| 真实完整静态包回放 | **PASS，仅算法输入与输出基线范围，退出 0** | `replay-X9zAmJPW/{frontend.log,resources.txt,exit_code.txt,manifest.json}` |
| 真正 ROS/DDS 回放 | **PASS，退出 0**；6 s 中 55/55 同戳对、坐标系正确、数值有限、时钟同域、无 TF 发布者 | `stage_a_20261026/dds_smoke_final/{result.json,frontend.log,player.log,view_manifest.json}` |
| 现有轮速时间对齐 | **PASS，7/7，退出 0**；合成延迟/6DoF 插值，不是实车误差 | `stage_a_20261026/existing_time_alignment.log` |
| 干净 HEAD 应用补丁并与本机源码比较 | **PASS** | `stage_a_20261026/patch_reproducibility.json` |
| 运动精度、物理同步、真实回环与重定位 | **未验证/缺运动数据** | 用户已确认无三传感器运动录包 |

表内 `stage_a_20261026/`、`replay-*/` 均以 `docs/fast_livo2_evidence/` 为根。13 项检查覆盖未对齐/带行填充点云、时间单位与排序、非法布局/字段/点时刻、非有限点/tag、真实五参数模型对照 OpenCV、回退/重复/断层与队列保护。

DDS 测试成功判据在代码中预先明确：播放器退出 0、6 s 至少 45 个完全同戳对、位姿有限且坐标系正确、无 TF 发布者、/clock 与传感器同历元、前端正常退出。最终 55 对满足全部判据。它不是运动/重定位成功率测试。

**最终完整回放**（日志 `replay-X9zAmJPW`，原生数据 `bags/fast-livo2-baseline-126a5fAo/result/`）：

- 读入 18,356 条选定话题消息；导出 829 对，覆盖 82.8002 s 有效传感器时间；829 个位姿时间戳唯一且轨迹全部有限。首段 IMU 初始化、首图时序及录包尾部不完整区间不伪造输出。
- 828 帧进入视觉求解；共 **4,222 次已执行视觉滤波状态更新迭代**，526 帧观察到非零视觉平移净改变量。首次建视觉地图的帧没有已知视觉匹配。以源码计数加前后状态差证明视觉参与，不以图像开关、订阅存在或 RGB 点云证明。
- 每帧本体系点数 min/median/max：4,257 / 4,390 / 4,527；内部 LiDAR/IMU/image 队列峰值 2 / 53 / 1。
- VIO 步耗时 median/p95/max：3.737 / 4.914 / 6.798 ms；这不是整条在线管线延迟。
- `/usr/bin/time -v`：wall 10.60 s，user 39.19 s，system 7.13 s，平均 CPU 436%，峰值 RSS 510,308 KiB（约 498 MiB）。这是**加速离线回放含读写**，不是实时部署 CPU 上限或长期内存验收。
- 名义静态段相对首输出位姿的最大位移 0.01423 m，首尾位置差 0.00574 m。这是实测输出离散度，**不是绝对定位精度、APE/RPE、闭环改善或标定精度**。

轨迹、逐帧 CSV、指标和图已另存为小体积证据，原生云 bag 留在忽略的大数据目录：

- [原始前端轨迹](fast_livo2_evidence/stage_a_20261026/baseline_final/trajectory_imu.tum)
- [逐帧诊断](fast_livo2_evidence/stage_a_20261026/baseline_final/frames.csv)
- [指标与声明边界](fast_livo2_evidence/stage_a_20261026/baseline_final/metrics.json)
- [轨迹/视觉参与/时间图](fast_livo2_evidence/stage_a_20261026/baseline_final/overview.png)

先前修复前的回放与最终迭代数不同，不能拿其数字混入最终基线。并行浮点运算下不承诺重复执行逐比特相同。

## 5. 关键命令与失败记录

以下已实际执行，未隐藏失败：

| 命令/动作 | 退出结果与处理 |
| --- | --- |
| `git status --short --branch`、逐仓库 rev-parse/status | 0；根仓库及嵌套修改保留 |
| 首次沙箱 `git ls-remote` | 128，连接失败；正常授权后成功联网，clone 均完成 |
| 用 a621ff 作为 clone 的 branch 参数 | 128，该引用不是分支；进一步检查源码实际要模板版，改为已锁定 Sophus 1.22.10，DETACHED |
| `bash scripts/build_fast_livo2.sh --execute` 早期 1–3 轮 | 1/2，分别发现无实际使用的 pcl_ros 依赖、漏声明 tf2_geometry_msgs、残留 camera_loader include；逐项修复，日志全部保留 |
| 第 4/5 轮构建 | 0；原生向量化配置仍导致运行退出崩溃，不能据构建成功声称运行成功 |
| `gdb --batch ...` | 沙箱 ptrace 被拒；授权后 backtrace 定位点云释放。`gdb_attempt2.log` 保留；临时复现输出 `/tmp/ranger_livo2_gdb_b_20261026` 不是正式交付资产 |
| `replay-QXmLNqdS`、`replay-Cmnp4g3Q` | 139；第一轮还暴露 VIO 临时云清空导致导出 0 对，第二轮发生于便携构建尚未安装前，仍是旧可执行文件；二者不计通过 |
| `replay-7lQalNbI` | 0，安装便携构建后短回放 20 对通过 |
| `replay-dtDhjPpY` | 0，修复后早期完整包通过；最终增加输入保护后的正式基线以 `replay-X9zAmJPW` 为准 |
| DDS `dds_smoke_a` / `b` | a 判定失败（clock 订阅 QoS 不兼容并有沙箱 socket 警告）；b 退出 1（Humble 无 qos_profile_clock 导出）。修正为显式 BEST_EFFORT Clock QoS 后 c/final 均退出 0；没有降低通过判据 |
| 最终 build / 13 检查 / 完整回放 / DDS / 原时间对齐 | 均退出 0，见上一节 |

## 6. 依赖与可复现入口

在本项目根目录，已有 Humble、PCL/Eigen/OpenCV/fmt/Boost、cv_bridge/image_transport/pcl_conversions、tf2_geometry_msgs、rosbag2_cpp/py 和 Livox 自定义消息。**本轮没有安装系统依赖。** matplotlib/numpy 仅用于审计/指标图。所有实际版本见依赖记录，候选算法版本见 lock；本地补丁不能用“最新版”代替。

```bash
# 默认只显示计划；源码已在本机，无需再次下载。
python3 scripts/fast_livo2/fetch_sources.py
bash scripts/build_fast_livo2.sh
bash scripts/run_fast_livo2_replay.sh

# 在新机器/缺源码时获取锁定的三个实际构建依赖；参考仓库用 --all-references。
python3 scripts/fast_livo2/fetch_sources.py --execute

# 隔离 CMake 构建，未声称 colcon 干净构建已经验证。
bash scripts/build_fast_livo2.sh --execute

# 真正前端、原传感器三话题，完全离线；每次独立日志/数据目录，不覆盖旧资产。
bash scripts/run_fast_livo2_replay.sh --execute \
  --bag /home/lab307/calibration/bags/stageB_sync_run2

source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
export LD_LIBRARY_PATH="$PWD/ws_fast_livo/deps_install/lib:${LD_LIBRARY_PATH:-}"
ws_fast_livo/install/fast_livo/lib/fast_livo/ranger_livo_checks

# 以下启动的只是录包播放器、前端与测试订阅者，domain 174/localhost。
export ROS_DOMAIN_ID=174 ROS_LOCALHOST_ONLY=1 OMP_NUM_THREADS=4
smoke_tag=$(date +%Y%m%dT%H%M%S)
python3 scripts/fast_livo2/dds_smoke.py \
  /home/lab307/calibration/bags/stageB_sync_run2 \
  --data "bags/fast-livo2-dds-$smoke_tag" \
  --out "docs/fast_livo2_evidence/dds-$smoke_tag"
```

构建脚本拒绝 commit 不符或超出审查补丁的已跟踪源码修改；不 reset 用户仓库。依赖本地安装到 `ws_fast_livo/deps_install`，算法到 `ws_fast_livo/install/fast_livo`；不写 `/usr/local`。复现命令不是底盘/现场/导航启动入口。

回退不需要改系统/网络：停止本次离线进程，继续原受管 Lightning 入口即可，生产 runtime 未改变。保留新工作区和所有日志/原始包，若需撤销主工程变更应按本报告文件清单逐项审阅，不能删除整个现有工作空间或硬重置用户修改。

## 7. 修改文件与交付边界

- 约定/状态/导航文档：`AGENTS.md`、`.gitignore`、`.gitattributes`（保留原始日志空白）、`README.md`、`docs/STATUS.md`、本报告、`fast_livo2_source_review.md`、`fast_livo2_interfaces.md`。
- `config/fast_livo2/`：sources.lock、本机三传感器试验参数、标定来源/状态、原标定小文件快照。
- `patches/fast_livo2_stage_a.patch`：全部前端第三方修改；构建时复制 replay/checks/input_guard 自有源码。
- `scripts/audit_fast_livo2.py`、`build_fast_livo2.sh`、`run_fast_livo2_replay.sh`；`scripts/fast_livo2/` 的审计、回放、DDS 测试、指标、版本获取、输入保护及定向检查。
- `docs/fast_livo2_evidence/`：版本、源码差异、依赖、所有失败/最终成功日志、轨迹和图。大点云/播放视图均留在忽略的 `bags/`，算法构建在忽略的 `ws_fast_livo/`，引用源码在忽略的 `vendor/`。

**未实现**：关键帧/回环/PGO 的本项目节点、优化地图包与原子保存加载、给定初值定位、无初值自动定位、失效状态机到 LocalizationEstimate 的适配、Nav2 新后端上线。现有 Lightning 地图与各参考格式没有已验证互转；没有伪造建图/保存/定位三个完整入口。

**未验证**：三传感器运动精度和同步物理相位、独立几何标定、真实回环/无回环/重复结构、重启加载、多初始位姿、遮挡/中断恢复、完整时钟跳变场景、长时视觉地图内存增长。无真值不计算/编造 APE/RPE；本轮没有自动重定位试验，因此没有成功次数/总次数可报告。

**下一步最少人工操作**：后续采集阶段由工程侧先修复并离线验证共享戳保护，再请用户在传感器固定、镜头设置不变的条件下提供一份包含三话题的运动闭环包和同期同步 CSV；优先手持采集，底盘启动/运动仍须本次明确授权。该数据补齐前不能将阶段 A 的静态结论升级为运动基线，也不自动启动 B–D 或高程地图阶段。组合许可的分发条件另有待核对，不影响以上本机运行事实。
