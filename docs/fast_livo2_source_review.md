# FAST-LIVO2 阶段 A 源码审查

2026-10-26。本机完整克隆并核对 HEAD；锁定信息见 `config/fast_livo2/sources.lock.json`。候选代码在 `vendor/fast_livo2_review/`，本轮只有 ROS 2 前端、Sophus 和 vikit 的必要子集进入构建。其他仓库均是审查参考，未安装或启动。

| 仓库 | 本次分支/状态 | 完整 HEAD | 许可证观察 |
| --- | --- | --- | --- |
| [fastlio_backend](https://github.com/liangheming/FASTLIO2_ROS2/tree/7baa59f7339ba88ce3b646fe35b23f61031c5d3f) | `main` | `7baa59f7339ba88ce3b646fe35b23f61031c5d3f` | pgo/localizer/interface/hba 各附 MIT 文本 |
| [global_lvba](https://github.com/xuankuzcr/Global-LVBA/tree/10493a2b0f695e32db83dde49dad843a4826e090) | `master` | `10493a2b0f695e32db83dde49dad843a4826e090` | MIT |
| [handheld_mapper](https://github.com/limshoonkit/handheld_mapper/tree/3abbde381e2ae6ec382a9cec8376fc5d47d95545) | `main` | `3abbde381e2ae6ec382a9cec8376fc5d47d95545` | 根/FAST-LIVO2 GPLv2；其他子组件分别标注 |
| [hardware](https://github.com/hku-mars/LIV_handhold_2/tree/30507028a7c00c99ac992322507701e0efe72a2a) | `main` | `30507028a7c00c99ac992322507701e0efe72a2a` | 根 GPLv2；Livox 驱动子目录 MIT |
| [holoagent](https://github.com/HorizonRobotics/HoloAgent/tree/ef14d3152ca6246d8ae64920694c6c74581d246c) | `main` | `ef14d3152ca6246d8ae64920694c6c74581d246c` | 根 Apache-2.0；fast_livo GPLv2 等子组件分立 |
| [longxiaoze](https://github.com/Longxiaoze/FAST-LIVO2-Humble/tree/6130a580e43bb9c3763477dffc70b6a9a148aa4c) | `main` | `6130a580e43bb9c3763477dffc70b6a9a148aa4c` | 本次仓库未发现 LICENSE 文件，未复制实现 |
| [official](https://github.com/hku-mars/FAST-LIVO2/tree/0d2c0346107b75b59934975adec9a6eeeb913c64) | `main` | `0d2c0346107b75b59934975adec9a6eeeb913c64` | GPLv2 LICENSE |
| [openflex](https://github.com/OpenFleX-Wheeled-Humanoid/mapping_localization_layer/tree/0e891ff05dc562868d84716933bb68fdc74476f4) | `v1.0_basic` | `0e891ff05dc562868d84716933bb68fdc74476f4` | 根 CC BY-NC-SA 4.0；部分子包另有 MIT/GPL |
| [openflex_docs](https://github.com/OpenFleX-Wheeled-Humanoid/OpenFleX/tree/980592bd06c355730e6867e79cb77bf6c86d91a5) | `v1.0_basic` | `980592bd06c355730e6867e79cb77bf6c86d91a5` | 根 CC BY-NC-SA 4.0，未复用代码 |
| [ros2_frontend](https://github.com/Robotic-Developer-Road/FAST-LIVO2/tree/837b7bbc1431cb04cf936528e52c83c835efba8e) | `humble` | `837b7bbc1431cb04cf936528e52c83c835efba8e` | GPLv2 LICENSE；上游 package.xml 旧 BSD 声明不一致 |
| [rpg_vikit](https://github.com/Robotic-Developer-Road/rpg_vikit/tree/4b7abc838f5d2ca9137f70f122eaaeff9eaf0f50) | `main` | `4b7abc838f5d2ca9137f70f122eaaeff9eaf0f50` | vikit_common/package.xml GPLv3；有逐文件许可 |
| [sc_fast_livo2](https://github.com/jxk6575/SC_FAST_LIVO2/tree/f39e3d0008af7cbd838b05ccfbad794c66161596) | `main` | `f39e3d0008af7cbd838b05ccfbad794c66161596` | GPLv2 |
| [sophus](https://github.com/strasdat/Sophus/tree/de0f8d3d92bf776271e16de56d1803940ebccab9) | `DETACHED` | `de0f8d3d92bf776271e16de56d1803940ebccab9` | MIT，tag 1.22.10，DETACHED |

## 1. 主要方案

选择 **Robotic-Developer-Road/humble 前端 + 以 liangheming 的 SimplePGO 为主要参考的独立后端**。本轮已构建前端；后端选择是下一阶段的实现依据，**本轮没有实现或验证 PGO/先验定位**。

前端候选的 `src/vio.cpp`、`src/IMU_Processing.cpp`、`src/voxel_map.cpp` 与 HKU 官方作了逐文件对照，差异归档于 `docs/fast_livo2_evidence/stage_a_20261026/official_humble_*.diff`。视觉的图像稀疏地图、光度残差、Jacobian、滤波状态与协方差更新仍在运行路径；主要迁移差异包括 ROS 2 API、模板版 Sophus、参数与时间戳处理。没有将视觉分支替换成 FAST-LIO2。

liangheming 后端规模小、输入是机体系点云与局部位姿，适合接入本轮明确的新前端接口。首版优先位置先验候选检索；是否引入全局描述子须由运动漂移数据决定。不能原样启动参考节点来声称集成完成。

## 2. liangheming/FASTLIO2_ROS2

- `pgo/src/pgos/simple_pgo.cpp`：`isKeyPose/addKeyPose` 按平移/转角选帧，以相邻局部位姿生成里程计因子；首帧先验固定 gauge。`getSubMap` 将各帧 body cloud 按各自优化位姿变换后拼接。
- `searchForLoopPairs`：至少 10 关键帧，KD-tree 半径候选和时间间隔；ICP 的输入已经在全局系，使用单位变换初值。判断只有 `hasConverged` 与 fitness 门槛，**缺少重叠率、相对变换合理性、重复结构/多候选一致性以及结构化拒绝日志**。fitness 不是成功概率。
- `smoothAndUpdate`：GTSAM iSAM2 更新所有关键帧，并计算当前全局刚体修正。历史地图优化成立的依据是逐关键帧位姿更新与重建，不是单独发布这一个修正。
- `pgo/src/pgo_node.cpp`：ApproximateTime 未约束最大配对差；`last_message_time` 未初始化；定时回调取队首后清空整个队列，会丢弃后续帧。默认 TF 发布也不能与公共桥接同时使用。
- `saveMapsCB` 确实逐帧用优化位姿重建 `map.pcd`，可保存 `patches/*.pcd` 和 `poses.txt`。但会删除已有 patches/poses，写盘结果检查、原子提交、时间戳、坐标系与配置元数据不足，不能直接用于本项目地图保存入口。
- `localizer/src/localizers/icp_localizer.cpp`：粗/细两级降采样 ICP，以传入 `guess` 初始化，不做地点全局搜索，也没有保证可收敛的米/度范围。
- `localizer/src/localizer_node.cpp`：`relocalize` 服务接收 pcd_path 和 x/y/z/roll/pitch/yaw。服务返回成功只说明地图加载和请求接收，不等于后续 ICP 成功。`localize_success` 成功后未随连续失败撤销，失败仍发送旧 offset，未区分失效状态；时钟使用 `rclcpp::Clock()`，不是节点模拟时钟。须替换外层节点、时间配对、状态与 TF 所有权。
- `interface/srv/SaveMaps.srv`、`Relocalize.srv`、`IsValid.srv` 为参考接口，不等于本项目已提供相同服务。`hba` 是可选后续离线地图精修，不替代在线 PGO。

## 3. OpenFleX 对照

远端 heads 查询和完整克隆确认 mapping_localization_layer 与配套 OpenFleX 文档此次都在 **v1.0_basic**；不能把 README 能力描述当运行验证。

`pgo/src/pgos/commons.cpp` 与 liangheming 对应文件字节相同；SimplePGO 和 pgo_node 具有相同结构并有扩展。文本比较记录于 `reference_source_audit.json`，相似度不是作者归属证明，但足以说明不能当两个完全独立后端重复引入。

- `pgo/src/pgos/simple_pgo.cpp` 实际调用 `makeAndSaveScancontextAndKeys` / `detectLoopClosureID`，随后 NDT→ICP（NDT 失败则 ICP）。新增时间与局部位置筛选。因此 SC/配准不是只有依赖声明。
- 回环时 source 已经位于全局系，却直接用 SC 相对航向绕全局原点旋转作初值，缺少绕关键帧中心的平移构造。不可不经非零位置/航向的数值验证直接复制。
- 保存路径包括 `scan_context/metadata.txt`、`polarcontexts.bin`、`invkeys.bin`、`vkeys.bin`、`invkeys_mat.bin`，并有关键帧与优化地图，不能假定与 HoloAgent 格式兼容。
- `icp_registration/src/icp_registration.cpp` 的初始化路径实际进入 `scRelocalize`，失败有粗初值周围网格搜索；配置为 5×5 XY 网格与离散 yaw，**不是任意位置连续全局搜索的保证**。持续定位实际调用 ICP，另有轮速一致性与连续拒绝检查。
- 初始化与部分持续路径中，同刻 TF 查询失败会回退到 `TimePointZero`；SC 机体系变换失败还可能使用原始云继续。它直接拥有 map→odom，并默认平面化/平滑，不能同时旁路已有采样时刻桥接，也不适合未经审查推广到坡道 6DoF。
- 仓库根许可证是 **CC BY-NC-SA 4.0**，部分子包另附 MIT，ScanContext 带第三方头。不能只看 package.xml 的 MIT 就给整个仓库作统一许可结论。本轮不复制其实现。

## 4. HoloAgent

定位到用户指定路径：`agentic_robot/core/src/fast_livo/src/online_relocalization.cpp`、`include/online-relo/pose_estimator.cpp`、`src/multi_session.cpp`。

- `pose_estimator::run` 同步 cloud/pose 队列；未初始化时实际调用 `globalRelo()`。SC 初始化会把多帧云按前端相对位姿变换到最近机体系，再计算描述子，检索先验关键帧和航向、构建子图并配准；也有 `/initialpose` 外部初值分支。
- 持续定位依靠前端相对增量预测，再调用 `relocalization()`；配准模式包含 PCL ICP/NDT 与 CUDA 方法。失败走 `lio_incremental()`，不能将其前端外推当新注册成功，也没有可直接复用的本项目健康消息状态机。
- 地图生成写编号的 `.pcd` / `.scd` 及轨迹/会话资产；定位依赖 sessions 的关键帧和 SC 数据结构，不能只把另一项目 `map.pcd` 放入目录就认为兼容。
- `publish_odometry` 使用节点 now，直接发布 **map→base_link**；与本项目 map→odom→base_link 所有权、原采样时间和传感器参考点要求不一致。
- `multi_session.cpp` 从两个已保存 session 初始化、调用固定次数 `run(iteration)` 再可视化，是离线多会话优化入口，不是本任务所需在线回环已完成的证据。
- 根许可证 Apache-2.0 明确不能覆盖 vendored fast_livo 的 GPLv2 与其他依赖。本轮仅审查，没有导入整个框架或 CUDA 组件。

## 5. SC_FAST_LIVO2、硬件参考与 Global-LVBA

SC_FAST_LIVO2 是 ROS 1。`pgoKeyframesCallback` 接收外部优化 Path，`updateStateWithPGO` 改滤波位姿/部分协方差，`rebuildLocalMapWithPGO` 重构/更新局部地图。实现不是原生完整 PGO，最新关键帧与当前滤波状态的时刻对应、视觉地图一致性仍须证明。本版采用外部全局修正，不移植这些滤波反馈。

LIV_handhold_2 的 `mvs_ros_driver/src/grab_trigger.cpp`、修改版驱动 `lddc.cpp` 与 Mid360 配置用于核对外触发/共享戳方案。本机已有 `ws_calib`，没有替换驱动。handheld_mapper 是带多依赖的工作空间，Longxiaoze 此锁定仓库主要是安装/使用文档，未发现许可证文件；没有因为教程完整而替换本机方案或采用外机 IP/标定。各仓库根与子模块许可分别记录，不统一推定。

Global-LVBA 的实际数据要求包括按时间戳命名的图像、机体系点云、image_poses/lidar_poses、COLMAP 对应关系。ROS 1/Ceres/SiftGPU 等构建与离线优化不进入第一版在线路径。README 的导出提示不能替代对前端实际写盘条件的检查；本轮保存了原 bag、成对本体云与轨迹，但**没有声称已经导出完整 LVBA 数据包**。

## 6. 实际构建依赖与许可边界

Sophus 选用模板版 1.22.10，完整 SHA 已锁定并标记 **DETACHED**；候选 README 推荐的非模板 a621ff 与当前 `SE3<double>` 源码不符，未照搬安装步骤。

vikit 只构建 pinhole_camera、math_utils、vision、robust_cost；使用同节点五参数相机配置，未启动其远程参数黑板或执行 popen 参数查询。`vikit_common/package.xml` 标为 GPLv3，robust_cost 带 GPLv3-or-later 文件头，部分其他头又含单独许可；不能将前端旧 package.xml 中 BSD 描述当整个组合的许可。FAST-LIVO2 本体 LICENSE 为 GPLv2；本次构建仅用于本机实验，**组合许可与对外分发条件尚未完成核对**，不以本报告声称分发授权已解决。

本轮没有复制 PGO/SC/定位参考代码，也没有安装 GTSAM、CUDA 或重装 ROS。后端实施前需保留实际复用模块各自的版权与许可证。
