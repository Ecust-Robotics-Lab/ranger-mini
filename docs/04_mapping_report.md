# 阶段 4：Lightning-LM 编译、离线输入验证与小地图待验

更新：2026-09-23。实物：Ranger Mini V2 + **Mid-360S**。范围为阶段 4，不进入定位部署、导航或坡道通行性评估。

> 当前雷达已更换并已实际识别收流：**Mid-360S，完整 SN `ARMCP4E0033801`，IP `192.168.1.101`，MAC `4c:43:f6:f4:9d:c2`，协议 type=35**。[地址切换只读检查](04_evidence/radar_switch_3801.json)、[成功驱动日志](03_evidence/driver-2v7kj2bx/)（退出 0；点云与 IMU 均 time_type=0）。此前静止包、完整 SN `ARMCP870038745`、MAC `ec:72:f7:da:cf:c8` 和其时间质量属于旧雷达，不能外推到新设备；新设备已由手持诊断包确认 CustomMsg/IMU 消息与频率（见第 4 节），但采集时**未安装在车体上**，安装外参仍未测。

## 结论

**三维验收完成（2026-09-23）：人工图审合格、安装约值已提供、尺度核对由用户明确免除；g2p5 二维图已按实测参数离线生成并验证；在线模式静止管线验证通过（保存服务首次实测成功）。阶段 4 建图链路闭环。**

| 项目 | 当前结论 | 证据 |
|---|---|---|
| 固定源码、项目内依赖、低并发编译 | PASS，有上游编译警告 | [构建结果](04_evidence/lightning_build_result.json)、[完整日志](04_evidence/lightning_build.log) |
| 外部 Livox 消息接口唯一性 | PASS，未生成第二套 Livox 接口 | [接口和动态链接审计](04_evidence/interface_link_audit.json) |
| 真实静止包经 Lightning RosbagIO 读取和预处理 | PASS，仅输入兼容性 | [真实统计](04_evidence/static_reader.json)、[退出及原包校验值](04_evidence/static_reader_result.json) |
| 专用配置初始化、外参转换、保存保护 | PASS，仅离线软件检查 | [保存保护](04_evidence/save_guard_result.json)、[工具检查](04_evidence/tool_tests.log) |
| 新雷达驱动与消息收流（SN …3801 @ .101） | PASS，退出 0；time_type=0 | [driver-2v7kj2bx](03_evidence/driver-2v7kj2bx/) |
| 手持诊断闭环包输入检查 | PARTIAL：500/500 帧逐点 offset 回退 | [validation-stage4.json](../bags/mid360-manual-loop-4uwmxbzb/validation-stage4.json) |
| 手持包两次离线运行 | 诊断性成功（exit 0、结构 PASS、35 关键帧），**不通过正式建图验收** | [第 4 节](#4-手持诊断闭环包与两次离线运行不通过正式建图验收)、[地图清单](04_evidence/map_inventory.json) |
| 正式车体闭环包输入检查 | 退出 0；逐点 offset 回退 122,536 次保留 | `bags/mid360-manual-loop-n2cgpiz0/`、[第 5 节](#5-正式车体闭环包两次离线运行验收进行中) |
| 正式包两次离线运行 + 同包比对 | exit 0、结构 PASS、49 关键帧；三维 PASS（人工图审合格；尺度用户免除，如实记 WAIVED） | [第 5 节](#5-正式车体闭环包两次离线运行三维已验收)、[地图清单](04_evidence/map_inventory.json) |
| g2p5 二维图及 YAML | 已生成并离线验证（`lightning-offline-suej7qsz`：map.pgm 0.05 m + map.yaml，结构 PASS、局部一致性 PASS） | [第 5.1 节](#51-g2p5-二维离线验证2026-09-23)、[acceptance](../maps/lightning-offline-suej7qsz/acceptance.json) |
| 在线模式 | PASS（静止管线验证：exit 0、保存服务首次实测 response=0、结构 PASS） | [第 5.1 节](#51-g2p5-二维离线验证2026-09-23)、[acceptance](../maps/lightning-online-aea6vhyt/acceptance.json) |

本轮 Codex 没有启动底盘或雷达驱动，没有发 CAN、速度、导航目标；雷达驱动由用户按采集说明自行启动。`maps/` 现有三个正式车体包实验目录：`lightning-offline-8k_qnpae/`、`lightning-offline-k2i9n8qw/`（三维已验收 PASS）与 `lightning-offline-suej7qsz/`（g2p5 二维验证）。手持诊断实验目录 `lightning-offline-t_4fi98t/`、`lightning-offline-7dwk87rr/` 已由用户于 2026-09-23 自行删除（确认不再需要），其"不通过正式建图验收"结论不变，诊断数字见第 4 节留档；对应 bag `bags/mid360-manual-loop-4uwmxbzb/` 与全部正式实验地图保留。

## 1. 源码与依赖

- `ws_slam/src/lightning`：官方 `https://github.com/gaoxiang12/lightning-lm`，固定 **1325fed8fa97a2506878360fc23427ea00a767da**，master 来源、**detached HEAD**。从阶段 3 已获取的同提交只读审查副本以 `git clone --no-hardlinks` 独立克隆，origin 改回官方地址；没有追逐远端更新。[获取命令及退出 0](04_evidence/acquisition.json)
- 读取了 [README_CN](../ws_slam/src/lightning/README_CN.md)、[顶层 CMake](../ws_slam/src/lightning/CMakeLists.txt)、[依赖 CMake](../ws_slam/src/lightning/cmake/packages.cmake)、[安装脚本](../ws_slam/src/lightning/scripts/install_dep.sh)、[Livox 示例](../ws_slam/src/lightning/config/default_livox.yaml)、输入/预处理/IMU/回环/保存源码。未直接执行上游 sudo 安装脚本。
- 外部 driver：`ws_livox/src/livox_ros_driver2`，**4a1def929e5b59c7a8122d19fce6efba581ce9f7**；SDK2：`vendor/livox/Livox-SDK2`，**08f523c930b2f0ba1e98a6afaa8d7476bf479908**。两者 detached HEAD；driver 的阶段 3 诊断补丁保留，SDK 干净。阶段 3 的 Lightning 审查副本未改。[完整来源、状态、补丁校验值](04_evidence/source_versions.json)
- 当前原生 Ubuntu 22.04.5、6.8.0-40-generic、ROS 2 Humble。已有 PCL 1.12.1、Eigen 3.4、OpenCV 4.5.4、TBB、yaml-cpp、OpenGL 等，实际包版本见 [环境记录](04_evidence/environment_final.json)。
- 缺失依赖以固定版本 Ubuntu deb 下载、校验、解包到 `ws_slam/deps/root`；没有写系统安装目录：glog `0.5.0+really0.4.0-2`，gflags `2.2.2-2`，epoxy-dev `1.5.10-1`，libunwind-dev/runtime `1.3.2-2build2.1`。[命令、deb 元数据与 SHA256](04_evidence/dependencies.json)、[unwind runtime](04_evidence/unwind_runtime.json)
- Pangolin **0.9.3** 来自上述固定 Lightning 提交内的归档；归档 SHA256 `bd22e2f7cbf7093fcc8e20927157dccd5a82057ab187c1450af37a112bce5807`，没有虚构独立 Git SHA。安装于 `ws_slam/deps/pangolin_install`。禁用示例、工具入口、Python 和非必要相机后端，保留 Lightning 所需库；配置/编译/安装均退出 0。[准确命令](04_evidence/pangolin_commands.json)
- **关闭运行 UI 不会去掉 Pangolin 构建依赖**：[packages.cmake:7](/home/lab307/ranger_nav/ws_slam/src/lightning/cmake/packages.cmake:7) 仍要求 Pangolin，lightning.libs 仍编译 UI 源文件。
- 编译使用 sequential colcon、`MAKEFLAGS=-j2`、`CMAKE_BUILD_PARALLEL_LEVEL=2`；Release 去除上游强制的 C++ 调试符号，保留 O2。没有假称测得峰值内存。

### 可追踪补丁

[stage4_lightning_integration.patch](../patches/stage4_lightning_integration.patch) 包含以下局部集成/正确性修复，不替换 ESKF、配准或闭环算法：

1. 不编译内置 `thirdparty/livox_ros_driver`，改为 `find_package(livox_ros_driver2)` 和外部命名空间 typesupport target；清除内置生成头路径。两份 `.msg` 字段逐项相同；build 树没有第二份 `livox_ros_driver2/msg/custom_msg.hpp`，ldd 只解析到 ws_livox 的 Livox C++ typesupport，未发现缺失动态库。
2. glog 使用导入 target，包含 miao；删除 package.xml 中源码未使用的 scrubber_common/agibot_robot 依赖，补真实 Livox/rosbag2_cpp 依赖。构建产物放 build/bin。
3. 离线 Livox 话题读取专用 YAML，不再硬编码；在线保留 rclcpp 的 SIGINT 处理，离线才使用上游退出标志。
4. 修复 Livox 空/长度不符消息的边界；邻点比较读取不可变原始消息，消除并行读取其他线程输出的竞争；括号逻辑修正，使 blind 距离约束作用于 x/y/z 各方向。仍保留上游点序抽样、时间单位和标签策略。[pointcloud_preprocess.cc:41](/home/lab307/ranger_nav/ws_slam/src/lightning/src/core/lio/pointcloud_preprocess.cc:41)
5. 关键帧使用 IMU 位姿，上游直接存 LiDAR 系点云；现在复制转换为 IMU 系后交给关键帧，保证回环、g2p5、导出使用一致坐标。原始消息和前端去畸变点云未就地改变。[laser_mapping.cc:366](/home/lab307/ranger_nav/ws_slam/src/lightning/src/core/lio/laser_mapping.cc:366)、[转换函数](/home/lab307/ranger_nav/ws_slam/src/lightning/src/core/lio/extrinsic_cloud.h:5)。已做非单位旋转+平移合成检查；真实动态效果待验，共用 LIO 对定位的影响也未作阶段 5 验收。
6. 保存时拒绝覆盖已有目标目录、拒绝无关键帧地图；离线清楚退出 2，服务返回失败并记录原因。输出 `keyframes.txt` 保存 LIO/优化后的完整 6DoF 位姿；二维 YAML 分辨率改用实际 `map.info.resolution`，不再写死 0.05。[slam.cc:154](/home/lab307/ranger_nav/ws_slam/src/lightning/src/core/system/slam.cc:154)

本地解包 glog 的 CMake 导出中含系统绝对 libunwind 路径，另存 [stage4_glog_local_prefix.patch](../patches/stage4_glog_local_prefix.patch)，只修项目内前缀。[补丁反向检查退出 0](04_evidence/patch_check.json)。不要对现有已修改源码重复应用补丁；复现时在干净的同提交副本应用一次。恢复无需改系统：保留日志和产物，使用另一份干净固定提交；不删除现有工作空间。

## 2. Ranger + Mid-360S 专用配置

配置：[config/slam/ranger_mid360s.yaml](../config/slam/ranger_mid360s.yaml)。参数是待运动数据验证的工程初值，未宣称标定。

| 项目 | 配置及实际代码含义 |
|---|---|
| LiDAR | `/livox/lidar`，`livox_ros_driver2/msg/CustomMsg`；不改成 RViz 派生 PointCloud2 |
| IMU | `/livox/imu`，`sensor_msgs/msg/Imu`，保留原始 g/rad/s 输入；不使用 orientation 作车体姿态 |
| 多余 PointCloud2 订阅 | `/ranger/unused_pointcloud2`，避免误接派生点云并重复喂数据 |
| QoS | 在线订阅 KEEP_LAST(10)、RELIABLE、VOLATILE；阶段 3 实际发布者为 RELIABLE/VOLATILE，兼容。离线直接读取序列化 bag，不进行 DDS QoS 匹配 |
| Livox 类型 | lidar_type=1；日志中“AVIA”是上游分支名称，不是实物识别结果 |
| scan_line | 4；driver 对 Mid360S 使用 kLineNumberMid360=4，但本分支的 line/tag 条件被上游注释，不能声称正在按该参数过滤 |
| 抽样与过滤 | point_filter_num=4、blind=0.1 m；LiDAR 系 z 范围 −20..20 m，保留地面和高差；这是宽 ROI，非实测地面/障碍物界限。range_min/max 不参与当前 Livox 分支 |
| 地图精度参数 | scan/map/ivox=0.2 m；点数不足时源码可能回退到 0.1 m。最终 global map 默认 0.1 m，定位分块也默认 0.1 m，不把运行时0.2和保存0.1混为一谈 |
| 回环初值 | 每隔 >5 关键帧检查，历史候选至少相隔20帧，候选范围5 m，NDT阈值1.3；需闭环数据验证，较短轨迹可能尚无合格回环候选 |
| 6DoF | loop_closing.with_height=false；imu_filter=false；force_2d=false。proj_kfs=false，不过调用本身在该提交中已注释，不把它当成高度约束开关 |
| 二维 | with_g2p5=false；地面、安装高度、障碍物上下界保留 null，误开启会被入口拒绝，不以默认值假装已标定 |
| 输出 | 每次 `maps/lightning-offline-*` 或 `lightning-online-*`，工作目录内 `data/new_map/`；不依赖 system.map_path 控制 SLAM SaveMap |

在线输入接口见 [slam.cc:94](/home/lab307/ranger_nav/ws_slam/src/lightning/src/core/system/slam.cc:94)，真实发布者见 [graph_before.json](../bags/mid360-static-qg8qtyiv/graph_before.json)。4线定义见 [comm.h:82](/home/lab307/ranger_nav/ws_livox/src/livox_ros_driver2/src/comm/comm.h:82)，Mid360S 分派见 [pub_handler.cpp:149](/home/lab307/ranger_nav/ws_livox/src/livox_ros_driver2/src/comm/pub_handler.cpp:149)。保存分辨率见 [laser_mapping.h:116](/home/lab307/ranger_nav/ws_slam/src/lightning/src/core/lio/laser_mapping.h:116)、[tiled_map.h:53](/home/lab307/ranger_nav/ws_slam/src/lightning/src/core/maps/tiled_map.h:53)。

### 时间、IMU 与外参方向

- Livox 使用 `offset_time / 1,000,000`，即 ns→ms：[pointcloud_preprocess.cc:68](/home/lab307/ranger_nav/ws_slam/src/lightning/src/core/lio/pointcloud_preprocess.cc:68)。`time_scale` 在 Velodyne 分支乘点时间：[同文件:225](/home/lab307/ranger_nav/ws_slam/src/lightning/src/core/lio/pointcloud_preprocess.cc:225)。本轮用同一消息分别设置 0.001 和 1，仅作代码路径验证，所得 Livox 时间完全一致；没有试调它解决动态误差。
- 去畸变前按点时间排序，随后 ms/1000 转秒：[imu_processing.hpp:264](/home/lab307/ranger_nav/ws_slam/src/lightning/src/core/lio/imu_processing.hpp:264)。排序能处理点序无序，不证明 host-receive 时间与实际采样时钟已同步。
- IMU 初始化依据加速度模长区间识别 g 或 m/s²：0.5–1.5 时乘重力常数，7–12 时保持 SI；本静止包模长约0.995659，符合 g 分支。原始 IMU orientation 未参与该估计。[imu_processing.hpp:334](/home/lab307/ranger_nav/ws_slam/src/lightning/src/core/lio/imu_processing.hpp:334)
- **A：IMU–LiDAR**：厂家 Mid-360S 手册名义值是 `p_L = p_I + [0.011,0.02329,-0.04412] m`，轴同向。Lightning 需要 `p_I = R_IL*p_L+t_IL`，故采用 **R=I，t=[−0.011,−0.02329,+0.04412] m**；并非车体安装高度。手册证据与方向解释见 [阶段3外参记录](03_mid360_validation.md)、[名义值与待测项](../config/mid360/extrinsics_pending.yaml)；实际运算见 [imu_processing.hpp:299](/home/lab307/ranger_nav/ws_slam/src/lightning/src/core/lio/imu_processing.hpp:299)。
- **B：整颗雷达相对 base_link**：xyz/rpy 待测，不发布虚构 TF。
- raw 点云和 IMU 的 header 都叫 `livox_frame`，但它们分别对应厂家 LiDAR/IMU 原点，不能据相同字符串忽略外参。内部状态/关键帧位姿是 `T_WI`；W 起始轴向取初始 IMU，重力作为向量估计，并未自动保证地图 z 轴等于现场铅直方向。
- `with_height=true` 会添加测量值 **0** 的高度先验，把关键帧 z 拉向零，因此关闭以保留坡道/多层高差。[loop_closing.cc:277](/home/lab307/ranger_nav/ws_slam/src/lightning/src/core/loop_closing/loop_closing.cc:277)
- 当前 **SlamSystem 没有 TF 广播器**，也不发布底盘控制。`system.pub_tf` YAML 在此固定提交未被读取；定位类另有默认 true 的 C++ `pub_tf_` 选项，不能声称 YAML=false 已关闭定位 TF。本阶段不启动定位。[loc_system.h:28](/home/lab307/ranger_nav/ws_slam/src/lightning/src/core/system/loc_system.h:28)、[loc_system.cc:60](/home/lab307/ranger_nav/ws_slam/src/lightning/src/core/system/loc_system.cc:60)

## 3. 真实静止包验收范围

输入：`bags/mid360-static-qg8qtyiv/raw/`，包含 SQLite3 数据库和 metadata.yaml，约29.8049秒。

| 实测项 | 结果 |
|---|---|
| CustomMsg / IMU | 298 / 5962，全6260条成功读取 |
| 原始点总数 | 5,959,776 |
| 专用配置对应过滤后的点总数 | 1,096,607（抽样4、blind0.1、z±20） |
| header 非递增 | 0 |
| 原始 offset 回退 | 39,103 次，保留；阶段3记录298/298帧存在 |
| 保留点时间最大值 | 100.959016 ms；这是过滤后保留点的最大值，不代替原始帧统计 |
| 原始加速度模长均值 | 0.99565917968963 g |
| 改变 time_scale 是否影响 Livox 时间 | 否，实际比较通过 |
| 原包前后 SHA256 | 完全一致 |

读取使用与 SLAM 相同动态库的 RosbagIO 和 PointCloudPreprocess；并执行非单位旋转/平移、原点云不修改、近点过滤、空点云的断言。[检查器源码](../tests/slam_reader/check_lightning_bag.cc)、[真实统计](04_evidence/static_reader.json)、[READ_ONLY日志](04_evidence/static_reader.log)。SQLite 完整性/metadata计数相符的既有真实结果见 [阶段3 validation.json](../bags/mid360-static-qg8qtyiv/validation.json)；运行入口还会在每个运动包建图前重新做数据库完整性和计数核对。

数据库 SHA256：`73115bef4c5a7975425db659fda9c780c7d84aaf44bb6ca3f1be0c8ad04fd027`；metadata SHA256：`81f199b448716f0161d77947ae0bac4bbf8cf16d5149553922252a0afed9b451`。

阶段3的 time_type=0 主机接收时钟、最大0.468837ms帧内回退及原检查退出2没有被追溯改成通过。当前只确认软件可读取、排序逻辑存在；动态去畸变误差必须用运动数据审查。

## 4. 手持诊断闭环包与两次离线运行（不通过正式建图验收）

2026-09-22 录得 `bags/mid360-manual-loop-4uwmxbzb/raw/`：约 49.98 s，500 帧 CustomMsg + 9996 条 IMU（≈10/200 Hz），9,999,072 原始点；发布者 `/mid360s_driver`（RELIABLE/VOLATILE），无底盘状态/TF（当时无发布者，未启动底盘）。**用户随后关键更正：该包为手持雷达走动录制，雷达并未牢固安装在 Ranger Mini V2 车体上。** 因此其两个离线实验只用于数据链路与算法管线诊断，**明确不通过正式建图验收**；车体三维地图、base_link 安装外参、绝对尺度、地面/坡道、g2p5、在线模式与导航均不得引用其结论。两个实验 manifest 中的 motion_data_note（“人工遥控录制”）表述不准确，作为原始运行记录保留不改写；两个实验目录的 acceptance.json 当时已更新为本结论。**2026-09-23：用户自行删除这两个实验目录（确认不再需要），下表数字为删除前留档；对应 bag 仍保留。**

诊断范围内的事实：

| 项目 | 结果 |
|---|---|
| 新雷达驱动（.101，type=35，SN ARMCP4E0033801） | 退出 0（约 301 s，SIGINT 收尾）；点云/IMU time_type=0；[driver-2v7kj2bx](03_evidence/driver-2v7kj2bx/) |
| 输入检查 | PARTIAL：header 无倒退/重复；500/500 帧逐点 offset 回退（64,701 次），time_type=0 动态时间质量问题在新设备未消失；[validation-stage4.json](../bags/mid360-manual-loop-4uwmxbzb/validation-stage4.json) |
| 离线运行 ×2（`lightning-offline-t_4fi98t`、`lightning-offline-7dwk87rr`，两目录已由用户删除） | 均 exit 0、map_check exit 0、35 关键帧、结构 PASS；global 点数 22,634 / 21,964 |
| 同包重复性（诊断性） | 对称最近邻中位数 0.0216 m、p95 0.060 m、max 0.285 m（comparison.json 已随目录删除，数字留档；REVIEW_REQUIRED） |
| 轨迹几何 | 优化轨迹长约 5.417/5.411 m；起止平移差约 0.098/0.096 m，但航向差约 174°/173°，不构成位姿闭合；轨迹 z 跨度约 0.44/0.46 m（手持携带高度变化所致） |
| IMU/时间行为 | 预处理保留 1,848,931 点；加速度模长均值 0.9977 g（g 检测分支一致）；运动中 acc 0.19–1.88 g、gyro 至 3.8 rad/s；time_scale 不作用，与静止包行为一致 |

以上数字只证明“新雷达链路可用、离线管线端到端可跑、同包重复性尚可”；cloud z 中位数约 1.64 m 与手持携带高度相符，进一步印证非车体安装。墙体重影、闭环错配、地面/尺度审查未做也不做——数据不合格，审查留给正式包。两实验目录内 manifest、config/extrinsics 快照、run.log、geometric_review、comparison 与全部图件保留原状。

## 5. 正式车体闭环包两次离线运行（三维已验收）

2026-09-23 用户确认 Mid-360S 已牢固安装回 Ranger Mini V2 后录制 `bags/mid360-manual-loop-n2cgpiz0/raw/`：约 95 s，950 帧 CustomMsg + 18995 条 IMU（≈10/200 Hz），18,997,056 原始点；录包 Ctrl-C 正常收尾、退出 0。输入检查退出 0：header 无倒退，预处理保留 4,303,005 点；**逐点 offset 回退 122,536 次（time_type=0 未变）**。路线细节、起止静止时长、参考距离与安装 xyz/rpy 用户尚未提供，acceptance 保持 PENDING。

| 项 | 第一次 `lightning-offline-8k_qnpae` | 同包重建 `lightning-offline-k2i9n8qw` |
|---|---|---|
| 运行/结构检查 | exit 0 / PASS | exit 0 / PASS |
| 关键帧 / global 点数 | 49 / 33,936 | 49 / 27,340 |
| 优化轨迹长 | 9.077 m | 9.143 m |
| 起止平移差 / 航向差 | 0.452 m / −4.27° | 0.487 m / −4.73° |
| 轨迹 z 跨度 | 0.117 m | 0.057 m |
| 回环最大修正 | 0.167 m | 0.044 m |

同包比对（`check_slam_map.py --compare`）：双向最近邻中位数 0.0439 m、p95 0.1192 m、max 0.6415 m，REVIEW_REQUIRED；max 主要来自远端离群点。**已知离群点 2 个**（水平距离 >30 m，轨迹仅 ~9 m 环）：(65.5, −5.3, 15.2) 与 (61.2, −13.5, 8.7)；前者即单点分块 `1.pcd`，主体地图在 `0.pcd`（33,933 点，x −4.3…21.1、y −14.4…11.2 m）。两次运行点数差（33,936 vs 27,340）主要来自离群/边缘点在体素降采样下的运行间抖动，主体结构一致（中位数 0.044 m）。

每实验目录有 `geometric_review.json` 与 4 张审查图（`map_review_xy/3d/slices/height_histogram.png`，画法见 `scripts/analyze_slam_experiment.py`）；`8k_qnpae` 另有查看辅助 `map_review_cloud_with_trajectory.pcd`（global 点 + 轨迹线 intensity=−1 + 朝向箭头 intensity=−2，供查看器按 intensity 着色，不参与验收）。地图坐标系：原点=起始静止时 IMU 位姿，z=0 为雷达安装高度、非地面。

**验收结论（2026-09-23）**：① 用户对四张审查图的人工结论——**合格**（重影/闭环/地面/整体错位均无问题，两实验同判）；② 现场量测参考距离——**用户明确免除**（认为算法尺度可信），两个实验 acceptance.json 的 `measured_scale` 如实记为 `WAIVED_BY_USER_NOT_INDEPENDENTLY_VERIFIED`，绝对尺度未与外部量测比对的缺口保留、不写成 PASS；③ 安装 xyz/rpy——用户约值：**车顶中间、距车尾约 0.25 m、离地约 0.60 m、装平**（约值；base_link 原点定义未核对，不换算成虚构 TF），已记入 `config/mid360/extrinsics_pending.yaml` 与两实验 acceptance.json。两个实验 `three_dimensional=PASS`；查看辅助：`8k_qnpae/map_review_xy_full_extent.png`（全范围未裁剪总览）。

### 5.1 g2p5 二维离线验证（2026-09-23）

配置 [ranger_mid360s_g2p5.yaml](../config/slam/ranger_mid360s_g2p5.yaml)：与已验收三维配置唯一差异是 `system.with_g2p5=true` 与 g2p5 实测值。**参数来源均为用户 2026-09-23 输入**：障碍物高度带离地 0.10–0.80 m；雷达离地约 0.60 m、装平（固定地面平面模型）。因关键帧点云为 IMU 系（阶段4补丁#5）、IMU 原点低于 LiDAR 原点 44.12 mm（手册 t_LI z=−0.04412），`lidar_height=+0.556`、`floor_height=−0.556`（0.60−0.04412）。入口 `--accepted-offline maps/lightning-offline-8k_qnpae` 通过三维 PASS 门槛。

运行 `maps/lightning-offline-suej7qsz/`：exit 0、结构检查 PASS、49 关键帧、global 27,321 点（0.pcd 27,318 + 单点离群 1.pcd）。**map.pgm + map.yaml 首次生成**：分辨率 0.05 m、528×560、origin(−5.15,−14.95)、trinary 0.65/0.25；占据 1201 / 自由 10597 / 未知 283882 格。

自动化交叉验证（详见该实验 acceptance.json）：占据格全部位于闭环附近（x −3.58…4.22、y −4.22…5.48 m），与 3D 地图按同障碍带投影的最近距离中位数 0.047 m（约 1 格）；优化轨迹 49 关键帧栅格全部自由；>50 m 两离群点未进栅格。已知边界：3D 带内点投影落格值占据/自由/未知约 605/2899/2691，自由与未知占比高源于逐关键帧固定平面与世界系投影的倾角差及 2.5D miss 射线清空；外框被 miss 射线端点撑大导致 96% 未知；固定地面平面仅平地+装平成立，坡道需 `esti_floor=true` 或重测（后续阶段）。查看辅助：`suej7qsz/map_pgm_view.png`。

**在线模式静止管线验证（2026-09-23，已完成）**：用户现场启动雷达驱动（唯一可靠发布者），运行 `maps/lightning-online-aea6vhyt/`：入口预检通过、车全程静止约 120 秒、exit 0、**保存服务 /lightning/save_map 首次实测成功（response=0）**、结构检查 PASS、产物齐全（单关键帧 2317 点 + 在线 map.pgm/map.yaml）。静止期 LIO 位移约 2 cm，符合预期。此实验验证在线链路，不产生正式地图（正式地图仍为上述离线实验）；在线运动建图与 time_type=0 动态时间质量仍属未验收项。

**下一步（不自动执行）**：阶段 4 建图链路已闭环。定位部署/导航（Nav2）属后续阶段，须用户明确指令后开始；底盘运动许可不跨进程重启（AGENTS.md）。

## 6. 入口、完整输出与复现

脚本默认 dry-run（只读检查器可直接读包）：

- [prepare_lightning_deps.py](../scripts/prepare_lightning_deps.py)：固定 deb/归档校验和项目内依赖复现；已有目录不整体覆盖。
- [build_lightning.sh](../scripts/build_lightning.sh)、[build_lightning_reader.sh](../scripts/build_lightning_reader.sh)：`--execute` 才编译。
- [slam_env.sh](../scripts/slam_env.sh)：只加载 Humble、外部 Livox workspace 和项目内依赖前缀。
- [check_lightning_input.sh](../scripts/check_lightning_input.sh)：完整 bag 目录→同源 C++ 读取器。
- [run_slam_offline.sh](../scripts/run_slam_offline.sh)、[run_slam_online.sh](../scripts/run_slam_online.sh)：共享 [slam_run.py](../scripts/slam_run.py)，唯一输出目录、记录真实命令/当前源码和库校验值、配置/外参/补丁快照。离线要求运动包说明；在线需 `--accepted-offline`，检查已有输入发布者及 QoS，只启动 SLAM，不启动任何驱动。在线保存服务随后正常结束流程尚未实测。
- [record_slam_loop.sh](../scripts/record_slam_loop.sh)：复用阶段3录制器，新增 `--label manual-loop` 唯一目录；不控制运动。默认 static 标签保持原行为。
- [check_slam_map.py](../scripts/check_slam_map.py)：定位分块索引、每块 PCD、global.pcd、6DoF关键帧文件、有限坐标与边界；若有二维图再核对图像、尺寸、YAML分辨率/原点；`--compare` 对同包同配置两次结果给出双向最近邻统计。结构通过不能自动给几何质量 PASS。

将来的每个实验应保留整个目录：`manifest.json`、配置和外参快照、源码/依赖/补丁、`input_reader.*`、`run.log`、`result.json`、`acceptance.json`，以及 **`data/new_map/` 内 index.txt、全部编号分块PCD、global.pcd、keyframes.txt**；二维启用后还须 `map.pgm` 和 `map.yaml`。索引加载会按地图目录定位编号分块，不能只拷 global.pcd。[tiled_map.cc:38](/home/lab307/ranger_nav/ws_slam/src/lightning/src/core/maps/tiled_map.cc:38)

g2p5 当前固定地面模型为当前关键帧点云系中的水平平面，需先核对实际安装倾角与地面位置；关键帧转换修复后这里是 IMU 系。`min_th_floor/max_th_floor` 是相对该平面的高度阈值；`lidar_height` 还参与射线清空处理。未知值不沿用仓库的 −1.2/0.5/1.0。[g2p5.cc:272](/home/lab307/ranger_nav/ws_slam/src/lightning/src/core/g2p5/g2p5.cc:272)、[参数读取:487](/home/lab307/ranger_nav/ws_slam/src/lightning/src/core/g2p5/g2p5.cc:487)

## 7. 命令与失败记录

| 命令/操作 | 退出及说明 |
|---|---|
| 固定克隆、checkout | 0；acquisition.json |
| 固定 apt-get download、项目内 dpkg-deb 解包 | 0；dependencies.json / unwind_runtime.json |
| Pangolin cmake configure/build/install | 均0；pangolin_commands.json及对应log |
| `bash scripts/build_lightning.sh --execute` | 最终0；前两次分别缺miao glog头路径、glog导出指向系统libunwind；失败日志 attempt1/2 完整保留 |
| C++ reader cmake build | 最终0；第一次缺tf2头依赖，reader_build_attempt1.log保留，补依赖后通过 |
| `bash scripts/check_lightning_input.sh .../raw` | 0；真实输入统计与哈希记录完整 |
| 专用配置+合成空包运行 | Init成功；无关键帧/已有目录均按预期拒绝保存，最终退出2、原sentinel完整；save_guard_result.json |
| `python3 tests/test_stage4_tools.py` | 0，3项检查：LZF、完整/缺失分块、dry-run和阶段入口限制 |
| shell语法/Python编译、补丁反向check、依赖dry-run | 0；最终工具检查与patch_check.json |

早期保存拒绝采用未捕获异常，负例退出−6；已改为离线退出2/服务错误响应，重新验证。旧 `save-guards-yxkfchvz` 日志保留，不混作正常建图结果。上游编译仍有未使用变量、成员初始化顺序、符号比较等警告，不把“退出0”描述为无警告。

## 8. 最少人工操作及下一步

正式车体闭环包三维验收与 g2p5 二维离线验证均已完成（第 5、5.1 节）。**验收输入已齐（2026-09-23）**：人工图审合格；尺度核对用户明确免除（如实记 WAIVED，不写 PASS）；安装约值已提供（车顶中间、距车尾约 0.25 m、离地约 0.60 m、装平）。两个三维实验 acceptance.json 均为 `three_dimensional=PASS`；g2p5 二维实验 `lightning-offline-suej7qsz` 的 map.pgm/map.yaml 已生成并验证。

**下一步需要用户做的最少人工操作**：无强制项——阶段 4 建图链路已闭环（三维验收 → g2p5 离线二维 → 在线静止管线验证均完成）。可选后续：① 在线运动建图实测（车动起来的在线模式，当前未验）；② 进入下一阶段（定位部署 / Nav2 导航）——须用户明确指令，助手不自动推进。注意：入口门槛要求 `--accepted-offline` 指向 `three_dimensional` 恰为 `PASS` 的实验（`8k_qnpae`/`k2i9n8qw`；`suej7qsz` 为 `PASS_BY_REFERENCE` 不满足严格校验）。`lightning-offline-t_4fi98t`、`lightning-offline-7dwk87rr`（手持，目录已由用户于 2026-09-23 删除）不作为任何后续阶段的输入依据。

底盘通信通过不等于停车安全通过；阶段2退出后负速度反馈问题仍未解决，自主导航不放行。**二维栅格生成成功也不等于坡道可通行性评估完成。**
