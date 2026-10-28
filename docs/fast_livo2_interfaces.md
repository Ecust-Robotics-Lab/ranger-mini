# 阶段 A 前端接口、时间与后续 TF

更新：2026-10-26，已含在线同步保护修复与目录迁移。适用于锁定前端加 `patches/fast_livo2_stage_a.patch`；不是未修改的上游接口说明。当前同步源码为 `ws_camera/src/{hik_camera_trigger_driver,timeshare_writer}`，标定在 `calibration/`，接手见 [HANDOFF](HANDOFF.md)。

## 当前输入

| 话题 | 消息与参考系 | 时间/处理 |
| --- | --- | --- |
| `/livox/lidar` | PointCloud2，`livox_frame`，LiDAR 原点 | header 为驱动组帧首点时刻；timestamp 字段 FLOAT64 **绝对纳秒**，不是相对秒；不是 CustomMsg |
| `/livox/imu` | Imu，原驱动 header 仍为 `livox_frame`，数据来自内置 IMU | 与点云传感器时间域一致；实际加速度模长约 0.995，驱动直接保留设备 g 值，前端使用静止初始化模长归一化；不能把此原始话题的加速度直接当标准 SI 输入给其他融合节点 |
| `/front/image_sync` | Image，rgb8，1280×1024，`front_optical_frame` | 从共享文件复制扫描戳；外触发曝光 20 ms 的历史配置已找到，但物理相位未证明 |

本机参数为 `config/fast_livo2/ranger_mid360s_hik_trial.yaml`。`require_livo=true`，LiDAR/IMU/图像都必须打开；启动时拒绝关闭任一输入的静默退化。`lidar_type=100` 是本补丁自己的 Mid-360S PointCloud2 路径，不是上游通用枚举。

点云解析按字段偏移和 row_step 读取未对齐的 26 字节布局，检查类型/长度/字节序，剔除不合法时间、非有限点与无效 tag，再按点时间排序。算法内部 curvature 为相对毫秒；不生成 ring、不伪造点时间。当前传入点下采样因子 3，具体比例与算法噪声是离线初始参数，未以运动数据调优。

输入订阅均 RELIABLE/VOLATILE，depth 分别 LiDAR 20、IMU 1000、图像 10。实测录包提供 RELIABLE，DDS 冒烟通过；不声称兼容任意 BEST_EFFORT 发布者。内部未消费队列上限分别 50/2000/30；重复/回退戳、超过 0.5 s 的传感器时间断层或队列耗尽会使会话失败退出，要求新会话，禁止沿用旧滤波状态自动继续。此阶段还没有完整数据故障健康消息及自动恢复状态机。

## 时间含义与回放

当前 driver `pub_handler.cpp` 用 packet timestamp 加逐点间隔，组帧 `base_time` 来自首点，`lddc.cpp` 的 PointCloud2 timestamp 直接写绝对纳秒。SDK 的同步类型进入原设备时间戳路径；无同步类型走主机接收时间。因此不能把一份历史日志的 time_type 当此次 bag 的硬件同步类型，bag 本身没有保存 time_type。

实际同步包图像/点云 834/834 header 精确相等、16,679,808 个点时间无负偏移/回退；这只能证明现有数据的写戳与消息配对。`timeshare_writer` 写到的是**刚组帧完成的扫描起始戳**；图像复制“当前共享值”，不携带扫描硬件序号映射。需要触发沿、扫描封包周期与曝光参考时刻的动态/硬件证据才能确认物理对应关系。

上述共享文件风险已在在线测试前修复：writer 不再在加锁前 O_TRUNC；reader/writer 配套使用奇数写入、偶数提交的 v2 协议，重复 writer 拒绝且不截断活动文件。原在线离线测试 5859 次快照、迁移后测试 5736 次快照通过，见 [在线报告](fast_livo2_online_report.md) 与 [整合报告](consolidation_report.md)。这修复的是共享文件一致性，曝光与扫描的物理对应仍未证明；不据此推定原 bag 必然正确或错配。

前端 LIVO 在 `img_time + exposure_time_init` 截断点云和 IMU，先 LIO 再 VIO；同 header 的图像通常对应**之前一个扫描区间的末端**。这种切分不能用于推导自动加/减 100 ms。本配置所有时间偏移和 exposure_time_init 均为 **0 的试验假设**，不宣称曝光中心或起点已经确定。

原 bag 的存储记录时间约为 2026 年，而传感器 header 约为 2020 年。直接 `ros2 bag play 原包 --clock` 的 /clock 跟随存储时间，与前端位姿 stamp 不在一个历元。提供两种验证：

1. `run_fast_livo2_replay.sh`：原包按存储顺序调用真实前端回调/同步/估计；保留原消息，时间指标使用 header。这验证算法输入路径，不能代替 DDS 测试。
2. `make_replay_view.py`：创建独立 bag，只将**新 bag 存储时间**设为原消息 header，原 header、逐点时间和序列化消息逐字节不改。逐话题 payload SHA256 与新文件独立读取对照通过。`dds_smoke.py` 实际用 `ros2 bag play --clock 100 --rate 1` 验证传输与时钟，原包未改写。这是回放调度视图，不是物理同步补偿。

## 专用输出

| 话题 | 内容 |
| --- | --- |
| `/fast_livo/odometry` | Odometry，parent=`fast_livo_local`，child=`fast_livo_imu`，完整 **T_local_imu(t)**；VIO 后的状态 |
| `/fast_livo/deskewed_imu` | PointCloud2，frame=`fast_livo_imu`；一段图像切分区间的去畸变全视野点云，在 **t 时刻 IMU 机体系**；强制与上面 odometry 完全同戳 |

原 `Process2` 在下一 VIO 步会清空临时去畸变云，所以补丁在 LIO 完成时保存该区间的 LiDAR 本体系云，VIO 后按 `T_imu_lidar` 转为 IMU 系，与同刻后视觉状态一起输出。不是从累积彩色世界云反推，也没有对世界点重复变换。

内部计算仍经过 double 秒，原始整数 header 到输出可能有亚微秒舍入；不声称输出与原 header 逐纳秒相等。新输出两话题内部完全同戳。后续配对以这对输入为准，并校验参考点/坐标系。

点云对外保证 x/y/z/intensity；当前序列化还保留 PCL normal/curvature 字段供调试，它们不是新的原始扫描时间接口。该云已统一到参考时刻，不应再次按 curvature 去畸变。

上游 `/cloud_registered` 是前端世界系且受 pub_scan_num、颜色/FOV 与缓存影响；`/aft_mapped_to_init` 在 LIO 更新后即发送、早于 VIO。`/path`、MAVROS 等为保留的上游接口，**不是本项目后端输入契约**。禁止仅将这些旧话题重映射到 PGO 的 body_cloud/odom 就认为接口成立。

## 外参与全局 TF

记 `T_a_b` 为将 b 中的点变换到 a。`Rcl/Pcl` 是 **T_camera_lidar**；`extrinsic_R/T` 是 **T_imu_lidar**，名义平移 `[-0.011,-0.02329,0.04412] m` 沿用现有 Mid-360S 手册方向，不是车体高度。相机参数保留本机真实 k1/k2/p1/p2/**k3**；原标定仍 PARTIAL。归档 Rcl 仅六位小数，运行时只对微小数值舍入作 SO(3) 投影，原文件不改；大于 1e-5 的正交误差拒绝。

本轮前端 `publish.native_tf=false`；DDS 图验证没有 `/tf` 发布者。没有向 `/localization/estimate` 发送虚构 VALID，也没有修改生产 runtime 的 Lightning 选择。

后续独立后端应先产生 T_map_imu(t)，由适配器使用安装外参得到：

```text
T_map_base(t) = T_map_imu(t) × inverse(T_base_imu)
T_map_odom(t) = T_map_base(t) × inverse(T_odom_base(t))
```

公共 LocalizationBridge 继续按原采样时刻插值 `/wheel/odom`，拥有唯一 map→odom 和 odom→base_link；物理静态链仍由既有受管会话持有。FAST-LIVO2 是连续局部里程计提供者，**其局部原点不自动等于 wheel odom**。不要同时让参考 PGO/localizer 广播 map→odom/map→base_link。现有 T_base_imu 仍是近似值，不能因相机–雷达标定存在就认为车体安装标定也已完成。

阶段 A 的成对云 rosbag 与 IMU TUM 轨迹只是前端基线资产。Lightning 原生地图、liangheming map/patches/poses、OpenFleX SC 二进制与 HoloAgent session 格式没有已实现的互转。阶段 A 尚无优化归档/保存/加载服务；B 后来实现了几何图快照导出，C1 加载/定位仍未实现。

## 阶段 B 复核（2026-10-26）

B 已沿 `sync_packages → Process2/UndistortPcl → processImu → handleVIO → publishBackendFrame` 复核上述语义，没有修改阶段 A 前端接口或补丁。完整静态输出只读解码仍为 829 对整数同戳；B 短 DDS 链路为 55 对前端输出、55 个后端有效采样，无公共 TF。当前 gravity_align_en=false，输出缺少显式 reset/session；B 使用 DDS GID 与 ROS 时钟代际，并禁止假定重力对齐。若以后增加同进程前端 reset，必须配套明确会话信号。详见 [B 契约](fast_livo2_backend_interfaces.md) 和 [B 报告](fast_livo2_stage_b_report.md)。
