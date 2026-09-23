# 阶段 3：Mid-360S 驱动、网络、时间戳与静止录包验证

日期：2026-09-15，Asia/Shanghai。**阶段 PARTIAL**。实物由用户更正为 **Mid-360S**，随后设备协议确认 type=35、SN=`ARMCP870038745`。历史 Mid-360 名称/配置不能作为本机型号证据。

## 结论与验收范围

| 项目 | 结果 | 证据与边界 |
|---|---|---|
| 采集时网络与设备识别 | PASS | enp2s0，主机 192.168.1.50/24；雷达 192.168.1.145，MAC ec:72:f7:da:cf:c8；实际协议 type=35、上述完整 SN；采集后载波丢失，见下文 |
| 固定源码与独立构建 | PASS | SDK2 + ROS2 driver 均构建成功；编译警告保留，没有运行 SDK 样例/升级程序 |
| 点云和 IMU 原始收流 | PASS | 298 帧 CustomMsg，5962 条 IMU，约 10/200 Hz |
| SQLite3 录包、读取和计数一致性 | PASS | 29.804909820 s，6260 条全部反序列化，数据库完整性 ok，与 metadata 一致 |
| header 时间戳 | PASS（本次窗口） | 两话题均无倒退/重复，无跨时钟域数量级错误 |
| 帧内逐点时间、硬件同步 | PARTIAL | time_type=0；298/298 帧存在逐点时间回退，尚不能作为运动去畸变时间基准验收 |
| 静止 IMU 合理性 | PASS（读数范围） | 加速度约 0.996 g、平均角速度接近零；没有有效 orientation，未验收姿态/轴向标定 |
| 遮挡、安装外参 | PARTIAL | 用户确认固定、窗口无遮挡；统计无完全空白方位区，但不能排除局部支架/车体遮挡；base_link 外参未知 |
| 网络持久化配置 | PASS（当前内核） | 索引权限问题已修复并复核，模块按名解析/签名、modules-load、autoconnect=yes 均通过；实际重启未测试 |
| 底盘状态、真实安装 TF | 本轮不可用 | can0 DOWN，无底盘状态发布者；未启动底盘，未编造 TF |

**未进入 SLAM、定位或导航；底盘阶段 2 停车安全仍未通过。** 本轮只启动雷达驱动，最终正常退出 0。未发送 CAN、速度或导航目标。

主要证据：

- [成功驱动日志](03_evidence/driver-4l6q_kwk/driver.log)、[退出结果](03_evidence/driver-4l6q_kwk/exit.json)。
- [完整 rosbag 目录](../bags/mid360-static-qg8qtyiv/raw/)、[metadata.yaml](../bags/mid360-static-qg8qtyiv/raw/metadata.yaml)。不要只复制 db3 而丢失 metadata。
- [完整读取验证](../bags/mid360-static-qg8qtyiv/validation.json)、[逐点时间审查](../bags/mid360-static-qg8qtyiv/point_timing_review.json)、[发布者/QoS 快照](../bags/mid360-static-qg8qtyiv/graph_before.json)。
- [ros2 bag info](03_evidence/rosbag_info.txt)、[录包退出结果](../bags/mid360-static-qg8qtyiv/record_result.json)、[录包原始日志](../bags/mid360-static-qg8qtyiv/record.log)。

## 1. 硬件、供电与安装

已实际读取 [Mid-360S 官方下载页](https://www.livoxtech.com/mid-360s/downloads) 对应 2026-06-01 用户手册；[本地 PDF](03_evidence/upstream/Mid360S_User_Manual_EN.pdf)、[可搜索文字](03_evidence/upstream/Mid360S_User_Manual_EN.txt)。不是用 Mid-360 手册代替 S 型号。

用户本次确认：**12 V、原厂一分三线、固定牢固、窗口无遮挡、采集期间可静止**，记录在 [user_confirmations.json](03_evidence/user_confirmations.json)。尚需人工核对的事项：

- 手册 External Power Supply：工作范围 9–27 V、推荐 12 V；常温启动有约 18 W 峰值。电源额定能力、实际带载电压及供电瞬态尚未测量。
- 手册 Connection：原厂电源线红正、黑负；RJ45 只接数据网络，**不接 PoE**。实际正负极接法、裸线绝缘、应力释放和接头锁紧待人工复核。未指导自制针脚接线，也未假设普通网线供电。
- 手册 Mounting：底部四个 M3、孔深 5 mm；核对有效螺纹啮合、不得顶底。留散热空间，避免玻璃/支架/线缆遮挡。散热底板与固定细节未量测；原厂调试分线不等于户外连接防护已验收。
- IMU 与点云坐标轴方向按 S 型号手册 Coordinates 图核对；静止均值不能验证全部三轴方向、比例或安装角。

本机 `systemd-detect-virt` 输出 `none`（退出 1 是未检测到虚拟化），原生 Ubuntu 22.04.5 / 6.8.0-40-generic。VMware 条件不成立，本阶段不修改宿主机桥接、USB 直通或网络。

## 2. 网络现状、备份及回滚

[开始快照](03_evidence/host_network_before_20260915T201813.json) 表明：本次开始时 enp2s0 不存在、YT6801 未加载；原 NM 配置仍存在但未激活。can0 DOWN，不能沿用昨天的 UP 结论。

用户执行恢复后，[本次网络证据](03_evidence/network_restored.json)：

- enp2s0 `192.168.1.50/24`，100 Mbit/s、full duplex、carrier=1。
- 到 `192.168.1.145` 经 enp2s0；邻居 MAC 与历史记录一致。先读路由/邻居，再只对这个已知地址 ping，3/3 响应；未扫描公司网络。
- SDK 的发现 socket 绑定主机雷达地址；`ip route get 255.255.255.255 from 192.168.1.50` 实测走 enp2s0。没有遍历 Wi-Fi 子网。
- 默认路由仍 `via 192.168.3.1 dev wlo1 metric 600`。雷达连接无 gateway，`ipv4.never-default=yes`，IPv6 disabled。
- 没有关闭/修改防火墙。`/etc/ufw/ufw.conf` 可读值为 ENABLED=no；本助手的 `sudo -n ufw status verbose` 因需要本机认证退出 1，因此不声称已完整审计运行中的 nftables/iptables 规则。实际 UDP 收流已通过。

### 恢复中发现并修复的脚本错误

第一轮 `restore_mid360_network.sh --execute` 退出 1：将 `/sys/class/net/enp2s0/device` 的解析后路径与 `/sys/bus/pci/devices/...` 符号路径直接比较，错误拒绝了正确网卡。已改为两端 `readlink -f`，在实际设备上比较通过；第二轮用户执行退出 0。保留失败，未声称一次通过。

Root-only 完整 NetworkManager 配置备份（可能包含其他连接秘密，不复制进项目）：

- 第一次：`/var/lib/ranger-nav-yt6801/stage3-restore-qo1CoVSt/`。
- 恢复成功：`/var/lib/ranger-nav-yt6801/stage3-restore-V2tsu0kS/`，含 `NetworkManager-system-connections.tar`、原地址/路由/profile、日志及 rollback.sh。
- 临时恢复回滚：`sudo bash /var/lib/ranger-nav-yt6801/stage3-restore-V2tsu0kS/rollback.sh`。先停止雷达驱动；该轮开始时模块已加载，回滚保留它，不删除原连接配置。

### 用户追加：重启自动恢复

已提供 [persist_mid360_network.sh](../scripts/persist_mid360_network.sh)。默认 dry-run；执行时先调用修复后的恢复脚本，再备份原设置，安装**已有签名模块**到当前内核 `/lib/modules/6.8.0-40-generic/updates/ranger-nav/yt6801.ko`，运行 depmod，建立 modules-load 配置，把原 UUID 的 autoconnect 设 yes。每次生成独立备份和回滚脚本；不重复 MOK、不重启、不改 Wi-Fi/CAN/防火墙。

用户已执行下列命令，退出 0；安装的模块文件可直接读取，version=1.0.34、签名正确，modules-load 内容为 yt6801，autoconnect=yes、never-default=yes、gateway 为空，Secure Boot 仍开启。首次复核证据：[persistence_verified.json](03_evidence/persistence_verified.json)。但普通用户按模块名查询失败，后续查明公共 modules.dep/modules.alias 索引因脚本 umask 077 变为 0600。**不能把 sudo 脚本退出 0 当作完整验收。** 下列命令仅保留为已执行记录：

```bash
sudo bash /home/lab307/ranger_nav/scripts/persist_mid360_network.sh --execute
```

修复已落入持久化脚本：保密备份仍用 077，只有 depmod 子进程用 022。在 /tmp 独立模块树复现 0600，并验证重建可恢复 0644，见 [depmod_permission_regression.json](03_evidence/depmod_permission_regression.json)。用户已执行一次最小修复 `sudo bash -c 'umask 022; depmod -a 6.8.0-40-generic'`。本助手随后实测索引恢复 0644，普通用户 `modinfo -n yt6801` 正确解析已安装文件，签名/version 查询和 modprobe dry-run 均退出 0；autoconnect=yes、never-default=yes、gateway 为空。证据：[persistence_final_verified.json](03_evidence/persistence_final_verified.json)。**当前内核的自动恢复配置已完成，无需重复手工加载；实际重启验证仍待验。**

**采集后网络变化**：20:30:38 NetworkManager 将 enp2s0 从 activated 变为 unavailable，reason=carrier-changed，当前 NO-CARRIER、无 IPv4 地址；模块仍加载。是否人为断电/拔线待用户确认，不能归因于持久化命令。静止包在 20:27:15 已结束，不受此变化影响。

本次持久化备份：`/var/lib/ranger-nav-yt6801/stage3-persist-FxqdJUHF/`；回滚为 `sudo bash /var/lib/ranger-nav-yt6801/stage3-persist-FxqdJUHF/rollback.sh`，恢复原 autoconnect 与原模块/加载配置，仅影响持久配置，不强制卸载正在使用的模块。此次恢复前置备份为 stage3-restore-qGbDPqL5。

仅适用于当前内核；未来升级内核需重新匹配/构建签名模块，不能声称已有 DKMS。真正重启后的恢复仍需独立验收，不自动重启本机。

## 3. 源码固定、构建与配置

三仓库来源为各自官方 master，均固定 detached HEAD；完整记录 [source_versions.json](03_evidence/source_versions.json)、[获取命令结果](03_evidence/upstream/acquisition.json)。

| 仓库 | 完整 SHA | 本地修改 |
|---|---|---|
| Livox-SDK2 | `08f523c930b2f0ba1e98a6afaa8d7476bf479908` | 无；构建/安装在 ws_livox/sdk_build、sdk_install |
| livox_ros_driver2 | `4a1def929e5b59c7a8122d19fce6efba581ce9f7` | 两处诊断日志：设备型号/SN/IP，以及点云/IMU time_type 和首个协议时间；未修改点云/IMU 数值或时间算法 |
| lightning-lm | `1325fed8fa97a2506878360fc23427ea00a767da` | 无，仅在 vendor/livox/lightning-lm-review 审查；未构建、未建 ws_slam |

[诊断累计补丁](../patches/livox_driver_stage3_diagnostics.patch)。ROS package.xml 从官方 package_ROS2.xml 原样复制，是上游忽略的生成文件；已记录 hash。底盘两仓库仍保持原 SHA 和已有补丁，本阶段未改动。

审查官方 [build.sh:30](/home/lab307/ranger_nav/ws_livox/src/livox_ros_driver2/build.sh:30) 发现会删除 workspace 的 build/devel/install。因此没有执行它；[项目构建脚本](../scripts/build_livox.sh) 采用 CMake + colcon，不清理现有目录、不 sudo 安装、不修改 bashrc/ROS/Conda。设置 SDK 的项目内链接目录与安装 RPATH，解决初次 ldd 中 SDK 动态库未找到的问题。

实际 `bash scripts/build_livox.sh --execute` 通过；SDK 及 ROS 编译警告保留。初始完整日志 [build-jnMdRFge.log](03_evidence/build-jnMdRFge.log)，后续 [RPATH 重建](03_evidence/rebuild-rpath-console.log)、[诊断重建](03_evidence/rebuild-diagnostics-console.log)，均记录 BUILD_EXIT=0。SDK sample 包含升级/IP 设置工具，仅编译，未执行。

配置副本：[MID360S_config.json](../config/mid360/MID360S_config.json)。正确设备键是 `Mid360s`，SDK 映射见 [parse_cfg_file.cpp:33](/home/lab307/ranger_nav/vendor/livox/Livox-SDK2/sdk_core/parse_cfg_file.cpp:33)，协议 type=35 见 [livox_lidar_def.h:69](/home/lab307/ranger_nav/vendor/livox/Livox-SDK2/include/livox_lidar_def.h:69)。`lidar_summary_info.lidar_type=8` 是驱动内部协议类别位，不是此处设备 type=35，不能简单改成 35。

- 主机数据端口 56101/56201/56301/56401/56501，雷达对应端口 56100/56200/56300/56400/56500；指定雷达 IP 列表，仅目标 192.168.1.145。
- xfer_format=1，publish_freq=10.0，multi_topic=0，use_sim_time=false，frame_id=livox_frame。
- 驱动会配置点格式、扫描模式、安装姿态参数并进入 Normal、启用 IMU；启动不是设备纯只读操作。配置 identity 表示保留原生点坐标，**不表示安装标定完成**，也不是 IMU-LiDAR 外参。
- 单实例文件锁、已运行程序检查和 UDP 端口占用检查阻止重复驱动。曾实际因锁占用拒绝第二进程；旧进程退出后才重新启动。
- 型号更正前两轮没有点云/IMU，启动分别保存于 driver-eobjd61p、driver-239bd2sn；SIGINT 未完成退出，监督器最终 SIGKILL，exit=-9。这两轮不验收。修正后的 driver-4l6q_kwk 正常收流并退出 0。

## 4. Lightning-LM 输入、单位及时间

在线输入：[slam.cc:87](/home/lab307/ranger_nav/vendor/livox/lightning-lm-review/src/core/system/slam.cc:87)、[loc_system.cc:33](/home/lab307/ranger_nav/vendor/livox/lightning-lm-review/src/core/system/loc_system.cc:33)：

- `common.livox_lidar_topic` → `livox_ros_driver2/msg/CustomMsg`；配置为 `/livox/lidar`。
- `common.imu_topic` → `sensor_msgs/msg/Imu`；配置为 `/livox/imu`。
- 两订阅用 `rclcpp::QoS(10)`，best_effort 调用被注释，默认 reliable/volatile。与本次真实发布者 reliable/volatile 匹配。
- 离线程序 Livox 入口硬编码 `/livox/lidar`，所以保留原名，见 [run_slam_offline.cc:68](/home/lab307/ranger_nav/vendor/livox/lightning-lm-review/src/app/run_slam_offline.cc:68)。
- [pointcloud_preprocess.cc:64](/home/lab307/ranger_nav/vendor/livox/lightning-lm-review/src/core/lio/pointcloud_preprocess.cc:64) 将 offset_time 除 1e6 得到毫秒；后续除 1000 换成秒。不要套用其他雷达 time_scale。
- [imu_processing.hpp:334](/home/lab307/ranger_nav/vendor/livox/lightning-lm-review/src/core/lio/imu_processing.hpp:334) 根据初始化加速度模长区分 g 与 SI；这属于选定源码行为，不是本次 SLAM 实测。

### 实际消息统计

录包时间 20:26:45.887828219–20:27:15.692738039，共 29.804909820 s。

| 项目 | /livox/lidar | /livox/imu |
|---|---|---|
| 消息类型 | livox_ros_driver2/msg/CustomMsg | sensor_msgs/msg/Imu |
| 发布者 | /mid360s_driver，唯一 | /mid360s_driver，唯一 |
| 数量 | 298 | 5962 |
| 按 bag 接收时间频率 | 10.000816 Hz | 200.000605 Hz |
| header 频率 | 10.000305 Hz | 199.999037 Hz |
| frame_id | livox_frame | livox_frame |
| header 相邻间隔 | 98.689–101.473 ms | 3.860–6.061 ms |
| header 倒退/重复 | 0/0 | 0/0 |
| 接收时间减 header | 99.921–104.614 ms | 0.039–0.951 ms |

发布 QoS 实测 reliable、volatile；RMW graph/metadata 将 history 报为 unknown、depth=0，**不把 0 解释成真实队列长度**。源码用 queue_size 构造发布者，[lddc.cpp:531](/home/lab307/ranger_nav/ws_livox/src/livox_ros_driver2/src/lddc.cpp:531)。原 graph 快照节点名因字符串拼接写成 `//mid360s_driver`，这只是记录格式问题，脚本现已修复，原快照保留。

### 时间戳与逐点时间异常

[成功驱动日志](03_evidence/driver-4l6q_kwk/driver.log) 实际点云与 IMU `time_type=0`，表示没有 PTP/GPS 同步。主机 `NTPSynchronized=yes` 不能改变这一结论。[pub_handler.cpp:279](/home/lab307/ranger_nav/ws_livox/src/livox_ros_driver2/src/comm/pub_handler.cpp:279) 在此分支丢弃设备时间而取主机 high_resolution_clock；ROS 时间接近系统时钟正是这种实现的表现，不能作为传感器采样同步精度证明。

原始 CustomMsg 保留 `timebase` 与每点 `offset_time`、xyz、reflectivity、tag、line。header=timebase，本包 298 帧均一致；生成过程见 [lddc.cpp:354](/home/lab307/ranger_nav/ws_livox/src/livox_ros_driver2/src/lddc.cpp:354)、[lddc.cpp:384](/home/lab307/ranger_nav/ws_livox/src/livox_ros_driver2/src/lddc.cpp:384)。

- 共 **5,959,776 点**；每帧 19,872–20,160 点。
- offset_time 单位 ns，每帧最小值 0，最大值范围 **98.854–101.210 ms**。
- **298/298 帧出现帧内回退，共 39,103 次**；回退幅度 **11–468,837 ns**，中位数 243,050 ns。
- 回退全在点索引的 96 倍数处，主要正步长 4947 ns；各帧末点仍为该帧最大 offset。
- 推断：这种边界回退与未同步模式下逐 UDP 包取主机接收时间、再按包内间隔插值的实现相符；没有保存连续 UDP 原始采样时间，不能据此完成因果标定。
- Lightning 在去畸变中会排序点时间（[imu_processing.hpp:262](/home/lab307/ranger_nav/vendor/livox/lightning-lm-review/src/core/lio/imu_processing.hpp:262)）；排序不能消除接收抖动或恢复丢失的真实采样时间。**没有通过调阈值、排序原包或重写时间戳把异常改判 PASS。**

下一步可在本阶段继续调查协议包原始时间和时间同步方案；没有安装/启动 PTP 服务、修改系统时间或进入 SLAM。

### IMU 与遮挡

[官方 Mid-360(S) 协议](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html) 定义角速度 rad/s、加速度 g。[lddc.cpp:480](/home/lab307/ranger_nav/ws_livox/src/livox_ros_driver2/src/lddc.cpp:480) 直接转发，**原 `/livox/imu` 的加速度不是 ROS 标准 SI m/s²**。

两消息均沿用上游 livox_frame 标签，不表示 IMU 测量原点与点云原点重合；两者的名义位置差见外参 A。

本次均值：acc=[0.006368, 0.006332, 0.995572] g；gyro=[0.000305, −0.000196, 0.001376] rad/s。加速度模长均值 0.995659 g，范围 0.970996–1.022191 g，符合本次静止读数预期。消息 orientation 全为 [0,0,0,1]、orientation_covariance[0]=0，但源码未提供姿态估计；**此默认四元数不是有效车体姿态**。

用户确认窗口无遮挡。去除 <0.1 m 点后 36 个 10° 方位区均有返回，未见完整空白方位区。约 25.8% 点为全零坐标（1,536,269 点），另有 819 点为非零但距离 <0.1 m；无 NaN/Inf。这些不能直接当作车体遮挡比例或无障碍证明，需结合现场和安装尺寸核对局部遮挡。

## 5. 两类外参严格分开

[extrinsics_pending.yaml](../config/mid360/extrinsics_pending.yaml) 是说明文件，不自动发布 TF。

**A. 雷达与内置 IMU**：本次读取的 Mid-360S 手册 IMU Data 小节给出 IMU 原点在点云坐标下 [11.0, 23.29, −44.12] mm，两坐标轴同向，即 `p_L = R_LI p_I + t_LI`，R_LI=I、t_LI=[0.011,0.02329,−0.04412] m。是厂家名义值，未现场标定。Lightning 使用 `p_I = off_R * p_L + off_t`（[laser_mapping.cc:714](/home/lab307/ranger_nav/vendor/livox/lightning-lm-review/src/core/lio/laser_mapping.cc:714)），直接使用时方向需取逆；此处不自动生成 SLAM 配置。上游 default_livox.yaml 的 [0,0,0.28] 未采用。

**B. 雷达整体相对 base_link**：安装平移 xyz、旋转 roll/pitch/yaw、雷达坐标原点相对支架/车体的定义均待量测，保存 null/NOT_MEASURED。离地高度单独测量，不能填写到 A 中。未填零宣称标定，未发布虚构 base_link→livox_frame。

## 6. 录包、派生格式和复现入口

脚本均默认 dry-run（检查脚本只读已有 bag）。环境准备只加载包路径，不启动底盘：

```bash
cd /home/lab307/ranger_nav
source /opt/ros/humble/setup.bash
source ws_livox/install/local_setup.bash
python3 scripts/start_mid360.py                 # 预览
python3 scripts/record_mid360.py                # 预览
# 已确认供电、固定和静止条件后，雷达启动与录制分别在两个终端执行：
python3 scripts/start_mid360.py --execute --seconds 120
python3 scripts/record_mid360.py --execute --seconds 30
# 离线读取，不播放、不连接雷达；--output 文件必须尚不存在
python3 scripts/check_mid360_bag.py bags/mid360-static-qg8qtyiv/raw
```

录制器写唯一 bags/mid360-static-* 目录，原包在其中 raw/，保存配置、发布者/QoS、命令、日志、退出结果和验证 JSON。静止录制不是固件/运动许可。底盘状态若有发布者会被纳入；本次全部不可用，因此未录，不能写成底盘接口已重验。未来检查含 ranger_msgs 的包时还需 source ws_base/install/local_setup.bash，仅为类型支持。

[bag_qos.yaml](../config/mid360/bag_qos.yaml) 对 `/tf_static` 指定 reliable + transient_local + keep_all。已用 localhost/ROS_DOMAIN_ID=178 隔离的合成测试，先发布一次静态消息，再启动录制器，确实收到保留的 1 条 TF：[tf_static_qos_test.log](03_evidence/tf_static_qos_test.log)。**合成测试不是本车外参 TF 验收**；实际采集图中 `/tf`、`/tf_static` 均无发布者，包中无 TF 是已知缺项。

可选 [mid360_derived.py](../scripts/mid360_derived.py)：`/livox/points_rviz` 为派生 PointCloud2，保留 offset_time 字段；`/livox/imu_si` 将 g 乘 9.80665 并标记 orientation_covariance[0]=-1。原 `/livox/lidar` 和 `/livox/imu` 不改。没有启动第二驱动；派生节点本次仅 dry-run/语法检查，未做实际 RViz 显示验收。

## 7. 关键命令、退出结果与修改文件

| 命令/操作 | 退出与证据 |
|---|---|
| 三官方 git clone、checkout --detach 固定 SHA | 各 0，acquisition.json |
| 官方手册下载、pdftotext | 0，upstream/ 两型号资料分开保留 |
| 恢复脚本第一轮 / 修复后第二轮 | 用户终端 1 / 0，root-only 备份路径见上 |
| build_livox.sh --execute 及必要重建 | 0，BUILD_EXIT=0，有上游编译警告 |
| persist_mid360_network.sh --execute | 用户终端 0；首次复核发现索引 0600，最小修复后普通用户按名查询/签名/权限均通过，persistence_final_verified.json |
| 默认启动/录包/派生脚本，bash -n/AST | 0，没有启动设备 |
| tests/test_mid360_bag.py | 0；合成正常包 PASS、倒退时间包正确 PARTIAL，synthetic_bag_checks.log |
| 错型号启动与录包前检 | 无真实消息；前检退出 1，static_record_console.log；驱动最终 -9 |
| 正确型号静止录制 | ros2 bag record=0；未强制终止，record_result.json |
| 完整实际包检查 | **2**，成功读取但发现逐点时间回退；不是读取损坏，也不能改写成全项通过 |
| ros2 bag info | 0，rosbag_info.txt |
| tests/test_tf_static_recording.py | 0；仅隔离合成环境验证迟加入录制 |
| 正确型号驱动退出 | 0，driver-4l6q_kwk/exit.json |

新增/修改：ws_livox/、vendor/livox/（三固定源码）、config/mid360/、scripts/build_livox.sh、start_mid360.py、record_mid360.py、check_mid360_bag.py、mid360_derived.py、restore_mid360_network.sh、persist_mid360_network.sh、两项 scripts/tests/、诊断补丁、docs/03_evidence/、静止 bag 目录、本报告、STATUS.md、AGENTS.md 型号更正。保留了首次持久化索引权限失败与最终修复证据；没有覆盖地图、旧包、底盘补丁；没有 ROS/系统/CUDA 重装。

[artifact_sha256.json](03_evidence/artifact_sha256.json) 记录配置、脚本、包描述、SDK/驱动动态库、官方手册 hash。修改诊断源码的回滚需先检查当前 diff，再逆向检查累计补丁并重建；不要覆盖现有工作树。保留原 bag 和失败证据。

## 最少人工操作与后续待办

1. 持久化和权限修复已经完成，不再重复执行。下次采集前确认雷达供电/网线；当前无载波是否人为断开尚未说明。实际重启恢复待后续合适窗口验收。
2. 记录电源能力/极性、安装尺寸与方向、局部遮挡；不需要为本次报告再移动底盘。
3. 逐点时间异常与 PTP/设备原始时间方案保持待处理。当前成果是可重复采集和可发现异常的原始数据链路，**尚未放行 SLAM/导航**。
