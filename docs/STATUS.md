# 项目状态

更新时间：2026-09-28。


## 阶段 6：Nav2 平地单点导航集成（当前）

- **PARTIAL：安全链与导航栈配置/离线验证完成；真机导航测试全部未执行——只可表述为"配置/离线测试完成，真机待验"，不写"导航已成功"。** 用户 2026-09-28 授权阶段 6 并给出速度链顺序要求（仲裁/平滑在 CM 之前、CM 最后直通底盘）。
- **安装**：ros-humble navigation2 全套 1.1.20（bringup/collision-monitor/mppi-controller/smac-planner/velocity-smoother/simple-commander）+ twist-mux 4.3.0，参数/插件名全部对照已装版本源码与二进制符号核实（例：MPPI 插件名 Humble 用 `::`、Ackermann 半径参数在 `FollowPath.AckermannConstraints.min_turning_r` 子命名空间、costmap width/height 必须整型、bt_navigator configure 时同时加载两个默认 BT）。
- **速度链**（用户指定顺序）：Nav2(/cmd_vel_nav)+遥控(/teleop/cmd_vel) → twist_mux（遥控 90>Nav2 10；4.3.0 输出话题实测 cmd_vel_out）→ velocity_smoother(OPEN_LOOP) → **ranger_motion_gate（本项目新节点：VOLATILE 粘滞许可/软急停锁存/定位 ready/点云新鲜度/限速/阿克曼运动学）** → collision_monitor → **/base/safe_cmd_vel（唯一发布者=CM，launch 重映射的底盘入口）**。全链 Twist。
- **运动模式约束**：底盘驱动源码证实 |vx|/|wz|<0.4764 切 SPINNING、vy≠0 切横移——门禁运动学强制拦截（先限幅后裁半径，输出不变量 R≥0.55 或 wz=0，单元测试 12/12 含回归用例与全网格扫描 tests/test_stage6_gate_unit.py）；禁倒车三重（MPPI vx_min=0 + smoother min=0 + Smac DUBIN）。
- **防撞输入**：ranger_obstacle_filter（去自身点/限距，空云也发布作新鲜度心跳）→ /livox/obstacles 同时供 CM（高度带 0.10–0.80 base 系=建图障碍带；Stop 多边形 footprint+0.08、严格大于语义 max_points=0 即 1 点停）与局部代价地图；未知区不可行驶（track_unknown+allow_unknown=false）。**门禁点云新鲜度监视 /livox/obstacles（与 CM 同源）——过滤器死亡时门禁立即阻断（审查 major 修复）**。
- **地图**：导航副本 `maps/.../new_map/nav/`（make_nav_map.py 清除建图起点 0.177 m 处 1 个不可能占据像元，原图未动；原版会使起点膨胀致死、Smac 拒规划）。footprint 0.74×0.52（用户确认外包络，PROVISIONAL）。**场地走廊净空紧张**（中轴线距墙 0.04–0.4 m 段存在）：v1 目标点须选开阔区，窄段通行属后续阶段。
- **验证**：tests/test_stage6_safety.py（隔离 domain 跨进程全栈）**37/37 全绿 exit 0**（safety-ych6w74e）+ 门禁单元 12/12；防空转设计（cmd_seen 时间戳截止、零速断言接受静默、T10 活跃零值消息、T9 实时图发布者数判定、T14 get_parameters 服务直读）；历轮 13 个失败证据目录保留。另实测禁用 FootprintApproach（1.1.20 空点云+动态 footprint 误判即刻碰撞，§7.1）。
- **对抗性审查**（6 维度×3 视角工作流）：确认 15 项（5 major：门禁看错云话题、限幅顺序 bug 可触发 SPINNING、--yaw 缺省全零四元数、--cancel 跨进程空操作、--timeout 不覆盖服务器不可用）全部修复；56 个验证代理因账户限流失败，驳回清单经人工复核拾回修复（launch 默认地图、MPPI critics 显式化、daemon 退化改服务直读等）。详见 06 报告 §5.1。
- **运维发现（如实记录）**：`ros2 run` 包装器 SIGINT 后子进程孤儿化（曾致 6 个带许可门禁泄漏发 0.15）；SIGKILL 残留 /dev/shm fastrtps_* 段与幽灵端点污染 DDS 图；调试会话 `ros2 topic pub` 僵尸污染输入话题——清理脚本 scripts/clean_dds_shm.sh，测试护栏已覆盖全部三类模式。
- **PENDING**：真机导航（隔离平坦空旷区，0.15 m/s 起步→缓弯→取消→软障碍→遥控接管→逐步放宽限速；不让人冲入车前测防撞）；footprint 实测；阶段 5 两项 PENDING 不变（独立数据验证、T_base_imu 精测）。
- 阶段 2 停车安全已由用户 2026-09-28 确认通过（见阶段 2 节）；运动许可仍逐次授权、不跨进程重启。

## 阶段 5：地图定位与 Nav2 接口适配（前一阶段，已完成含两项 PENDING）

- **PARTIAL：定位管线与接口适配完成并经离线自测+合成测试；真实冷启动定位与独立数据验证待用户，未放行导航。** 用户 2026-09-23 授权阶段 5；不启动自主导航。
- **源码审计（不虚构接口）**：定位初始化依赖地图功能点（我们地图仅 `start`=建图起点），无任意全局搜索（YawSearch 被注释）；外部初始位姿仅 C++ API，**无 /initialpose/重定位服务**；`Localize()` 成功判定恒真（上游缺陷，置信度门槛失效）→ 桥接自设门槛补救。上游 TF 把传感器位姿硬编码 child=`base_link`（语义错误），补丁更名 `imu_link`。**Lightning 未融合底盘轮速**（ProcessOdomMsg 被注释）；/wheel/odom（odom→base_link 平面里程计，50 Hz，TF 关闭）保持独立观测。
- **适配（新增，最小化）**：[stage5_loc_interface.patch](../patches/stage5_loc_interface.patch)（run_loc_online 加 --init_*/--pub_tf/--tf_child；新增 /lightning/loc_status + /lightning/loc_confidence）；定位配置 [ranger_mid360s_loc.yaml](../config/slam/ranger_mid360s_loc.yaml)（地图=suej7qsz，`save_dyn_when_quit:false` 防 Finish() 回写已验收地图——`has_set_pose_` 上游永不置位）；桥接包 `ws_nav/src/ranger_localization_bridge`（**唯一发布** odom→base_link 与 map→odom=T_map_base×inv(T_odom_base) 同刻同参考点；loc TF 取最新+显式年龄检查；navigation_ready=loc TF 新鲜+status GOOD+conf≥1.0+innovation+odom 新鲜；速度取 /wheel/odom.twist；外参 T_base_imu 为用户约值显式 APPROXIMATE，base_footprint 不引入）。3D/2D 地图同坐标系构造+数值一致（占据格 vs 障碍带投影中位 0.047 m）。
- **验证**：离线定位自测 `docs/05_evidence/loc-offline-6ggxi299/`（同源建图包，exit 0，FP init 成功，**73** 匹配全成功 conf≈3.0，终点与建图轨迹终点一致；**同源自测不作独立依据**）；桥接合成测试 `tests/test_stage5_bridge.py` v2 **11/11**（9 轮 8 轮全绿）。**对抗性审查（15 代理工作流）确认 10 项缺陷已全部修复**：`/navigation_ready` 无看门狗致轮速断流滞留 True（已加 0.2 s 看门狗+`odom_present` 诊断）、innovation yaw 误差未参与判定（原地旋转航向发散不可见，已加 20° 判据）、零四元数输入崩溃（已加防护）、Localization::Finish 空指针（Init 失败路径，已保护+早退）、v1 测试多项空转（jump 检查可被任意满足/TF 仅查存在性/stall 归因错误，破坏性实验证实后 v2 全重写：TF 数值比对、纯门禁归因、相位前沉淀基线、跨话题配对竞态修正、批量排空防测试饥饿）、文档 74→73 匹配数更正。失败记录保留：mfrsk5zz 缺 slam_env 退出 127；桥接首版 quat_rotate bug 由数学测试抓出已修；单进程多节点 DDS 发现在本环境不可靠→测试改跨进程。
- **现场冷启动（2026-09-23 晚，静止部分）实测 PASS**：车摆位偏差 ~0.43 m/−10° 仍 FP 收敛（conf 3.04 @200 Hz、status GOOD）；`/navigation_ready`=true、map→base_link z=−0.53 m 与外参换算一致；用户经 foxglove_bridge 确认二维地图+TF+实时点云可视化正常。插曲与看门狗现场验证（雷达驱动 600 s 到时退出→ready 正确翻 false）见 [05 第 7 节](05_localization_interface.md)。运动短距验证（可选强化）未做。
- **PENDING**：① 独立数据验证（新运动包，先离线后在线）；② T_base_imu 精测（当前近似）。
- 阶段 2 停车安全结论已于 2026-09-28 更新为通过（用户确认，见阶段 2 节顶部）；运动许可仍逐次授权、不跨进程重启。

## 阶段 4：Lightning-LM 编译与离线输入验证（已完成）

- **新雷达已识别并实际收流：Mid-360S，完整 SN `ARMCP4E0033801`，IP `192.168.1.101`，MAC `4c:43:f6:f4:9d:c2`，协议 type=35。** `config/mid360/MID360S_config.json` 已指向 .101；主机 enp2s0=192.168.1.50/24，默认上网路由仍走 Wi-Fi。驱动日志 `docs/03_evidence/driver-2v7kj2bx/`：设置数据类型/扫描模式/姿态、启用 IMU，**退出 0**（运行约 301 秒，SIGINT 正常收尾）。点云与 IMU 均 **time_type=0**（主机接收时间，未 PTP/GPS 同步）；旧设备的逐点 offset 回退问题在新设备上依旧存在。
- **三维验收完成（2026-09-23）：用户人工图审合格；安装约值已提供（车顶中间、距车尾约 0.25 m、离地约 0.60 m、装平）；尺度核对由用户明确免除。** 两个正式实验 acceptance.json 已如实填写：`three_dimensional=PASS`（依据=人工图审+同包重复性）、`measured_scale=WAIVED_BY_USER_NOT_INDEPENDENTLY_VERIFIED`（不写成 PASS）。g2p5 二维配置 `config/slam/ranger_mid360s_g2p5.yaml` 已按用户输入建立（障碍带 0.10–0.80 m、floor/lidar_height=∓0.556 m IMU 系换算、装平固定地面平面），同包离线验证 **完成**：`maps/lightning-offline-suej7qsz/` exit 0、结构 PASS、map.pgm(0.05 m, 528×560)+map.yaml 首次生成；占据格与 3D 障碍带投影中位差 0.047 m、轨迹 49 关键帧全部自由、离群点未进栅格，二维 `PASS_STRUCTURAL_AND_LOCAL_CONSISTENCY`（边界与 caveat 见其实验 acceptance.json）。**在线模式静止管线验证完成（2026-09-23）**：用户现场启动雷达驱动后运行 `maps/lightning-online-aea6vhyt/`——入口预检（唯一可靠发布者）通过、exit 0、**保存服务 /lightning/save_map 首次实测成功（response=0）**、结构 PASS、产物齐全（含在线 map.pgm/map.yaml）；车全程静止，单关键帧 120 秒漂移约 2 cm 符合预期。在线**运动**建图未做（无需，正式地图已由离线产生）。用户已授权阶段4，不自动进入定位部署/导航。本轮 Codex 未启动底盘驱动、未发送 CAN/速度；雷达驱动由用户按采集说明启动。
- **正式车体闭环包与两次离线运行（2026-09-23）**：用户确认 Mid-360S 已牢固安装回 Ranger Mini V2 后录制 `bags/mid360-manual-loop-n2cgpiz0/raw/`（约 95 s，950 帧 CustomMsg + 18995 条 IMU，18,997,056 原始点，录包退出 0）。输入检查退出 0：header 无倒退，预处理保留 4,303,005 点；**time_type=0 逐点 offset 回退仍有 122,536 次，动态时间质量问题未解决**。两次离线运行 `maps/lightning-offline-8k_qnpae/`（首次）与 `maps/lightning-offline-k2i9n8qw/`（同包重建）：均 exit 0、结构检查 PASS、49 关键帧；轨迹长 9.08/9.14 m，起止平移差 0.45/0.49 m，航向差 −4.3°/−4.7°，轨迹 z 跨度 0.117/0.057 m，回环最大修正 0.167/0.044 m；双向最近邻中位数 0.044 m、p95 0.119 m、max 0.642 m（REVIEW_REQUIRED）。已知离群点 2 个：水平距离 >30 m 的 (65.5,−5.3,15.2) 与 (61.2,−13.5,8.7)，后者即单点分块 `1.pcd`；主体地图（`0.pcd`，33,933 点）范围 x −4.3…21.1、y −14.4…11.2 m。每实验已生成 geometric_review.json + 4 张审查图；`8k_qnpae` 另有查看辅助 `map_review_cloud_with_trajectory.pcd`（intensity −1=轨迹线、−2=朝向箭头，不参与验收）。两个实验 acceptance.json 已于 2026-09-23 填写：`three_dimensional=PASS`（依据=用户人工图审合格）、`measured_scale=WAIVED_BY_USER_NOT_INDEPENDENTLY_VERIFIED`（用户明确免除尺度核对，缺口如实保留）；`8k_qnpae` 另增全范围查看辅助 `map_review_xy_full_extent.png`。
- **验收输入记录（2026-09-23 齐备）**：① 人工图审——合格（两实验同判，已写入 acceptance.json）；② 现场参考距离——**用户明确免除**（"核尺度就不用了，算法基本算出来的没问题"），acceptance.json 记为 WAIVED，绝对尺度未独立核对的缺口如实保留；③ 安装 xyz/rpy——用户约值（车顶中间、距车尾约 0.25 m、离地约 0.60 m、装平；base_link 原点定义未核对，不换算成虚构 TF），已记入 `config/mid360/extrinsics_pending.yaml` 与两实验 acceptance.json。g2p5 二维验证：新配置 + 同包新目录离线运行 map.pgm/map.yaml，通过后在线模式最后验证。
- **构建 PASS**：`ws_slam/src/lightning` 固定 `1325fed8fa97a2506878360fc23427ea00a767da`，master 来源、detached HEAD，有可追踪本地补丁。Pangolin 0.9.3 与缺失依赖安装在项目内，最多2个编译任务；最终退出0，有上游警告。没有把关闭UI当成免除Pangolin依赖。
- **外部 Livox 接口 PASS**：使用 ws_livox 已固定的 `livox_ros_driver2`；不生成内置副本。消息字段一致，无内部生成头遮蔽，动态链接无缺失。SDK/driver及阶段3审查副本保留原状态。
- **真实静止包读取 PASS（限兼容性）**：同源 C++ RosbagIO/预处理读取298帧CustomMsg、5962条IMU、5,959,776原始点；预处理保留1,096,607点。header无非递增，原包SHA256不变。静止加速度模长均值0.995659g。
- Livox offset_time按ns→ms，time_scale不作用于此分支；实际比较已证实。去畸变前有按点时间排序，但39103次原始offset回退和time_type=0主机接收时钟问题仍保留，运动时间质量未验收。
- 专用 `config/slam/ranger_mid360s.yaml` 使用厂家名义IMU–LiDAR变换逆向：t_IL=[-0.011,-0.02329,+0.04412]m、R=I。base_link安装外参待测；不发布虚构TF。with_height=false避免z=0先验；保留6DoF。源码SLAM无TF广播器，不能以未被读取的system.pub_tf YAML推断定位TF关闭。
- 局部补丁处理外部接口/构建、Livox预处理边界与距离条件、关键帧点云转IMU系、拒绝覆盖/空地图、6DoF关键帧导出、真实二维分辨率和正常退出；合成检查通过，真实动态效果待验。
- **手持诊断闭环包（不作为建图验收）**：`bags/mid360-manual-loop-4uwmxbzb/raw/`，约 49.98 s，500 帧 CustomMsg + 9996 条 IMU（≈10/200 Hz），9,999,072 原始点；无底盘状态/TF（当时无发布者，未启动底盘）。**用户 2026-09-22 关键更正：该包为手持雷达走动录制，雷达并未牢固安装在 Ranger Mini V2 车体上**；实验 manifest 中“人工遥控录制”的说法不成立，原始 manifest 保留不改写。因此两个离线实验 `maps/lightning-offline-t_4fi98t/`、`maps/lightning-offline-7dwk87rr/` **明确标为手持诊断数据，不通过正式建图验收**（各自 acceptance.json 已更新）：车体三维地图、base_link 安装外参、绝对尺度、地面/坡道、g2p5、在线模式与导航一律不得引用其结论。
  - 诊断性事实（仅证明链路与管线可用）：两次运行均 exit 0、结构检查 exit 0、各 35 关键帧，优化轨迹长约 5.41/5.42 m，起止平移差约 0.096/0.098 m（航向差约 173°–174°，不构成位姿闭合）；同包重建对称最近邻中位数 0.0216 m、p95 0.060 m、max 0.285 m（comparison.json，REVIEW_REQUIRED）。
  - 输入检查 PARTIAL：500/500 帧存在逐点 offset 回退（64,701 次），time_type=0 的动态时间质量问题在新设备上未解决；预处理保留 1,848,931 点，header 无倒退。
  - 地图与 bag 保留情况更新（2026-09-23）：用户自行删除两个手持诊断实验目录 `lightning-offline-t_4fi98t/`、`lightning-offline-7dwk87rr/`（确认不再需要）；"不通过正式建图验收"结论不变，诊断数字以阶段4报告第 4 节留档为准。手持 bag `bags/mid360-manual-loop-4uwmxbzb/` 与全部正式实验地图仍保留。
- 若人工审查不通过（明显重影/错位/漂移），在当前阶段内先诊断原因（time_type=0 时间质量、外参、配置），不自动进入后续阶段；必要时重录。
- 输出：[阶段4报告](04_mapping_report.md)、[人工采集说明](04_manual_capture.md)、`docs/04_evidence/`（含已更新的 [map_inventory.json](04_evidence/map_inventory.json)）、离线/在线/录制/检查脚本。
- 阶段2停车安全未通过的结论不变，自主导航不放行。二维成功也不代表坡道通行性评估完成。

## 阶段 3：Mid-360S 驱动、网络、时间戳与静止录包（旧雷达、阶段3结束时记录）

- **实物更正并验证：Mid-360S**（此前 Mid-360 为历史称呼）。用户确认 12 V、原厂一分三线、固定牢固、窗口无遮挡、可以静止；协议识别 type=35，SN **ARMCP870038745**，IP **192.168.1.145**。
- **阶段 PARTIAL**：真实收流与 SQLite3 完整录包/读取 PASS；逐点时间质量、安装外参及重启后恢复仍待验。没有运行 SLAM/导航，没有启动底盘或发送 CAN/速度。
- 当前原生 Ubuntu 22.04.5 / 6.8.0-40-generic / ROS2 Humble。采集时 enp2s0=192.168.1.50/24、100 Mbps/full duplex；20:30:38 后载波丢失、无 IPv4，是否人为断电/拔线待确认。上网默认路由继续 wlo1→192.168.3.1。
- 本次开始 enp2s0 不存在，can0 DOWN；已复用已有签名 YT6801 和 NM 连接恢复。首次恢复脚本因符号路径比较错误退出 1，修复后用户执行退出 0，失败记录保留。
- **网络持久化配置 PASS（当前内核）**：用户执行 persist_mid360_network.sh 退出 0；首次复核发现 umask 077 导致公共索引 0600，已修脚本并在 /tmp 复现验证。用户执行最小 depmod 修复后，本助手确认索引 0644、普通用户按名解析/签名查询通过，modules-load、autoconnect=yes、never-default=yes 均正确，Secure Boot 开启。同内核正常重启无需重复手工加载；实际重启未测试，不宣称 DKMS/未来内核适配。最终证据 docs/03_evidence/persistence_final_verified.json。
- 持久化 root-only 备份和回滚：/var/lib/ranger-nav-yt6801/stage3-persist-FxqdJUHF/rollback.sh；只读证据 docs/03_evidence/persistence_verified.json。没有重复 MOK、关闭防火墙或修改 Wi-Fi/CAN。
- 独立 ws_livox 构建 PASS（有上游警告）：SDK2 `08f523c930b2f0ba1e98a6afaa8d7476bf479908`；driver2 `4a1def929e5b59c7a8122d19fce6efba581ce9f7`，仅加设备/时间类型诊断日志。Lightning-LM `1325fed8fa97a2506878360fc23427ea00a767da` 只审查输入，未构建。三个仓库 master 来源，均 detached HEAD。
- 最新成功驱动 docs/03_evidence/driver-4l6q_kwk/，**退出 0，当前已停**。型号更正前两轮无数据、最终退出 -9，不列为通过。单实例锁曾拒绝重复启动。
- **真实静止包** bags/mid360-static-qg8qtyiv/raw/：SQLite3、metadata 完整、29.8049 s、298 帧 CustomMsg + 5962 条 IMU（约 10/200 Hz），6260 条全部反序列化，计数一致、integrity_check=ok。没有底盘状态或真实 TF 发布者，未虚构/补录。
- header 均无倒退/重复；**time_type=0，未 PTP/GPS 同步**，驱动取主机接收时间。298/298 点云帧存在逐点 offset 回退，共 39103 次，最大回退 0.468837 ms；原始数据不改，检查退出 2，时间质量 PARTIAL，不能放行运动去畸变。
- 原始 IMU 加速度单位 g、角速度 rad/s；静止加速度模长均值 0.995659 g。orientation 默认四元数不是有效姿态。原始 CustomMsg 保留逐点 ns；可选 RViz/SI IMU 派生话题独立，未替换 SLAM 输入。
- /tf_static reliable/transient_local 录制配置通过隔离合成迟加入测试；本车外参 TF 尚不存在。厂家 S 型号 IMU-in-LiDAR 名义外参与未知 base_link 安装外参分开记录，不用离地高度代替。
- 报告：[03_mid360_validation.md](03_mid360_validation.md)。后续仅待用户指定，不自动进入 SLAM/导航。底盘阶段 2 的停车安全结论不变。

## 阶段 2：底盘通信、ROS 与控制测试

- **【2026-09-28 结论更新：停车安全通过（用户确认）】** 用户明确确认"阶段 2 停止安全测试已经通过"。据此本阶段的停车安全（STOP safety）判为 **通过（依据=用户现场确认，非新增自动化测试证据）**；下文 2026-09-2x 历史记录中"驱动退出后负向速度反馈、原因未明"等异常观察作为原始记录保留不改写。运动许可逐次授权的约束不因此放松。
- **最新用户观察**：5 秒测试停车时未观察到后退。记录为现场结果，不等同于所有停车机制通过。
- **最新日志（22:32）**：ros-motion-ebju6tr2 中 204 条非零 CAN 指令均 0.05 m/s，持续约 5.0015 秒；主动发零后反馈归零。驱动退出后仍有 5 条负向反馈，最低 −2.544 m/s，随后归零；这是解码反馈，不能据此声称实物同速后退。用户未指定精确日志或观察窗口。审查：docs/02_evidence/ros-motion-ebju6tr2/observation_review.json。
- 22:30、22:31 两轮分别有零速稳定不足、前进速度反馈 0.144 m/s 超阈值中止。最新一轮完成不能追溯认定此前通过。本次仅分析本地记录，未启动驱动或发控制；停车反馈异常仍待定位。

- **此前配置修改（22:29）**：按用户“放宽保护”要求，前进纵向反馈绝对值阈值 0.10 → 0.12 m/s，顶部参数 `MAX_FORWARD_FEEDBACK_SPEED_MPS` 可调。发送速度仍 0.05 m/s、时长 5 秒；其他检查保留。离线边界/故障/退出瞬态检查和 dry-run 通过，未运行真机。证据 ros_motion_preflight/feedback_limit_012.json；不代表停车安全通过。
- **此前用户自测（22:25）**：5 秒参数已生效，但约 1.96 秒时 vx 反馈达到 0.101 m/s，超过脚本 0.1 阈值而提前中止。78 条非零 CAN 指令均为 0.05 m/s；退出后最低反馈 −0.801 m/s。5 秒持续前进未完成；当时未提高阈值，之后修改见上条。证据 docs/02_evidence/ros-motion-4mquwslq/review.json。
- 已修复首次中止原因被退出异常覆盖的日志问题，离线回归和默认 dry-run 通过；本助手未重新运行真机。停车原因仍未确定，安全未通过。
- 实物：用户确认 Ranger Mini V2；铭牌/SN/固件版本待记录。阶段 **PARTIAL**，停车安全未通过，自主导航不放行。
- **CAN 通信 PASS（已测范围）**：复用 can0 500000 bit/s、sample-point=0.875、restart-ms=100；没有重配。两次前进试验实际发送控制，收到运动反馈；接口 ERROR-ACTIVE，错误累计无增长。
- **ROS 接口 PASS（限范围）**：/system_state、/motion_state、/actuator_state、/wheel/odom 持续收流；22:16 轮各 524 条。实测 V2 参数、/wheel/odom、底盘 TF 关闭、/cmd_vel → /base/safe_cmd_vel 的 Twist 订阅正确。BMS 尚无数据。
- 22:05 首次启动因静止阶段未刷新命令而拒绝前进，未发送非零。之后测试脚本静止阶段增加零速刷新。22:11 和 22:16 分别获新授权后各执行一次 0.05 m/s、约 0.8 s 前进和主动发零。用户确认最后一次车体“确实运行了”。
- **停车安全未通过**：两轮主动发零后反馈归零，但驱动退出后均出现连续负向速度反馈，最低分别 −0.425、−0.392 m/s，最后又归零。无反向指令，原因未明；上述短脉冲的实物回动尚无明确现场结论；新的 5 秒现场观察见顶部。不能用驱动退出 0 或末帧为零判安全。
- 22:16 驱动 PID 23670 正常退出 0，测试脚本退出 2（退出后反馈异常）；没有自动重启。原脚本遗漏退出中途瞬态，已修复并用原始数据离线回放通过，重复轮实际捕获异常。
- 两仓库 detached HEAD：ranger_ros2 b6ea21a275ca5e7168130cc6470e61474681d679，ugv_sdk f2704eacdc90357078cd93ec60aae08bb4baab35，均有已存累计补丁。4 包构建 PASS：docs/02_evidence/build-JaMO1xCg.log。没有改变本轮驱动二进制。
- 测试脚本已按用户要求增加顶部 `FORWARD_DURATION_SECONDS` 参数，当前按用户要求设为 5.0 秒（速度仍 0.05 m/s）；截止时间、提示及日志同步使用。有限正数校验、默认/自定义 dry-run、状态拒绝与退出瞬态回放离线检查通过。5 秒配置仅运行默认 dry-run，退出 0，未启动驱动或发送控制；随后用户自行执行，既有中止记录，也有最新约 5 秒完成记录，详见本节顶部。此前参数化证据 ros_motion_preflight/duration_parameter_offline.log。
- 仍无生产主机运动看门狗；退出补零不覆盖崩溃、SIGKILL、CAN 中断。停发超时、RC 接管、硬件急停等未验收。测试脚本默认 dry-run，不属于生产安全控制器。
- 报告：[02_ros_motion_test.md](02_ros_motion_test.md)；源码审查：[02_base_interface.md](02_base_interface.md)；人工步骤：[02_base_manual_tests.md](02_base_manual_tests.md)。
- 新证据：docs/02_evidence/ros-motion-mp69pbm2/、ros-motion-9yzy_0mk/；历史 ros-motion-79ltuvud/。各轮保留实际脚本快照和版本记录。用户请求自行测试的命令已提供；本助手不再自动运行。

## 已完成的前序阶段（历史结果）

- 阶段 0 环境审计：PARTIAL，历史报告 docs/00_environment_audit.md。
- 用户追加任务“先连接雷达”：**PASS，仅板载直连 IP 通信。**
- YT6801 1.0.34 签名模块已加载，MOK 已登记，Secure Boot 保持开启；板载接口 enp2s0，192.168.1.50/24。
- 192.168.1.145 经 enp2s0 ping 5 发 5 收、0% 丢包，平均 1.829 ms；邻居 MAC ec:72:f7:da:cf:c8。
- wlo1 192.168.3.18/24，上网默认网关 192.168.3.1 保持原样；雷达配置无默认路由。
- NM ranger-mid360-8745 已激活，UUID 7d3735fd-229f-4461-bebd-421146eb06d2；autoconnect=no，模块未安装供开机加载，重启后不保证恢复。
- 设备完整 SN、Livox UDP 点云/IMU、ROS 话题与录包尚未验收；没有进入建图/导航阶段。
- 当时 can0 DOWN，未运行底盘驱动；阶段 2 最新检查见上。
- 详情、已修复脚本问题、命令和回滚：docs/LIDAR_CONNECTION.md 最后章节。
- 成功证据：docs/mid360_ip_connected_20260914.txt；原配置/激活日志及回滚：/var/lib/ranger-nav-yt6801/connect-jQpZBTzH/。
- 雷达 UDP/ROS 等后续阶段尚未授权；雷达临时连接持久化尚未实施。

交接：docs/HANDOFF.md 原文是阶段 2 之前的历史交接。当前已授权阶段 5（完成，剩两项 PENDING 见顶部阶段 5 节）；主仓库与四个上游源码仓库均已 git 化——源码本地修改提交在各仓库 `ranger-local` 本地分支（lightning@b973e7f 基点 1325fed8、ranger_ros2@7b75bc2 基点 b6ea21a2、ugv_sdk@e2f5764 基点 f2704eac、livox_ros_driver2@cc34f0a 基点 4a1def92；SDK2 与审查副本保持干净 detached HEAD），完整记录见 [source_versions.json](04_evidence/source_versions.json)；主仓库最新 commit 15d71e9（bags/ 与第三方源码不入库，补丁以 patches/ 追踪）。**现场一键启动**：`bash scripts/field_session.sh [--execute|--status|--stop]`，参数在 [config/field_session.env](../config/field_session.env)（组件开关/雷达时限/初始位姿/外参/桥接阈值）。后续阶段不自动推进，底盘运动许可不跨进程重启。
