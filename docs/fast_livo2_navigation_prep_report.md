# FAST 导航前置接入交付

日期：2026-11-06。范围由用户“规划控制算法我要大改，在此之前你先把前面工作做好”确定：完成导航前的地图、定位/TF软件接入、车体几何和时间输入核查。用户随后说明没有当前三传感器＋wheel同录包、只知道雷达距车头约26cm，并授权查Ranger Mini模型选择合适的base_link原点。本轮没有启动实物、底盘驱动、CAN、规划器、控制器或导航任务；未提交/推送本轮修改。

后续入口变更：用户另行要求录包自动启动底盘、仍由遥控器控制，已交付[默认自动wheel与只接收驱动](fast_livo2_recording_wheel_report.md)。下文“要求已有wheel发布者”描述本报告交付当时的入口；当前无需手动先启wheel，无发布者时自动自启receive-only。真实同录/同步待验边界不变。

## 交付与结果

| 项目 | 本轮结果 | 验收边界 |
| --- | --- | --- |
| 正式三维图→平地占据图 | PASS：305关键帧逐帧优化位姿投影，5cm栅格，PGM/YAML及来源manifest | 候选图，现场地面/门口/车体通行性未验；不是可直接行驶的最终图 |
| Nav2地图服务→C1适配→唯一公共TF桥接 | PASS：新增隔离受管入口，复用已有桥接，真实GICP C1合成6DoF测试 | 同时刻wheel为已知合成输入；真实wheel/传感器同步仍PARTIAL |
| 导航图身份绑定 | PASS：占据图绑定3D归档UUID/SHA；另一张图的有效C1结果也拒绝 | 本入口不自动跟随C1换图；换图须停止并重开相应地图会话 |
| 车体/传感器配置 | PASS：模型参考、原点约定、估算T_base_imu及静态TF已落地 | 工程估算，实测外参未完成；不阻塞软件开发 |
| 录包准备 | PASS：复用已有`--record-wheel`，8话题dry-run通过 | 没有启动录制；需要已有wheel发布者，且同ROS域/可发现 |
| 规划/控制大改、真实障碍costmap/安全闭环 | 未实施 | 等用户后续设计；保留现有生产参数和安全链 |

代码入口：`scripts/export_fast_livo2_nav_map.sh`、`scripts/bridge_fast_livo2_offline.sh`；两者默认dry-run。核心为`scripts/fast_livo2/navigation/`。既有C1适配器仅新增可选的`expected_map_uuid`/`expected_manifest_sha256`绑定，不改变未绑定旧入口的行为。

## 正式来源与占据图

源图：`outputs/fast-livo-demo-online-mapping-20261105/full_replay_12/map`，UUID`253f573d7a7eabbc363a225ab192cc14`，manifest SHA`e81d4a05a8da6cf3c1badfc0a6171295020bec59e5615408796c92a85b49b247`，graph419、305KF、114接受回环、1829386点。导出先调用当前隔离安装的`archive_tool`检查来源身份与完整性，结果[archive_validation](../outputs/fast-livo-demo-navigation-prep-20261106/navigation_map_final/archive_validation.json)为PASS，`physical_sync_validated=false`。原3D归档不变。

最终候选：`outputs/fast-livo-demo-navigation-prep-20261106/navigation_map_final/`，453×835格，分辨率0.05m，原点`[-10.45,-35.45,0]`。图中302738未知、61776自由、13741占用。查看[预览](fast_livo2_evidence/navigation_prep_20261106/navigation_map_preview.png)和[manifest副本](fast_livo2_evidence/navigation_prep_20261106/navigation_map_manifest.json)。蓝线为优化后的传感器轨迹，不是外部真值。

方法：从优化地图中靠近传感器且低于传感器的点拟合近水平地面，估计`z=-0.02350264*x+0.00968960*y-0.58404698`；采样12000点，7306内点，内点竖直残差P95约4.10cm。这个平面只用于投影高度筛选，不旋转/压平源3D图，不改变C1/C2姿态。逐帧读取IMU机体系`full.pcd`，仅应用一次对应`X_map_imu`；12m内、高于估计地面0.10–1.20m为障碍，地面带±0.08m。每帧720个方位射线，最近相关回波截止、该帧障碍遮挡；未知区保持未知。按关键帧融合hit/miss（+0.85/-0.4），占用阈值0.6，自由阈值-0.4。

首版全历史障碍取并集的图仍在`navigation_map/`，会令50个关键帧原点落在占用格；已改成多观测融合，正式候选减至2个（KF19/102，见[清单](fast_livo2_evidence/navigation_prep_20261106/occupied_origin_review.json)）。没有按轨迹或起点强制擦除障碍。这两处、门口及动态残留须在后续地图/车体复核时确认；图可加载不等于整条轨迹具备车体净空。中间`navigation_map_evidence/`也保留，不覆盖历史输出。

## base_link与外参估算

本轮选定base_link为**前后轴中点、左右居中在地面的投影**，X前/Y左/Z上。这是项目工程约定，不宣称照搬某个CAD原点。核对了[厂家V2模型](https://github.com/agilexrobotics/ugv_gazebo_sim/blob/27633a956c845903ee630538afeb17fe70afdd84/ranger_mini/ranger_mini_v2/urdf/ranger_mini.xacro)，其中部分仿真尺寸/单位碰撞盒与实车规格不一致，未整包导入。采用[厂家手册](https://static.generation-robots.com/media/ranger-mini-user-manual-en.pdf)的738×500×338mm及[官方驱动V2参数](https://github.com/agilexrobotics/ranger_ros2/blob/b6ea21a275ca5e7168130cc6470e61474681d679/ranger_base/include/ranger_base/ranger_params.hpp)的494mm轴距、364mm轮距；固定来源SHA见[模型审查](fast_livo2_evidence/navigation_prep_20261106/model_sources.json)。

配置在`config/fast_livo2/navigation_prep.yaml`，独立于生产runtime：

- 估计`T_base_lidar.xyz=[0.109,0,0.600]`m；X由`0.738/2-0.260`得到，假设车体前后对称、26cm量到雷达坐标原点。
- Y居中、0.600m高度和水平朝向沿用既有安装近似；yaw对齐也属假设，不把“装平”当作偏航已测定。
- 厂家`T_lidar_imu.xyz=[0.011,0.02329,-0.04412]`m，假设两者和base轴对齐；故`T_base_imu.xyz=[0.120,0.02329,0.55588]`m，RPY均0。此变换用于IMU位姿→base位姿换算；不对已经去畸变的IMU云重复应用雷达外参。
- 新入口发布base_link→fast_livo_imu、base_link→livox_frame静态TF。公共map→odom、odom→base_link仍只由原桥接发布。
- 规划包络建议保留原0.740×0.520m参考矩形。当前生产两个costmap实际半径仍0.10m，不能包住该矩形，审查结果[geometry_audit](fast_livo2_evidence/navigation_prep_20261106/geometry_audit.json)。本轮不修改用户先前窄口试验参数，后续规划控制大改时统一车体包络。

历史“距车尾25cm”记录保留；本轮用更新的“距车头26cm”，两者不合并。软件开发可以使用上述估算，不再以不知道base_link原点为由阻塞；实测校核与真实接车验收分别记录。

## wheel同录包的具体用途

需要用同次运动中的FAST位姿和wheel轨迹，确认两者是否能在**同一个原采样时刻**对应，区分历元偏移、传输/驱动延迟、漂移和插值缺口；再检验`T_map_odom(t)=T_map_imu(t)*inverse(T_base_imu)*inverse(T_odom_base(t))`。不能用FAST局部里程计代替wheel，也不能把两次独立采集拼成同步验收。

只读审查：旧`bags/mid360-manual-loop-p42bk0yi/raw`约89.704s，仅雷达/IMU/wheel等，没有当前相机和FAST三融合。其LiDAR、IMU、wheel header范围重叠，wheel存储时刻减header的中位数约0.337ms；这不证明物理同步。新282s独立查询包没有wheel，LiDAR存储时刻减header中位数约203755747.692s，与主机历元明显不一致。[审查数据](fast_livo2_evidence/navigation_prep_20261106/input_audit_02.json)保留原整数纳秒，未改header、未自动应用估计时间偏移。新包将用于查清当前安装的时间关系，不能靠旧包推定当前已经同步。

用户后续准备采集时，在**已有当前wheel发布者、与FAST同域175且可发现**的条件下：

```bash
bash scripts/record_fast_livo2.sh \
  --seconds 180 --label wheel_alignment_visit01 \
  --visualization none --min-free-gb 100 --record-wheel
```

这会启动录包入口管理的传感器/前端，要求已有wheel数据，但不会启动底盘驱动。先等`CAPTURE RECORDING`，静止15秒，再人工遥控做直线、左右转弯、几次停车，最后再静止15秒；约2–3分钟即可。不需要自动导航。底盘驱动另有控制模式副作用，不应为得到wheel而直接复用未知启动命令。原包保留传感器和wheel原戳；取得包后再做时间诊断/有依据的派生映射。

## 复现与软件验证

导出只写新目录，默认不执行；下面命令的输出已存在，复现时改成新的输出路径：

```bash
bash scripts/export_fast_livo2_nav_map.sh \
  --archive outputs/fast-livo-demo-online-mapping-20261105/full_replay_12/map \
  --identity-file outputs/fast-livo-demo-online-localization-20261106/visit01_auto_pcl_01/source_identity.json \
  --output outputs/fast-livo-demo-navigation-prep-20261106/navigation_map_final --execute

# 以下默认dry-run；仅离线同域输入就绪后才显式--execute。
bash scripts/bridge_fast_livo2_offline.sh \
  --navigation-map outputs/fast-livo-demo-navigation-prep-20261106/navigation_map_final
bash scripts/bridge_fast_livo2_offline.sh --status
```

桥接入口只启动地图服务/lifecycle、静态TF、既有适配器/桥接及会话监视。默认domain219、localhost_only=1、use_sim_time=true；需调用者提供同域离线`/clock`、私有C1和wheel。它不启动C1、前端、硬件、障碍过滤、costmap、规划控制或速度发布者，不把未映射的真实异历元输入拼接。原始包不能直接据主机存储时刻充当同采样时刻；没有当前FAST＋wheel包，真实联调仍PARTIAL。

| 实际命令/检查 | 退出码/结果 | 证据 |
| --- | --- | --- |
| `audit_inputs.py --bag bags/mid360-manual-loop-p42bk0yi/raw --bag bags/fast-livo-relocalization_visit01-trpozzyj/raw --output …/input_audit.json` | 1，numpy bool不能JSON序列化；修复显式bool转换 | `input_audit.log`及outputs内原部分JSON保留 |
| 同命令另存`input_audit_02.json` | 0，采样审查完成，真实同步未验 | `input_audit_02.log/json` |
| 上述正式导出 | 0，archive_tool校验及新目录写出PASS | `export_final.log`及`navigation_map_final/` |
| `python3 -m unittest discover -s tests -p test_fast_livo_navigation_map.py -v` | 0，4项通过 | `grid_tests_final.log` |
| `python3 tests/test_fast_livo_navigation_ros.py`，ROS＋ws_nav＋隔离localizer环境 | 0，2项通过，10.889s | `navigation_ros_static_tf.log` |
| 原C1的`test_adapter_rejects_retired_sessions_and_future_stamp`、`test_initialization_external_base_and_same_time_bridge_latch`，覆盖最新节点＋small_gicp | 0，2项通过，5.684s | `adapter_regression.log` |
| 录包`--record-wheel --dry-run`、桥接默认dry-run、bash语法、`git diff --check` | 0 | `record_wheel_dry_run.json`、`bridge_dry_run.json` |

ROS测试在domain196、localhost执行：已知6DoF外参和运动、最新隔离small_gicp C1、原公共桥接；同刻插值数值正确、只有一个动态TF发布者、wheel缺口撤销就绪、创新跳变锁定、退休会话/未来戳拒绝、错地图拒绝、无速度发布者、正式PGM每格与Nav2输出一致、静态TF数值一致。重复调试执行记录保留，表中8项为4+2+2，不把重跑相加。未修改C++，没有重新编译/替换旧安装；入口明确使用源码Python路径。会话退出后核对只清理本轮受管进程，不能据此说整机设备已停止。

全部小证据位于`docs/fast_livo2_evidence/navigation_prep_20261106/`；大地图在outputs。本轮起点HEAD`542d03f7626eaabfd43789571ab53199679f75a4`、工作树干净，基线保留在`.runtime/backups/navigation_prep_20261106_210954/`。当前未提交工作留给后续保存；没有删除原包、地图、失败证据或备份。

最少待办：用户随后录一次当前FAST＋wheel包；用它核查真实时间关系和外参估计，现场复核候选图门口/残留。规划与控制算法方案另按用户新设计推进，不预设继续Smac/MPPI。
