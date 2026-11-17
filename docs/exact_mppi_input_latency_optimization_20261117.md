# 输入延迟拆分与首轮优化（2026-11-17）

用户要求“能否想办法减少数据延迟”。本轮授权按软件实施、构建、离线验证执行；实车仍暂停，没有启动传感器、底盘、CAN、导航目标或非零速度。boot已变为6c018ad9…，不继承上次运动许可。全部离线ROS使用localhost、domain201/202/203；早期234启动因DDS端口范围失败，未形成回放，失败日志保留。

结论：发现并修复了三融合前端多等下一包雷达的调度缺陷；同包8400帧对照中5639帧可提前约100ms完成，录制输入时序中的输出间隔P95从200.83ms降至101.43ms。导航侧减少重复工作，实录点云处理P95约9.19→6.63ms，忙时协调器回调约1.38→.048ms。前端修正仅安装于独立候选目录，正式定位入口仍使用原版；地图来源兼容、实机端到端延迟和运动连续性尚未验收。不能宣称走停已解决。

## 1. 先拆分延迟，排除心跳统计混淆

工具：`scripts/analyze_exact_input_latency.py`。只读已正常封存的`bags/fast-livo-localization-z7vdr1vj/raw/raw_0.db3`，按原采样戳匹配，只统计每个测量首次VALID发布；窗口为上次door_center_step约28.45秒。

| 记录接收链路 | 中位ms | P95 ms |
| --- | ---: | ---: |
| FAST odometry → 同戳deskewed云 | .140 | .422 |
| FAST odometry → C1私有新有效结果 | 35.07 | 52.50 |
| C1私有 → 公共新有效定位 | .497 | .777 |
| deskewed云 → 导航障碍观测 | 34.07 | 117.99 |

C1记录配准elapsed_ms中位24.79/P95 46.17；待处理job P95=0，当前主要问题不是C1长期积压。前端新采样在记录时钟下的年龄中位约104.93ms，私有/公共新定位约144.73ms。此前约165ms统计含重复心跳，不是新样本首发延迟或纯CPU耗时。录制clock离散、会话映射含接收偏差；上述跨topic差包含DDS/调度/录包接收影响，不能冒充物理同步或每个进程精确CPU耗时。

证据：`recorded_stage_latency.json`；原包只读，未改header或正式+100ms。

## 2. 前端发现：LIO完成后，VIO错误地还要等新雷达

实际在线来源`ws_fast_livo/online_frontend_source/src/LIVMapper.cpp`的`sync_packages`在入口无条件检查raw lidar/IMU队列非空。但LIO可能刚好取完所有当前雷达点，将剩余点留在`pcl_proc_next`；下一步VIO所需图像与IMU区间均已同步好，仍被空raw lidar队列挡住，直到约100ms后新包到来。VIO分支还读取了未使用的`imu_buffer.front()`。

修正仅对`slam_mode_==LIVO && meas.lio_vio_flg==LIO`（已完成LIO、待做同帧VIO）取消新raw lidar/IMU等待，删除该未使用读取。仍要求缓存图像存在，新的LIO区间继续执行全部原传感器可用性与时间覆盖检查。未跳过视觉、未用LIO姿态代替VIO后姿态，原同戳云/位姿、安装外参、图像+100ms均保留。

可复现生成器`scripts/fast_livo2/prepare_latency_frontend.py`从原在线manifest逐文件SHA核对并生成baseline/candidate，保护vendor、原派生源码与原安装。补丁`patches/fast_livo2_finish_pending_vio.patch`应用于原在线I/O派生版本。`scripts/build_fast_livo2_frontend_latency.sh --execute`只构建/安装，不启动设备；候选位于`ws_fast_livo/latency_frontend_install/fast_livo`。

### 离线核对

| 检查 | 结果 |
| --- | --- |
| 实际C++ sync_packages断言 | 原版复现等下一包；候选在原图像存在、无新raw lidar/IMU时完成本帧VIO；缺图像、新LIO缺雷达/IMU仍拒绝 |
| 原生预处理/时间检查 | 13 passed、0 failed |
| 300帧同包对照 | 201帧提前，0帧延后；中位提前99.72ms；300位姿消息一致，所有声明点字段绝对容差1e-7内一致 |
| 长片段 | baseline8401帧、candidate8400帧，8400共同原采样戳全部匹配；5639帧提前、0帧延后；中位提前99.76ms |
| 长片段释放间隔 | 原版P95 200.83ms、最大202.61ms；候选P95 101.43ms、最大103.57ms |

长段是同一原包前约840秒的真实传感器输入，包含实车转向/移动，轨迹累计约14.15m是前端估计，不是真值。上限在同一输入内处理完整批次，故baseline比candidate多1个尾帧，不将其当成丢失中间帧。时序提前指在原录制消息到达序列中更早具备执行条件，不是新实机测得100ms改善；离线wall_seconds受磁盘、并行竞争影响，不作为比较指标。

**长段数值没有完全一致：** 原版与候选位置差P95约9.37mm、最大43.16mm；原版重复回放自身P95约9.80mm、最大41.58mm，均从约第489帧起出现差异。说明数值差异非修正版独有；尚未唯一定位不确定性来源，也不能据此证明精度无损或更优。长段16800共同原生消息有15821个不满足严格比较（位姿完全一致/点字段1e-7），结果原样保留。序列化CDR/PCL填充字节与声明点字段分开比较，不用padding字节不一致冒充几何变化。

证据：`frontend_replay_comparison.json`、`frontend_motion_comparison.json`、`frontend_repeatability.json`及三个motion/repeat目录；所有原始失败日志保留。

## 3. 导航侧已落地的优化

协调器仍5ms检查输入、异步计算结果及撤销输出；优化器忙时不再反复准备无法提交的点集/路径/覆盖输入。准备控制输入按20Hz运行，覆盖不足时也遵守该频率，避免200Hz重复重算；只在确实需要提交时分层采样障碍。NavigateToPose反馈调整为20Hz，取消/终态仍立即处理。增加`input_timing.observation_build_publish_ms`和`observation_sample_age_ms`，供下一次明确授权实测拆分构造耗时与样本年龄。

覆盖历史本来就存于odom对齐、5cm固定格网上，改用整数索引重叠切片合并，避免每帧、每个历史格重新构造世界坐标。TTL、原始时间、未知区域和新障碍遮挡撤销逻辑保留。

81帧真实点云、同采样wheel插值回放，新旧完整障碍点、control网格及独立安全覆盖网格逐帧完全相等。4次交错、无profiler对照：更新P95 9.19→6.63ms，约降低28%。冻结真实输入的忙worker场景300次production tick对照：P95 1.377→.0479ms，约降低96.5%；这是忙时CPU回调成本，不是JAX/全链路延迟。

## 4. 验证、安装状态与剩余项

- GPU相关回归107项通过；新增与相关最终合并检查20项通过（有重叠，不相加称127独立项）。实际sync两个variant通过，原生13项通过。
- 导航colcon构建退出0；源码与`ws_nav/install_exact`中coordinator/obstacles SHA一致。
- 前端baseline/candidate构建、候选安装退出0；隔离原包回放3次均退出0。首次候选安装因未构建ranger_livo_checks而失败，补齐构建目标后复验成功；失败日志保留。系统Python无JAX和venv自动加载ROS测试插件缺lark的两次测试环境失败也保留；已使用项目JAX环境及禁用无关插件的命令完成验证，没有安装/升级系统依赖。
- `git diff --check`、Python编译与构建脚本语法检查通过。无提交推送，无设备/运动。

**正式入口状态必须区分：** 导航两项优化已经更新install_exact；前端候选单独安装，mapping/localization默认仍指向online_frontend_install。前端来源身份包含二进制SHA，不能直接替换后冒充旧地图来源一致；尚需正式的来源兼容验证/迁移，再按用户当次授权测端到端延迟与运动连续性。未改原地图/旧来源身份来绕过检查。250ms门限、定位几何接受条件及末端防撞均未放宽。

下一步软件重点是前端候选与C1/旧地图的来源兼容核对、重复回放差异来源及完整链路调度测试；实车仍保持暂停，坡道/出门/走廊往返仍PARTIAL。
