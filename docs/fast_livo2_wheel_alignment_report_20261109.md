# 180秒FAST＋wheel同录核验、反馈修复与工作外参

完成日期：2026-11-09；原包录制于2026-11-06。用户录完后要求继续检查/开发，并在解释坐标轴后明确授权约RPY=[1°,21°,-179°]先使用。本轮仅源码、隔离构建、只读分析和离线ROS检查，没有启动硬件、CAN控制、真实录制或导航，没有提交/推送。此前未提交成果均保留。

## 结论

| 项目 | 实际结果 |
| --- | --- |
| 真实receive-only录制流程 | PASS：自动拥有wheel，定时正常停录，全部8话题审查通过，自启进程已清理 |
| 转向反馈方向 | 确认V2反馈角与ROS车体正方向相反；已修源码并构建隔离录包版本，六组运动学检查通过 |
| 工作外参 | 用户授权暂用，已统一FAST profile、C1/C2入口和公共适配器；不是实测标定通过 |
| 时间关系 | 确认两个历元不同；候选映射支持本包离线对应，物理同步仍PARTIAL，不自动用于真机 |
| 实车公共wheel/TF与导航D | 未通过验收；本轮没有用候选映射运行实车公共TF或控制 |

## 原包和真实录制事实

路径：bags/fast-livo-wheel_alignment_visit01-pwr5lx7u/raw。

原SQLite SHA256为dd3a84c771ac803df92702a3c2f43a28be6773d30947d0c922cc50551d227b1e，11月6日与11月9日重新计算一致。完整结果capture_complete=true、forced=false、exit=0，实际录制180.047607秒；CAPTURE_COMPLETE存在，data_checks=PASS、无失败/警告，physical_sync=PARTIAL。数据库跨度180.085950秒，共55831条消息。

| 实际数据库话题 | 消息数 |
| --- | ---: |
| /livox/lidar | 1801 |
| /livox/imu | 36021 |
| /front/image_sync | 1800 |
| /front/image | 1801 |
| /front/camera_info | 1801 |
| /fast_livo/odometry | 1801 |
| /fast_livo/deskewed_imu | 1801 |
| /wheel/odom | 9005 |

worker打印的observed_messages还覆盖录前预检收流，不能与最终数据库计数直接相减当作丢包。wheel约50Hz、header严格递增，中位周期19.999551ms、最大20.233225ms；FAST位姿/云1801对同戳，原图1801帧、同步图1800帧；整个传感器会话1818条相机同步记录均OK、无硬件帧号跳变。原始IMU/点时间检查无非有限值或扫描内逆序。

manifest.wheel.source=owned_receive_only，实际argv含receive_only=true和TF关闭；归档二进制SHA为474cadbb61ca744235ffda6bf23b5fab3709809597ef705ccaad47382c6af835，已重新校验归档文件，属于本次修正反馈方向前的版本。原包和其sources档案不修改。录包会话run-vl2iktwk已STOPPED且owned=[]；当前旧base/预览会话也不存活。这里只证明本次受管会话收尾，不据此宣称整机所有设备已停。

## 转向问题的证据和修复

原wheel路径积分约37.377m，FAST估计轨迹长度39.550m，均不是外部真值。速度变化大体对应，但行驶转弯时wheel角速度多段与IMU/FAST相反，原地旋转又一致，不能靠统一时间偏移或对所有角速度一律取负解决。

[厂家Mini V2手册](https://static.generation-robots.com/media/ranger-mini-user-manual-en.pdf)第26–28页说明：反馈转向左负右正，控制转向左正右负；原地旋转逆时针为正。现有UpdateOdometry直接把反馈角当作ROS左正方向，造成此差异。

已在Mini V2的UpdateOdometry入口仅将反馈angle取反，覆盖行驶和斜行反馈，spin仍使用原angular值，命令编码不改、其他车型不改。没有改变当前角度幅值换算，也没有用拟合gain强行校准轮距/比例。新增增量补丁patches/ranger_ros2_v2_feedback_sign.patch，叠加在已有receive-only补丁之后，ws_base源码仍由补丁追踪。

11月6日执行bash scripts/build_base_recording.sh --execute完成，11.3秒，隔离安装节点SHA为ae5f3f1ae633cc9b968c3c0e740810388423a4e7554d2b22c787d582c12ce069；11月9日核对构建成功、源码/二进制及相关回归。旧ws_base/install节点仍是36c56486b018d87a785260710b770da25d91af613807d71d23bc7d33007f23f9，没有覆盖。旧控制回调的编译警告保留，不据此声称普通控制算法已修复。

新脚本scripts/fast_livo2/navigation/analyze_wheel_alignment.py只读提取原包的wheel、FAST位姿和IMU。诊断旧wheel时，根据有无平移twist区分行驶与spin，对行驶的yaw增量和机体侧向增量反向，再重积分到新派生数组；原包没有原始CAN模式话题，所以该重建属于诊断候选，不能追认为已录得修复后真实wheel。

将IMU角速度投影到数据估计的车体旋转轴、允许独立bias/gain用于比较后，原始角运动相关性约0.353，修正诊断约0.989，拟合gain约0.999。FAST估计总yaw变化0.1254rad，原wheel为1.3364rad，诊断修正为0.2909rad。修正方向后仍有残差，不能报告角度精度已标定。

使用候选外参和一个初始刚体对齐、不拟合轨迹尺度，FAST与wheel的平面轨迹分歧中位数由11.32m降到0.53m、P95由21.42m降到2.34m。这是两条估计轨迹的一致性诊断，不是APE/RPE或绝对定位精度，也没有经过独立盲测。

## 用户授权的工作外参

此前零安装旋转意味着IMU三个轴与车体X前/Y左/Z上完全同向。新包平面spin估计的车体旋转轴在IMU系为[-0.3602,0.0158,0.9327]，相对IMU Z约21.14°；再用有符号直行速度与FAST速度估计车体前向，得到当前T_base_imu旋转：

    RPY(rad) = [0.0169366422492582, 0.3685116044554271, -3.1260938402392293]
    RPY(deg) ≈ [0.9704, 21.1142, -179.1120]

分别用前半包和后半包自己的spin/直行样本估计，得到[1.0222°,21.1493°,-179.0939°]及[0.7703°,21.2175°,-179.1825°]。这支持作为当前工作估计，但同包分段不是独立实测标定，也仍假设车体平面旋转、直行wheel vx方向正确。

用户11月9日已授权暂用约[1°,21°,-179°]，配置保留上述估计精度，状态为USER_AUTHORIZED_WORKING_ESTIMATE_NOT_FIELD_CALIBRATED，不再以没有实测外参为由阻塞软件联调。

采用既有车头距离/高度估计T_base_lidar.xyz=[0.109,0,0.600]m，以及厂家T_lidar_imu.xyz=[0.011,0.02329,-0.04412]m，按T_base_imu=T_base_lidar*T_lidar_imu重新计算平移，得到：

    T_base_imu.xyz = [0.11485917776401003, -0.023945937986232936, 0.5552533734244779] m

位置和安装高度没有重新实测，不能因更新旋转而追认XYZ精度。T_base_lidar与T_base_imu采用同一候选旋转，前提是厂家名义两传感器轴对齐；去畸变IMU云不重复应用雷达外参。

已统一到config/fast_livo2/navigation_prep.yaml及FAST专用profile。C1/C2离线入口默认选择FAST profile，在线入口也明确给节点传同一XYZ/RPY，保证人工base初值转换、私有IMU结果和公共适配器一致。生产config/runtime.yaml和生产底盘安装不变，未切换生产定位后端或启动硬件。没有改FAST前端滤波/图像时间偏移、3D地图、PGO或地图坐标。

## 时间关系和边界

wheel header为主机2026历元，传感器/FAST仍在旧历元；直接用原header做wheel插值，1801个FAST采样中0个可对应，不能直接拼TF。

IMU存储时间减header的整数纳秒中位数为203782752741349211ns。以此作为粗映射，消息到达残差P1/P99约-10.98/+6.32ms，只说明本次接收行为，不代表物理采样延迟已测定。修正符号并考虑旋转轴后，全包拟合的附加lag约+2ms，前半包+2ms，后半包-18ms；该20ms差异可能包含传输、反馈延迟和模型误差，不能当作精确时钟漂移。

诊断候选映射t_host=t_sensor+203782752743349211ns，仅绑定本包，不写成跨重启/跨会话常量。在该映射下，1799/1801个FAST时间戳可由原wheel历史在100ms最大间隔内插值，另2个不外推。live_clock_mapping_enabled=false、physical_sync_verified=false。没有重打now、重写原包或将派生候选接入真实公共TF。

## 复现与验证

输出必须改为新的目录：

    source /opt/ros/humble/setup.bash
    OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 scripts/fast_livo2/navigation/analyze_wheel_alignment.py --bag bags/fast-livo-wheel_alignment_visit01-pwr5lx7u/raw --output outputs/fast-livo-demo-wheel-alignment-20261106/NEW_OUTPUT

最终分析在outputs/fast-livo-demo-wheel-alignment-20261106/analysis_20261109_split/。较早的analysis_20261109使用共同spin轴做分段估计，保留为过程结果；最终版本对两个半包各自估计旋转轴。小证据在docs/fast_livo2_evidence/wheel_alignment_20261106/，包括[完整统计](fast_livo2_evidence/wheel_alignment_20261106/analysis.json)、[运动曲线](fast_livo2_evidence/wheel_alignment_20261106/motion_diagnostics.png)、采集事实、原包哈希和命令日志。

| 实际检查 | 退出码/结果 |
| --- | --- |
| 只读输入审查、完整分析脚本、原DB重新SHA | 0，哈希一致 |
| check_base_feedback_sign.py | 0，V2左/右/倒车、spin保留、斜行及其他车型6组通过 |
| check_base_receive_only.py / check_base_model_selection.py | 各0，接收模式无控制副作用及四机型参数回归通过 |
| test_fast_livo_wheel_alignment.py | 0，5项已知运动检查：整数历元、延迟、旋转轴、前向、行驶/旋转重建 |
| test_fast_livo_navigation_ros.py | 0，2项、11.666秒；工作外参静态TF、实际地图服务及GICP C1→原公共桥接隔离检查 |
| 录包owned-wheel单项回归 | 0，1项、4.364秒；合成wheel录制归档及清理 |
| C1受管入口故障清理回归 | 0，1项、3.598秒；故障注入为预期行为，未接硬件 |
| 三入口实际dry-run参数一致性、AST、增量补丁重放、Git空白检查 | 通过 |

测试数量不把重复执行或C++断言混加为独立实车验收。全部ROS测试隔离domain/localhost；没有把合成或诊断结果称为实车公共TF通过。

当前包足够继续离线工作，无需为了修正符号或暂用外参立刻重录。后续重点是带明确有效范围/不确定性的时间映射和真实C1/wheel公共链验收，新的实物测试另按当次授权；规划控制仍待用户新方案。完整故障字段32位、旧ROS16位截断和模式枚举差异仍是独立底盘接口待办，不能用旧error_code=0自动放行。
