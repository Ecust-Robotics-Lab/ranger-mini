# 变化场景重建与首轮实车分项测试

日期：2026-11-10。当前阶段进行中。用户手动完成新录包后明确授权“随意可以开始实车测试”，随后确认已释放急停、静止并切换上位机/CAN档。本轮没有重启、提交、推送或修改系统驱动。

## 新包与新地图

用户录制 `bags/fast-livo-office_corridor_changed_20261110-s10xmu6g`，约206.6秒，正常 Ctrl+C 退出0，CAPTURE_COMPLETE；存储2060雷达、41183 IMU、2064同步图像、2067 FAST同戳位姿/区间云、10332 wheel。相机整会话2094条同步OK，缺帧ID/非递增ID均0；物理曝光同步仍PARTIAL。原包不改写。

原DB SHA256 `0f4edafdef03f8dea2d7a04ec74b8cd88f3bc1b0c514f6021aaaeab04bb6ff8d`。完整身份与采集质量见 `fast_livo2_evidence/changed_scene_capture_20261110/capture_identity.json`、`capture_summary.json`。

实际离线步骤均退出0：

```bash
source /opt/ros/humble/setup.bash
python3 scripts/fast_livo2/make_replay_view.py \
  bags/fast-livo-office_corridor_changed_20261110-s10xmu6g/raw \
  outputs/fast-livo-demo-changed-scene-20261110/replay_view --seconds 240
bash scripts/map_fast_livo2.sh --execute \
  --replay-view outputs/fast-livo-demo-changed-scene-20261110/replay_view \
  --domain 228 --visualization none --seconds 300 \
  --output outputs/fast-livo-demo-changed-scene-20261110/mapping_replay_01
```

冷启动三融合+B重放完整处理2055同戳对，133关键帧、37接受/优化回环、图版本170，403归档文件校验通过。新3D图为 `outputs/fast-livo-demo-changed-scene-20261110/mapping_replay_01/map`，UUID `160f8572a77a61856cf87e43514be769`，manifest SHA256 `93628bcf34742e8fbdb1f08afa9501f44aaa1f9dab4d9ef2c80914b7a62fc105`。

新占据图 `outputs/fast-livo-demo-changed-scene-20261110/navigation_map_05`，5cm/749×338、原点[-6.75,-5.60]，自由32268、占用5812、未知215082格，传感器原点占用0。沿用原0.1–1.2m高度投影，没有擦除障碍或缩小车体。PGM SHA256 `b072b4e8050cff78b32d70d45c468681c633a1d6df68c75741ff84e2beb80a44`。

新地图坐标与旧图不同。候选路线配置 `config/exact_mppi/office_corridor_changed_20261110.yaml`；从新图办公室经门口、走廊约x=18.3m端点掉头返回。窄档完整0.74×0.52m车体、静态每侧合计5.5cm，全部7/7段、三个初始模式通过SE(2)搜索及独立扫掠检查。无需继续降至1.5cm。交互页 [新路线复核](../outputs/exact-field-preparation-changed-scene-20261110/index.html)，[新路线图](../outputs/exact-field-preparation-changed-scene-20261110/route_review.png)。离线成功不代表已完成实车路线，`field_route_accepted=false`。

FAST轨迹约43.76m、首尾估计0.266m；wheel约41.06m、首尾2.18m，两者并非真值精度。新包wheel来自旧receive-only安装，后续发现的反馈符号配置不会追溯修改原包。新建图使用三融合，不使用wheel约束建图。

## 当前实车发现与修复

本轮boot `31de1c53-fc87-45d0-9b02-966b8a628ec2`，localhost/domain215，显式选择新地图、窄档、低速0.10m/s和0.20rad/s。完整日志及微动作JSON见 `exact_mppi_evidence/live_changed_scene_20261110/`；受管启动命令为 `robot_session.py base --execute --control-authorized`、`localize_fast_livo2.sh --execute --mode auto --public-tf --record-navigation ...`、`run_navigation.sh --execute --control-authorized`。生产runtime没有切图。

1. 初始反馈vehicle_state=2/control_mode=3，保持停车；用户切档后，实际0x211为vehicle0/control1、32位故障0，随后进入测试，没有绕过故障检查。
2. 初次定位录制因 `/livox/lidar` 被探针声明为旧CustomMsg且实际为PointCloud2而失败。已修复探针类型选择：base不订原雷达，FAST导航订PointCloud2；隔离图检查通过，失败包与日志保留，后续录制成功启动。
3. 直行、倒车微动作产生相应方向wheel反馈且最后归零。横移虽安全输出非零，CAN实际只发零速预转向。原始证据显示指令+1570mrad、实际反馈+1569mrad，旧驱动反号后永远无法确认转向到位。
4. 新增 `v2_feedback_steering_sign`，默认保留-1兼容旧约定，当前实车配置+1；里程计与到位判断共用该参数，命令编码不变。模拟SDK验证两种反馈约定、反向轴拒绝、真实模式确认、过期停车、遥控和receive-only隔离，均退出0；隔离安装重建退出0。备份 `.runtime/backups/live_steering_20261110/`，身份 `steering_fixed_identity.json`。原receive-only二进制保留。
5. 修正后左移0.05m/s×2s，wheel估计+0.099m，定位体轴侧移+0.106m；右移相同脉冲wheel-0.086m、定位-0.095m；结束均反馈零。正旋0.08rad/s×2s定位yaw+0.105rad、结束零；这些只说明方向和有限动作响应，不是绝对精度/制动距离验收。

## 验收边界

新采集、离线三融合重建和新路线静态7/7 PASS。真实分项测试进行中；全程办公室—走廊—办公室、门口、避障、取消、接管、急停及≥30分钟重复任务尚未完成，仍PARTIAL。运行期定位短暂撤销仍由原条件停车，不放宽有效期；所有微动作都通过 `/teleop/control` 与独立安全链，结束撤销motion_enable，未直接发CAN运动包。

本报告将在本轮分项结束后追加实际结果和最终会话状态。完整原包、旧地图、失败证据和旧安装保留。


## 后续覆盖/时延及物理车头核对（当前结论）

用户授权覆盖算法更积极，并明确车头为相机镜头朝向的一端、雷达原点距同一车头端约26cm。检查 `R_base_imu * R_imu_lidar * R_camera_lidar^T * [0,0,1]`，原配置相机朝向为base约[-0.9997,-0.0225,0.0079]，证明原工作车体前向与用户定义相反。原小动作的“前后/左右”只指当时SDK坐标，不是已核实的物理车头方向。

现工作外参RPY=[0.0169366422492582,0.3685116044554271,0.0154988133505638]rad，保留约21°倾角；雷达XYZ仍[0.109,0,0.6]，按同一厂家LiDAR–IMU平移重算IMU XYZ=[0.10314082223598997,0.02394593798623294,0.5552533734244779]。FAST profile同步；生产runtime不改。驱动 `v2_sdk_front_opposite_base=true`，指令XY到SDK取反、反馈按相反纵轴积分，yaw率保持原符号；转向反馈符号+1另独立保留。相机朝向在新base为[+0.9997,+0.0225,+0.0079]。47组协议往返、真实反馈积分/生命周期/receive-only编译检查及73项相关Python回归通过。

实际正向试验 `micro_forward_camera_positive.json`：+0.05m/s×2秒，经独立安全链，wheel估计+0.09363m、定位体轴+0.09337m，最终反馈零。用户现场明确“确认是正方向”，这是本轮首个经人工核实的物理前进方向证据。旧包/旧外参/旧安装保留，新receive-only录制入口选择同一install_exact二进制并显式receive_only=true，避免旧坐标wheel混入新会话。

旧工作坐标下30cm目标失败的记录逐项核对：静态图与全保留障碍均通过；当前单帧覆盖不通过。冻结原请求与21帧原云/同戳插值wheel见 failed_goal_plan_request.npz、coverage_raw_frames.npz。单纯减小角度膨胀、延长地面回波到地面、使用多高度远回波，均不足以令该旧方向目标21帧中任一帧通过；试验结果保留，后两种策略没有进入交付代码，不能声称已找到唯一传感器盲区根因。

用户授权后的最终覆盖调整：保留180扇区，取消相邻扇区额外膨胀，并对栅格覆盖的全部扇区求精确最小距离，防止只检查四角遗漏中间短回波；空扇区仍未知。全局指导路线使用已建图占据格及实时保留障碍，不再要求全局近处目标整车当前可见；局部指导截断、完整局部预测/制动及独立最终输出仍必须通过当前覆盖。没有把全局未知格改成自由，也没有缩小车体/障碍距离。冻结旧目标新全局路线5.5ms可生成、独立几何通过，完整路线当前覆盖仍被明确拒绝。这说明可规划远期指导不等于允许执行未观测轨迹。覆盖/几何65项回归与4个新角度边界检查通过（后者包含在65中）。

时延优化：复用同一独立几何评估预处理，去掉跟踪阶段无用粗云下采样，GICP4线程；节点5ms轮询但不放大未变化心跳；原真实匹配约83ms降到约43ms，仍有年龄偶发250ms越界，60秒自主目标已取消且停车，未报到点成功。追加在线严格同戳配对零人工reorder延迟、PCL4线程法向量；backend/localizer两组CTest均通过，初次缺libgtsam的测试启动失败单独保留，正确LD_LIBRARY_PATH后运行通过。实际采样、匹配质量和250ms停车门限未放宽。

另修正提前到达DDS数据的处理：future云保留队列直至原采样时间；future wheel不遮挡历史中仍新鲜且具有同戳TF的采样，禁止外推/latest TF，2项ROS采样边界回归通过。

车头修正后重新计算候选点，另存 `config/exact_mppi/office_corridor_camera_front_20261110.yaml` 与 `outputs/exact-field-preparation-camera-front-20261110/`。原录制起点换算为约(-0.106,-0.014,-0.016)，新候选为6/7静态通过，最后回办公室且恢复出发朝向的搜索在三模式5秒预算内超时，不能沿用旧车体坐标下7/7的成绩。3D图及2D投影来自IMU归档/地面拟合，不依赖车体朝向，原资产不需旋转或重写；改变的是车体位姿、轮廓检查及路线。

最新时延安装的短目标试验因持续踌躇取消，随后导航、定位、底盘受管会话全部停止；整条走廊路线未发送。后续修复与复验见下节。


## 踌躇故障、上游复核与修复

失败证据 `goal_camera_forward_latency_final.json`：60秒净估计移动约2.9cm，39次segment_start_pose_replan、9次mode_confirmed；502个非零输出中|vx|中位数约0.0041m/s。有效输入377次，仅2次采样新鲜度失败、5次就绪缺失。不能继续把这次停滞主要归因于定位计算延迟，也不能把未实测的电机死区当已确认根因。

已确认的软件原因：旧搜索只有距目标≤0.27m才尝试精确连接；0.30m同朝向目标仍拆成DRIVE/PARALLEL/SPIN。停稳等待中5mm/5mrad起点检查会因微小定位变化触发整个路线重搜，再次清空优化历史。全局目标无进展计时也会被模式确认重置，造成长时间重复尝试。

本次修改：

- 全局搜索首先检查固定朝向直达加终点旋转的完整车体扫掠；全部通过才输出精确段，否则继续SE(2)搜索。30cm同朝向目标为一个PARALLEL段。
- 停稳时可重新对齐PARALLEL段起点；只处理≤3cm/0.03rad的小偏移，并用全部保留障碍与静态图独立重验。较大跳变仍重规划；没有把运动中的指令裁剪成另一条轨迹。
- 15秒无进展预算归属整个目标，重规划/模式确认不重置；无有效输入或人工控制的时间不计入。平移段只转动朝向不算平移进展，旋转段保留角度进展。
- 新直达段只有起终点，扩大闭环验证发现门口/走廊离线控制被稀疏路径拉回。核心现在先按原几何加密再裁取局部指导，统一在线/离线契约。初次8/10闭环失败结果保留 `churn_closed_loop.json`；修复后结果另存，不覆盖失败。

上游复核：本项目已使用caseypen/EXACT-mppi提交bdc5456938a54267408f2b15870b00ff4e91c58f的JAX核心，所有锁定原文件SHA仍匹配（`churn_upstream_integrity.json`）。其[rangerminiv3.py](https://github.com/caseypen/EXACT-mppi/blob/bdc5456938a54267408f2b15870b00ff4e91c58f/EXACT_MPPI_core/exact_mppi/mppi_jax/rangerminiv3.py)使用20控制步模式保持、进入/退出不同阈值，证明抑制模式抖动是适用思路。当前SDK的V2/V3轮距0.364m、轴距0.494m、主要转向约束相同，最小转弯半径0.4764/0.47644m仅舍入差异；不以型号不同否定复用。但上游0.15m/s横移进入阈值高于本轮0.10m/s总限速，且优化后模式裁剪不能直接代替本项目模式内预测与实际反馈确认。

官方[Nav2 MPPI文档](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/)提供VelocityDeadbandCritic；本轮未直接启用未测量阈值。模拟敏感性对照 `churn_stiction_sensitivity.log`：固定PARALLEL、0.10m/s限速，假设死区0/0.005/0.010m/s均能约3.2秒进入30cm目标6cm内部到点范围。缩小随机采样噪声反而更慢，因此没有凭经验改该参数。这些是合成plant结果，不能作为真实死区/制动标定。

已执行75项核心/任务/采样/覆盖/外参回归通过（`churn_progress_regressions_final.log`）、10项GPU优化器检查通过（`churn_optimizer_dense_tests.log`），colcon构建退出0（`churn_dense_build.log`）。初次GPU测试受ROS pytest插件未初始化影响、初次旋转测试未模拟停稳确认的测试设置错误均保留失败日志，分别在关闭无关插件、补全停稳确认后通过。实车复验结果随后追加。


修复后软件闭环 `churn_closed_loop_dense.json`：10/10通过（包含门口、65cm窄走廊），GPU独立计算P95约10.47ms/P99约10.99ms。这是无定位竞争的合成plant结果，不代替实车整链时延。首次修复实车 `goal_camera_forward_churn_fixed.json` 仍FAIL：估计约1.6cm，状态6/no_progress_timeout；起点误差重规划0次，但覆盖不足7次、输入无效及异步就绪撤销仍触发24次接受路线。结束输出及wheel零，随后受管设备全部停止。不能把这次实车报为已修复。

追加协调层OBSERVING状态：短暂实时覆盖不足时保留已验证路线和当前模式、输出零速；每次新观测重检指导，只有停稳、实际模式一致、小位姿变化条件满足后才提交新的完整局部优化/独立验证。真实障碍仍走重规划，定位/会话失效仍撤销原任务历史，15秒无进展预算保留。76项相关检查通过、构建退出0，证据 `churn_observation_hold_tests.log` / `churn_observation_hold_build.log`。随后仅复验相机端30cm目标，未发送走廊路线。


OBSERVING实车复验 `goal_camera_forward_observation_hold.json`：30秒沿相机车头方向估计前进约18.3cm，结束yaw变化不足0.001rad；未完成内部到点，按期限取消（status5），最终输出/反馈归零。非零平移指令P50=0.01244m/s、P95=0.03726m/s、最大0.05295m/s；约32.1%采样输出非零，全程平均0.00501m/s。低速不仅来自0.10m/s试验上限，也来自短覆盖指导、反复停止和重新初始化，不能仅抬高上限宣称解决。

后续修正恢复起点约束：允许沿已验证平移段正常前进，限制横向偏移≤3cm、yaw≤0.03rad及段端点范围；仍独立重验全部障碍/地图扫掠。52项相关检查、构建均退出0。证据 `churn_along_resume_tests.log` / `churn_along_resume_build.log`。继续同一绝对目标的实车 `goal_camera_forward_along_resume.json` 仍在30秒期限取消，最终估计位置(0.39676,0.20437)、目标(0.45339,0.21295)，位置差约5.73cm，yaw差约0.0505rad；没有收到NavigateToPose成功结果，不能标PASS。最终输出/wheel零，随后按navigation→localization→base顺序停止受管设备，日志 `churn_final_stop.log`。真实踌躇/到点稳定性与巡航速度仍PARTIAL，不能发送整条走廊路线。


## 多帧覆盖与停车流程优化（本轮单点实车PASS，整车仍PARTIAL）

用户要求彻底放宽不必要的运动限制。实际修复：局部规划使用0.5秒内原始采样的已观测自由格，原始帧固定于odom格网并按采样时间过期，合并结果不写回原始历史、不刷新旧证据。新/保留障碍按体素角宽立即撤销其遮挡区域的旧自由证据；整云为空、时间回退、数据断流仍撤销。初始未知区域不能凭时间记忆变自由。全局未知格、真实轮廓、障碍距离、失定位撤销及独立末端防撞保持。参数visibility_control_memory=0.5；默认核心0兼容历史测试。

模式真实反馈已确认且反馈停稳后取消协调层重复0.5秒等待；独立Safety仍在发模式请求前要求停稳驻留，底盘仍确认实际转向角。最终路径段使用原计划15cm/0.15rad容差，连续停稳一次0.5秒即可成功；中间段仍严格控制横向/朝向误差。

只读回放bags/fast-livo-localization-95eecsy_/raw的前500帧原始FAST区间云与同戳wheel历史，旋转/平移按正式工作外参：当前车体覆盖由356/500升至495/500，前方5/15/30cm检查点由431/474/499升至全部500，过滤P95均约9.9ms。这是覆盖检查点统计，不是完整轨迹或任务成功率。见continuity_recorded_comparison.json及tools/check_temporal_coverage.py。单帧对照也应用了新障碍阴影撤销，以单独比较记忆效应。

77项相关回归通过（continuity_final_tests.log），覆盖记忆过期、禁止自行刷新、新障碍即时遮挡、空云/会话重置、最终与中间段不同容差。构建退出0；窄档10/10闭环通过（continuity_closed_loop.json）。

实车goal_camera_forward_continuity.json：NavigateToPose返回status4/SUCCEEDED，目标前进30cm，定位估计实际前进约26.1cm，终点位置误差约4cm，结束速度输出和wheel反馈均零；约25.8秒包含客户端准备和结束记录。非零指令P50约0.0125m/s/P95约0.0487m/s，零输出采样约33.8%，仍偏慢。覆盖不足事件5次，仍有定位撤销和重规划。没有外部真值，以上不是绝对停车精度。成功不能仅归因于覆盖修复：最终容差及重复等待也同时修改。

结束后按navigation→localization→base停止全部受管设备，continuity_stop.log退出0。未提交/推送。当前单次短目标PASS，连续运动速度、重复成功率、动态障碍、全走廊往返和30分钟实车仍PARTIAL；不得把此次成功宣称为整套导航已彻底解决。


## 1.5秒覆盖、提速与零输出原因排查

用户明确允许更多覆盖记忆、大提速，并以降低正常运动零速度占比为目标。已将visibility_control_memory从0.5增至1.5秒，上限不超过障碍记忆2秒；保留原始采样过期与新障碍遮挡撤销。实车速度通过显式RANGER_NAV_MAX_SPEED=0.30传入新launch max_speed参数；协调器和独立安全链使用同一上限，默认首测档0.10仍保留，角速度维持0.20，平移加速度0.30。实际worker参数已核对0.30，并非只修改未安装配置。

优化器首次批次改为以模式内加减速参考轨迹为采样中心，而非纯零中心；完整优化、制动候选及独立验证保留，未在验证后强行放大命令。该参考投影JIT显式缓存并预热，避免重置后的重复投影编译开销。固定实录输入的代价/温度对照保存speed_ablation.log、speed_temperature.log：去掉覆盖软代价在这份末端请求中无差异，降低temperature会产生反向抖动，因此没有把上述对照参数放入交付。该请求距目标约0.12m，不能据它声称已确认正常巡航的唯一慢速根因。

定位恢复会话缺陷：无session元数据的故障心跳会把原有效会话加入retired_loc，导致恢复后的同会话消息一直被拒收。本轮首次提速准备因此未能发目标（goal_camera_forward_speed，goal=null）。现修正为无编号故障只撤销，不淘汰已知会话；真正会话替换仍拒绝旧消息。3项ROS采样/会话检查通过（speed_session_fix_tests.log）。

计算超时此前触发全局重规划和优化历史重置，形成反复起步。control超时/过期及最终验证超时现在撤销输出，进入保留路线模式的OBSERVING，等待新的有效局部控制；不延长100ms计算期限，不执行过期结果。实际障碍等其他错误仍重规划。相关52项检查通过，日志speed_gap_tests.log。

统一任务窗口（从接收到任务至终止，不含客户端前后等待）比较见speed_motion_comparison.json：早先0.1m/s短目标零指令占比26.97%；首轮0.3m/s提速试验升至68.46%、30秒估计0.225m且取消；修复超时重规划/JIT后降至26.15%、30秒估计0.251m且取消。两个提速试验使用同一绝对目标的不同剩余距离，不能将其作为严格同初始条件速度排名。两轮结束均输出/反馈零，随后所有受管设备停止。0.30m/s上限尚未变成实际0.30m/s巡航。

末端逻辑另修正：精确规划会因为很小yaw差追加SPIN，原来因此让前一平移段被迫使用中间点6cm门限。若后续只剩目标处旋转且实际位置/朝向满足整体15cm/0.15rad容差，允许一次停稳后直接成功，避免无意义微动/换模式；真正需要转向的目标仍必须旋转完成。75项相关检查通过（speed_optional_spin_tests.log）。此前83项核心/采样/launch/覆盖回归、10项GPU检查、10/10窄档闭环分别见speed_final_regressions.log、speed_projection_jit_tests.log、speed_closed_loop.json；最终末端修改为任务层纯逻辑，后续实车单独记录。


最终追加：末端微旋转判定修正后，受管会话qpq_rsqi的C2一直处于验证/退避，首次20秒及追加60秒准备均未满足导航输入；goal_camera_forward_speed_final、goal_camera_forward_speed_final_ready两次goal=null，无导航目标发出。未放宽定位有效性。本轮停止日志speed_delivery_stop.log；实际受管设备均停止。

目标进展代价由10提高到50，仍只朝预测时域内截断的指导终点，不拉向弯道远端；碰撞、模式、加速度、覆盖及独立验证保留。冻结真实末端输入（距目标约12cm、初速零）对照的5个首样本速度由约0.0015–0.0047提升至0.0056–0.0097m/s，见speed_ablation.log；这是同输入离线敏感性结果，不能称为真实巡航速度提升。最终源码10项GPU检查及10/10窄档闭环通过，见speed_progress_optimizer_tests.log、speed_progress_closed_loop.json；新进展代价与末端微旋转修正尚无成功实车复验。单纯降低采样temperature的反向抖动试验不进入源码。

交付当前参数：覆盖记忆1.5秒；显式提速入口RANGER_NAV_MAX_SPEED=0.30；模式/计算中断保留路线、过期结果不执行；无编号故障心跳不淘汰原定位会话；初始指导采样中心与投影JIT缓存；最终到点可跳过容差内无意义旋转。当前实车零输出约26%仍未达到用户要求的正常顺畅运动，不能把源码提速上限或离线通过冒称整车提速完成。完整身份speed_delivery_identity.json，未提交/推送。
