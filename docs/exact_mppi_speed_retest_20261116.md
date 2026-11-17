# 2026-11-16 实车提速续测

当前结果：通信准备PARTIAL，未发送导航目标或非零速度。用户本日要求继续既定实车提速测试，并确认仍在原已建图区域；物理供电/接线状态等待本日核对。未提交/推送。

## 基线与操作

- HEAD仍e8245fa80310a4e91b5d2eb303f33883bb965216；原有未提交导航及11月11日P0实验/时钟/后端优化修改均保留，没有覆盖或清理。
- 新boot：3489e27b-21ab-4e9c-b48b-271a6df865af。进入时没有受管设备/导航/底盘进程，can0初始DOWN，适配器与海康相机已枚举。RTX5060 Laptop/580.178.04正常枚举。
- 计划沿用经绑定的新场景图160f8572a77a61856cf87e43514be769；本轮尚未启动定位、雷达、相机或导航。现有速度上限显式0.30m/s、覆盖记忆1.5秒、GoalCritic权重50；没有继续增加到未经测试的更高上限。
- `sudo -n /usr/sbin/ip link set can0 up type can bitrate 500000`：退出0；此固定命令使用已有免密授权，没有改系统授权。
- 被动监听3秒：0外部帧、0发送；见can_passive_initial.json。
- `ROS_DOMAIN_ID=215 ROS_LOCALHOST_ONLY=1 python3 scripts/robot_session.py base --execute --control-authorized --startup-timeout 15`：正常启动新底盘二进制，但未取得新鲜wheel反馈，受管启动超时、自动清理；base_handshake.log及会话run-a1d6xoyr保留。底盘日志errno=105发送队列不足，与can0已UP不能混为通信正常。链路累计RX/TX各3帧不作为已收到车辆反馈的证据。
- 驱动退出后按固定命令down/up同一500kbit/s，清空报错队列；随后被动监听15秒仍0帧、0发送，见can_passive_after_reset.json。原始链路配置及前后统计均保存；初始状态为DOWN，阶段结束可在无其他拥有者时恢复DOWN。

证据目录：`docs/exact_mppi_evidence/live_speed_20261116/`。baseline.json保存boot、HEAD、关键源码/配置SHA；can_before.txt、can_after_failed_handshake.txt及被动监听JSON保存当前事实。11月10日的定位成功/底盘故障/非零速度证据不冒充本轮状态。

## 当前人工待办

已询问本日底盘上电、CAN/上位机档和底盘端/USB-CAN端插头；等待反馈。通信恢复后才启动同一定位会话并进行0.30m/s上限短目标验证，统计实际速度、正常任务窗口零输出占比、重规划次数及停稳结果。


## boot b62be450续测：CAN恢复，等待定位运动基线

当前boot b62be450-225c-40c8-8a86-395c669b2cf8，与本报告前一轮boot分开。用户再次要求继续。所有原有源码、11月11日P0修改、地图与原包保持；HEAD仍e8245fa。

- 初始can0 DOWN，没有受管设备进程。按已有固定免密命令启用500kbit/s，3秒被动监听收到2661帧、发送0帧，PASS。不能据此断定此前errno105的唯一根因。
- 正常受管底盘握手run-wftzms_x进入RUNNING。实际vehicle_state=0/control_mode=1/error_code=0、约55.1V，停稳反馈正常。当前v2_sdk_front_opposite_base=true、转向反馈sign=+1，物理车头约定不变。
- 启动FAST三融合+C2定位并记录bags/fast-livo-localization-65k767hm，绑定原变化场景图UUID160f8572a77a61856cf87e43514be769。传感器时钟映射有数十毫秒残差波动，本轮观察时仍未锁定。
- 导航以RANGER_NAV_MAX_SPEED=0.30、narrow启动准备；有限客户端forward_060_cap030等待120秒后因输入未就绪退出，goal=null，没有发出目标/非零速度。导航会话随后显式停止，无待发目标；底盘与定位采集保留以等待辅助基线。
- 已从当前原包只读提取C2原因与候选质量：主要global_no_geometric_match及候选基线不足。取得几何接受候选时source_overlap约0.91325、rmse_m约0.12512、eigen_ratio约0.24819；静止强几何分支要求RMSE≤0.12，故stationary_strong_geometry=false，不能在没有运动基线时直接交给导航。没有为继续运动放宽定位有效性条件。
- 已请求用户遥控小幅移动约20cm/10°后回CAN停稳，以在同一会话取得真实运动基线；当前自主目标已取消，等待反馈。该动作由用户现场判断空旷区域，Agent尚未发送非零速度。

证据boot_b62be450/：passive_initial.json、base_handshake.log、pre_motion_probe.json、initialization_reasons.json、forward_060_cap030.json、navigation_stop.log、identity.json。当前结果：CAN握手PASS；本次提速及导航PARTIAL/未执行。


随后本次定位采集因录包器SQLite error(5): database is locked异常退出(-6)，会话自动清理。Agent曾对仍在写入的原包执行只读SQL长查询，存在阻塞SQLite写提交的风险；这次诊断操作不应对活动原包执行。已明确在线只订阅ROS、原包停录后再查，原失败包保留。此后人工运动辅助请求暂缓，车辆保持静止；底盘仍RUNNING、导航未使能，后续恢复另建限时会话。


用户随后要求优先清理11月5日前录包。已停止全部受管设备/采集及待发导航目标，本轮没有发送非零速度。清理结果30个旧数据文件/44.5GiB，见recording_cleanup_before_20261105_20261116.md；本日及其他11月5日以后的原包保留。此前“底盘与定位采集保留等待基线”不再是当前运行状态。


## 静止重定位改进与CAN档位再核对

用户问为何需要移动，并说明已清理无用录制包。实测空闲约791GB；此前定位诊断原包DB已由用户清理，目录中留删除记录，正式建图包及地图保留。没有代用户删除其他资产。

补采30秒静止包wwun8uxh：录包器正常退出0，得到303个FAST同戳对；整体localize入口退出2因定位验收未就绪，不能混称录制失败。全部采集停止后读取SQLite，在线诊断不再访问活动数据库。

隔离domain229、localhost，同一包前20秒按原header 1x输入：原2帧/4000点/.2m及2帧/8000点/.1m均零VALID；8帧/12000点/.2m约4.456秒首VALID、原采样4.200秒，157个不同有效确认。细采样单独无效，增加时间窗口/点数联合有效；没有放宽RMSE/重叠率/多帧一致性/地点歧义门限。配置仅改global_window_frames=8、global_maximum_points=12000，原配置备份.runtime/backups/static_refinement_20261116/relocalizer.yaml。实车新会话g8i0xh67也得到map_match_confirmed；原始日志与对照见static_refinement/。

相机静止样本current_camera.png显示车头朝柜子/物品。独立预检基于原采样TF、实时障碍和占据图，前进0.6m全车扫掠通过，其余三方向拒绝（directions.json）。随后forward_060_window8及统一覆盖口径后的forward_060_coverage2均因guidance_unobserved/no_progress_timeout失败，实际输出全部零、优化器未进入控制计算（metrics为空）。没有成功实车提速。

规划与末端安全覆盖记忆分别1.5/2秒存在口径差异，已统一visibility_control_memory=2.0，保留新障碍阴影即时撤销、原始采样过期、不自行刷新旧证据。对应时效边界/任务/采样检查通过（coverage_consistency_tests.log），构建退出0；该一致性修正仍未解决上述起步覆盖拒绝，不能标已修复。

用户随后表示此前未开启底盘CAN控制并要求再试。最新采样实际control_mode=3（遥控）；已停止导航/定位并重新启动底盘请求CAN，system_after_handshake.yaml仍vehicle_state0/control_mode3/error_code0、约49.9V。当前只有受管底盘节点，未发目标/非零速度，正在等待用户把遥控器控制权档切到上位机/CAN并停稳。此当前模式与前面control_mode1快照须分开。后续需协议确认模式1再继续同一测试。


## CAN确认后的成功运动与走停分析

用户切档后最新协议control_mode=1、vehicle_state=0、error_code=0。forward_060_can_confirmed返回SUCCEEDED，任务约17.56秒、估计平移0.474m，非零指令P50约0.026/P95约0.087/峰值0.107m/s，任务窗口零输出25.74%，接受路线8次。随后前方1m预检被实时点障碍拒绝，0.8m通过；forward_080_can_confirmed也SUCCEEDED，约16.42秒、估计平移0.694m，非零指令P50约0.043/P95约0.142/峰值0.149m/s，零输出21.19%，接受路线4次。两次末尾均零输出/零wheel；实际峰值尚未达到0.30m/s上限。比较数据见success_motion_comparison.json，两个距离不同不能视为严格同条件算法A/B。

用户问轮速/指令是否需标定。完整动作加停稳区间的receipt-steady积分：指令约0.502m、wheel约0.444m、定位净位移约0.472m。存在差异，但包含响应动态、接收时间偏差、定位误差，不能直接把定位当真值自动乘轮径系数。正式比例校核应以外部尺量的同一车体参考点位移为真值，做正反向/不同稳定速度重复测试；轮速比例、指令响应与转向几何分别校核，当前没有改速度/里程计比例。见forward_060_distance_check.json。

80cm完整记录中零输出原因含短暂定位撤销、重新接受路线及segment_cross_track_replan，安全门禁多数采样passing。已实现瞬时输入/DEGRADED暂停保留PARALLEL/SPIN路线，立即清空控制并撤销pending结果代际；恢复必须停稳/实际模式一致、重新检查路径/当前障碍/覆盖并提交新优化结果。真正LOST、定位会话变化、地图变化、时钟跳变及遥控接管仍清空路线。89项相关回归通过，构建退出0。PAUSED不授权非零速度，旧代际结果不能恢复输出。原有“输入无效总清路线”测试按新语义迁移，初次1失败日志与最终通过日志均保留。

用户询问定位短暂撤销门限。本轮11个不同not_converged样本均结束于迭代49（共50次），求解器收敛步长为1e-4m/1e-4rad；几何门限重合45%、RMSE0.20m、退化比1e-4，导航输入有效期0.25秒，连续3次拒绝或1秒无确认进入LOST。收敛步长是优化停止标准，RMSE是点云残差，都不是绝对定位精度。隔离同一移动包前50秒将最大迭代50→100后，同四个原采样时刻仍未收敛（迭代49→99），故没有修改50次上限，几何/收敛/时效门限均未放宽。证据moving_iter50、moving_iter100、nonconverged_details.json。

首次修复后回程return_pause_fix虽然返回SUCCEEDED，但指令/wheel均零、未实际倒车，不能当运动验收。用户确认80cm测试后自行移动过车，因而旧绝对起点已接近新当前位置；没有据此宣称跨会话定位错误或恢复算法成功。随后从当前位姿生成新的相对60cm目标forward_060_pause_fix作真正复验，结果随后追加。


## 暂停恢复修复后的最终实车复验

用户确认80cm测试之后曾自行移动车辆，因此return_pause_fix的零指令到点不作为跨会话定位一致性或运动通过证据。随后从当前位姿生成相对目标forward_060_pause_fix：SUCCEEDED，任务约8.635秒，定位估计平移0.504m，非零指令P50=0.04999/P95=0.15393/最大0.16458m/s，零输出18.287%，仅1次plan_accepted，3次revalidate_after_pause；末尾输出和wheel均零。见pause_fix_motion_comparison.json及对应原始goal JSON。与之前60cm的17.56秒/25.74%/8次接受路线相比有改善，但起点/场景状态不同，不能声称严格控制变量A/B或完全消除走停。

本轮未达到0.30m/s持续巡航、未执行整条走廊往返、未完成独立尺量速度/轮速比例标定。未放宽定位有效性/求解收敛精度、几何或250ms时效门限；100次迭代对照无改善，因此保留50次。现有改进是采样积累与任务暂停恢复流程，真正失效仍停车。

结束按navigation→localization→base受管停止（final_stop.log）。源码/配置最终身份见final_identity.json；89项相关回归通过、colcon构建退出0。此后没有自动提交/推送，没有代用户清理任何原始资产。
