# 复制到新对话

> 请接手 `/home/lab307/ranger_nav`，分支 `refactor/pluggable-navigation`。先读 `AGENTS.md`、`docs/HANDOFF.md` 和 `docs/STATUS.md` 最新小节，再查 `docs/FAST_LIVO2_BACKEND_PLAN.md`。核对本机 Git、实际 YAML、安装来源和当前受管会话，以最新日志为准。
>
> 项目是 Ranger Mini V2 的 FAST-LIVO2 三融合定位与 EXACT-MPPI 导航。正式工作台见 `workbench/README.md`，正式前端由 `config/fast_livo2/active_frontend.lock.json` 锁定，导航运行参数从 `ws_nav/src/ranger_navigation/config/exact_navigation.yaml` 读取。用最近会话的 worker 状态确认参数实际加载，改 YAML 后需新建会话。保持现有 +100 ms 图像偏移、坐标约定和安全门禁。
>
> 2026-11-23 交接时，YAML 为平移上限 0.50 m/s、旋转上限 0.80 rad/s、`min_radius: 0.48`、覆盖总外扩和保护范围各 0.015 m、边界未知格额度 16、内部额度 6。0.25 m 弧线曾被 V2 转角限制拒绝，驱动模型下限约 0.4764 m。相机若 USB3 已打开却持续 `MV_E_NODATA`，优先检查外同步信号和 Line0，并对照雷达 `time_type`。最新现场记录有成功单点，也有失败和取消，完整办公室出门、走廊掉头回返仍为 PARTIAL。详见 HANDOFF 最新小节。
>
> 先给我简短接手摘要，再按我的下一条指令推进。开设备、CAN、实车目标、非零速度和重启均需当次明确授权。按 boot 和 PID 核对归属，保护原包、地图、标定、安装资产及未提交修改。用隔离 ROS 做离线验证，现场结论以实际记录为准。底盘两套嵌套仓库通过 `config/exact_mppi/sources.lock.json` 中的锁定源码和补丁重建。Git 保存的是源码及精选验证记录，原始资产仍在工控机。

## 历史接手记录

2026-11-17最新：后角边界格容忍与速度档已修，见docs/workbench_boundary_speed_report_20261117.md及STATUS/HANDOFF顶部。新增coverage.allow_boundary_unknowns=true（部分接触保护边界的格共用8格额度，完整内部未知仍拒绝；false恢复严格），总2.5cm/点障碍保持，无整块后方豁免。正式low_speed默认false，默认读YAML motion .30m/s/.50rad/s，网页显示实际进程上限；改值需新导航会话。87核心/49GPU/后续25参数/65工作台/6隔离ROS/5浏览器通过（有重叠），模拟峰值0.290m/s非实车；正式nav已构建，最后用户会话均无存活，未操作设备/实车目标或速度、未提交推送。旧“仅外1cm/保护任意相交必拒”和“默认首次低速档”由本条覆盖。

2026-11-17最新：实时观测覆盖已统一总2.5cm（含扫掠），修复此前仅静态图2.5cm、覆盖仍5.5cm的遗漏。车体+1.5cm必须观测，仅最外1cm带最多8未知格，点障碍净空/体素项保持；前段/JAX/独立验证/门禁一致，网页显示原因/格数/25mm。正式nav已构建，61核心/58GPU相关/18系统ROS/20相关/65工作台/5隔离实际协调器+GPU+门禁/浏览器通过（有重叠），记录失败输入非零候选和制动复核通过，不是实车运动。见docs/workbench_coverage_policy_report_20261117.md及STATUS/HANDOFF顶部。用户会话已停后安装，Agent未启设备/CAN/实车目标或速度；需正常启动自有会话加载，新实车仍PARTIAL，无提交推送。

2026-11-17最新：已新增独立“重置规划控制”，失败/无有效定位可结束自有目标、清旧规划/历史、重建仅自有计算子进程；不重启设备/定位、不自动发目标。受管35秒显式预热条件修复1秒误停，运动权限保持。70/67/2GPU/7ROS及实际GPU+协调器+guardian、浏览器通过，正式nav已构建，见docs/workbench_navigation_reset_report_20261117.md及STATUS/HANDOFF顶部。实车仍PARTIAL，无提交推送。

2026-11-17最新：用户已明确要求直接应用此前优化，并最终将静态总裕量指定2.5cm。正式FAST各入口已用低延迟前端（active_frontend.lock.json），旧地图走精确来源迁移声明，定位采样400ms、实时链期限保持；见docs/workbench_goal_latency_frontend_report_20261117.md及STATUS/HANDOFF顶部。109/65/46及隔离ROS/浏览器/真实地图加载通过；四受管会话均STOPPED，Agent未启设备或运动；新实车仍PARTIAL。下方“前端候选未切正式”仅历史。

2026-11-17最新：工作台7/8匹配阶段显示及静止确认弱证据锁定已修，见docs/workbench_static_confirmation_report_20261117.md、STATUS/HANDOFF顶部。45核心/59工作台/4隔离ROS/浏览器通过，正式localizer已构建；现场尚待用户重启工作台后显式加载复验，设备当前均停，Agent未启动或移动。旧日志不足以证明现场唯一根因，新增支持计数与位姿差诊断已保存。0.05停稳保持，无提交推送。

2026-11-17最新软件：用户要求执行恢复/历史/实际执行反馈优化，见docs/exact_mppi_control_recovery_20261117.md及STATUS/HANDOFF顶部。generation与history_generation分离；只有最终验证ACK的轨迹作warm-start，按实测进展推进，同模式可余速重验。新LeasedControlCommand固定原100ms期限，旧ControlCommand不变；新增执行观察和驱动会话重启enable边沿。base/nav已安装，补丁可重建；148回归、后续31相关GPU、38生命周期、ROS20故障项及单次A/B仿真通过。实车仍暂停，前端低延迟候选未切换。当前是bb41832之后的新未提交工作，禁止reset/覆盖；此前9次保存不涵盖本轮代码。

2026-11-17最新Git保存：用户已授权并完成分段本地提交，检查点见docs/git_checkpoint_navigation_20261117.md，未推送。实车仍暂停，前端低延迟候选仍独立安装，正式来源兼容未验。下方历史“未提交”不表示当前Git状态；现查git log/status。

2026-11-17最新：用户要求减少数据延迟，已完成首轮软件优化/离线验证，实车保持暂停。先读docs/exact_mppi_input_latency_optimization_20261117.md及STATUS/HANDOFF顶部。前端LIO→VIO多等下一雷达包已复现，候选8400帧中5639帧可提前约100ms；仅在latency_frontend_install独立安装，正式入口仍用原online_frontend_install，勿绕过地图来源SHA。长段候选/原版重复都存在厘米差异，未证明精度等价。导航重复输入准备/覆盖合并优化已安装，81帧全点/网格一致，107回归/最终20相关检查及构建通过。新boot6c018ad9，没有设备或目标启动，运动许可不续用。

2026-11-17最新：用户要求暂停实车并已停车；不自动续用设备/运动许可。先读docs/exact_mppi_ramp_corridor_attempt_20261117.md及STATUS/HANDOFF顶部。精朝向已成功，向门中心仅部分移动，仍未到坡面/走廊。主要证据指向输入延迟/250ms过期→高频暂停与代际重置，未证明覆盖坐标偏移；odom/map同戳回放均通过。当前整理讨论，不继续软件或实车执行。

2026-11-16最新：先读docs/exact_mppi_door_retest_ramp_20261116.md。复位续测两段接近成功（估计67cm/42cm），转向仍未过，尚未出门，设备全停。SPIN GoalAngleCritic现60、其他模式12；87回归/10闭环及构建过，但实车无进展未解决。长停链是重规划后STOPPING遇短暂失效→OBSERVING，feedback_settling曾恢复，并非唯一根因。用户新增确认门口6cm/50cm下坡（约7°）；现有导航按平地，未自动做地形支持。记录俯仰中位0.26°、地面采样不连续，不能确认坡高，更不能把远处11–13cm低分位当坡高或自动改外参。新snwbprb6/d_pha29a封存包保留。旧结束位姿不可当当前就绪状态，下一次按当次指令及实际定位；不盲发全段目标。

2026-11-16此前门口实机：见docs/exact_mppi_office_corridor_live_20261116.md。DRIVE固定曲率/轮角确认及方向约束分项已完成，整圈仍PARTIAL。

2026-11-16巡航准备历史：见docs/exact_mppi_office_corridor_preparation_20261116.md；新office_corridor_preparation_20261116路线新增门外转向后倒车/平移返回办公室，8/8段、24/24静态检查通过，约40.39m。原6/7为历史候选。候选起点不是当前位姿；后续设备运动须当次授权，不直接自动发送整条路线。

2026-11-16最终覆盖说明：先读STATUS/HANDOFF最上方本日最终状态。CAN模式1已确认，静止定位8帧/12000点有效；覆盖2秒。短暂定位/输入失效暂停保留平移/旋转路线并撤销旧结果，恢复重验；LOST/会话/地图/跳时/遥控清路线。89回归与构建通过，最新60cm实车成功约8.6秒、零速18.3%、计划1次、峰值0.165m/s；0.30持续巡航/全走廊/独立速度标定仍未验。设备已停。用户自行移动过车辆，return_pause_fix零命令成功不算运动证据。详见docs/exact_mppi_speed_retest_20261116.md。

最新录包清理（2026-11-16）：用户要求只保留11月10日预建图原包。已执行，全项目实际录包数据仅剩bags/fast-livo-office_corridor_changed_20261110-s10xmu6g/raw/raw_0.db3。其他录包包括标定原始录包/回放/测试数据均已删除；地图、标定参数和结果、日志保留。可用约791GiB。所有设备已停止，不要沿用下文旧原包“仍存在”假设。删除完整清单见docs/cleanup_evidence_20261116/；未提交/推送。

当前最新状态补充（2026-11-10提速阶段）：先读STATUS/HANDOFF最上方和变化场景实车报告最新章节。覆盖1.5秒，显式RANGER_NAV_MAX_SPEED=0.30；已修计算抖动重规划、无session故障误淘汰、末端微旋转等问题。最近实车零指令约26%但仍慢，末端/进展代价最终修改未获实车复验（C2连续未就绪，两次goal=null），当前设备均停止。不得宣称已完成提速或走廊往返。

# 新上下文接手

先读 AGENTS.md、STATUS.md 顶部当前实车阶段、HANDOFF.md 和 [实车报告](exact_mppi_changed_scene_live_report_20261110.md)，再读EXACT计划与对应接口。旧报告中的未开设备、旧车头约-179°、旧地图门口堵塞均属历史快照。

同一实施会话已获软件和本次实车授权，用户现场确认新正方向；独立新接手须核对最新消息与受管boot/PID，不默认复用设备许可。当前车头为相机镜头朝向端，雷达距此端约26cm；工作RPY约[1°,21°,0.9°]，精确值读配置，不恢复旧-179°或零倾角。SDK原生纵轴相反，由install_exact驱动统一转换；receive-only也必须使用同一新坐标二进制与显式receive_only=true，旧安装保留。

新图UUID160f8572a77a61856cf87e43514be769，归档outputs/fast-livo-demo-changed-scene-20261110/mapping_replay_01/map，导航navigation_map_05。车头修正后候选camera_front路线6/7静态通过，最终回办公室恢复朝向仍搜索超时；不是实车路线成功。当前优先完成短单点与连续采样时延问题，不直接发送全程。用户正向实测约9.3cm已确认。

全部修改未提交/推送。保留原bag、地图、失败日志、旧安装和.runtime/backups。生产runtime仍Lightning/wheel，Tag关闭，正式+100ms三融合，公共TF唯一、采样同刻、wheel不外推、定位/障碍过期停车和独立末端防撞不放宽。新源码/安装身份与所有本轮失败对照在实车报告及证据目录。当前任务不是重新选择算法或恢复旧Smac/MPPI调参。
