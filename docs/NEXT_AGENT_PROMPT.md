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
