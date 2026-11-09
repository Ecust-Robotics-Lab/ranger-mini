# 相机分线程与局部恢复修正后的90秒实机短测

日期：2026-11-09。用户明确授权“那再进行一次实机短测吧”。复用上一轮90秒/public TF/receive-only wheel范围，车辆由用户遥控；Agent未启动导航控制器、未发速度或目标。本次正常结束，无自动重启、无提交/推送。

## 执行与版本

```bash
bash scripts/localize_fast_livo2.sh \
  --archive outputs/fast-livo-demo-online-mapping-20261105/full_replay_12/map \
  --public-tf --auto-wheel \
  --navigation-map outputs/fast-livo-demo-navigation-prep-20261106/navigation_map_final \
  --domain 215 --seconds 90 --min-free-gb 100 --visualization none --execute
```

Git起始694b8b6d463f07a340b2c66b6b5cab4dd2938668，起始工作树干净。boot_id dc7ccd75-a884-43af-921e-f285f18e2264。开始前无相关设备/底盘/定位进程，can0已UP/500k/ERROR-ACTIVE，磁盘空闲约524GiB，未改变系统/网络。核对上一轮`binary_hashes.json`各节点/库与runtime全部一致。

本次包`bags/fast-livo-localization-co968xa3`，会话`run-ef5z0dya`。真实进程`/proc/PID/exe`及maps确认使用`ws_camera/latency_install`节点/相机库、`online_localization_install`定位器；receive-only驱动df58f4…，原相机/底盘安装保留。正式+100ms、三融合、v2/KISS/small_gicp、既有工作外参和地图身份保持。

实际录制90.044472890秒，recorder exit0、forced=false，外层exit0，录后摘要OBSERVED。原bag有903条FAST位姿，录制器观察901对，计数口径分别保留。受管收尾STOPPED、owned=[]、guardian_cleanup_complete=true；结束按/proc核对无相机/雷达/底盘/前端/定位残留，本次只读ROS观察器也已正常结束。

## 实测结果

| 项目 | 结果 |
| --- | --- |
| 相机同步CSV | 921帧，全部OK，BACKWARD/MISMATCH/队列丢帧均0 |
| 相机取帧返回间隔 | 最大101.097ms，P95 100.185ms |
| SDK取帧调用等待 | 中位97.114ms，最大110.388ms；10Hz下等待下一帧属于正常机制 |
| 转RGB | 最大7.881ms |
| 转换结束到raw发布开始 | 最大0.303ms |
| raw发布 | 最大5.261ms |
| raw发布结束到sync发布结束 | 最大6.107ms |
| FAST位姿接收间隔 | 最大201.076ms |
| C1有效确认 | 591个不同采样，无LOST/ERROR心跳 |
| 首次VALID | 距首FAST采样30.700秒，距首FAST记录接收30.994秒 |
| 公共就绪 | 首次就绪后3次短暂撤销，约120.478/119.966/60.522ms |
| 派生wheel | 4641条，零领先最新已记录clock，零采样戳回退，时钟无锁定错误 |
| TF | 记录odom→base_link 4641、map→odom 2946；运行观察无重复动态TF端点 |
| 速度发布者 | 全部图观察为0 |

三个就绪撤销均与C1 `not_converged`/DEGRADED吻合，随后`map_match_confirmed`恢复；对应health为backend_valid=0。没有此前“private有效但odom缺失”的观察，分析统计为0。原包C1状态计数INIT1855、VALID2946、DEGRADED15。

图观察共91个有wheel/TF的样本：90个明确识别动态TF唯一发布者为ranger_localization_bridge，1个启动发现样本名字仍为_NODE_NAME_UNKNOWN_，但只有1个端点。保留该未知样本；不能将程序中的“所有名字均识别”布尔false当成发现重复TF，亦不能声称91个都已识别名称。其余19个为空图或清理阶段。实际加载记录见`active_linkage.json`。

wheel积分路程约13.04m、最大反馈速度0.567m/s，仅代表里程计观测，不是独立路径真值。现场CAN原始0x211样本`00 03 01 F6 00 00 00 00`，按既有格式为正常0、遥控模式3、完整故障0；仅对应取样时刻。CAN收尾保持既有UP500k，无控制请求。

## 结论与局限

相机分线程、有效同步输出、实际版本加载以及C2→C1/public TF链路本次功能观察PASS。相机本轮没有重现此前527ms交付积压，故分段计时能确认本轮没有大等待，但不能唯一解释上一轮SDK/USB或调度根因，也不能证明所有未来积压都已消除。

本轮没有进入LOST，未触发短时局部恢复分支；因此不能将两帧离线恢复或本次无LOST报告为实机1–2秒恢复已验收。首次无初值确认仍需约31秒，且有三次短暂几何拒绝，连续就绪整体仍PARTIAL；不同轮次轨迹/视角不同，不将31秒与此前50秒直接当算法同条件加速结果。

下一步可基于本包只读分析三次局部几何未收敛与首次全局搜索等待，决定C1计算/全局候选优化优先级；若需现场验证局部恢复，须另设计明确的短时故障场景并取得该次设备许可。本轮没有继续开设备、调整门槛、切换SDK策略/QoS或改规划控制。物理同步、绝对精度和自主闭环未验。

## 证据与复现

证据`docs/fast_livo2_evidence/field_latency_retest_20261109/`，含启动命令输出、preflight、真实maps、CAN样本、每秒图/健康观察、相机日志、录后结果、最终会话状态和原包只读分析。原包及sensor_assets中的完整CSV保留。

分析命令：

```bash
source /opt/ros/humble/setup.bash
source ws_fast_livo/online_localization_install/setup.bash
python3 docs/fast_livo2_evidence/field_latency_retest_20261109/analyze.py
```

exit0，结果`analysis.json`。只读SQLite，不重写原header或重放传感器。唯一性图统计与原bag消息统计分开，录后OBSERVED只表示曾完成链路，不代替连续就绪、恢复触发或真值验收。

## 后续澄清：约31秒首次VALID不等于31秒计算

用户指出此前已有首次定位优化，复核确认：2026-11-06相同282秒访问的旧v1/PCL首次51.494秒，v2/PCL为24.394秒，v2/GICP为22.315秒，优化及默认仍有效，见检索对照报告。不能与本轮不同运动时序的31秒直接比较退化。

本轮`localization_samples.jsonl`在首次VALID前记录87个不同耗时/状态的`verifying_insufficient_time_or_motion_baseline`结果，第一项距首FAST接收约0.7秒即出现，C2确认交接约在28.8秒，随后约2.2秒C1初始化/新鲜扫描接续才VALID。实际FAST相对首帧位移在10/20/25秒分别约1.9/1.0/3.1mm，姿态变化极小；到28秒才约7.35cm/0.127rad。主要等待发生于近静止采集阶段的运动基线确认，不是检索/配准连续消耗31秒。

当前C2要求至少两个不重叠查询、0.25秒及0.15m或0.08rad运动支持；这是现有验收策略，静止初始化可用性需另行设计，不能仅通过减少线程耗时解决。C1交接两次初始化约1091/1067ms，确有重复计算优化空间，但约为末尾2秒，不应将整段31秒统称计算瓶颈。本轮未修改运动门槛或初始化策略。
