# 第二次授权现场90秒联调

日期：2026-11-09。用户明确要求“重新启动”后执行。命令沿用上一份[现场报告](fast_livo2_field_localization_report_20261109.md)，90秒、domain215、无可视化、public-tf＋auto-wheel，只接收底盘/用户遥控，无导航控制器、速度命令或目标。本轮私有重定位功能已观察到成功，公共TF连续就绪仍PARTIAL。

## 本次真实结果

- 目录`bags/fast-livo-localization-jbwq4yhi`，会话`run-sqbzr49k`，实际90.026777秒，正常定时结束，录包exit0、forced=false，外层exit0。STOPPED、owned=[]、受管清理完成，结束后未自动重启设备。
- 稳定窗口修正已实际生效：wheel时间状态无错误，派生wheel4766条；启动首帧仍有积压，但没有再污染锚点。物理同步仍未验证。
- 记录903对FAST位姿/去畸变区间云。C2无人工初值确认后交给C1，486个不同有效确认采样，首次VALID距首帧FAST输出42.399878秒。消息有VALID/短暂DEGRADED，无LOST/ERROR状态；不据此报告位置真值精度。
- 原wheel4890条，里程计积分路程约4.49m、最大反馈速度0.472m/s，说明录得运动；该路程不是独立真值。
- 动态TF记录包含1898条odom→base_link、1564条map→odom；navigation_ready有3897条true，但发生296次状态段切换。因此录后摘要的OBSERVED只代表曾经完成公共接入，不等于持续就绪通过。
- 现场原始0x211样本为`00 03 01 F2 00 00 00 00`，按此前核对的V2格式为正常状态0、遥控模式3、完整故障0；不能外推为自主控制就绪。

## 新发现与修正

一、原转发wheel立即发布，4766条中有3167条采样戳领先最新已记录的`/clock`，最大约20.33ms。桥接严格拒绝未来采样，造成odom_present/map_transform等条件反复失效。没有放宽fresh()或重新打now。

已修为有限队列等待时钟：原戳与位姿保留，等后续时钟刻度已越过该采样再发，给下游上一刻度回调留时间。队列有容量/年龄限制，暂停或回退仍拒绝。基于原记录时钟顺序的离线模拟4765条释放、零未来释放；这不是新现场测试，仍需实机确认时序改善。

二、图发现底盘即使publish_odom_tf=false也会构造TransformBroadcaster，产生第二个空TF发布端点。原代码在false时不发实际TF，但该端点仍违反唯一发布者检查。已按publish_odom_tf条件创建广播器，更新可复现receive-only补丁和源码生命周期shim；只重建隔离recording_rx安装，原ws_base/install保持SHA不变。新驱动SHA为`df58f4c6015c6c23145a75e4776dde7bcc170c4e52c53139d906b423163f63fa`，本次实际采集使用的是修正前ae5f3f…驱动，不能追认新端点行为已实测。

## 修正验证与证据

证据位于[本轮目录](fast_livo2_evidence/field_localization_20261109/)，新字段/报告含field_03或gated前缀，保留前两次失败证据。

- `test_fast_livo_live_chain_ros.py`：隔离domain228，实际monitor/时钟转换/适配器/桥接，启动积压、wheel和receipt断流恢复、锚点异常锁定均通过，exit0，7.927秒。
- `test_fast_livo_live_clock_ros.py`：明确验证未来wheel先暂存、时钟推进后原戳原位姿转发、断流停止，exit0，2.084秒。
- `bash scripts/build_base_recording.sh --execute`：隔离底盘构建exit0，12.4秒；既有控制路径steer_cmd编译警告保留，该路径未用于只接收运行。
- `scripts/check_base_receive_only.py`：实际源码shim通过不请求模式、不订阅速度、不退出发零速；关闭TF时不构造广播器、打开时构造。补丁reverse check通过。未启动真实驱动来做此检查。
- 运行时图：零速度发布者，唯一wheel源、唯一C1源；当时TF端点2个的问题如上保留。结束后无本次传感器/底盘/定位进程，can0仍为本轮既有UP500k，未改网络/系统或生产runtime。

后续只需在明确授权的新实物会话中复核等待时钟后的公共TF就绪稳定性与TF唯一端点，同时观察C1短暂几何拒绝。当前不将软件修正后的隔离通过算作新实车通过。所有原包、地图、失败证据与源快照保留，尚未提交或推送，规划控制未改。
