# 新 Agent 开场提示词

复制下方内容到新上下文。先核对并汇报接手理解，后续实施范围由用户另行指定。

```text
请接手 /home/lab307/ranger_nav，先做只读交接，暂不继续实施。

依次读 AGENTS.md、docs/HANDOFF.md、docs/STATUS.md、docs/FAST_LIVO2_BACKEND_PLAN.md、docs/FAST_LIVO2_APRILTAG_PLAN.md，再重点读：
- docs/fast_livo2_online_mapping_report.md
- docs/fast_livo2_online_mapping.md
- docs/handoff_cleanup_online_mapping_20261105.md（最新清理）
- docs/fast_livo2_corridor_classroom_capture_report.md
- docs/local_git_checkpoint_20261105.md
时间/Tag历史再查HANDOFF指定报告，勿直接执行历史实验脚本。

核对实际Git分支/HEAD、未提交改动、会话的boot/PID身份、配置、二进制与动态库、原包和地图，保护全部未提交成果。先简短汇报当前已完成能力、剩余限制和建议下一步；不自动开设备、重放实验、提交或推送。

当前重点是完整在线预建图。map_fast_livo2.sh已统一三融合前端、B回环优化、C++收流计数、录包、显示及停止封存，默认dry-run，显式--execute才运行。record_fast_livo2.sh单独录包默认执行/启动传感器/读取FAST profile，现补存去畸变云，但不启动B；不要把入口默认行为当Agent的硬件许可。

软件链路已用390秒真实包原速DDS重放验证：3900对输出，305关键帧、114条接受回环，地图归档和共用读取器加载PASS。正式成果：outputs/fast-livo-demo-online-mapping-20261105/full_replay_12/，地图UUID253f573d7a7eabbc363a225ab192cc14，map/map.pcd与map_trajectory.png可直接查看。来源原包：bags/fast-livo-corridor_classroom_map01-182fdzyj/raw。它原先没有去畸变云，停录后审查中断的事实仍保留；另有审查PASS证据。

这还不是成熟系统验收：此前确有约1.11秒实际接收延迟；交付配置整包IMU最大记录到达间隔12.51ms，但仍有45个LOST后端心跳。真机同时运行、连续新鲜度、长时、误回环泛化、绝对精度、独立访问定位/重定位、wheel同步与D仍待验。不得把旧停顿统称误报或把地图加载当定位通过。最终整包收尾曾有SHM警告但成功，随后仅UDP控制面调整另做6秒回归，未再次重跑390秒。

新程序在ws_fast_livo/online_frontend_install与online_install，可追溯派生源码在online_frontend_source；原安装保留。C1优化隔离版本在install_c1_confirmation_20261104，与旧backend_install不可混用。2026-11-06用户已另行授权分组保存并上传Git，提交分组和核对见docs/git_checkpoint_20261106.md；分支refactor/pluggable-navigation，当前HEAD和远端同步状态须现查，不沿用旧的领先15提交快照。

正式前端img_time_offset固定+0.1秒、重力对齐开启。新390秒前端/B整包已完成；原两90秒固定+100ms对照及C1/C2评估仍待完成，历史+110ms结果不能追认为新配置结果。Tag默认关闭、独立时间未确认、没有真实VALID地标表，所有已报真实C2成功均Tag关闭。tag36h11:1实测140mm黑色边界，跨两90秒包Tag及传感器安装未变，用户已确认，不必再问，当前无需重录同类包。

最新清理仅删除可从原包重建的replay_view和Python缓存，重建方法见清理报告。原包、全部地图、失败记录、构建与.runtime/backups均保留，不继续批量清理。最近会话指针是已停止的6秒final_control回归，不是正式full_replay_12；会话历史不能代替当前活进程核对。

后续依我的新指令决定是真机在线建图验收、新地图独立访问定位，还是原两90秒/Tag离线工作；不默认续跑旧计划。生产Lightning/wheel/公共TF/导航链保持，私有sensor-clock不等于wheel物理同步。
```
