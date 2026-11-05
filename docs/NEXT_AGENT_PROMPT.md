# 新 Agent 开场提示词

直接复制下方内容到新对话。该提示词先做只读接手；之后发“继续固定 +100 ms 的离线对照”即可明确下一阶段范围。

```text
请接手 /home/lab307/ranger_nav。先读 AGENTS.md、docs/HANDOFF.md、docs/STATUS.md、docs/FAST_LIVO2_BACKEND_PLAN.md 和 docs/FAST_LIVO2_APRILTAG_PLAN.md，再重点读：
- docs/fast_livo2_timestamp_phase_report.md
- docs/fast_livo2_c1_correction_report.md
- docs/fast_livo2_tag_two_capture_report.md
- docs/handoff_cleanup_20261105.md
- docs/local_git_checkpoint_20261105.md

核对实际Git分支/HEAD、未提交工作树、会话状态、原包/地图和真实二进制位置，保护既有改动。先简短汇报接手理解和下一步，暂不运行新实验。

2026-11-05已有成果已按我后续指令分7组本地提交，未推送。FAST五个受管会话未运行，但只读接手时field存活且DEGRADED，旁边ranger_nav_k_integration另有雷达/Nav2进程；重新现查，不擅自停止或接管。

关键事实：正式前端试验 img_time_offset 已由我指定为 +0.1 s，完整+100ms评估尚未跑；历史+110ms的GICP结果不能追认为+100ms。Tag默认关闭，独立配置仍offset0/time_verified=false，没有真实VALID地标表；所有已报真实C2成功均Tag关闭。跨访问17.32→6.11px是整条定位与图像观测的一致性，不是Tag单独精度；初始像素门槛不是普适真值。

当前Tag是tag36h11:1，打印后实测140mm黑色边界，Tag和传感器安装跨两新包均未改，不必再问。已有turn03和straight04两份90秒原包，当前不用重录。旧155mm实例不要合并。

后续我让你继续时，优先复用锁定源码及原零gyro初始化的隔离构建，用已有两包固定+100ms完整三融合重放，另存新输出，与既有0/+110ms结果按同集合/冻结规则比较；需要时重建新地图版本，再推进Tag时间映射、地标登记和真实C2/B验收。不要推倒重做B/C1/C2，不反复扫描时偏后冒充独立验证，不随意放宽门槛。

ws_fast_livo/backend_install 是既有受管默认，install_c1_confirmation_20261104 是已验隔离新C1；核对实际节点/库，不能混用版本。硬件、底盘/Nav2、部署、Git提交/推送都不在本次授权内。清理仅按20261105报告已完成的具体名单，不继续批量删除原包、地图、失败证据或备份。
```
