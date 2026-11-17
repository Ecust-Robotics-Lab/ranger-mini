# 单 P0 首次有效闭环结果与分析修复

日期2026-11-11。输入为用户实际运行的新组 `experiments/fast_livo2_p0_closure/runs/20261111_171817-liu6_7ya`。本次Agent只读原包并离线修复/重算，没有启动或接管设备。

## 数据与结果

第1轮标签first，用户确认出发和返回P0中心/朝向；行走166.763882秒、原始前端估算行程31.470285m。原包检查无失败/告警，2136条相机同步记录全部OK，100关键帧、52接受回环/52回环因子，最终图版本152。地图UUID `e009aa98967fa6b33cb84b7a2803888a`，定位会话 `27b0eb1139795731b7e8c87df493771e`。映射会话退出码0，归档完整提交。

停稳后11～13秒窗口主结果：

| 轨迹 | XY闭环位置误差 | 航向误差 | 按共同前端估算行程归一化 |
|---|---:|---:|---:|
| 原始前端 | 0.463cm | 0.139° | 0.0147% |
| 最终图校正历史 | 1.961cm | 0.094° | 0.0623% |
| 当时在线后端 | 1.981cm | 0.116° | 0.0630% |

停稳后1～3秒即时窗口分别0.633cm/0.163°、1.834cm/0.119°、1.816cm/0.119°。上述百分比不是独立实测路程指标，独立漂移率字段保持null。

结论：本轮在线与最终历史结果接近；后端平移返回差大于原始前端，航向差略小。不能据此宣称后端精度优于前端。前端起点静止窗口位置跨度约1.60cm，终点即时/等待窗口约3.90/2.97cm；0.463cm的中位数首尾差不能解释为毫米级精度。后端等待窗口高度差约−3.72cm，仅作为6DoF诊断，不与XY主指标混合。

观察器记录的行程内有效新鲜不同采样覆盖约82.37%，最长采样间隔约0.3001秒。这是只读观察器的采样覆盖，不是定位成功率；观察器BEST_EFFORT、后端心跳20Hz与输入10Hz等会影响可观测采样集合。主指标所需各窗口均满足至少10采样、跨度70%、最大缺口0.5秒条件，无5cm/2°步进告警。P0由人工复位，没有独立全程真值，不报告APE/RPE。

## 原分析失败与修复

原分析将整个进程生命期所有非空session_id混在一起检查。发现前端发布者之前，B正常发送session=`1ada2dd358623280dd88b4860298e3d7`、stamp=0、INITIALIZING、pose_valid=false的心跳；建立输入身份后生成最终会话。前一会话最后消息在出发前约30.44秒。真正测量区间（起点静止窗口开始至终点15秒记录结束）仅包含最终会话。

因此原汇总FAILED来自后处理误判，不是本轮建图或回环失败。`analysis.py`改为对测量区间严格核对会话，区间内任意新会话即使无效也拒绝；区间外初始化心跳单独记录。新增分析源码SHA、测量会话及忽略的前置会话字段，重算写新目录。

原failure.json、采集退出记录、原包和地图均保持；组内before_analysis_recovery_*文件保留原汇总/进度，progress中的analysis_recovery记录误判与新结果路径。当前第1轮COMPLETE，2～5轮PENDING，组保持STOPPED待用户继续。

为继续剩余4轮，采集配置/源码/二进制身份保持冻结；只对离线分析代码升级及来源检查函数修正允许同组继续，runner在检查函数以外须保持AST一致。原source_hashes.json不改，后处理版本登记analysis_revisions/；采集逻辑或二进制改变仍拒绝混入。

## 命令、验证与证据

- `bash experiments/fast_livo2_p0_closure/run.sh --analyze /home/lab307/ranger_nav/experiments/fast_livo2_p0_closure/runs/20261111_171817-liu6_7ya/trial_01_attempt_01`：退出码0，PASS_DATA，仅表示数据及指标可计算。
- Humble+B消息环境下 `python3 -m unittest discover -s experiments/fast_livo2_p0_closure/tests -v`：退出码0，21项通过，覆盖前置初始化会话忽略、测量期会话改变拒绝、来源修订和采集身份变化拒绝。
- 只调用 `suite.verify_frozen` 与`summarize`，核对采集身份不变并恢复汇总，没有执行resume或启动设备。

指标、原始窗口统计、轨迹图与日志见[证据目录](fast_livo2_evidence/p0_closure_first_result_20261111/)。实际完整轨迹CSV保存在本次组内analysis/result-1794389803044942979/。软件改动与原进度备份 `.runtime/backups/p0_analysis_session_scope_20261111_173631/`。未提交或推送。

当前只能评价这一轮。剩余4轮可由用户在P0就位后运行：

```bash
bash /home/lab307/ranger_nav/experiments/fast_livo2_p0_closure/run.sh --resume /home/lab307/ranger_nav/experiments/fast_livo2_p0_closure/runs/20261111_171817-liu6_7ya
```

该命令由用户运行，会按原流程从第2轮开始；本次Agent未执行。
