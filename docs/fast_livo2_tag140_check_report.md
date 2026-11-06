# 140 mm Tag 检查包：录制与离线筛查

日期：2026-10-29。用户自行录制，包为 `bags/fast-livo-tag140_check_01-dguhgx2l/`。本轮 Agent 只读分析，没有启动/重启传感器、连接底盘、改变同步或登记地标。

**结论：录包成功，新包可进入后续离线跨帧一致性分析，暂不必仅为增加检测数量重录。** Tag 图像覆盖与条件性平移基线明显增加；录制期间仍有重复共享戳导致同步图像跳帧，物理同步和定位准确性仍未验收。

## 1 录制与完整性

录制约 60.001 s，`exit=0`、`session_exit=0`、`forced=false`、`capture_complete=true`、CAPTURE_COMPLETE 存在；受管会话最终 STOPPED、owned=[]、guardian_cleanup_complete=true。`observed_messages` 是预检/录制监视器看到的消息数量，不能作为 SQLite 内实际保存数。

| 包内实际话题 | 消息数 |
| --- | --- |
| LiDAR | 600 |
| IMU | 12001 |
| 原始图像 / CameraInfo | 各 600 |
| 同步图像 | 593 |
| 前端 odometry | 593 |

原包 `raw/raw_0.db3` 为 5017432064 bytes，离线筛查前后 SHA256、mtime、文件大小均一致；SHA256 为 `190637437b9cd7cb8e27ca4cda3168cc59a6af11c425334bf12b44a330432479`。SQLite 以只读/query_only 打开，不运行 ROS graph、bag play 或前端。归档配置源文件逐份 SHA 验证；包内全部 CameraInfo 与本次归档模型一致。

所有已存话题 header 单调，图像与 LiDAR 593 次同戳；这仍只证明写戳配对。11999904 个点时间没有负偏移、非有限值或扫描内回退，IMU 无非有限消息。不能据此宣称曝光物理同步。

## 2 同步 PARTIAL 的原因

整个设备会话 CSV 为 OK 613、BACKWARD 8，硬件帧 ID 未缺失/回退。按已录同步图像共享戳范围，1 次 BACKWARD 在录前约 0.400 s，另 7 次位于窗口内约 10.700、33.800、38.100、42.200、53.700、56.200、57.800 s。

源码 `readTimeshareConsistent()` 将 `value <= last_used_low_` 统称 BACKWARD。本包 8 次异常与前一 CSV 行的共享 seq 和 stamp **完全相同**，不是 stamp 变小；异常图像保留在 `/front/image`，未发布到 `/front/image_sync`。包内 600 原图与 593 同步图像的数量差与窗口内 7 次拒绝相符。

这说明相机读共享值与 writer 更新的跨周期对应仍需排查，不能直接推断触发线故障、真实时钟倒退或某个确定时偏。没有自动修改驱动、共享协议、曝光参数或时间补偿。

## 3 全部同步图像的 Tag 检测

使用本次归档相机模型和 [140 mm 尺寸表](../config/fast_livo2/apriltag_sizes_capture_140mm.json)，复用已有官方 Detector 与有界流式图像工具。没有重做检测算法，也没有使用旧 155 mm 尺寸或旧前端回放轨迹。

| 指标 | 实际结果 |
| --- | --- |
| 全量处理同步图像 | 593/593，1280×1024 rgb8 |
| 检出图像 / 观测 | 556 / 556，仅 tag36h11:1 |
| 观测时间跨度 | 58.800 s |
| 单帧像素/质量初始门槛通过 | 556/556 |
| 位姿假设 | 每帧保留 2 支；每帧仅 1 支通过当前像素门槛 |
| 逐帧 hypothesis 0 RMS 中位数 / 最大 | 0.760 / 2.262 px |
| 逐帧 hypothesis 1 RMS 中位数 / 最大 | 4.273 / 11.792 px |
| decision margin 最小 / 中位数 | 119.86 / 199.99 |
| 两支估计距离范围 | 0.632–0.936 m |
| Tag 观测的本包局部位姿关联 | 552/556；550 包围 SLERP、2 精确 |
| 被拒绝的关联 | 4 次，间隔约 200 ms，超过原 150 ms 限制 |
| 已关联正观测的最大相机平移基线 | 条件性估计 0.546 m |

556/593 是检出帧占比，**不是召回率**；没有全面人工可见性标签。hypothesis 编号只在单帧内有效，不应当作跨帧分支身份。每帧有一支通过初始像素门槛，不证明全局定位唯一或正确。

关联只在本包在线局部 odometry 上进行，保留 int64 纳秒、包围样本和原门槛，没有外推或拼接旧图轨迹。4 次约 200 ms 间隔发生在相对首个正观测约 10.900、34.000、38.300、53.900 s，已经拒绝，而不是放宽门槛。

局部基线、距离依赖用户声明尺寸、当前外参、未验证的零时偏和前端估计，不是外部真值；本轮没有计算 C1 地图位姿或固定 Tag 地图姿态。原图首/中/尾三帧保存于输出目录，抽查中间原图可见完整且可辨认的标签。

## 4 命令与证据

新增只读筛查入口 [screen_capture.py](../scripts/fast_livo2/tags/screen_capture.py)，默认 dry-run，输出必须为新目录。首轮遗漏项目内 detector 共享库搜索路径，退出 1，失败日志与输出保留；补齐项目内路径后在 v2 新目录重跑，退出 **0**，不改系统链接器配置。

```bash
source /opt/ros/humble/setup.bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/time -v python3 -u \
  scripts/fast_livo2/tags/screen_capture.py --execute \
  --capture bags/fast-livo-tag140_check_01-dguhgx2l \
  --sizes config/fast_livo2/apriltag_sizes_capture_140mm.json \
  --output outputs/fast-livo-apriltag-tag140-check-20261029-v2
```

全命令耗时 28.98 s，峰值 RSS 219484 KiB；detector 处理时延中位数/P95 为 33.30/39.78 ms，属于离线资源记录。输出为 summary、593 行帧记录、556 行观测/双解/局部关联和 3 张原图。本轮是数据筛查，没有增加算法单元测试；复用此前检测与时间工具的已有测试，不能把此前测试数当成本轮重跑。

证据：[summary](fast_livo2_evidence/apriltag_tag140_check_20261029/summary.json)、[成功日志](fast_livo2_evidence/apriltag_tag140_check_20261029/screen.log)、[首轮失败](fast_livo2_evidence/apriltag_tag140_check_20261029/first_attempt.log)、[实际包审查](fast_livo2_evidence/apriltag_tag140_check_20261029/capture_audit.json)、[最终核对](fast_livo2_evidence/apriltag_tag140_check_20261029/verification.json)。完整输出在 [本机目录](../outputs/fast-livo-apriltag-tag140-check-20261029-v2/summary.json)。

## 5 状态与后续

**PASS**：采集完成/受管清理；新包原始 SHA 不变；归档模型/尺寸；全量离线检测；两支假设保留；缺时间支持的观测拒绝。

**PARTIAL**：7 次录制窗口内重复共享戳；曝光与扫描物理相位、固定安装声明/跨帧固定性、全局坐标和精度；新的独立访问评测。旧包的固定声明不自动适用于这次新打印标签。

**未实施**：修改同步驱动/时偏/安全门槛、创建真实 VALID Tag 表、C1/C2/B 全局提交、Agent 启动设备或底盘、Git 提交/推送。

下一步优先利用此包排查重复共享戳及时间对应，再在确认标签确实固定的前提下做跨帧固定地标一致性检查；没有因为这次 PARTIAL 自动要求重录。独立验证访问仍应另包采集，正式更长包前先解决会影响几何解释的同步问题。

后续更新 2026-10-29：用户已独立确认新标签固定未移动；[同步与固定地标一致性分析](fast_livo2_tag140_consistency_report.md)完成。writer 漏收已录扫描与重复共享戳对应，跨帧残差仍不足以登记；上文待确认事项以该后续报告为准。
