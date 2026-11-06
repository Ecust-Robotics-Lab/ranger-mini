# 同步漏收缓解与下一次车载诊断录制准备

日期：2026-10-29。用户希望通过再录包排查并解决潜在同步问题，确认传感器固定在车上、由用户手动操控。本轮完成软件修改、构建、隔离合成测试和录制安排；没有启动真实雷达/相机、底盘、导航或采集，没有提交/推送 Git。

**当前可用：writer 改为可靠订阅，新增可追溯的交付诊断，下一次录制入口已准备好。真实漏收是否消失、曝光关联是否正确仍待新包验证；没有设定 100–125 ms 补偿。** 先前证据见 [140 mm 一致性分析](fast_livo2_tag140_consistency_report.md)，用户执行命令和 120 秒动作表见 [录制指南](fast_livo2_recording.md#本次-140-mm-标签的车载同步诊断)。

## 1 修改与保留的边界

| 实际文件 | 修改与作用 |
| --- | --- |
| `ws_camera/src/timeshare_writer/src/timeshare_writer_node.cpp` | `lidar_reliability` 默认 reliable，可显式 best_effort 对照；`lidar_queue_depth` 默认 5、有界 1–100；错误参数在碰共享文件前退出 2。增加实际 QoS/RMW、DDS 丢失/不兼容事件及逐样本来源/时序记录 |
| 同包 `CMakeLists.txt` / `package.xml` / `launch/timeshare_launch.py` | 显式声明既有 rmw 依赖，launch 暴露可靠性选择；没有新增第三方算法或升级系统 |
| `scripts/fast_livo2/online_session.py` | 受管启动 argv 显式记录 reliable / depth 5，录包和预览共用此入口 |
| `scripts/fast_livo2/record_session.py` | 归档 writer/相机源码和实际二进制，均保留原字节及 SHA |
| `scripts/fast_livo2/record_worker.py` | 新增录前 `subscriptions_before.json`，记录订阅端实际 QoS/GID；原发布端快照格式保持 |
| `tests/test_timeshare_delivery.py` | 实际 writer 的大消息对照、10 Hz/压力条件、队列停顿、参数错误与 QoS 不兼容测试 |
| `tests/test_fast_livo_recording.py` | 在相关既有用例检查可靠启动参数、订阅快照与二进制归档 |

未改变原始扫描戳、v2 seqlock、共享文件所有权和相机重复戳拒绝；未改图像戳、相机驱动、曝光、标定、前端融合或 Tag 开关。可靠交付可能重传或积压，故不能把“完整收到”当成“及时且正确对应曝光”。本轮记录延迟供核查，不凭主机处理时间自动重写传感器时间。

CSV 原前五列保持，新增 `callback_system_ns`、`dds_source_ns`、`dds_received_ns`、`publication_sequence_number`、`publisher_gid_hex`、`dds_lost_total`。arrival 是 steady，callback 是 system，DDS 时间语义来自中间件；未知时间/序号为空。消息序号必须结合 publisher GID；DDS 事件不是所有丢失途径的总账。若 RMW 不支持事件，明确告警、事件列留空并保留相同 QoS，不静默降为 BEST_EFFORT。不兼容发布者明确记录错误，原受管就绪等待/失败清理继续有效。

当前 RMW 为 `rmw_fastrtps_cpp`；本机 `ros-humble-rclcpp 16.0.19-1jammy.20260724.021311`、`ros-humble-rmw-fastrtps-cpp 6.2.10-1jammy.20260724.002510`。接口以本机 `rmw/types.h`、`rclcpp/qos_event.hpp`、`subscription.hpp` 核查；没有调整 DDS/内核/网络参数。

## 2 合成证据与实际命令

证据目录：[sync_capture_diagnostics_20261029](fast_livo2_evidence/sync_capture_diagnostics_20261029/)。最终 [大消息对照](fast_livo2_evidence/sync_capture_diagnostics_20261029/delivery-comparison.json)每组单独进程、相同 600000 字节合成消息、队列深度 5、localhost/domain 178，负载批次的消息不重发；小消息探针只确认批次前后通路可用。没有接入真实 topic 或设备。

| 合成条件 | BEST_EFFORT 收到/发送 | RELIABLE 收到/发送 |
| --- | --- | --- |
| 请求 10 Hz，实际均约 10 Hz | 0/30 | 30/30 |
| 请求 50 Hz，实际分别约 43.85 / 41.81 Hz | 1/80 | 80/80 |

这些是本机一次合成条件下的计数，不能外推实际机器人的丢失率，也不证明旧包 7 次漏收的每次原因；结果支持使用可靠订阅进行下一次实采对照。DDS 丢失事件在 BEST_EFFORT 对照中可见。两个模式均能收到批次前后的小消息，避免把未发现端点误报成大消息丢失。

实际执行，最终均退出 0：

```bash
source /opt/ros/humble/setup.bash
# 在 ws_camera 目录执行；只编译 writer，不打开设备。
PYTHONDONTWRITEBYTECODE=1 colcon build --packages-select timeshare_writer \
  --executor sequential --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
  --event-handlers console_direct+

# 以下在仓库根目录顺序执行；不要与受管录制测试并行启动 writer。
ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 \
  EVIDENCE_DIR=$PWD/docs/fast_livo2_evidence/sync_capture_diagnostics_20261029 \
  python3 tests/test_timeshare_delivery.py

ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 \
  python3 scripts/fast_livo2/test_timeshare_writer.py

PYTHONDONTWRITEBYTECODE=1 \
  EVIDENCE_DIR=$PWD/docs/fast_livo2_evidence/sync_capture_diagnostics_20261029 \
  python3 tests/test_fast_livo_recording.py \
  CaptureUnit.test_plan_no_motion_and_qos_and_no_sim_clock \
  CaptureUnit.test_validation_and_sync_evidence \
  CaptureROS.test_finite_capture_archives_and_preserves_external_preview \
  CaptureROS.test_owned_synthetic_session_is_cleaned_after_finite_capture

bash scripts/record_fast_livo2.sh --start-sensors --seconds 120 \
  --label tag140_sync_diag_02 --visualization foxglove
```

- [最终构建](fast_livo2_evidence/sync_capture_diagnostics_20261029/build-final.log)：1 包，退出 0。
- [交付诊断测试](fast_livo2_evidence/sync_capture_diagnostics_20261029/delivery-tests-acceptance-v2.log)：7/7，13.840 s，退出 0。停顿测试只向自建合成 writer 发 SIGSTOP/SIGCONT，确认延迟可见且原扫描戳不改；不操作设备进程。
- [既有共享协议测试](fast_livo2_evidence/sync_capture_diagnostics_20261029/seqlock-test.log)：1 个检查脚本，5976 个一致快照；重复 writer 拒绝且不截断共享文件；退出 0。
- [录包回归](fast_livo2_evidence/sync_capture_diagnostics_20261029/recording-tests-final.log)：4/4，8.818 s，退出 0；ROS 部分隔离 domain 189。
- [用户命令的 dry-run](fast_livo2_evidence/sync_capture_diagnostics_20261029/capture-dry-run.json)：退出 0，没有 `--execute`，未生成真实录包或打开设备。
- [收尾核查](fast_livo2_evidence/sync_capture_diagnostics_20261029/verification.json)：源码/二进制 SHA、6 个 Python 文件语法、文档链接、配置原字节与原包 stat 核查通过；两真实受管会话均 STOPPED、live=false、owned=[]。

失败证据保留：首次编译的变量遮蔽和日志计数类型问题已修正；新测试曾使用非法数字开头 topic、错误地等待 BEST_EFFORT 必须送达末帧，以及单次发现探针丢失，均在测试夹具中修正，负载批次没有补发或删除丢失记录。首次录包回归与另一个 domain 的合成 writer 并行，跨 domain 设备占用检查正确拒绝启动；改为顺序执行后通过，没有放宽保护规则。初次日志及目录均保留，只有上列最终日志代表通过结果。

## 3 下一包判据与停止边界

1. 用户按 120 秒车载动作表采集，保持标签、安装、曝光和既有标定。Agent 未启动设备或自动控制运动。
2. 对原 bag 扫描、writer 原始戳/发布序号、相机帧号做逐条对应，分别报告缺失、重复、DDS 丢失事件及交付/回调延迟；没有缺失也不宣称物理同步通过。
3. 比较静止、直线、左右转向及不同节奏的残差；优先用原始 IMU 与图像转动做时序诊断，减少把融合轨迹当真值的循环论证。单次旋转不足以独立确认所有外参，不据此反标所有参数。
4. 若新包仍支持稳定相位差，再以独立派生输入/输出做前端重放对照，保留原 bag；不同运动段结果不一致则继续排查动态延迟、标定和轨迹误差。没有充分依据不采用 125 ms，不将有效消息数当定位精度。

**PASS**：本地构建、7 项交付测试、4 项录包回归、seqlock 检查、诊断归档和 dry-run。

**PARTIAL / 待新数据**：实际设备下是否不再漏收、交付尾延迟、曝光/扫描物理相位、原标定准确性、跨帧一致性；尚不能登记 VALID 地标。

**未实施**：实际录制/驾驶、相机曝光或时间补偿调整、全局 Tag 地图、C1/C2/B 新验收、提交/推送。

改动前源码、入口、文档及旧 writer 二进制在 [baseline.json](fast_livo2_evidence/sync_capture_diagnostics_20261029/baseline.json)所指备份。必要时仅回退本报告列出的 writer 和启动参数变更，保留本轮其他未提交成果；不恢复或覆盖原包。节点仍支持显式 `lidar_reliability:=best_effort` 用于隔离对照，受管入口固定显式 reliable，不建议为消除报错自动降级。
