# 录包输入缺失与启动报错修复

日期：2026-10-29。用户要求没有雷达、相机或前端时明确报错。仅修改软件并使用合成测试，没有启动真实设备、改网络、提交或推送。

## 问题与变更

用户运行 `record_fast_livo2.sh --execute --start-sensors` 后看到 CAPTURE_DIR、VISUALIZATION、SESSION_DIR，误以为已在录制。实际该会话雷达驱动 `bind failed`、输入计数为空、录包器 WAITING，最终启动超时退出，没有 raw 数据。失败证据保留在 `bags/fast-livo-tag_check_01-nsyqvjp1/` 与 `.runtime/sessions/fast_livo_record/run-rfj4tpbp/`。

原逻辑本来会拒绝无数据录制，但等待/失败提示不足。现在：

- `record_session.py` 在自建会话启动驱动之前检查雷达配置的本机 IPv4。当前配置要求 `192.168.1.50`，本机仅有 `127.0.0.1`、`192.168.3.18`，会立即报缺少采集地址、退出 2；不创建录包目录、不联系设备或修改网络。dry-run、status、attach 不执行该启动地址检查。
- 启动时明确打印 `CAPTURE PREPARING（尚未录制）`；等待时列出未就绪组件，状态变化或每 10 秒输出。只有原有录包就绪条件满足后才打印 `CAPTURE RECORDING`。
- 共用 `session_manager.py` 的失败分支在清理前向终端打印 SESSION ERROR 和日志目录。启动超时原因新增未就绪组件/状态/健康键；默认超时、健康条件、进程身份与清理逻辑保持。
- `record_worker.py` 的输入预检区分具体话题无发布者、重复发布者、类型错误、QoS 错误或有发布者但未收到数据；录中断流说明具体话题。原输入诊断作为 `recorder_error` 保留，最终也打印在终端。
- 启动总超时仍为 90 秒；输入预检 12 秒；录中断流门槛 2 秒。设备存在但上游未就绪时允许正常启动等待，超时后失败；没有仅凭进程或话题存在认定成功。

当前网卡检查的实际只读输出见 [network_precheck.log](fast_livo2_evidence/recording_readiness_20261029/network_precheck.log)。拥有所需 IP 也不代表传感器已连接，仍须通过实际输入检查。生产参数、同步/标定、TF、底盘和导航未改变。

## 验证

基线 HEAD `acffa879cc8d52330276089cd3df83ad6a43f112`，保留已有未提交修改；修改前备份路径见 [baseline.json](fast_livo2_evidence/recording_readiness_20261029/baseline.json)。

以下实际命令均退出 **0**，共 **29 项测试通过**：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 tests/test_fast_livo_recording.py CaptureUnit
PYTHONDONTWRITEBYTECODE=1 python3 tests/test_session_manager.py

source /opt/ros/humble/setup.bash
export EVIDENCE_DIR="$PWD/docs/fast_livo2_evidence/recording_readiness_20261029"
export PYTHONDONTWRITEBYTECODE=1 ROS_LOCALHOST_ONLY=1
python3 tests/test_fast_livo_recording.py CaptureROS
# 上条执行时仍为既有 6 项；随后新增以下两项并单独执行：
python3 tests/test_fast_livo_recording.py \
  CaptureROS.test_camera_publisher_without_data_is_reported_before_recording \
  CaptureROS.test_frontend_stream_loss_reports_topic_and_stops
```

| 范围 | 数量 | 日志 |
| --- | --- | --- |
| 录包单元/合成进程 | 10 | [unit.log](fast_livo2_evidence/recording_readiness_20261029/unit.log) |
| 原录包隔离 ROS 生命周期 | 6 | [ros.log](fast_livo2_evidence/recording_readiness_20261029/ros.log) |
| 新增无相机数据/前端断流 | 2 | [missing_streams_ros.log](fast_livo2_evidence/recording_readiness_20261029/missing_streams_ros.log) |
| 共用进程管理回归 | 11 | [session_manager.log](fast_livo2_evidence/recording_readiness_20261029/session_manager.log) |

新增单元覆盖本机采集 IP 缺失、地址存在但无传感器流、五类话题诊断，以及分别缺少雷达/相机/前端的完整合成启动失败与清理。新增 ROS 测试在独立 domain 189、localhost 下，仅发布合成消息：相机话题存在但不发数据时不启动 rosbag；已录制后前端 odometry 停发时失败停止。两项均核对终端可见具体原因、退出码 2、无 CAPTURE_COMPLETE、外部合成发布源仍存活；断流测试还核对受管 owned=[]。

**PASS**：上述 29 项；输入缺失不能冒充录制；错误和日志位置可见；真实录制命令不变；原有成功录制、Ctrl+C、附着外部会话和清理回归通过。

**PARTIAL / 未验证**：本轮没有接真机进行拔线/断电测试，不能把合成流故障写成硬件故障实测。缺失本机采集 IP 已只读复现。

**未实施**：实际启动/重启设备、改变网卡或同步、底盘运动、删除失败目录、Git 提交/推送。

最终源码 SHA、dry-run、语法、文档链接、diff 和生产会话只读状态检查见 [verification.json](fast_livo2_evidence/recording_readiness_20261029/verification.json)。
