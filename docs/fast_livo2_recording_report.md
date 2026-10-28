# 2026-10-27 三传感器录包入口与可视化

用户要求完整录包脚本、远程Foxglove，并允许先开RViz查看。未授权底盘控制、导航、C1/C2或真实运动自动采集。本次保留全部已有工作树、数据和标定；只追加采集脚本/配置、测试和文档，复用既有会话管理器，不修改前端估计器或公共TF桥接。

## 已实现

- `scripts/record_fast_livo2.sh`、`scripts/fast_livo2/record_session.py`与`record_worker.py`：默认dry-run，60秒初值；自动选择启动设备或接入已有受管预览，可显式指定。
- 精确输入类型/唯一发布者/QoS/收流检查、录包器订阅就绪、定时/Ctrl+C结束、磁盘保留、发布者变化/断流失败处理。
- 会话所有权隔离、进程身份记录、异常清理、配置/源码/SHA/Git状态/同步CSV及日志归档、录后审查；禁止覆盖已有结果，失败无成功标记。
- `online_session.py`复用设备环境/占用检查，增加rviz/foxglove/both/none；原默认RViz保留。
- `config/fast_livo2/foxglove.yaml`复用已安装3.5.0，loopback监听/只读能力和话题白名单；远程SSH隧道，无系统依赖/网络改动。
- `audit_bag.py`只对实际Livox输入解析逐点时间，空包安全统计，并补充非有限IMU计数。

操作命令、参数和结果语义见[录包指南](fast_livo2_recording.md)。Lightning默认、连续wheel、Nav2和安全链未改。

## PASS：隔离离线证据

证据目录：`docs/fast_livo2_evidence/capture_20261027/checks-u0pBPFI3/`。测试使用domain189/188、localhost和合成消息；不打开实物。

```bash
source /opt/ros/humble/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export EVIDENCE_DIR="$PWD/docs/fast_livo2_evidence/capture_20261027/checks-u0pBPFI3"
python3 tests/test_fast_livo_recording.py
python3 tests/test_fast_livo_foxglove.py
python3 tests/test_project_consolidation.py
bash -n scripts/record_fast_livo2.sh scripts/run_fast_livo2_live.sh
git diff --check
```

- 录制8/8、退出0：[最终日志](fast_livo2_evidence/capture_20261027/checks-u0pBPFI3/recording_tests_verified.log)。真实ROS录包器、合成输入下完成定时/中断落盘、外部预览保留、自有会话清理、重复发布者拒绝、录包器SIGKILL、磁盘不足拒绝、禁止覆盖及证据归档。
- Foxglove 1/1、退出0：[日志](fast_livo2_evidence/capture_20261027/checks-u0pBPFI3/foxglove_tests_final.log)、[握手/收云结果](fast_livo2_evidence/capture_20261027/checks-u0pBPFI3/foxglove_result.json)。实际启动已安装Bridge，HTTP101、只读connectionGraph能力、白名单过滤、订阅收到PointCloud2。
- 现有整合回归10/10、退出0：[最终日志](fast_livo2_evidence/capture_20261027/checks-u0pBPFI3/project_regression_verified.log)。生产默认、profile路由、三融合强制与标定路径保持。
- 初次WebSocket测试误用旧`foxglove.websocket.v1`子协议，返回400；对照官方源码及本机SDK协议改为`foxglove.sdk.v1`后通过，保留首次失败日志与Bridge日志，未升级系统组件。

不将上述测试称为实车运动录包或回环验收。

## 本次预览尝试

实际命令：`bash scripts/run_fast_livo2_live.sh --execute --visualization both`，受管目录 `.runtime/sessions/fast_livo_live/run-lkcc1jgc/`，同步资产 `sensors-3oleuxcw/`。

雷达连接成功：实际日志SN ARMCP4E0033801、IP192.168.1.101，PointCloud2与IMU收流、writer写入。此次实际日志`time_type=0`、header接近主机时间，与上次旧历元观测不同，不能沿用上次time_type或宣称物理同步因此通过。仍观察到约200ms扫描/到达间隙。

相机SN DB1216907连接成功，Line0、20ms曝光，但连续`MV_E_NODATA`超时，没有图像；因此前端、RViz和Foxglove尚未启动。本次预览受阻，尚不能显示真实三融合效果。已主动通过受管`--stop`退出0，没有切换自由运行或丢弃图像绕过。

停止证据：[状态](fast_livo2_evidence/capture_20261027/checks-u0pBPFI3/preview_state_stopped.json)，`live=false`、`STOPPED`、`owned=[]`、`guardian_cleanup_complete=true`；设备日志与CSV副本见同证据目录的`preview_logs/`。

需操作员确认外触发线、触发源及供电，再继续检查触发沿/采集状态；当前证据不能直接判定线缆故障。未录制新的真实运动bag。

## PARTIAL与未实施

物理同步、三传感器运动地图/闭环精度、实车外参、实时wheel历元、长时负载及远程客户端实际界面仍PARTIAL。C1/C2、真实底盘控制及导航未实施。预览故障只影响当前实物显示与采集就绪，不影响已经通过的脚本离线测试。

## 同日追加：用户确认传感器就绪后预览成功

用户明确“已经准备好传感器了”后，执行`bash scripts/run_fast_livo2_live.sh --execute --visualization both`。受管目录`.runtime/sessions/fast_livo_live/run-4e1fptkc/`，同步资产`sensors-typrg78w/`。本次Line0图像恢复；未更改曝光、外参或时间偏移，未切自由运行。

PASS：所有组件READY、会话RUNNING，RViz OpenGL4.6并实际显示点云与VIO特征图像；Foxglove3.5.0监听`127.0.0.1:8765`，通道已公布。5秒独立观测50条雷达、1000条IMU、50条同步图像，前端云/位姿50对同戳，约10/200/10Hz，各输入唯一发布者且观测窗口无重复/回退；`/tf`、`/tf_static`、`/cmd_vel`和`/base/safe_cmd_vel`发布者均为空。

证据：[观测数据](fast_livo2_evidence/preview_20261027/ready-f9fe0j37/live_validation.json)、[运行摘要](fast_livo2_evidence/preview_20261027/ready-f9fe0j37/summary.json)、[RViz窗口可见部分](fast_livo2_evidence/preview_20261027/ready-f9fe0j37/rviz.png)。同目录状态、健康、日志和CSV是活动会话快照，非停止后的完整结果。

会话继续运行供用户查看，未启动录包器、底盘、回环后端或公共TF桥接。可执行`bash scripts/record_fast_livo2.sh --execute --attach --seconds 60`接入采集；停止预览用`bash scripts/run_fast_livo2_live.sh --stop`。物理曝光同步、运动精度和远程Foxglove客户端界面仍未验收；实际输入header仍为2020历元，与主机/wheel不可直接拼接。

随后用户确认显示成功，要求关闭原程序准备录包。实际执行`bash scripts/run_fast_livo2_live.sh --stop`退出0；[停止核查](fast_livo2_evidence/preview_20261027/ready-f9fe0j37/stopped_summary.json)为STOPPED、live=false、owned=[]、guardian_cleanup_complete=true，所有已记录组件身份均退出。本轮没有自动录制。后续应使用`record_fast_livo2.sh --execute --start-sensors --seconds 60`，不再attach已停止会话。

## 同日修复：普通SSH终端无DISPLAY

用户执行录包命令，在创建输出目录或启动设备之前被默认RViz的DISPLAY检查拒绝。原命令未兼顾远程无桌面环境。修复录包入口默认`--visualization auto`：DISPLAY非空选择RViz，否则选择已安装的localhost只读Foxglove；显式rviz/both仍要求DISPLAY，缺失时输出参数错误而非Traceback。单独预览入口默认值保持，记录manifest新增请求和实际可视化选择。

PASS：`env -u DISPLAY bash scripts/record_fast_livo2.sh --start-sensors --seconds 60`退出0、dry-run选择Foxglove，组件不含RViz；[计划证据](fast_livo2_evidence/capture_20261027/headless-fix-efzQFAA3/no_display_plan.json)。执行`source /opt/ros/humble/setup.bash`后`PYTHONDONTWRITEBYTECODE=1 python3 tests/test_fast_livo_recording.py`，10/10、退出0，含无DISPLAY/桌面选择、显式RViz拒绝、无桌面默认路径的合成定时录制及既有故障/清理回归；[完整日志](fast_livo2_evidence/capture_20261027/headless-fix-efzQFAA3/tests.log)。`bash -n scripts/record_fast_livo2.sh`与`git diff --check`均退出0。修复测试使用隔离domain189和合成输入，没有启动真实传感器或录包。

用户可用原命令重试，或明确指定`bash scripts/record_fast_livo2.sh --execute --start-sensors --seconds 60 --visualization foxglove`。远程通过前文SSH隧道访问8765；无需设置虚假的DISPLAY。

## 同日追加：Foxglove可见话题但没有显示

用户自行启动录制后，`fast_livo_record/run-gzanp076`为RUNNING、Bridge监听127.0.0.1:8765。只读排查没有停止、重启或修改本次录制。检查前Bridge日志只见用户客户端的connectionGraph订阅，尚未见具体传感器通道订阅。

使用相同WebSocket地址和`foxglove.sdk.v1`，主动订阅并用ROS2反序列化验证4秒：图像40帧、当前云40帧、Path40帧、累计云8帧。图像1280×1024 RGB8、step3840、data3932160字节；累计云示例33869点、point_step16、XYZ/intensity字段、data541904字节，显示坐标系fast_livo_local。该证据证明主机端Bridge可发送并正确解码数据，不证明远程客户端GPU/面板渲染通过。

[详细证据](fast_livo2_evidence/foxglove_diagnosis_20261027/check-dRcY7B3r/websocket_delivery.json)，同目录保留活动会话状态/健康和Bridge日志。建议用户添加Image与3D面板，显式选择图像并启用累计点云，将Display frame设为fast_livo_local；仍需用户提供具体面板错误才能确认客户端渲染问题。未发布额外TF、未重打时间戳。
