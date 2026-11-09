# FAST-LIVO2 三传感器运动录包与远程预览

更新：2026-11-06。录包入口默认直接执行、独立启动传感器并读取FAST profile，现默认同时记录wheel：已有新鲜发布者则复用，没有则自动启用can0并启动只接收反馈的底盘驱动；`--no-wheel`用于纯传感器录制，`--dry-run`仅显示计划。自启wheel驱动不请求CAN控制模式、不订阅速度、退出不发送停车命令，保持操作员使用遥控器；不启动Nav2、公共定位桥接或发布公共TF。实现/验证见[自动wheel报告](fast_livo2_recording_wheel_report.md)。本页不代表运动建图精度或物理同步通过。

正式前端试验配置现按用户要求使用 `time_offset.img_time_offset: 0.1`（+100 ms），与 LIV_handhold_2 锁定示例一致；前端对原图 header 内部映射一次，原 bag/图像/雷达戳不改。曝光中心与剩余相位尚未实测，Tag 默认关闭；历史 +110 ms 离线结果不是新 +100 ms 的验收结果。此修改对下一次新建会话生效，接入已有会话不自动热加载 YAML，须核验实际参数与归档。源码依据和两包证据见 [时间相位核查](fast_livo2_timestamp_phase_report.md)。

当前已有约136.6秒三传感器运动包，后续预览与录包均已停止；首次无图像故障是历史记录，不是已确认仍存在的当前阻塞。新前端配置已开启初始重力对齐，新采集归档须保留实际配置，不能与旧图直接混用世界系；见 [STATUS](STATUS.md) 和 [重力报告](fast_livo2_gravity_alignment_report.md)。

需要边采集边进行B回环和全局地图优化时，使用[在线预建图入口](fast_livo2_online_mapping.md)。本页的独立录包入口运行前端并保存数据，仍不启动回环后端。2026-11-05后续已补存 `/fast_livo/deskewed_imu`；此前走廊包没有该话题，已通过保留的原始数据离线重放生成，不追认旧包原本已保存它。

## 开始录制

2026-11-09补充：首份180秒自动wheel真实包已完整归档。后续诊断确认Mini V2反馈转向符号问题，当前隔离录包驱动已修正；原包保留原值，不追认成修复后采集。工作外参按用户授权更新，具体版本、哈希与限制见[本轮报告](fast_livo2_wheel_alignment_report_20261109.md)。

```bash
cd /home/lab307/ranger_nav
bash scripts/record_fast_livo2.sh --dry-run --seconds 60   # 只显示计划，不启动设备
bash scripts/record_fast_livo2.sh --seconds 60             # 启动传感器/前端和适合当前终端的可视化并录包
bash scripts/record_fast_livo2.sh --seconds 60 --no-wheel  # 不连接底盘的采集
```

默认使用 `config/profiles/fast_livo2_frontend.yaml` 并启动自己的传感器会话，省去 `--execute --start-sensors --runtime-config config/profiles/fast_livo2_frontend.yaml`。旧的两个显式开关仍兼容；需要其他配置时仍可传 `--runtime-config PATH`。仅设置环境变量 `RANGER_RUNTIME_CONFIG` 不替代本入口默认profile，入口参数可显式覆盖。

已有本工程的FAST预览会话时，默认拒绝启动，须显式 `--attach` 接入，或由操作员通过所属入口停止原会话；不自动接管、停止或重启设备。终端出现 `CAPTURE RECORDING` 和“录制已就绪”后才开始采集动作。`--seconds` 从录包器订阅全部目标话题就绪后计时，不包含启动时间；范围1～3600秒，默认60秒。

启动提示 `CAPTURE_DIR`、`SESSION_DIR` 只表示创建目录/会话。现在会明确显示 `CAPTURE PREPARING（尚未录制）` 和 `CAPTURE WAITING（尚未录制）`，列出雷达、相机、前端等未就绪组件；等待状态变化或每 10 秒刷新。`STARTING` 表示已启动但未就绪，`WAITING` 表示等待上游条件。

自建采集会话在启动驱动前检查配置要求的本机雷达采集 IP。缺失时立即打印 `CAPTURE ERROR` 并以退出码 2 结束，不创建新录包目录、不改网卡配置。有该 IP 仅说明可绑定地址，后续仍检查实际数据：启动就绪总超时保留 90 秒，超时明确列出未就绪组件并清理；录包器输入预检最多 12 秒，区分无发布者、多发布者、类型/QoS 错误和有话题但无数据。录中单路断流超过 2 秒则报具体话题并失败退出。失败原因、日志路径直接打印到终端，原始/部分数据保留，不生成成功标记。实现与 29 项离线检查见 [就绪报错报告](fast_livo2_recording_readiness_report.md)。

录包入口的`--visualization`默认`auto`：有`DISPLAY`时用RViz，无`DISPLAY`（普通SSH终端）时用本机只读Foxglove，不要求远程X11。可显式指定`foxglove`、`none`；显式`rviz`/`both`而没有`DISPLAY`仍在启动设备前拒绝，不伪造桌面环境变量。启动时显示实际`VISUALIZATION`，归档同时保存请求值与实际选择。已有预览的显示设置不受影响。

```bash
# 明确接入已有预览；结束只停止录包，已有预览继续运行
bash scripts/record_fast_livo2.sh --attach --seconds 60

# 默认拥有本次设备会话；有其他预览占用时拒绝，不接管
bash scripts/record_fast_livo2.sh --seconds 300 --label loop01

# 无GUI录制，或用Foxglove代替RViz
bash scripts/record_fast_livo2.sh --seconds 300 --visualization none
bash scripts/record_fast_livo2.sh --seconds 300 --visualization foxglove

# 状态/停止；Ctrl+C也会正常收尾，SSH断开由受管会话清理
bash scripts/record_fast_livo2.sh --status
bash scripts/record_fast_livo2.sh --stop
```

本次走廊/教室大范围采集可用：

```bash
bash scripts/record_fast_livo2.sh --seconds 900 --label corridor_classroom_map01 --visualization none --min-free-gb 100
```

自行启动的设备会话和wheel驱动在录制结束时一同停止；接入/复用已有会话或wheel不会停止对方。CAN接口保持启用，其录前状态和回退命令写入manifest。关闭RViz本身不停止采集。`--attach`模式不重新启动可视化，但仍按默认wheel策略复用/启动反馈。

默认新目录 `bags/fast-livo-<label>-<随机串>/`。可用 `--output /绝对路径/新目录` 指定，父目录须存在，禁止覆盖。默认磁盘保留 5 GB，录前及录中检查；`--min-free-gb` 可调整，最少 1 GB。未开启压缩/分卷，兼容本工程现有单 SQLite 审查与回放工具。双份 1280×1024 RGB8 图像约 79 MB/s，另有点云与数据库开销，应按实测文件增长规划时长。

本次 Tag 补录按用户新声明采用 `tag36h11`、ID `01`（整数 1）、Black-border width **140 mm**。新包尺寸表为 `config/fast_livo2/apriltag_sizes_capture_140mm.json`，后续离线处理显式选用该表；录包命令无需更改，配置会自动归档。旧包仍使用原 155 mm 表。140 mm 指黑色外边界四角之间的边长，不包含白边/纸张；新标签须在两次登记/验证访问间保持固定，不能与其他位置的同家族同 ID 标记同时部署。

## 记录内容与结果

| 话题 | 类型/作用 |
| --- | --- |
| `/livox/lidar` | PointCloud2，保留原逐点时间 |
| `/livox/imu` | Imu，原始输入 |
| `/front/image_sync` | Image，当前三融合实际使用的同步戳图像 |
| `/front/image` | Image，同步检查前的图像；主机写戳不代表曝光时刻 |
| `/front/camera_info` | CameraInfo，驱动内参 |
| `/fast_livo/odometry` | Odometry，在线前端基线；回环后端可离线重新运行 |
| `/fast_livo/deskewed_imu` | PointCloud2，与上述位姿严格同戳的去畸变IMU机体系区间云 |
| `/wheel/odom` | 默认启用；底盘反馈积分的Odometry，odom/base_link、原主机header |

默认自动记录`/wheel/odom`，`--record-wheel`保留为显式兼容选项；`--no-wheel`才关闭。缺少wheel时检查跨ROS域的底盘进程，发现已有驱动则拒绝重复启动；否则需要已构建`ws_base/install_recording_rx`。can0已为500k/ERROR-ACTIVE时不改配置；DOWN时用固定免密`sudo -n /usr/sbin/ip link set can0 up type can bitrate 500000`启用，没有权限则报错。自启驱动为receive_only、TF关闭；已有wheel发布者只复用、不改变其模式。新鲜度/坐标系/类型/QoS不符时拒绝，不替换已有节点。保留原header/逐点时间，录包器使用主机存储时间，不使用`--use-sim-time`；不处理传感器与wheel历元差异。新包回放仍需有依据的派生时间视图，不能修改原始包。

录前检查每个话题恰好一个发布者、消息类型、RELIABLE QoS及实际收流；等待录包器订阅就绪。录中检测发布者消失/GID改变/重复发布者、数据断流与磁盘空间，失败保存部分结果并停止，不自动重启。复用原会话设备占用检查、资源租约和精确进程身份清理。

2026-10-29 同步排查更新：受管入口显式使用 writer `lidar_reliability:=reliable`、`lidar_queue_depth:=5`，节点默认也为 reliable。保留原扫描戳、v2 原子协议和相机重复戳拒绝；没有采用假设的 100–125 ms 补偿。可靠交付仍可能有延迟，不能证明曝光与扫描物理对齐。

输出包括：

- `raw/`、`record.log`、实际录制命令、录前发布者/GID信息、录包器进程身份。
- `sources/`：配置、标定配置、脚本、锁文件和补丁的原始字节与SHA256；Git HEAD、工作树状态和tracked diff。
- `sources/` 另含 writer/相机源码与实际二进制；2026-11-04 起也归档实际前端 `fastlivo_mapping` 二进制，防止仅靠配置追认源码版本。`subscriptions_before.json` 保存录前订阅端 QoS/GID，`graph_before.json` 保持原发布端格式。
- `session_logs/`：本次受管会话、设备日志、相机/时间writer CSV。不复制仍可能活动的共享内存文件。
- `manifest.json`、`recorder_result.json`、`audit.json`、`result.json`；完整收尾且必需数据检查通过时才生成 `CAPTURE_COMPLETE`。
- `manifest.wheel`记录disabled/external/owned_receive_only、已有发布者身份或自启命令及CAN前后配置；自启时另归档实际wheel二进制、驱动源码和增量补丁。结束只清理本次拥有的进程。

`CAPTURE_COMPLETE` 仅表示录包完成及必需输入检查通过。相机同步异常、超过0.5秒的间隔会列入 `result.json`，数据质量可为 PARTIAL；该标志不表示物理同步、回环或定位精度通过。审查检查整数header顺序、数量/间隔、点时间和非有限IMU值。同步CSV统计覆盖整个设备会话，接入已有长会话时可能包含录制时间之外的数据。接入会话的日志属于活动文件快照，明确标记可能不完整；原会话资产继续保留。

接入会话会保存原启动argv，并检查其配置路径及domain与所选profile一致；配置文件哈希是归档时读到的文件内容，不能证明外部运行进程从未修改过参数。

## RViz与Foxglove

```bash
# 仅预览，不录包
bash scripts/run_fast_livo2_live.sh --execute --visualization rviz
bash scripts/run_fast_livo2_live.sh --execute --visualization foxglove
bash scripts/run_fast_livo2_live.sh --execute --visualization both
bash scripts/run_fast_livo2_live.sh --status
bash scripts/run_fast_livo2_live.sh --stop
```

上述三个启动命令互为选择，不能并行抢占同一设备。单独的`run_fast_livo2_live.sh`默认仍为RViz；录包入口则默认`auto`。`none` 可关闭可视化进程。Foxglove使用本机已安装的Bridge 3.5.0，监听 `127.0.0.1:8765`，可用 `--foxglove-port` 修改。只公布指定预览/图像话题，禁用客户端发布、参数修改和服务调用，不开放互联网端口，也不调整主机网络。

在远程电脑建立SSH隧道（将主机地址换成实际可访问地址）：

```bash
ssh -N -o ExitOnForwardFailure=yes -L 8765:127.0.0.1:8765 lab307@<机器人主机地址>
```

在Foxglove中选择Foxglove WebSocket，连接 `ws://localhost:8765`。3D面板固定/显示坐标系设为 `fast_livo_local`，添加：

- `/fast_livo/view/map`：前端累计云，未经过回环优化。
- `/fast_livo/view/scan`：当前云，已转到同一显示坐标系。
- `/fast_livo/view/path`、`/fast_livo/view/pose`：轨迹和IMU位姿。
- Image面板选择 `/front/image_sync` 或 `/rgb_img`（VIO使用的特征图像）。

显示云和轨迹本来就在同一局部系，不需要新增TF。首次先订阅地图和一幅图像，按网络情况增加显示项；原始录制不依赖远程客户端连接。ROS仍保持localhost/domain隔离。

话题列表/Topic Graph可见只表示发现了话题。还需添加Image面板并选择图像，以及在3D面板的Topics中启用点云话题。先启用`/fast_livo/view/map`，将Display frame设置为`fast_livo_local`；Image可先选`/rgb_img`查看VIO特征图。若为空，先在Raw Messages面板选择`/fast_livo/odometry`确认内容持续更新，再记录具体面板报错。不要用添加任意公共TF或改写原始采样时间的方法绕过显示配置问题。

Bridge连接及参数依据[官方ROS2说明](https://docs.foxglove.dev/docs/getting-started/frameworks/ros2)和[官方Bridge参数](https://github.com/foxglove/foxglove-sdk/blob/main/ros/src/foxglove_bridge/README.md)。本机3.5.0通过 `foxglove.sdk.v1` 子协议的真实握手/收云测试；老客户端若只提供旧子协议可能返回400，需使用兼容的Foxglove客户端，不修改全局依赖绕过。

## 采集路线与验收

### 本次 140 mm 标签的车载同步诊断

此项已完成采集及分析，当前不再等待或要求重录120秒包。现有输入为 `bags/fast-livo-tag140_sync_diag_02-35f_m8nd/`，1200扫描均有writer对应；动作按实际数据分段，不要求追认旧计划的时间顺序。历史动作方案、时序/Tag分析及失败证据见[120秒诊断报告](fast_livo2_tag140_sync_diag02_report.md)、[同步诊断准备报告](fast_livo2_sync_capture_diagnostics_report.md)。

writer CSV 的 `arrival_ns` 是steady clock，`callback_system_ns`是主机system clock，DDS时间为中间件元数据；均不能直接与传感器虚拟UTC相减当曝光偏差。来源身份用publication sequence与GID，未知字段留空。没有重复戳只证明这一类异常未出现，不代表曝光中心已测定。

### 新一轮分离动作诊断（2026-11-04）

转向包 `bags/fast-livo-tag140_turn_03-bk8_pjyy/`、直行包 `bags/fast-livo-tag140_straight_04-ryzykq2l/` 已录完并完成多轮离线分析，见[两包报告](fast_livo2_tag_two_capture_report.md)、[GICP对照](fast_livo2_c1_correction_report.md)。用户已确认140mm标签与传感器安装跨两包未改变，不重复询问或执行旧采集命令。

当前下一步是利用这些原包固定正式 **+100ms** 另存三融合对照；旧0/+110ms结果保持原配置，不能追认为新配置验收。此两包已参与方法选择，未来新盲测访问另行设计，当前无需重复录同类动作。具体范围和实际版本见[HANDOFF](HANDOFF.md)。

### 后续一般建图采集

同步诊断完成后，一般建图先60秒短包：静止15秒、缓慢直行与转弯、最后静止15秒。检查图像、消息间隔和CSV后再采集3～5分钟闭环，起终点做标记，回到起点尽量同朝向，并重复行进起始路段。可先尝试30～80m、有纹理和多方向几何结构的平缓路线，避免第一次就使用重复长走廊和急转。当前后端位置搜索3m、排除最近30秒/20关键帧，应给历史候选足够时间与运动基线。

后续用同一原包重跑前端和B后端，检查VIO参与、原始漂移、回环候选/质量/拒绝原因、优化轨迹与地图结构、资源消耗及导出。不将闭合误差当作全轨迹绝对精度；无独立真值不报告APE/RPE。物理曝光/扫描对应、车体外参和实时wheel时间对齐仍需单独验收。

本次实现与测试证据见[采集入口报告](fast_livo2_recording_report.md)。
