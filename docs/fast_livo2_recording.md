# FAST-LIVO2 三传感器运动录包与远程预览

更新：2026-10-29。脚本不会启动底盘驱动、CAN、Nav2、公共定位桥接或发布公共 TF。运动由操作员另行控制。本页描述采集入口，不代表运动建图精度或物理同步通过。

当前已有约136.6秒三传感器运动包，后续预览与录包均已停止；首次无图像故障是历史记录，不是已确认仍存在的当前阻塞。新前端配置已开启初始重力对齐，新采集归档须保留实际配置，不能与旧图直接混用世界系；见 [STATUS](STATUS.md) 和 [重力报告](fast_livo2_gravity_alignment_report.md)。

## 开始录制

```bash
cd /home/lab307/ranger_nav
bash scripts/record_fast_livo2.sh --seconds 60             # dry-run，不启动设备
bash scripts/record_fast_livo2.sh --execute --seconds 60   # 启动传感器/前端和适合当前终端的可视化并录包
```

已有本工程的 FAST 预览会话时，默认自动接入该会话；没有时启动自己的传感器会话。终端出现 `CAPTURE RECORDING` 和“录制已就绪”后才开始采集动作。`--seconds` 从录包器订阅全部目标话题就绪后计时，不包含启动时间；范围 1～3600 秒，默认 60 秒。

录包入口的`--visualization`默认`auto`：有`DISPLAY`时用RViz，无`DISPLAY`（普通SSH终端）时用本机只读Foxglove，不要求远程X11。可显式指定`foxglove`、`none`；显式`rviz`/`both`而没有`DISPLAY`仍在启动设备前拒绝，不伪造桌面环境变量。启动时显示实际`VISUALIZATION`，归档同时保存请求值与实际选择。已有预览的显示设置不受影响。

```bash
# 明确接入已有预览；结束只停止录包，已有预览继续运行
bash scripts/record_fast_livo2.sh --execute --attach --seconds 60

# 明确要求拥有本次设备会话；有其他预览占用时拒绝，不接管
bash scripts/record_fast_livo2.sh --execute --start-sensors --seconds 300 --label loop01

# 无GUI录制，或用Foxglove代替RViz
bash scripts/record_fast_livo2.sh --execute --seconds 300 --visualization none
bash scripts/record_fast_livo2.sh --execute --seconds 300 --visualization foxglove

# 状态/停止；Ctrl+C也会正常收尾，SSH断开由受管会话清理
bash scripts/record_fast_livo2.sh --status
bash scripts/record_fast_livo2.sh --stop
```

自行启动的设备会话在录制结束时一同停止；接入已有会话不会停止对方。关闭 RViz 本身不停止采集。`--attach` 模式不重新启动可视化，使用被接入会话原有的可视化设置。

默认新目录 `bags/fast-livo-<label>-<随机串>/`。可用 `--output /绝对路径/新目录` 指定，父目录须存在，禁止覆盖。默认磁盘保留 5 GB，录前及录中检查；`--min-free-gb` 可调整，最少 1 GB。未开启压缩/分卷，兼容本工程现有单 SQLite 审查与回放工具。双份 1280×1024 RGB8 图像约 79 MB/s，另有点云与数据库开销，应按实测文件增长规划时长。

## 记录内容与结果

| 话题 | 类型/作用 |
| --- | --- |
| `/livox/lidar` | PointCloud2，保留原逐点时间 |
| `/livox/imu` | Imu，原始输入 |
| `/front/image_sync` | Image，当前三融合实际使用的同步戳图像 |
| `/front/image` | Image，同步检查前的图像；主机写戳不代表曝光时刻 |
| `/front/camera_info` | CameraInfo，驱动内参 |
| `/fast_livo/odometry` | Odometry，在线前端基线；回环后端可离线重新运行 |

`--record-wheel` 可增加已有 `/wheel/odom`，但不会启动底盘；不存在时拒绝，不伪造 wheel 数据。默认不录 wheel，也不处理传感器与 wheel 历元差异。保留原 header/逐点时间，录包器使用主机存储时间，不使用 `--use-sim-time`。新包的回放调度需要遵循现有 header-clock 视图流程，不能修改原始包。

录前检查每个话题恰好一个发布者、消息类型、RELIABLE QoS及实际收流；等待录包器订阅就绪。录中检测发布者消失/GID改变/重复发布者、数据断流与磁盘空间，失败保存部分结果并停止，不自动重启。复用原会话设备占用检查、资源租约和精确进程身份清理。

输出包括：

- `raw/`、`record.log`、实际录制命令、录前发布者/GID信息、录包器进程身份。
- `sources/`：配置、标定配置、脚本、锁文件和补丁的原始字节与SHA256；Git HEAD、工作树状态和tracked diff。
- `session_logs/`：本次受管会话、设备日志、相机/时间writer CSV。不复制仍可能活动的共享内存文件。
- `manifest.json`、`recorder_result.json`、`audit.json`、`result.json`；完整收尾且必需数据检查通过时才生成 `CAPTURE_COMPLETE`。

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

先60秒短包：静止15秒、缓慢直行与转弯、最后静止15秒。检查图像、消息间隔和CSV后再采集3～5分钟闭环，起终点做标记，回到起点尽量同朝向，并重复行进起始路段。可先尝试30～80m、有纹理和多方向几何结构的平缓路线，避免第一次就使用重复长走廊和急转。当前后端位置搜索3m、排除最近30秒/20关键帧，应给历史候选足够时间与运动基线。

后续用同一原包重跑前端和B后端，检查VIO参与、原始漂移、回环候选/质量/拒绝原因、优化轨迹与地图结构、资源消耗及导出。不将闭合误差当作全轨迹绝对精度；无独立真值不报告APE/RPE。物理曝光/扫描对应、车体外参和实时wheel时间对齐仍需单独验收。

本次实现与测试证据见[采集入口报告](fast_livo2_recording_report.md)。
