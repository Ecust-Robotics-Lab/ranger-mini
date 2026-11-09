# 首次授权现场联调与启动锚点修正

日期：2026-11-09。用户在收到90秒现场方案后明确回复“确认”，授权本次传感器、只接收wheel、采集与定位/公共TF联调，车辆由用户遥控。Agent未启动导航控制器、发送速度/目标或请求底盘控制模式。本次现场功能验收FAIL；软件修正后的隔离验证PASS，尚未再次启动真实设备。

## 实际启动与收尾

boot_id为`dc7ccd75-a884-43af-921e-f285f18e2264`。开始时没有相关设备/底盘进程，旧在线会话已STOPPED且属于前一次boot，can0为DOWN，磁盘可用约543GiB。只接收驱动SHA仍为`ae5f3f1ae633cc9b968c3c0e740810388423a4e7554d2b22c787d582c12ce069`，含V2反馈符号修复；已复核receive_only不请求模式、不订阅速度、退出不发零速。

实际执行：

```bash
bash scripts/localize_fast_livo2.sh \
  --archive outputs/fast-livo-demo-online-mapping-20261105/full_replay_12/map \
  --public-tf --auto-wheel \
  --navigation-map outputs/fast-livo-demo-navigation-prep-20261106/navigation_map_final \
  --domain 215 --seconds 90 --min-free-gb 100 --visualization none --execute
```

首次准备目录`bags/fast-livo-localization-iqt9d470`、会话`run-hl243mhj`在wheel加载器阶段退出127：缺少`libranger_msgs__rosidl_typesupport_cpp.so`。该驱动未进入CAN接收，雷达、相机和录包均未启动。受管清理完成。修复在线shell入口：显式auto-wheel时加载现有`ws_base/install_recording_rx/setup.bash`；底盘消息库来自其既有依赖`ws_base/install/ranger_msgs/lib`。ldd确认可解析，未重编或升级系统库。

随后在本次授权下完成设备首次启动，目录`bags/fast-livo-localization-um88vo7b`、会话`run-58oo6bv3`。真实receive-only wheel、雷达、相机和三融合前端均有数据，但时间锚点异常导致定位没有VALID、没有navigation_ready=true、没有派生wheel/动态TF。本次提前停止，不计90秒完整通过。录包进程退出0、forced=false、实际录制37.945秒；原始bag还包含就绪前的约数秒输入。外层退出2，受管状态STOPPED、owned=[]、guardian_cleanup_complete=true。两次目录/日志/失败证据全部保留。

实际停止命令为`bash scripts/localize_fast_livo2.sh --stop`，退出0。结束后按/proc检查无本次传感器、底盘或localizer残留进程。停止阶段DDS观察速度发布者为0；此图快照只代表观察时刻。can0由本次固定免密命令从DOWN启为UP、500k、ERROR-ACTIVE，按现有采集约定保留UP；未修改系统配置。需人工恢复初始链路状态时使用`sudo -n /usr/sbin/ip link set can0 down`，本轮未执行该回退。

## 失败数据与原因

原bag含419条LiDAR、8382条IMU、397组图像/信息、386对FAST位姿/区间云、2228条wheel。C1心跳2227条，但无有效确认；就绪消息3329条全为false。录后摘要正确标记INCOMPLETE_OR_REJECTED，没有把正常封包算成定位通过。

receipt显示：首帧头戳1590192125242454340ns，首接收steady=5103486145917ns；首个非空统计已count=17，末帧头戳1590192128200244010ns、steady=5103539804222ns。即17帧的头戳跨2.958秒，接收只跨53.658毫秒，符合启动积压/批量送达特征。以第一帧建锚导致约2.904秒残差；随后残差基本恒定，原固定首帧锚点不可用。本轮没有据此推断物理同步或修改原始时间戳。

## 已实施的软件修正

仅对真实在线定位的启动阶段增加稳定窗口获取锚点：观察新的雷达receipt至少1秒、至少5个新统计；候选头戳/steady偏差超过50ms时重新观察，10秒内不能稳定则报错。一旦建立就冻结，不因后续异常或重启重建有效定位的锚点。已有建图默认与header-clock离线重放路径保持原行为。

mapping_monitor作为单一锚点来源，原子保存`clock_anchor.json`，wheel转换等待并使用同一锚点。文件同时保留原始首帧身份用于检测源会话变化；不独立选择两个锚点，不复制旧包偏移，不改原wheel/雷达头戳。运行中原100ms偏差拒绝、20ms主机步变拒绝及断流/过期机制保留。首帧接收延迟和时间漂移的物理量仍未实测。

## 修正验证

证据：[field_localization_20261109](fast_livo2_evidence/field_localization_20261109/)，含两次实际结果、状态、启动日志、原receipt trace、修正源码SHA与复验。

- 原失败包receipt离线复验：count28时建立稳定锚点，剩余408个统计残差−382970至3425721ns，无错误。此结果不是新的现场通过。
- `test_fast_livo_live_clock.py`：6项通过，新增启动积压与权威锚点/源身份检查。
- 修改后的实际ROS链路测试使用真实mapping_monitor建立锚点，合成输入首帧带2.9秒偏差；通过wheel断流恢复、receipt断流恢复以及源锚点异常锁定。退出0，7.928秒。输入C1是合成已确认消息，不冒充真实C2重新验证。
- 两次早期集成复验在receipt恢复断言失败，日志保留；检查器就绪/健康订阅改为depth1，避免累积旧状态，再复验通过，未放宽桥接门限。
- wheel转换ROS传输检查、Python/shell静态检查见对应日志。没有在修正后自动重启真实设备。

下一项是用户确认后的新一轮90秒现场联调，复用上述命令，先静止再遥控短行/转弯/停车。正式地图、工作外参、规划控制和生产runtime未改；源码与新证据尚未提交或推送。
