# 阶段 B 报告：外触发相机 + 雷达-相机时间戳桥接

日期：2026-10-21　工作空间：`~/calibration/ws_calib`（独立，未触碰 ranger_nav）
结论：**全部验收项通过**。834/834 帧图像与雷达扫描逐帧对齐，零错配、零丢帧、零异常状态。

## 一、实现清单

| 组件 | 路径 | 说明 |
|---|---|---|
| timeshare_writer | `ws_calib/src/timeshare_writer/` | 订阅 `/livox/lidar`，整数纳秒取 header.stamp，mmap 写 `~/timeshare` `{int64 high=写序号, low=扫描起始ns}`；flock 单写者；回退/重复/间隔/积压检测 |
| 相机驱动副本 | `ws_calib/src/hik_camera_trigger_driver/` | 复制自 ws_camera（改名包名），新增 Line0 外触发 + timeshare 时间戳 + `/front/image_sync` + 逐帧 CSV 仪表 |

副本源码修改（相对原版 `hik_camera_node.cpp`）：
1. 新增头文件（mman/flock/csv）
2. `declareParameters()`：+6 参数（trigger_enable/trigger_source/sync_image_topic/timeshare_path/timeshare_stale_ms/sync_csv_path）
3. `configureCamera()`：触发分支（TriggerSource→Line0、TriggerMode→On、绕开帧率迭代、直接设曝光/增益），**不写不支持的 TriggerSelector**
4. `initializeRosInterfaces()`：sync 发布器（QoS 随 `qos_reliability`）
5. `captureLoop()`：记录到达时刻（steady ns）与设备时间戳（100 MHz ticks）；每帧 `processSyncFrame()`
6. timeshare 一致性读取协议：high→low→high 双读校验 + 过期/回退/无写者检测；**异常帧跳过 sync 发布并计数，绝不静默回退主机时间**
7. 析构清理 + 5 秒汇总日志

配置：`config/camera_params_trigger.yaml`（Line0、20 ms 曝光可调 40 ms、增益 10、reliable QoS）

## 二、验证结果（bag：stageB_sync_run2，83.5 s）

| 指标 | 结果 |
|---|---|
| bag 计数 | lidar 834（10.0 Hz）、image 834（10.0 Hz）、**image_sync 834（10.0 Hz）**、imu 16688（200 Hz） |
| 丢帧 | 相机 FrameID 0 跳变；sync 状态 **1528/1528 OK**（含 bag 外时间） |
| 单调性 | image_sync / lidar / imu 时间戳全部严格递增（bag 实测） |
| 时间域 | image_sync 首末戳与 lidar **逐纳秒一致**（1590193436100175970…）；每帧图像戳=某扫描起始戳（834/834） |
| 触发周期 | 设备时间戳间隔 100.00 ms（min=max，100 MHz 分辨率） |
| 错配分析 | Δhost=图像到达−所用扫描到达：min 20.4 / 中位 22.1 / max 24.0 ms；**零**疑似跨周期错配（1528 帧） |
| Δhost 语义 | ≈曝光 20 ms + 传输 ~2 ms → 图像用的正是自身触发时刻开始的扫描，配对确定性成立 |
| writer 异常 | 3591 扫描 0 回退；仅启动期 1 次 1400 ms 扫描间隔（雷达驱动重启窗口，非录制期） |

## 三、过程中发现并解决的问题

1. **大图像 DDS 传输丢包（原始 4.3 fps 现象的真因）**：3.9 MB×2 话题 best_effort 下订阅端丢失 10–60% 消息（含 bag）。FastDDS XML 调优在本机 2.6.2 解析器不兼容（sendSocketBufferSize/max_message_size 被拒，已弃用）。**解法：图像话题 QoS 改 reliable**，bag 100% 收全。
   注：Python 监测脚本自身在高吞吐下也会掉样（测得 9 Hz 而 bag 实际 10 Hz）——以后以 bag/CSV 为准，不以 rclpy 监测为准。
2. yaml 顶层键必须匹配节点名（`/**:` 通配解决）。

## 四、可复现启动命令

```bash
source /opt/ros/humble/setup.bash
source ~/ranger_nav/ws_livox/install/setup.bash   # 雷达（未改动）
ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -p xfer_format:=0 -p multi_topic:=0 -p data_src:=0 -p publish_freq:=10.0 \
  -p output_data_type:=0 -p frame_id:=livox_frame \
  -p user_config_path:=/tmp/MID360s_test.json \
  -p cmdline_input_bd_code:=livox0000000001 -r __node:=livox_lidar_publisher &

source ~/calibration/ws_calib/install/setup.bash
ros2 launch timeshare_writer timeshare_launch.py &          # 先启动（写 ~/timeshare）
sleep 3
ros2 launch hik_camera_trigger_driver trigger_launch.py exposure_us:=20000.0
# 静态标定时曝光可传 40000.0
```

停止：直接终止各进程；`~/timeshare` 残留 16 字节文件无害（writer 启动时 O_TRUNC 重建）。

## 五、日志与数据位置

- 驱动逐帧仪表：`~/calibration/logs/camera_sync_*.csv`（frame_id, dev_ts, 到达ns, 写序号, 所用扫描ns, 状态, 发布ns）
- writer 逐帧：`~/calibration/logs/timeshare_writer_*.csv`
- 验证 bag：`~/calibration/bags/stageB_sync_run2/`（原始 4 话题，未覆盖）
- 运行日志：/tmp/{livox_driver,timeshare_writer,hik_trigger}.log

## 六、回退方式

- ranger_nav 两工作空间零改动（ws_camera 源码原有两处本地修改为历史遗留，与本轮无关）
- 卸载本方案：停进程，`rm -rf ~/calibration/ws_calib ~/timeshare` 即完全移除
- 相机设置全部为易失项（关机/关闭句柄即恢复自由采集默认）

## 七、对后续阶段的判断

**FAST-Calib 静态标定：现有 timeshare 方案足够。**
板与传感器均静止，配对误差 ≤1 扫描周期（100 ms）不产生几何误差；实测配对确定性 ±2 ms、相位对齐已被 Δhost 分布证实（中位 22 ms=曝光+管线，零离群）。

**FAST-LIVO2 运动场景：本方案与上游 LIV_handhold_2 完全同构，可直接开始。**
残余风险两点，均有观测手段：①"扫描起始=触发瞬间"假设由同步器保证，已实测稳定，但无硬件沿直接测量（如需更强证据，可用动态事件实验：快速挥动物体入画，比对图像与点云中事件出现的帧序号）；②20 ms 曝光期间的运动模糊/卷帘效应属成像问题而非时间问题。若后续轨迹出现时间性伪影，升级路径为 FrameID↔扫描序号的相关性匹配（CSV 已含全部所需字段）。

## 八、遗留事项

- 相机内参 `camera_info.yaml` 仍是旧 1280×720 占位（属另一台相机），内参标定后需替换
- 雷达 IP 配置仓库副本（.5/.12）与实际（.50/.101）不符，标定期间用 /tmp 探针配置；修正仓库配置需另行授权
