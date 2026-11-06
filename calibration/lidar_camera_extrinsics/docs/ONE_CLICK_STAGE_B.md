# 一键启动已验证的 Stage B 系统

先协调好内参 Agent 的相机使用时间，并接通 LiDAR、相机、硬件同步器。

```bash
cd ~/calibration/lidar_camera_extrinsics
./scripts/start_stage_b.sh
```

默认20ms曝光，与 Stage B 报告一致。若静态板过暗，可停止后改为40ms启动：

```bash
./scripts/start_stage_b.sh --exposure-us 40000
```

启动脚本自动 source Humble、现有 ws_livox、现有 ws_calib，按原报告参数启动 LiDAR → timeshare_writer → 等3秒 → 外触发相机。终端保持前台运行。启动时三个进程存活不等于数据同步通过，随后采集仍需审计。

正常停止：启动终端按 **Ctrl+C**。也可从另一个终端执行：

```bash
~/calibration/lidar_camera_extrinsics/scripts/stop_stage_b.sh
```

查看会话状态：

```bash
~/calibration/lidar_camera_extrinsics/scripts/stage_b.py status
```

只预览命令，不访问硬件：

```bash
./scripts/start_stage_b.sh --dry-run
```

每次日志位于 `logs/stage_b_时间戳/`：lidar.log、writer.log、camera.log、timeshare_writer.csv、camera_sync.csv；停止时保存 session.json。所有子进程工作目录为本次日志目录，ROS日志位于本任务 logs/ros。

`config/mid360s_stage_b.json` 是从报告引用的、当时仍存在的 `/tmp/MID360s_test.json` 原样复制，使用 Mid360s/type8、主机192.168.1.50、雷达192.168.1.101。这样重启清理 /tmp 后仍可启动。没有改动 ranger_nav 或 ws_calib。若现场IP改变，需要重新确认网络配置，脚本不会自动改网卡或猜测IP。

已有同用户 Livox、timeshare_writer 或海康相机驱动进程时拒绝再启动，不会自动终止其他任务。会话停止使用 supervisor PID和启动时刻校验，按记录通知它清理自己建立的进程组；没有全局 pkill/killall。强制 kill -9 启动器会绕过清理，因此使用 Ctrl+C 或 stop 脚本。

系统运行后，另开终端录正面场景：

```bash
cd ~/calibration/lidar_camera_extrinsics
source scripts/env.sh
./scripts/record_scene.py scene_01 --seconds 8
```

只启动本脚本不会自动录包、求解、改内参或集成FAST-LIVO2。原 Stage B 驱动仍按原方案访问共享 `~/timeshare`；桥接机制未修改。

交付验证：命令 dry-run、Bash/Python语法检查、使用两个普通Python休眠进程的清理测试和PID身份校验均通过。未执行真实硬件启动，实际硬件启动和话题验收待现场使用。

## 同时预览同步图像和点云

启动 Stage B 后，另开本机桌面终端：

```bash
cd ~/calibration/lidar_camera_extrinsics
./scripts/preview_stage_b.sh
```

配置为 Fixed Frame=livox_frame；PointCloud2订阅/livox/lidar；Image订阅/front/image_sync；两者Reliable/Volatile。Image显示不需要已知外参或相机CameraInfo，不要为了查看图像设置虚假的LiDAR相机TF。配置语法已检查，GUI显示需用户在桌面实际打开。

应看到同步图像持续更新，板和四ArUco完整、无过曝模糊；点云持续刷新，板对应的平面有覆盖。鼠标左键旋转、滚轮缩放；用Select选择板附近点，从Selection面板查看实际LiDAR坐标，以便配置ROI。单帧看不清圆孔时，录静态bag后用离线50帧诊断，不因肉眼看似有孔就判定四孔提取成功。Image面板若没有显示，可用Panels菜单添加Image显示对应的面板，或关闭/重新启用Image显示；先检查话题是否实际有数据。

预览之后录scene_01，再运行calib.py image、cloud和diagnose_lidar.py。RViz显示不能证明header配对或物理触发沿精度，录制审计及CSV仍需通过。

2026-10-21 17:49启动会话在检查时出现相机MV_E_NODATA重复超时/重连，camera_sync.csv尚无数据；雷达与writer继续运行。这说明当时相机还未正常出帧，不应采集标定数据。先核对同步器供电、外触发输出到相机Line0的信号/接线和公共地；具体原因未确认，不能仅凭日志认定硬件故障。日志是当时观察结果，后续以当前会话为准。

## 单独启动相机排查

先停止占用相机的节点。若由本任务一键启动，可执行 `scripts/stop_stage_b.sh`（同时停止本次雷达、writer和相机）。不自动终止其他任务或MVS。

`./scripts/start_camera_only.sh` 默认自由运行：读取本任务 config/camera_free_run.yaml，trigger_enable=false，10Hz、20ms曝光、原始/front/image；不依赖雷达/同步器，不发布/front/image_sync。配置复制自已安装驱动并只在任务目录修改，原驱动不改。Ctrl+C停止。

`./scripts/start_camera_only.sh trigger` 只启动Line0外触发相机，仍须硬件脉冲才出原始图像；/front/image_sync还需要正在更新的timeshare桥接。不能把单独启动外触发节点当成绕过触发问题。

自由运行看图：启动 `scripts/preview_stage_b.sh` 后将Image Topic改为/front/image；PointCloud2没有雷达时可关闭。自由运行测试成功后，Ctrl+C停相机，再启动完整Stage B恢复硬件同步采集。当前驱动的CameraInfo可能是旧占位文件，不用于外参求解；本任务求解读取已校验的真实内参文件。

脚本命令dry-run和语法/配置检查通过，未由Agent实际启动相机。

## 未参与拟合的独立验证数据

保持本次标定的相机/LiDAR刚性关系及镜头设置不变，只把板移到不同距离或高度，静止后执行：

```bash
cd ~/calibration/lidar_camera_extrinsics
source scripts/env.sh
./scripts/record_scene.py validation_01 --seconds 10
```

输入 `RECORD validation_01` 确认。新bag保存在datasets/validation_01/时间戳/bag；仅采集和审计，不自动加入求解清单。使用既有联合外参和已归档的joint/camera_info_used.yaml投影，不重新拟合：

```bash
./scripts/calib.py project datasets/validation_01/实际时间戳/bag \
  --intrinsics results/multi_20261021_183657_578407/joint/camera_info_used.yaml \
  --transform results/multi_20261021_183657_578407/joint/extrinsics.yaml \
  --frames 1 --out results/independent_validation_01
```

若传感器相对安装或镜头设置改变，这份数据不能用于验证原外参。旧Stage B bag虽未参与拟合，但无法确认其安装关系与本次标定相同，不能默认当作有效独立验证数据。
