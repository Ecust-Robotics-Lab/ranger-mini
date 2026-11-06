# 本机标定与同步流程

正式位置为 `/home/lab307/ranger_nav/calibration`。2026-10-26 从 `~/calibration` 原子迁入，原路径现在仅是兼容链接。新入口、相机包及新编译的 FAST-Calib 已在临时移除兼容链接的条件下通过离线检查。

## 目录与数据来源

| 路径 | 职责 |
| --- | --- |
| `camera_intrinsics/` | 棋盘采集、质量检查、五参数内参求解、合并会话、原始图片及历史结果 |
| `camera_intrinsics/results/camera_intrinsics.yaml`、`camera_info.yaml` | 本机现有真实内参，保留原验收状态和 k3；不是本轮重新标定 |
| `lidar_camera_extrinsics/` | ROS 2 FAST-Calib 本机适配、板参数、场景配置、求解与独立验证流程 |
| `lidar_camera_extrinsics/config/` | 原始场景、ROI、配置哈希及求解清单；历史绝对路径在读取时解析到新目录，原文件字节不改写 |
| `lidar_camera_extrinsics/datasets/`、`results/` | 原始场景、审核证据、试验外参与独立验证结果 |
| `bags/stageB_sync_run2/` | 原始三传感器静态同步检查包；不属于运动闭环数据 |
| `ws_calib/` | 旧工作空间，保留追溯；新入口使用工程根目录的 `ws_camera/` |
| `../ws_camera/src/hik_camera_trigger_driver/`、`timeshare_writer/` | 当前维护和构建的相机、共享戳 writer 源码 |

大体积图片、bag、构建产物和运行日志不进入主仓库。流程、源码、配置、内参和来源清单已随 `0ced46c` 本地提交；原始数据和SDK仍只保留在本机，当前Git状态以工程HANDOFF为准。旧标定仓库原 SHA `82936bbbd12c5d5d93cf9f9d2564ec6a5c37fac8` 的完整历史保存到 `.runtime/backups/consolidation_20261026/calibration_history.bundle`；原 `.git` 元数据也保留于该备份目录。FAST-Calib 上游部分版本当年未能核实，见原 `upstream/SOURCE_MANIFEST.json`，本轮没有将其改称已验证提交。

## 构建

在工程根目录执行。入口默认只显示计划，`--execute` 才编译；构建不访问传感器。

```bash
bash scripts/build_camera_sync.sh --execute
bash scripts/build_calibration.sh --execute
```

使用 ROS 2 Humble、本机现有 OpenCV/ArUco、PCL、Eigen、cv_bridge、rosbag2 依赖。相机 SDK 复用 `ws_camera/src/hik_camera_ros2_driver/hikSDK`，新驱动通过相对链接引用；SDK 是本机依赖，不随 Git 分发。SDK 缺失时构建入口明确报错。FAST-Calib 使用 `lidar_camera_extrinsics/ws/build_integrated` 和 `ws/install_integrated/fast_calib`，避开旧位置的 CMake 缓存。

## 统一入口

以下命令显示计划，不启动实物，也不覆盖已有结果：

```bash
bash scripts/run_calibration.sh camera
bash scripts/run_calibration.sh capture -- --topic /front/image_intrinsics
bash scripts/run_calibration.sh intrinsics -- --input calibration/camera_intrinsics/SESSION --output calibration/camera_intrinsics/NEW_RESULTS
bash scripts/run_calibration.sh sync
bash scripts/run_calibration.sh extrinsics -- --help
bash scripts/run_calibration.sh solve -- --help
```

`camera` 为内参采集的自由运行相机，发布 `/front/image_intrinsics`；使用主机时间，不是硬同步模式。`capture` 仅订阅、检查并保存图片。顺序是 camera、另一个终端 capture、停相机、审查图片，再离线 intrinsics。结果目录选新名称；详细参数可用 `intrinsics --execute -- --help` 查看，此 help 调用不求解。

`sync` 为外触发流程：Livox PointCloud2 → writer → Line0 相机，共用本次会话的私有 timeshare 文件，CSV 落入日志目录。先获得本次传感器测试授权，再增加 `--execute`；执行后前台 Ctrl+C 停止。同步会话也可查询/停止：

```bash
bash scripts/run_calibration.sh status
bash scripts/run_calibration.sh stop
```

这两个控制命令仅管理标定同步会话；FAST-LIVO2 在线会话仍用 `run_fast_livo2_live.sh --status/--stop`。二者会检查设备占用，不应并行启动同一套驱动。

外参流程保留原工具顺序：`extrinsics` 下的 `audit`、`import-intrinsics`、`image`、`cloud`、`make-config`；审核板尺寸/方向、场景与特征对应后，`solve single|multi|diagnose MANIFEST`；独立验证继续用 `lidar_camera_extrinsics/scripts/validate_trial.py`。统一入口增加 `--execute` 后才调用子命令；参数见 `extrinsics --execute -- --help` 与 `solve --execute -- --help`。

板与采集要求见 [BOARD_AND_CAPTURE.md](lidar_camera_extrinsics/docs/BOARD_AND_CAPTURE.md)，结果见 [TRIAL_CALIBRATION_REPORT.md](lidar_camera_extrinsics/TRIAL_CALIBRATION_REPORT.md)。原目录中的历史报告保留当时路径和命令，当前操作以本文和统一入口为准。

## 当前验收边界

- 内外参维持 **PARTIAL / 试验值**；本轮只迁移流程和验证接口，没有重新标定，也没有自动更新 FAST-LIVO2 外参。
- 共享文件一致性和重复 writer 拒绝已离线验证；数值戳配对不证明曝光与扫描物理同步。雷达历元、曝光参考点与运动数据质量仍待验。
- 五个畸变系数完整保留；`compatibility_camera_info_k3_zero.yaml` 是历史兼容实验，当前前端不使用它。
- 旧 `ws_calib/`、FAST-Calib build/install 和结果保留；其中含历史部署链接与追溯信息。
