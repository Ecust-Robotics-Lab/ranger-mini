> 整合后入口说明（2026-10-26）

本目录已随标定资产迁入 ranger_nav。当前统一流程、构建位置、同步入口及验收边界以 [上级 README](../README.md) 为准。下面保留原项目说明；其中旧绝对路径为历史记录，新的构建使用 build_integrated/install_integrated，同步默认 dry-run。

# Mid-360S + Hikrobot FAST-Calib 外参工作区

本目录独立部署 FAST-Calib ROS 2，使用 Zhi-Calf 的算法副本并修正本机兼容性。只读现有 bag 和内参文件；不启动传感器、不改驱动或 Stage B、不写 FAST-LIVO2。

当前状态见 [STAGE_C2_REPORT.md](STAGE_C2_REPORT.md)。三个真实场景的标记、ROI和四孔检查已完成；经用户明确接受当前 PARTIAL 五参数内参，单场景与联合试算已完成，见 [试算报告](TRIAL_CALIBRATION_REPORT.md)。独立几何验收尚未完成。不要把输入读取、优化返回、特征匹配、几何验收混为一谈。

## 文件

- `upstream/`：通过网页访问取得的候选源码快照；不是成功 git clone，具体来源和 SHA256 见 `upstream/SOURCE_MANIFEST.json`。
- `ws/src/fast_calib`：本地独立算法副本，含单场景、联合 SVD、输入探针和 LiDAR 诊断程序。
- `config/board.yaml`、`config/target_params.yaml`：正式确认的米制几何尺寸；ArUco 半间距 0.55/0.35。
- `config/scenes.yaml`：三个场景真实 ROI 待填写。
- `config/record_qos.yaml`：可靠录制 QoS。
- `scripts/`：构建、采集、输入审计、内参导入、ArUco/ROI/圆孔诊断、显式求解和投影。
- `docs/BOARD_AND_CAPTURE.md`：板测量、字典/ID、坐标、现场操作指南。
- `docs/CALIBRATION_REPORT_TEMPLATE.md`：后续实际标定报告模板。

## 构建和测试

```bash
cd ~/calibration/lidar_camera_extrinsics
source /opt/ros/humble/setup.bash
./scripts/build.sh
source ws/install/fast_calib/share/fast_calib/local_setup.bash
/usr/bin/python3 tests/test_interfaces.py
ws/install/fast_calib/lib/fast_calib/offline_checks
```

构建脚本使用 ament/CMake，单编译进程，不安装系统依赖。沙箱里 colcon 停在命令环境子进程阶段，因此使用可独立复现的直接构建；常规终端也可以在 `ws` 下执行 `colcon build --packages-select fast_calib --executor sequential --cmake-args -DBUILD_TESTING=OFF`。当前已用直接方式编译安装并运行接口检查，未声称 colcon 构建成功。

输入接口检查不需要 DDS、相机或雷达：

```bash
ws/install/fast_calib/lib/fast_calib/input_probe ~/calibration/bags/stageB_sync_run2
./scripts/calib.py audit ~/calibration/bags/stageB_sync_run2 --out results/stageB_input_audit.json
```

SQLite3 已安装。MCAP 需要系统 `rosbag2_storage_mcap` 插件，当前未安装；源码 reader 可选择插件不等于本机能读 MCAP，因此录制固定 SQLite3。没有尝试改动系统安装。

## 一键启动 Stage B

协调好相机使用时间后执行 `./scripts/start_stage_b.sh`，默认20ms曝光；Ctrl+C或另一个终端执行 `./scripts/stop_stage_b.sh` 停止本次会话。使用现有驱动，未改驱动代码；启动不自动录制。命令预览可用 `--dry-run`。详见 [一键启动说明](docs/ONE_CLICK_STAGE_B.md)。

## 真实采集与特征检查

先读 [操作指南](docs/BOARD_AND_CAPTURE.md)。用户协调好内参任务、手动启动 Stage B 已验证驱动后，执行：

```bash
./scripts/record_three_scenes.sh
```

每场景分别确认、静止录约 8 秒；采集完成只审计，不求解。每次独立目录，保留原始图像/点云、header 时间戳、topic 类型及 publisher QoS 快照。保存驱动 CSV 以核对硬件 FrameID。

对于每场景的实际 BAG：

```bash
./scripts/calib.py image BAG --out results/scene_01_image
# 依据 LiDAR frame 的实际位置编辑 config/scenes.yaml；ROI 无示例实机默认值
./scripts/calib.py cloud BAG --scene scene_01 --frames 50 --out results/scene_01_roi
./scripts/diagnose_lidar.py BAG --scene scene_01 --frames 50 --out results/scene_01_lidar
```

三个工具均离线读取真实消息，保存 PNG、PCD、JSON 和日志。ArUco 工具返回非零表示四 ID/排列/方向/裁边检查不全；仍保存预览。LiDAR 工具复用固态算法，没有机械 ring 逻辑。最后一个命令不需 K/D；空 ROI 或四孔失败会非零退出，保存原因建议。点云与板需完全静止才允许累积。

检查 `aruco.png` 是 **DICT_6X6_250**，打印面正视 **左上1、右上2、左下3、右下4**、每个标记不单独旋转，确认后设置 `config/board.yaml: orientation_verified: true`。实物已通过 scene_01 的真实图像验证，当前配置 orientation_verified 已为 true；后续更换板需重新核对。

## 真实 K、D 的导入和正式参数生成

```bash
./scripts/calib.py import-intrinsics
```

自动读取 `~/calibration/camera_intrinsics/results/camera_intrinsics.yaml`，与同目录 `camera_info.yaml` 核对，保存 `config/intrinsics.imported.json`。支持 ROS camera_info 的扁平 data 矩阵及本次内参 Agent 的嵌套矩阵。只接受 plumb_bob 4/5 系数，完整保留 k1,k2,p1,p2,k3；未知模型/尺寸/非法矩阵拒绝，绝不补假内参。

当前真实内参状态 PARTIAL：可导入和离线预览，但默认生成求解参数会拒绝。等待内参 Agent 完成检查；如果用户评估后接受当前 PARTIAL 结果，可在下面命令明确添加 `--accept-partial-intrinsics`，并在 provenance 中记录该决定。未自动选择 k3=0 的兼容性内参文件。

板方向、尺寸和各场景 ROI 全部确认，内参状态通过后：

```bash
./scripts/calib.py make-config BAG --scene scene_01 \
  --result-dir "$PWD/results/planned/scene_01" \
  --out "$PWD/config/scene_01/qr_params.yaml"
```

scene_02 / scene_03 分别执行，生成各自配置与 provenance。只有 make-config 才产生正式 `qr_params.yaml`。`config/target_params.yaml` 仅含尺寸，不可单独求解。安装包默认配置 `configuration_validated: false`，直接启动不会使用源码示例内参/尺寸进行真实求解。

生成之后可以用内参和板几何得到图像孔中心预览：

```bash
./scripts/calib.py image BAG --intrinsics ~/calibration/camera_intrinsics/results/camera_intrinsics.yaml \
  --board config/board.yaml --out results/scene_01_board_pose
```

图像 ArUco 角点重投影误差与推算孔中心有输出。推算孔中心并非在图像上独立观测出的孔边界。

## 单场景和多场景求解（首轮试算已完成）

复制 `config/quality_review.template.yaml`，为每场景填入真实 bag 路径、图像/点云诊断 JSON 路径、reviewer 和现场静止/对应确认。人工检查后才把四个确认项设为 true。复制 `config/solve_manifest.template.yaml`，填写至少三个不同 bag 对应的配置和 review 绝对路径。

```bash
./scripts/solve.py single config/solve_manifest.yaml --scene scene_01
./scripts/solve.py single config/solve_manifest.yaml --scene scene_02
./scripts/solve.py single config/solve_manifest.yaml --scene scene_03
./scripts/solve.py multi config/solve_manifest.yaml
```

multi 会在新的本次结果根目录下重跑所列各场景并合并精确场景清单。每个 single 保存独立中心记录；合并文件只由这些新运行的一块/场景记录组成，历史记录不参与。联合程序已从只读最后三块改成使用清单内所有场景（至少三个）。算法是原始无权/加权接口的闭式 SVD 刚体最小二乘，不宣称为非线性束调整。

输出 `extrinsics.yaml` 包含 T_cam_lidar、T_lidar_cam、行列式/正交性/逆检查、逐孔/逐场景米制残差、由 ArUco 板位姿推算的孔中心重投影差。上游无协方差输出，`uncertainty: null`，不虚构不确定性。几何验收始终默认 false，需要独立数据、视图叠加和重复性验证。

可直接运行包节点（应优先用上面的隔离流程）：

```bash
source ws/install/fast_calib/share/fast_calib/local_setup.bash
ros2 launch fast_calib calib.launch.py config:=/absolute/generated/qr_params.yaml
# 不启动 RViz，不需要 /clicked_point；默认求解后退出
```

## 畸变一致的投影验证

```bash
./scripts/calib.py project BAG \
  --intrinsics ~/calibration/camera_intrinsics/results/camera_intrinsics.yaml \
  --transform results/实际求解目录/scene_01/extrinsics.yaml \
  --frames 1 --out results/scene_01_projection
```

`overlay.png` 叠加在**原始带畸变图像**，使用完整五参数 OpenCV projectPoints。只投影相机正深度点，像素内近点覆盖远点。单帧默认；静态时可显式增加帧数。投影日志保留实际 header frame_id 和累积时间范围。相机 x右/y下/z前，LiDAR x前/y左/z上；`p_cam = R_cam_lidar p_lidar + t_cam_lidar`。输出两个方向并验证互逆，不自动发布 TF 或写 FAST-LIVO2。

叠加图没有独立标注就不能给“重投影精度”；工具明确保持独立重投影误差为 null。求解残差较小、优化成功和实物几何正确是三个不同判断。详见报告模板。

## 官方样例

官方 Drive 下载与仓库 `dev_humble/calib_data/mid360_11` 二进制在当前网络条件均不可取得。失败日志在 `logs/clone*.log`、`logs/download_example.log`、`logs/sample_and_pin_access.txt`。提供 `scripts/download_example.sh` 供网络恢复后下载真实例子，不覆盖已有文件；下载成功仍需使用该样例自身内参/话题/图像和板配置做专门回归，不能把样例内参替换实机内参。

当前 Stage B 图像未包含 FAST-Calib 板，0 标记检测是预期的负面输入检查，不是样例标定通过。`tests/` 中合成几何仅用于数值和接口自检，绝不当成真实标定结果。

实时预览：Stage B运行后，在桌面另开终端执行 `./scripts/preview_stage_b.sh`，同时显示 `/front/image_sync` 与 `/livox/lidar`。只订阅已有话题，不启动相机；具体判断标准和无图像排查见 [一键启动说明](docs/ONE_CLICK_STAGE_B.md)。

## 当前试算结果

用户已明确接受当前真实PARTIAL五参数内参，仅用于试算。结果位于 `results/multi_20261021_183657_578407/`，联合变换为该目录 `joint/extrinsics.yaml`，包含两个方向。联合特征中心RMS约3.85mm；不是独立真实精度。各场景孔边界和点云投影在 `validation/scene_XX/`，留出场景误差与单场景差异见 [试算报告](TRIAL_CALIBRATION_REPORT.md)。几何验收仍为false，未写FAST-LIVO2。

为已有结果生成孔边界叠加与逐场景留出诊断：

```bash
./scripts/validate_trial.py results/multi_20261021_183657_578407
```
