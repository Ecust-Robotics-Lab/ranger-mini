# STAGE C2：FAST-Calib / Mid-360S + 海康相机外参准备报告

日期：2026-10-21。任务根目录：`/home/lab307/calibration/lidar_camera_extrinsics`。

**当前结论：独立部署和三场景实物检查已完成；经用户明确接受当前真实PARTIAL五参数内参，已完成三个单场景与联合试算、双向变换检查和投影验证。联合特征中心RMS约3.85mm，尚未通过独立几何精度验收，未写FAST-LIVO2。详细结果见TRIAL_CALIBRATION_REPORT.md。下文保留初次部署和后续更新的历史证据，以最新结果为准。**

未修改 `ranger_nav`、`ws_calib` 或 `camera_intrinsics`，未改写原始bag/header时间戳或FAST-LIVO2。硬件采集由用户通过独立启动/录制脚本执行；本Agent离线检查真实数据，并在用户接受当前内参后运行真实外参试算。

## 独立场景投影更新

用户新录validation_01/20261021_185640_899318未参与拟合。使用既有联合外参和标定时归档的真实五参数内参，生成单帧和静态5帧叠加图，结果位于results/independent_validation_01_20261021_185640。原bag首帧图像无对应点云，其余98对header精确相等；仅选择共同header投影，未删改原消息或修改原审计。没有重新拟合；几何精度验收仍未自动通过。

## 最新结果：已获授权并完成三个单场景与联合试算

用户明确接受当前真实PARTIAL五参数内参用于试算，结果保留待独立验收。有效结果目录为results/multi_20261021_183657_578407，详见[TRIAL_CALIBRATION_REPORT.md](TRIAL_CALIBRATION_REPORT.md)。

联合T_cam_lidar已输出，t=[-0.003503,-0.088245,0.000141]m；两个方向矩阵、行列式/正交性/逆检查通过。联合12对中心RMS3.846mm；三个单场景RMS0.930/2.802/0.788mm，联合逐场景RMS3.901/4.890/2.289mm。逐场景留出RMS5.749/6.633/3.316mm。三场景原图五参数点云投影和孔边界叠加已保存并查看。

单场景之间最大差异约0.466°/19.33mm，尚需独立数据验收；这些残差不是绝对外参精度。内参PARTIAL状态未修改，uncertainty=null，geometric_calibration_accepted=false，未写FAST-LIVO2。源码版本df5d566d6ab0e229eb41c8b571331d16fd3f23b9；已修复Humble YAML别名兼容性问题，七项接口测试通过。

此更新取代下文历史记录中的“等待内参决定/未生成正式配置/未求解”状态，其余历史证据保留。

## 最新更新：三个真实场景均完成输入和特征检查

scene_01为78对/7.7秒，scene_02和scene_03各99对/9.8秒；全部header精确配对，所有帧四ArUco完整，同期驱动FrameID连续、sync状态全OK。三个场景以50帧和完整录制均成功提取四孔；50帧板面RMS分别4.177/3.742/3.917mm，累积敏感性最大2.356/3.291/2.373mm（重叠数据，非独立不确定度）。真实ROI、各场景质量复核和求解清单已保存。

详见results/THREE_SCENE_REVIEW.md和results/three_scene_input_review.json。此更新取代历史记录中“左右场景待采集/ROI未配置”状态。当前真实内参仍为PARTIAL（边缘覆盖不足），等待按当前内参试算或补充内参的选择；尚未生成正式qr_params.yaml、未执行真实外参求解、未写FAST-LIVO2。

## 实物数据更新：正面场景已检查（2026-10-21）

首份正面bag为datasets/scene_01/20261021_181558_055290/bag。78/78精确配对、约7.7秒；78帧均检测四ArUco，字典/ID/朝向符合预期，板方向已在配置中标记验证。原算法以50帧和78帧均成功提取四孔，板面RMS约4.1mm，两个累积结果的中心最大差约2.36mm（重叠数据敏感性，不是独立精度）。同期驱动FrameID连续且78条全OK。

详细报告：results/scene_01_20261021_181558/SCENE_01_REPORT.md。此更新取代下文历史记录中的“尚无实板数据/方向未验证”状态；左右两场景、PARTIAL内参评估、真实外参求解及独立精度验收仍待完成，未进行任何外参求解。

## 内参更新：2026-10-21 17:29 版本（当前有效）

按用户通知，已重新导入内参 Agent 的新结果，并与新 camera_info.yaml 交叉核对通过。下文第5节的旧 K/D 仅保留为初次部署记录，**当前以本节和 config/intrinsics.imported.json 为准**。

```yaml
image_width: 1280
image_height: 1024
distortion_model: plumb_bob
fx: 1299.9988190926874
fy: 1299.8402206328024
cx: 640.2595320942488
cy: 484.9367557425784
k1: -0.08288339607839673
k2: 0.15032436811294939
p1: -0.0005625494644882011
p2: -0.00039491110944710704
k3: -0.08810366754215963
```

来源 generated_at：2026-10-21T17:29:06.754169+08:00；63张输入，50张训练、13张留出；训练 RMS 0.07678px，留出 RMS 0.08514px。原始文件 SHA256：`466cb1595c4d296f51e6cbe1011ddc1c5ba329b51f7b28053d0faede7ba8926f`。

六项接口测试重新通过。新五参数投影公式与 OpenCV 最大差0px，迭代去畸变最大差约2.27e-13px；在相同测试网格把新 k3 清零，最大投影差约2.241px。保留完整五参数。

新结果仍为 **PARTIAL**，警告为边缘/角落覆盖不足，故 ready_for_solve 仍为 false，没有自动放行正式求解。未修改内参任务文件、未启动硬件。旧导入快照及测试日志归档于 `results/intrinsics_history/20261021_165207/`，更新比较记录在 `results/intrinsics_update_20261021_172906.json`。此前没有正式 qr_params.yaml 或真实外参，不涉及旧外参继续使用的问题。

## 1. 源码选择及版本

选择 [Zhi-Calf/calibration 的 FAST-Calib](https://github.com/Zhi-Calf/calibration/tree/master/FAST-Calib)。源码已有 rosbag2、bag 内图像、固态/机械分支、单场景和多场景 SVD。完整审查见 [SOURCE_AUDIT.md](docs/SOURCE_AUDIT.md)。

| 项目 | 核查结果 |
|---|---|
| [HKU-MARS 官方](https://github.com/hku-mars/FAST-Calib) main | 官方算法基准；catkin/ROS1 bag，不直接适配 Humble |
| [ichangjian](https://github.com/ichangjian/FAST-Calib-ROS2/tree/dev_humble) dev_humble | ROS2 single；PointCloud2 bag + 独立图像；main 分支仍是 ROS1，不能误取 |
| Zhi-Calf master/FAST-Calib | ROS2 single + multi；本轮选用并独立修复 |

终端 `git clone` 全部失败：本机代理连接失败，清除代理后 DNS 解析失败。因此利用网页 raw 文本取得核心源码，逐行连续性检查后保存至 `upstream/`，并非完整 Git clone。未获取的装饰性 color.h 使用本地空颜色宏兼容头替代；GPLv2 LICENSE 取官方文本。

GitHub [子目录历史](https://github.com/Zhi-Calf/calibration/commits/master/FAST-Calib) 显示最后相关 commit 为 `84d92f846a61d188bae2511dcba6d346f7b72485`，2026-03-24。固定 SHA raw URL 无法访问，**未验证所取 master 快照与该 Git tree 相同**。不把这个观察到的 commit 冒充经过 clone 验证的源码版本。实际文件 SHA256 与来源见 [SOURCE_MANIFEST.json](upstream/SOURCE_MANIFEST.json)，本地代码版本见本报告末尾。

## 2. 编译和接口适配

本地工作区 `ws/src/fast_calib`，系统依赖已具备：Humble、PCL 1.12、OpenCV 4.5.4（含 aruco）、Eigen、cv_bridge、rosbag2_cpp/storage、pcl_conversions。

- **直接 ament/CMake 编译安装成功**，单编译进程；`scripts/build.sh` 可复现。
- colcon 在沙箱内停在命令环境子进程阶段，已终止，未声称 colcon 通过；独立 CMake 不依赖该步骤。
- 产物：`fast_calib`、`multi_fast_calib`、`input_probe`、`lidar_diagnostic`、`offline_checks`。ROS 包索引和 launch 参数解析通过。
- 日志：`logs/cmake_configure.log`、`logs/build_manual.log`、`logs/install.log`、`logs/build_final_console.log`；早期 colcon 记录在 `ws/log/` 与 `logs/build_console.log`。

主要修复：

1. 完整 k1,k2,p1,p2,**k3** 参数链；上游第五项固定为零已修改。
2. OpenCV 4.5 的 drawFrameAxes、姿态累加初始化、Rodrigues 矩阵类型一致性。
3. 要求四个精确 ID、正深度和板角点粗略重投影一致性（RMS ≤2px）；四孔不全则禁止求解。
4. Livox 原生 x前/y左/z上情况下，排序临时变换修为 `(-y,-z,x)`，SVD 仍使用原 LiDAR 坐标；保留实物对应复核要求。
5. 固态处理保留原 PCL 平面/边界/圆检测，不引入机械 ring 模型；修复法向旋转、缺失平面/圆候选保护和组合数溢出/越界。
6. max_frames 限制静态累积，非有限点过滤，PointCloud2 类型/内存布局校验；未知布局显式拒绝。
7. `use_point_pick:false`、无 RViz launch、离线 ROI 参数；默认保存后退出。
8. 联合 SVD 使用本次全部场景，至少三个，拒绝秩不足；每次结果目录和中心记录完全独立，禁止历史 append 污染。
9. 未验证配置不会使用源码示例值求解；正式参数只能在板/内参/ROI 检查后生成。

差异保存在 [local_changes.patch](docs/local_changes.patch)。

## 3. 真实输入验证和测试结果

只读 bag：`~/calibration/bags/stageB_sync_run2`。此 bag 是 Stage B 同步验证数据，**不是 FAST-Calib 标定板样例**。

| 检查 | 实测 |
|---|---|
| `/livox/lidar` | PointCloud2，834 条 |
| `/front/image_sync` | Image，834 条；1280×1024、rgb8、step=3840 |
| header 时间范围 | 1590193436100175970 ～ 1590193519400055810 ns |
| header 配对 | 834/834 精确相等，无回退/重复，无 >150ms 间隔 |
| frame_id | livox_frame / front_optical_frame |
| 点云字段 | x/y/z/intensity FLOAT32，tag/line UINT8，timestamp FLOAT64 |
| PointCloud2 布局 | point_step=26，小端、无行填充；无 ring，有 line |
| C++ 原算法预处理输入探针 | 前两帧有效点 28698；图像1280×1024；`INPUT_OK lidar_type=1`（Solid） |
| SQLite3 | Python rosbag2 和 C++ Reader 都通过 |
| MCAP | 本机无 mcap storage plugin；当前不支持实际读取/录制，源码框架支持插件不等于安装可用 |

完整结果：`results/stageB_input_audit.json`、`logs/input_probe_real.log`。bag 记录时间属于主机时间，算法使用消息 header 同域时间，不误把二者混合。图像里没有硬件 FrameID 计数器；正式采集仍需保存驱动 CSV 独立检查硬件丢帧。

测试情况：

- Python 六项接口/数值测试通过：真实五参数投影与迭代去畸变、Mid-360S 26字节点布局及行填充预览、rgb8/bgr8、板方向门禁、非法旋转拒绝、变换互逆。
- 实际 K、D 测试网格：OpenCV 投影与五参数公式最大差 0px；迭代去畸变最大差约 `2.27e-13px`。同网格仅把 k3 清零，最大投影差约 `2.635px`；这只是该测试网格的差异，不是实际标定误差估计。
- C++ offline_checks 通过：k3 入参保持、Livox/optical 对应排序、缺失特征拒绝、合成 ArUco PnP 接口、合成三场景联合 SVD 与退化输入拒绝。合成测试未保存为真实标定结果。
- 真实 Stage B 图像预览：检测 **0个标记**，保存 `results/stageB_image_preview/aruco.png` 与质量 JSON；图像没有当前板，不能据此判定板或算法兼容性失败/通过。
- 真实房间点云、10帧累积、仅测试 ROI：142912个输入有效点，33515个体素后 ROI 点，2956个平面点，2089个边界点；约5.67mm平面 RMS（房间表面，不是板面精度）。找到9个圆候选，但没有符合板几何的四孔组合，最终 **0个板孔中心，非零退出，未求解**。负例产物见 `results/stageB_room_lidar_negative/`。
- 空 ROI 负例：0个板面/圆心，正常失败，无外参输出；见 `results/stageB_empty_roi_negative_v2/`。

早期 `stageB_empty_roi_negative` 为诊断程序迭代过程中的旧版失败日志，正式负例以 `_v2` 为准。

## 4. 仓库真实样例和 CAD

已尝试官方 [单场景](https://drive.google.com/drive/folders/1W87Dx3MUuPhTpCLvaavWqNUJZV03yU6L)、[多场景](https://drive.google.com/drive/folders/1g__plgFqp5tsk-TY7Ioh4RXru62AdLmr)、[CAD](https://drive.google.com/file/d/1hdC8xGCHNP47a-wSLPyjr_tpOeynNFEG/view) 链接；当前工具不可访问，终端网络也失败。另尝试 ichangjian 自带 `dev_humble/calib_data/mid360_11` 的真实 db3/png，同样不可获取。

**官方样例标定回归未完成，未生成任何“标定通过”结果。**可执行编译、真实消息读取、负例和接口测试已按上述方式完成。失败记录：`logs/clone*.log`、`logs/download_example.log`、`logs/sample_and_pin_access.txt`。网络恢复后可运行 `scripts/download_example.sh` 下载仓库真实样例，不覆盖已有文件；需使用样例自身话题/内参/图像做单独回归，不可替代实机数据。

## 5. 内参与板配置

真实内参 Agent 文件已经存在，已自动读取并交叉校验 camera_info；来源状态仍为 **PARTIAL**，因此当前不自动放行正式求解。

- K：fx=1299.0436163581198，fy=1298.9710109488706，cx=639.1500474476223，cy=487.08425474910274。
- D：[-0.08257009711650069, 0.15253109433900297, -0.00012880838136669937, -0.0005877995846604111, **-0.10365792131876635**]。
- 模型 plumb_bob，1280×1024；完整保留 k3，没有使用 k3=0 兼容性输出。
- `config/intrinsics.imported.json` 保留来源/指纹/状态/警告，`ready_for_solve:false`。
- 等待内参任务完成或用户明确评估接受 PARTIAL；工具不会修改其结果或假定已验收。

用户已确认标准板六参数，已正式写入 `config/board.yaml`、`config/target_params.yaml` 和源码包内 target_params：

```yaml
marker_size: 0.20
delta_width_qr_center: 0.55
delta_height_qr_center: 0.35
delta_width_circles: 0.50
delta_height_circles: 0.40
circle_radius: 0.12
```

整体1.4×1.0m。ArUco 完整中心距1.1/0.7m，各取一半；圆孔中心距0.5/0.4m使用完整值。源码预期 DICT_6X6_250，打印面正视左上1、右上2、左下3、右下4，构造顺序 `[1,2,4,3]`。所有贴纸的自身角点方向必须一致。

几何确认已设 true，**orientation_verified 仍 false**；三场景 ROI 仍 UNMEASURED。尚未生成正式 `qr_params.yaml`。这符合当前还未核对实物与场景的状态，不会偷偷使用默认 ROI/示例内参。

## 6. 已交付程序与启动命令

完整流程见 [README.md](README.md) 和 [BOARD_AND_CAPTURE.md](docs/BOARD_AND_CAPTURE.md)。

| 交付 | 路径/状态 |
|---|---|
| 独立可编译算法工作区 | `ws/src/fast_calib`，已编译安装 |
| Mid-360S 输入与 bag 检查 | `input_probe`、`scripts/calib.py audit`，真实 bag 通过 |
| 内参导入/校验 | `scripts/calib.py import-intrinsics`，五参数通过，PARTIAL门禁保留 |
| 板模板/测量指南 | `config/board.yaml`、`config/target_params.yaml`、`docs/BOARD_AND_CAPTURE.md` |
| 三场景采集 | `scripts/record_three_scenes.sh` / `record_scene.py`，未执行真实采集 |
| ArUco 预览 | `scripts/calib.py image`，真实图像负例通过 |
| ROI/圆孔诊断 | `scripts/calib.py cloud` / `diagnose_lidar.py` / `pcd_diagnostic.py`，实际点云负例通过 |
| 单/多场景求解 | `scripts/solve.py`，已编译、SVD接口自检；真实流程未执行 |
| 畸变投影及双向变换检查 | `scripts/calib.py project`，数值接口测试通过；真实外参叠加待求解 |
| 报告模板 | `docs/CALIBRATION_REPORT_TEMPLATE.md` |

```bash
cd ~/calibration/lidar_camera_extrinsics
./scripts/build.sh
source scripts/env.sh
./scripts/calib.py import-intrinsics
# 用户完成硬件协调、手动按 Stage B 报告启动现有驱动以后：
./scripts/record_three_scenes.sh
```

脚本每场景都要求输入 `RECORD scene_XX` 确认，约8秒，检查一个可靠发布者/话题，SQLite3 + reliable QoS，保存新目录；录完自动审计、验证 FAST-Calib 读取，**不求解**。保存原始消息、metadata、计数、时间范围、字段和分辨率。审计异常不能视为采集成功，需检查日志重采新 bag。

## 7. 下一步由用户完成的实际操作

1. 协调内参 Agent 的使用时间，保持相机/雷达刚性连接、镜头设置与内参一致，手动启动已验证 Stage B 系统。
2. 正面摆放板，四标记/四孔无遮挡、板后留深度间隔。依次录正面、左偏、右偏，每次完全静止后确认录约8秒；建议偏转从约15–30°尝试，以实际可见性为准。
3. 每场景运行 `calib.py image BAG --out results/scene_XX_image`。核对真实 DICT_6X6_250、1/2/3/4编号、角点朝向与裁边情况。通过后才设置 board 的 orientation_verified true。
4. 依据实际 LiDAR 坐标填写 `config/scenes.yaml` 每个 ROI；运行 `calib.py cloud` 查看 PNG/PCD，再运行 `diagnose_lidar.py`。要求四孔提取成功并人工核对，查看平面残差、边界支持和候选歧义；失败先修布置/密度，不强行求解。
5. 等真实内参状态通过、交叉核对 K/D/分辨率，再用 `calib.py make-config` 为三个场景分别生成正式参数。若用户最终接受 PARTIAL，需显式 `--accept-partial-intrinsics` 并记录来源决定。
6. 填写每场景 quality_review 和 solve_manifest，显式运行 `solve.py single ... --scene scene_XX`，再 `solve.py multi ...`。独立目录防历史污染；至少三个不同、通过特征检查的场景。
7. 查看 T_cam_lidar / T_lidar_cam、正交性/行列式/逆检查、逐场景残差和完整五参数投影；另用未参与拟合的视角/距离验证。优化成功、匹配正确、精度合格分别记录。上游无协方差，不虚构不确定性。

**兼容性判断：当前标准板几何、Mid-360S实际 PointCloud2 和海康 rgb8 输入在软件接口上兼容，可进入真实三场景采集和特征检查；尚不能断言实板四孔可稳定提取或外参精度合格。** 残余验证项是实物标记、静态板孔回波、场景ROI、内参验收与真实标定回归。FAST-LIVO2 集成保留待用户检查及后续授权。

## 8. 本地源码版本

本报告引用的本地代码快照 commit：`0ff9efe4289dc103daee40d68c7074b8628e1f56`。此为独立任务目录本地 Git commit，包含构建源码、适配、工具、配置、文档与测试，不是上游远端 commit。运行日志和数据在磁盘保留，不纳入 Git。最后的报告文档提交可由 `git log -1` 查看。

## 后续增补：Stage B 一键启动

新增 scripts/start_stage_b.sh、stop_stage_b.sh 和 stage_b.py，按原报告顺序调用现有驱动，默认20ms曝光、writer后等待3秒；本次日志与CSV均保存在任务目录。原/tmp的已验证Mid360S JSON原样复制到config/mid360s_stage_b.json，消除临时文件依赖。加入重复进程检查、会话锁、PID启动时刻核对、仅本次进程组清理及启动失败回收。详见 docs/ONE_CLICK_STAGE_B.md。命令dry-run和三个进程管理测试通过，没有实际启动硬件。
