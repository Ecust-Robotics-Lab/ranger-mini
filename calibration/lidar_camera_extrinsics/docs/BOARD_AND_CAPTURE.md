# 标定板与三个静态场景操作指南

所有开发、输出都在 `~/calibration/lidar_camera_extrinsics`。程序不会启动硬件驱动。需先由用户协调内参 Agent 的相机使用时间；采集脚本每个场景要求手动输入 `RECORD scene_XX`。

## 已确认的尺寸

2026-10-21 用户确认使用官方标准尺寸。已写入 `config/board.yaml` 和 ROS 参数文件 `config/target_params.yaml`（也复制到源码包 config）。禁止将上游示例相机参数用于实机。

| 参数 | 已确认值（m） | 几何定义与核对位置 |
|---|---:|---|
| 板外形 | 1.400 × 1.000 | 板外缘完整宽、高；算法不直接使用外形 |
| marker_size | 0.200 | ArUco 黑色正方形外边界边长，包含黑框，不含外围白色留边 |
| delta_width_qr_center | 0.550 | 同一排两 ArUco 中心完整间距 1.100 的一半 |
| delta_height_qr_center | 0.350 | 同一列两 ArUco 中心完整间距 0.700 的一半 |
| delta_width_circles | 0.500 | 同一排左右两圆孔的完整中心距 |
| delta_height_circles | 0.400 | 同一列上下两圆孔的完整中心距 |
| circle_radius | 0.120 | 实际开孔半径，完整孔径为 0.240 |

卷尺测长距离、卡尺核对黑框边长和孔径。孔中心距离也可用同侧孔边之间距离测量（等直径时等于中心距）。测量上下两排、左右两列，检查板面平整、打印比例、孔径一致、纸张未拉伸。保留测量照片和测量不确定度。尺寸既已由用户确认，不再标成 UNMEASURED；实际 ID、朝向和 ROI 仍待验证。

## 字典、ID、方向与坐标

源码 `qr_detect.hpp` 指定 **DICT_6X6_250**；打印面朝向传感器、板竖直摆正时：

```text
        板 +y 向上
       ID 1       ID 2
          ○     ○       上孔
          ○     ○       下孔
       ID 3       ID 4
                    板 +x 向右
```

ArUco 构造顺序是 `[1, 2, 4, 3]`，即左上、右上、右下、左下；所有标记的自身标准方向须与板一致，不能单独旋转贴纸。板原点在标记中心矩形与孔中心矩形的共同中心，板 z=0，+z 朝打印面外。角点按每个标记的左上、右上、右下、左下定义。

孔的板坐标原始顺序为 `(-0.25,+0.20,0)、(+0.25,+0.20,0)、(+0.25,-0.20,0)、(-0.25,-0.20,0)`。图像中并非直接找黑色圆形：从 ArUco 板位姿和这些已知几何位置推算相机坐标系中的孔中心。LiDAR 则从板平面回波边界拟合实际圆孔的中心，不是孔后背景回波的中心。

相机 optical frame：x 右、y 下、z 前；Livox 原生 frame：x 前、y 左、z 上。这里假设驱动发布原生 Livox 坐标，没有预先旋转；实际 header 为 `livox_frame` / `front_optical_frame`。本地修正将 LiDAR 点转换为 `(-y,-z,x)` **仅用于对应特征排序**，最终 SVD 仍使用原 LiDAR 点坐标。不能把这一步误当作已求出的外参。场景应竖直、朝向近似前方，避免板倒置、很大 roll 或传感器倒装；这会使矩形孔排序产生对称歧义。

实际字典、ID 和朝向尚未通过真实板图像确认。先运行图像预览，检查 `aruco.png` 的编号、角点 0..3 和 `image_quality.json`，再将 `config/board.yaml` 的 `orientation_verified` 改为 true。识别失败时检查清晰度、曝光、字典、贴纸旋转、遮挡及打印分辨率，禁止跳过检查强行求解。

CAD 的官方 Drive 链接在当前工具中无法访问；以上定义来自实际选用源码的坐标构造，并结合用户确认尺寸。未声称已核对 CAD 二进制文件。

## 现场布置

1. 相机、雷达固定在同一刚性支架，三个场景期间不可移动相对位姿，镜头对焦、光圈、分辨率与内参采集条件保持一致。
2. 板完整进入相机视野，四个 ArUco 不裁边，四孔均对雷达无遮挡。板后留出明显深度间隙，避免贴墙、手、支架遮住孔。不要让与板同平面的墙占据 ROI。
3. scene_01 正面；scene_02 左偏；scene_03 右偏。可先尝试约 15–30° 偏转，以四孔回波和四标记清晰可见为准。这是布置建议，不是预设验收阈值。每次调整后完全静止，采集期间不扶板晃动。
4. 用户按 Stage B 报告启动现有驱动和同步系统。本任务不重启、不替换相机或 LiDAR 节点。
5. 从已 source Humble 的终端执行：

```bash
cd ~/calibration/lidar_camera_extrinsics
source /opt/ros/humble/setup.bash
./scripts/record_three_scenes.sh
```

每场景提示确认后录制约 8 秒。完成一个场景后再调整板、静止、确认下一个场景。也可单独执行：

```bash
./scripts/record_scene.py scene_01 --seconds 8
```

脚本先检查两话题各有一个 reliable 发布者，再录制原始 `/livox/lidar` 与 `/front/image_sync`，可靠 QoS、volatile、SQLite3、512 MiB 缓存。每次保存到 `datasets/scene_XX/时间戳/bag`，不覆盖、不改 header 时间戳、不自动求解。磁盘需预留数 GB。

保存产物：`record.log`、`publisher_snapshot.json`、`capture_manifest.json`、`bag_audit.json`。审计检查完整消息数量、时间范围、字段、编码、分辨率、严格递增、>150ms 间隔、精确 header 配对、metadata 一致性，并用编译的 FAST-Calib input_probe 读取。两话题录制首尾可能因订阅建立时机相差一帧；审计会明确失败，不静默裁帧，检查记录日志、延长稳定等待后重录新 bag。

硬件 FrameID 不在 sensor_msgs/Image 中，不能仅凭相等的 bag 计数证明硬件零丢帧。另保存 Stage B 相机驱动同期 CSV，检查 FrameID 跳变、sync 状态和发布数。header 同域配对也不代表测得物理触发沿精度。

## 录制后的预览和 ROI

下面的 BAG、OUT 替换为本次实际路径；OUT 应在本任务目录内，诊断用新目录。

```bash
./scripts/calib.py image BAG --out results/scene_01_image
./scripts/calib.py cloud BAG --scene scene_01 --frames 50 --out results/scene_01_roi
```

cloud 命令前编辑 `config/scenes.yaml`，填入该场景真实的 `x/y/z_min/max`（米，LiDAR frame）。若还不知道 ROI，可先在单独的临时 scenes 文件设置适合实景的宽范围，查看 `roi.png` 和原始 `roi.pcd`，再收紧。**仓库不提供默认实机 ROI**。PNG 是无需 GUI 的三维点云预览和源图；可自行用点云查看器打开 PCD，程序无 RViz 点选依赖。

进一步使用原 FAST-Calib 固态分支检测圆孔，无需等待 K、D：

```bash
./scripts/diagnose_lidar.py BAG --scene scene_01 --frames 50 --out results/scene_01_lidar
```

保存 ROI、板面、压平平面、边界、候选圆心和最终中心 PCD，`lidar_quality.json`、`circle_support.json`、原算法日志及 `lidar_diagnostic.png`。查看板面距离、平面 RMS、点数、每圆候选边界点数与 24 角度扇区覆盖。建议比较 10/20/50/100 帧：静态累积应改善孔边界支持；板或传感器运动会使边界变厚。默认 50 帧约 5 秒，最多 100 帧，避免整个长 bag 无限制累积。

原算法：5 mm 体素、1 cm 平面距离阈值、3 cm 法线/边界邻域、圆半径 ±3 cm、圆拟合 2 cm 阈值、几何相对公差约 8%。这些是继承的提取参数，**不是标定精度标准**。少于四孔、几何歧义、点过稀或平面支持不足均失败退出，生成建议，不求外参。

完成三场景的图像和点云检查后，再按 README 的内参导入、配置生成和求解步骤操作。
