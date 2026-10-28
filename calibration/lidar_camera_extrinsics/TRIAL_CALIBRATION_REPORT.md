# 三场景 LiDAR–Camera 外参试算报告

日期：2026-10-21。状态：**试算完成，独立几何验收未完成，未集成FAST-LIVO2**。

本次目录：`/home/lab307/calibration/lidar_camera_extrinsics/results/multi_20261021_183657_578407`。本地源码 commit：`df5d566d6ab0e229eb41c8b571331d16fd3f23b9`。

用户已明确选择“使用当前真实五参数内参进行单场景和联合试算，结果保留待独立验收”。接受记录保存在本次目录 `user_trial_authorization.json`；内参原始状态仍为PARTIAL，没有改成PASS。

## 输入和运行隔离

- scene_01：datasets/scene_01/20261021_181558_055290/bag，78对、约7.7秒。
- scene_02：datasets/scene_02/20261021_182754_515500/bag，99对、约9.8秒。
- scene_03：datasets/scene_03/20261021_182958_009120/bag，99对、约9.8秒。
- 录制时间戳、原始bag和内参Agent目录均未改写。没有启动传感器驱动，求解节点只读离线bag。
- 每个单场景累积前50帧，图像取同bag首帧。静止条件依据用户录制确认、全帧图像和驱动CSV复核。
- 三场景各保存一个新的四孔中心记录，联合文件严格三块、12对。没有使用历史append数据。
- 三场景均用真实1280×1024、plumb_bob、完整k1/k2/p1/p2/k3。内参来源17:29版本，SHA256 `466cb1595c4d296f51e6cbe1011ddc1c5ba329b51f7b28053d0faede7ba8926f`。
- 每场景参数、内参和板provenance、人工/程序特征复核文件、输入审计和运行日志保存在本次目录各scene_XX下。

## 联合候选外参

方向是 **T_cam_lidar（LiDAR → 相机 optical frame）**：

`p_cam = R_cam_lidar * p_lidar + t_cam_lidar`，长度单位米。

LiDAR实际frame_id为livox_frame，原生x前/y左/z上；相机为front_optical_frame，x右/y下/z前。输入没有先应用未知外参或改写坐标。排序临时坐标不参与最终变换定义。

```text
T_cam_lidar =
  [ 0.012996000, -0.999911000,  0.003075000, -0.003503000]
  [ 0.367388000,  0.001915000, -0.930066000, -0.088245000]
  [ 0.929977000,  0.013217000,  0.367380000,  0.000141000]
  [ 0.000000000,  0.000000000,  0.000000000,  1.000000000]

T_lidar_cam =
  [ 0.012996210,  0.367387815,  0.929977006,  0.032334537]
  [-0.999910638,  0.001914791,  0.013216851, -0.003335580]
  [ 0.003074860, -0.930065671,  0.367380060, -0.082114674]
  [ 0.000000000,  0.000000000,  0.000000000,  1.000000000]
```

平移t_cam_lidar = [-0.003503, -0.088245, 0.000141]m，它是LiDAR原点在相机坐标系的位置，不是相机在LiDAR坐标的位置。相反方向须用提供的逆矩阵，不能只把t取负。

- det(R) = 1.000000353852。
- ||RᵀR-I|| = 6.377e-07。
- ||T*T_inverse-I|| = 2.238e-16。
- C++文本输出R/t保留6位小数，因此有约1e-6量级舍入误差；保留原始输出并作矩阵有效性检查，没有静默投影到其他旋转。
- 联合矩阵还用NumPy无权SVD重算核对，最大元素差约4.23e-7，符合文本舍入差异。

标准机器可读结果：[joint/extrinsics.yaml](/home/lab307/calibration/lidar_camera_extrinsics/results/multi_20261021_183657_578407/joint/extrinsics.yaml)，同时含R/t构成的两个方向完整4×4矩阵与逐孔残差。

## 单场景和联合结果

算法使用原始闭式刚体SVD。所谓optimization_converged在此表示求解成功，不存在非线性优化迭代收敛精度的独立证明。

联合12对特征中心RMS = **3.846mm**。

| 场景 | 单场景拟合 RMS mm | 联合逐场景 RMS mm | 联合推算孔中心投影 RMS px | 单场景相对联合旋转差 ° | 平移差 mm |
|---|---:|---:|---:|---:|---:|
| scene_01 | 0.930 | 3.901 | 1.347 | 0.214 | 7.850 |
| scene_02 | 2.802 | 4.890 | 2.100 | 0.363 | 14.363 |
| scene_03 | 0.788 | 2.289 | 1.588 | 0.132 | 5.666 |

这些米制残差比较的是LiDAR提取孔中心与ArUco板位姿推算的相机孔中心。像素列也由这两组中心投影得到，**不是与图像实际孔边界独立标注的误差**。不得把3.85mm称为实际外参绝对精度。

单场景之间最大旋转差约0.466°，最大平移差约19.33mm。平面四点标定中的相机板位姿误差、孔边界采样偏差和几何可观测性会影响这些量；它们说明仍需独立验证，不能把单场景小残差当作充分证据。

## 逐场景留出检查

每次只用另外两个场景的8对孔中心求刚体变换，再评价未参与该次拟合的4对中心：

| 留出场景 | 其余两场景拟合后，留出中心 RMS mm | 推算孔中心投影 RMS px |
|---|---:|---:|
| scene_01 | 5.749 | 1.763 |
| scene_02 | 6.633 | 2.598 |
| scene_03 | 3.316 | 2.298 |

留出中心RMS约3.32–6.63mm，高于拟合残差。它是场景留出检验，仍共享同一板和相机内参，也没有独立孔中心真值，因此不能称作完全独立的绝对精度测量。

## 原始图像上的投影

全部使用原始带畸变图像，五参数OpenCV projectPoints，未用k3=0方案。全点云叠加使用5帧；孔边界叠加使用各场景50帧的板面/边界诊断。

孔边界图中：青色为LiDAR板面边界投影，红色为变换后的LiDAR孔中心，绿色为ArUco推算孔中心。边界从原算法压平平面坐标还原到同一拟合板面，不是额外求解外参。

- scene_01：[孔边界与中心叠加](/home/lab307/calibration/lidar_camera_extrinsics/results/multi_20261021_183657_578407/validation/scene_01/feature_overlay.png)；[全点云叠加](/home/lab307/calibration/lidar_camera_extrinsics/results/multi_20261021_183657_578407/validation/scene_01/overlay.png)。
- scene_02：[孔边界与中心叠加](/home/lab307/calibration/lidar_camera_extrinsics/results/multi_20261021_183657_578407/validation/scene_02/feature_overlay.png)；[全点云叠加](/home/lab307/calibration/lidar_camera_extrinsics/results/multi_20261021_183657_578407/validation/scene_02/overlay.png)。
- scene_03：[孔边界与中心叠加](/home/lab307/calibration/lidar_camera_extrinsics/results/multi_20261021_183657_578407/validation/scene_03/feature_overlay.png)；[全点云叠加](/home/lab307/calibration/lidar_camera_extrinsics/results/multi_20261021_183657_578407/validation/scene_03/overlay.png)。

已查看三个场景：孔附近边界总体对应，板外缘及少数区域仍有可见偏差。没有以肉眼大致重合直接宣告验收；实际孔边界标注误差仍未测量。像素误差图/数值使用的绿色中心来自ArUco推算，保持这一限制。

## 质量状态和不确定性

| 判断 | 当前状态 |
|---|---|
| 输入读取与配对 | 通过 |
| 实物字典/ID/布局与四孔提取 | 已按真实图像/点云复核 |
| 单场景和联合刚体求解 | 成功 |
| 旋转/行列式/双向逆矩阵 | 检查通过 |
| 外参几何质量合格 | **未独立验收，false** |
| 相机内参状态 | PARTIAL，经用户接受仅用于试算 |
| 协方差/绝对精度置信区间 | 上游不提供，uncertainty=null |
| FAST-LIVO2集成 | 未执行 |

累积帧数比较、单场景差异、留出误差都是诊断信息，不冒充协方差。header精确配对不代表独立测得物理触发沿精度。

## 软件问题与复现

首轮运行发现PyYAML复用同一个节点参数dict，输出Humble不支持的YAML锚点/别名，导致rcl参数解析失败。本地生成器已改为每节点独立参数map，并加入无别名回归测试，七项接口测试通过。失败目录multi_20261021_183614_650720保留，里面没有成功外参；有效结果仅为本报告指定目录。

沙箱运行日志中有DDS网卡枚举/UDP权限提示，离线Reader和算法仍正常完成并返回0；本次不依赖这些节点在线发布，不将这些提示当作硬件同步状态。

```bash
cd ~/calibration/lidar_camera_extrinsics
source scripts/env.sh
./scripts/solve.py multi config/solve_manifest.yaml
```

命令将在新的时间戳结果目录重新运行三个独立求解再联合，不复用旧中心记录。当前正式qr_params.yaml已有完整真实内参、板尺寸、实际ROI及显式接受PARTIAL的provenance。

下一步建议保持相机/LiDAR刚性关系不变，另录未参与拟合的不同距离或高度场景，仅用本次候选外参投影验证，不加入本次拟合。先核对机械基线、边界匹配与应用验收标准，再决定是否补内参边缘视角或重采；未获后续授权前不写FAST-LIVO2。

## 后续补充：未参与拟合场景叠加图已生成

使用datasets/validation_01/20261021_185640_899318/bag及既有外参/归档内参生成投影，没有重新拟合。有效单帧图为results/independent_validation_01_20261021_185640/overlay.png；更稠密的静态5帧图为其accumulated_5frames/overlay.png。原bag首帧图像无对应点云，其余98对精确配对；仅选择严格共同header，原bag和原始审计保持不变。详细溯源与限制见该结果目录REPORT.md及validation_provenance.json。此图没有独立像素真值，不自动改变几何验收状态。
