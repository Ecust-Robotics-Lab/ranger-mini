# 候选源码审查与本地适配

审查日期：2026-10-21。终端 GitHub 代理连接失败、直连 DNS 失败；利用可用网页工具取得 raw 文本，检查连续行完整性后保存。`upstream/` 是关键源码快照，不是完整 clone；三仓库远端 HEAD 未用 Git 验证。所有取得文件 SHA256 存于 `upstream/SOURCE_MANIFEST.json`。不得将本地 Git commit 冒充远端 commit。

| 候选 | 已核对版本 | 输入/依赖 | 选择判断 |
|---|---|---|---|
| [HKU-MARS](https://github.com/hku-mars/FAST-Calib) | main 文本快照 | catkin/roscpp、rosbag ROS1、PCL、OpenCV；CustomMsg + PointCloud2、独立图像 | 官方算法基准，含单/多场景；不直接在 Humble 构建 |
| [ichangjian](https://github.com/ichangjian/FAST-Calib-ROS2/tree/dev_humble) | **dev_humble**，不是 main | ament/rclcpp、rosbag2_cpp、PointCloud2→PCL XYZ、独立 imread 图像；仅 single target | 可用 ROS2 起点，自带裁剪 Mid360/11 回归 bag；本轮该二进制不可下载。main 仍为 ROS1，不能误取分支 |
| [Zhi-Calf](https://github.com/Zhi-Calf/calibration/tree/master/FAST-Calib) | master/FAST-Calib 文本快照 | ament、rclcpp、sensor_msgs、geometry_msgs、rosbag2_cpp/storage、cv_bridge、pcl_conversions、PCL、OpenCV aruco | 选用，已有 bag 内图像、无 ring 固态分支和多场景 |

Zhi-Calf [FAST-Calib 历史](https://github.com/Zhi-Calf/calibration/commits/master/FAST-Calib) 最后显示影响此子目录的 commit 为 `84d92f846a61d188bae2511dcba6d346f7b72485`（2026-03-24）。固定 SHA raw URL 当前无法访问，故仅记录观察到的版本线索，不声称快照已与该 Git tree 校验。Zhi-Calf 的 color.h 未获取成功，用本地空颜色宏兼容头代替，纯装饰性；LICENSE 取官方 GPLv2 文本。报告保留这一来源差异。

## 已实际核查的接口

- [data_preprocess.hpp](https://github.com/Zhi-Calf/calibration/blob/master/FAST-Calib/src/data_preprocess.hpp)：rosbag2 Reader 通过存储插件处理 db3/mcap，真正能读何种格式取决于本机插件。当前注册 sqlite3、两个测试插件，无 mcap，因此 MCAP 未验证且当前不可用。
- PointCloud2 以字段名和 point_step 读取 x/y/z，ring 存在时进入 mechanical，否则 solid；Livox 的 `line` 不被误认为 ring。实际 Stage B 字段为 x,y,z,intensity,tag,line,timestamp、point_step=26，小端连续布局；本地 C++ 已读取。没有将 Mid-360S 当成普通 Mid360 的扫描线模型。
- Image 使用 `cv_bridge::toCvCopy(...,"bgr8")`，实际 rgb8 已验证，bgr8 也有转换测试。补充 topic 消息类型校验，避免同名压缩图被错误反序列化为原始图。
- 上游默认 `use_point_pick:true` 会等待 RViz2 /clicked_point。当前生成配置设 false，直接 3D bounds，launch 无 RViz 节点，不依赖点选。
- 图像几何以四个 DICT_6X6_250 标记估计板位姿，再推算孔中心；LiDAR solid 使用平面压平、法线边界、去直线边缘、RANSAC 圆拟合和矩形组合筛选。

## 本地更改

1. 保留 k3：Params、参数读取、QRDetect、原图投影/去畸变、结果输出均保留五参数。原始源码第五项固定 0 已修正；不采用 k3=0 兼容性文件。
2. OpenCV 4.5 使用 drawFrameAxes；初始化姿态平均累加量；Rodrigues 输出显式转 float 再写 float 板变换，避免矩阵类型不一致；要求精确四 ID、正深度和 ArUco 角点 RMS ≤2px。这个阈值是粗略特征一致性保护，不是最终几何验收标准。
3. Livox 孔排序用 (-y,-z,x) 临时坐标，修复 Zhi-Calf 原先 (x,-z,y) 对 x前/y左/z上坐标不适用的问题。SVD 使用原 LiDAR 坐标。C++ 合成几何对应测试覆盖此项；实物仍需对应复核。
4. 输入累积帧数上限、XYZ 类型/大小/端序校验、非有限和原点过滤；原始 bag 未改动。Python 预览支持 padded row/big endian，C++ 对这类未验证布局显式拒绝；当前实机 layout 不受影响。
5. 平面/边界不足、圆候选不足保护；组合函数不再在 N<4 构造非法 bitmask；组合数避免原始阶乘溢出。法向旋转改用单位四元数 FromTwoVectors，避免旋转轴未归一化和平行法向零轴问题。
6. 四孔失败禁止继续 SVD；缺失输入非零退出；单场景默认保存后退出，无无限发布循环。默认未验证配置禁止使用示例值求解。
7. 联合 SVD 改为读当前文件内全部场景，至少三场景；拒绝秩不足中心对应。外层脚本每次创建新目录并绑定清单，防止 append 历史中心污染。
8. 新增不建立 DDS participant 的 input_probe、offline_checks 和 LiDAR --offline 诊断，均不访问硬件；诊断保存 PCD/JSON/日志。求解节点仍为 ROS2 节点。

完整文件级差异见 `docs/local_changes.patch`。未修改依赖的系统驱动，未进行算法扫描模型替换。

## 样例和 CAD 限制

官方 [单场景](https://drive.google.com/drive/folders/1W87Dx3MUuPhTpCLvaavWqNUJZV03yU6L)、[多场景](https://drive.google.com/drive/folders/1g__plgFqp5tsk-TY7Ioh4RXru62AdLmr) 与 [CAD](https://drive.google.com/file/d/1hdC8xGCHNP47a-wSLPyjr_tpOeynNFEG/view) 在网页工具中不可访问；终端网络也失败。ichangjian 仓库的 mid360_11 db3/png 下载同样失败。没有替代为伪造样例；现有真实 Stage B bag 仅作输入/诊断负例，合成几何仅作接口数值测试。正式样例回归、实物四孔提取与真实外参验证仍待实际数据。
