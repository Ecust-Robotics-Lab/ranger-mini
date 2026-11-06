# 2026-10-28 真实采集数据建图演示视频

用户要求删除第二份180秒无用数据，将第一份可用数据可视化成视频供展示。本次只做数据清理和离线处理，不启动实物、底盘、导航、公共TF或C1/C2。

## 选包与删除

- 保留第一份可用包：`bags/fast-livo-motion-w5phdosj/raw`，采集开始于主机时间2026-10-27 23:13:19，实际约136.6秒。1367条雷达、1367条同步图像、27343条IMU和1367条在线位姿。末尾录制日志有输入发布者变化错误；视频使用此前保留下来的数据，不将停止原因隐藏为完整180秒成功。
- 删除第二份：`bags/fast-livo-motion-ez09stbr`，主机时间23:16:33开始，完整180秒，按用户要求移入系统回收站。删除前确认会话停止且无进程打开目录内文件；原路径已不存在。清单及原manifest/result保存在`.runtime/capture_cleanup/20261028_122508_441848/manifest.json`。
- 另有`fast-livo-motion-58obp75g`失败启动目录，不含raw数据库，不是用户所指两份有效录包，本次保留。

## 视频交付

本地输出目录：`outputs/fast-livo-demo-20261028-fHJTbGCU/`，已按生成物加入Git忽略，源脚本仍可跟踪。

- [1080p完整演示](../outputs/fast-livo-demo-20261028-fHJTbGCU/FAST-LIVO2_Demo_1080p.mp4)：1920×1080、H.264/MP4、20fps、142.15秒，含约136.1秒真实时间回放及6秒最终地图虚拟视角展示。
- [720p两倍速分享版](../outputs/fast-livo-demo-20261028-fHJTbGCU/FAST-LIVO2_Demo_share_720p_2x.mp4)：1280×720、H.264/MP4，约71秒，画面标记2x播放。
- [封面](../outputs/fast-livo-demo-20261028-fHJTbGCU/FAST-LIVO2_Demo_1080p.jpg)、[开头/结尾对照](../outputs/fast-livo-demo-20261028-fHJTbGCU/contact_sheet.jpg)。

画面左侧为彩色三维局部地图逐步重建及IMU轨迹，右侧为原录包相机画面和局部XY轨迹。相机保持原纵横比。字幕明确“真实采集数据·离线回放”“前端重建展示·未应用回环优化”。无合成环境或补画的观测结果。

## 数据与坐标处理

使用采集时归档的`bags/fast-livo-motion-w5phdosj/sources/8_ranger_mid360s_hik_trial.yaml`，重放实际LiDAR–IMU–Camera前端，得到1362对严格同整数纳秒的IMU系去畸变云和局部位姿、1362帧VIO、10911次视觉求解更新；重放退出0，耗时23.2616秒。

绘图只进行一次`p_local = R_local_imu * p_imu + t_local_imu`；不重复应用LiDAR–IMU外参。0.065m体素仅用于显示。三维视图按轨迹范围外扩XY各5m、Z各2.5m聚焦；远距离回波仍在完整native_frames中，不修改原始数据。高度着色和局部坐标视角不证明实际重力已标定。

相机按原整数header选择最近录制图像，强制相差不超过150ms；本次实际最大差为0.000238ms，仅为数值写戳对应证据，不能当作物理曝光/扫描同步精度。视频中显示的16.7m路程是重放前端轨迹累计长度，非独立真值。

首版全范围视图被少量远距离回波拉远，已保存于`draft_full_extent/`而未交付；最终版改用明确标记的局部视图，保留了完整重放数据。原始包以只读SQLite访问，硬件始终未启动。

## 可复现命令与证据

在工程根目录执行；所有输出目录必须新建，禁止覆盖已有结果。

```bash
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=191 OMP_NUM_THREADS=4
export LD_LIBRARY_PATH="$PWD/ws_fast_livo/deps_install/lib:${LD_LIBRARY_PATH:-}"

# 实际重放命令（output必须为新的路径；已生成的交付目录不可再次运行覆盖）
ws_fast_livo/install/fast_livo/lib/fast_livo/ranger_livo_replay \
  --ros-args --params-file bags/fast-livo-motion-w5phdosj/sources/8_ranger_mid360s_hik_trial.yaml \
  -p "offline.bag:=$PWD/bags/fast-livo-motion-w5phdosj/raw" \
  -p "offline.output:=$PWD/outputs/fast-livo-demo-20261028-fHJTbGCU/frontend" --log-level warn

export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 PYTHONDONTWRITEBYTECODE=1
python3 scripts/fast_livo2/render_demo.py \
  --native outputs/fast-livo-demo-20261028-fHJTbGCU/frontend/native_frames \
  --raw bags/fast-livo-motion-w5phdosj/raw \
  --output outputs/fast-livo-demo-20261028-fHJTbGCU/FAST-LIVO2_Demo_1080p.mp4
python3 scripts/fast_livo2/make_share_video.py \
  outputs/fast-livo-demo-20261028-fHJTbGCU/FAST-LIVO2_Demo_1080p.mp4 \
  outputs/fast-livo-demo-20261028-fHJTbGCU/FAST-LIVO2_Demo_share_720p_2x.mp4
```

PASS：前端退出0、严格配对1362/1362；主视频2843/2843帧、分享版1422/1422帧全段解码，两次编码命令均退出0；人工查看开头/中段/结尾画面；非零旋转/平移的点变换及逆变换数值检查、非法四元数拒绝。完整版49,838,516字节（142.15秒），分享版12,925,657字节（71.1秒）。编码使用本机OpenCV4.5.4已有H.264编码器，没有安装或升级全局软件。

详细原始证据均在视频输出目录：`replay.log`、`replay_exit.txt`、`replay_resources.txt`、`frontend/summary.json`、`render.log`、`share_encode.log`、两个视频同名JSON（分辨率/帧数/完整解码/时长/SHA256）、`provenance.json`（来源、配置及脚本SHA）、`delivery_checks.json`（交付核查）。

PARTIAL：此交付是运动数据的前端建图展示，尚未做该包的后端回环验收、独立真值精度评估、物理同步或车体外参验收，不将画面效果称为这些项目通过。C1/C2及实车导航未实施。
