# 第4轮建图、在线位姿与RGB组会视频

日期：2026-11-11。按用户要求生成第4轮组会视频。仅读取原实验资产，没有启动设备、底盘、ROS节点或导航，没有提交/推送。

交付目录：[视频及讲解说明](../outputs/p0-trial04-meeting-video-20261111/README.md)。

| 文件 | 时长 | 大小 |
|---|---:|---:|
| `trial04_meeting_2x_1080p.mp4` | 95.7秒 | 12,543,453字节 |
| `trial04_mapping_rgb_1x_1080p.mp4` | 180.4秒 | 16,064,903字节 |

均为1920×1080、20fps、H.264/MP4，无音轨。片头3秒、主过程、最后地图旋转查看4秒、结束卡4秒。主过程覆盖出发前3秒、行走151.282秒和返回后约15秒；2倍速版仅加速主过程。`preview.jpg`可作PPT封面，`encoded_contact_sheet.jpg`为编码后抽帧检查。

## 来源和显示

源：`experiments/fast_livo2_p0_closure/runs/20261111_171817-liu6_7ya/trial_04_attempt_01/`。地图UUID`1937ef8a785df14b8b6b6f1d184361af`，manifest SHA256`6a9b819bda3079db7665aaae7f6a19afef66f5819187a9f69dd6fe1a03191254`。

- 左侧直接使用bag中256条`/fast_livo/backend/optimized_path`，根据当时位姿逐步重建已出现关键帧。没有重新配准或优化，没有将最终图提前用于过去帧。显示采用8.5cm体素及声明视口，原完整云保留。
- 图快照的录包时间经此前最近的已录`/clock`及经过的主机时间关联至软件时间轴。在线位姿来自原观察记录，按实际接收steady时间呈现，保留采样年龄与等待状态。
- RGB来自原`/front/image_sync`，在演示关联时使用正式+100ms一次，与显示位姿原采样戳匹配；有效软件戳差最大0.00033ms，不等于曝光/扫描物理同步。原bag/header不改。
- 右下为原FAST前端与B在线输出的base_link轨迹，沿用原工作外参。最后记录图快照与归档110个关键帧逐矩阵最大差0，结束56次接受回环事件。

## 命令与验收

新增[render_trial_video.py](../experiments/fast_livo2_p0_closure/render_trial_video.py)，复用已有render_demo解码和P0分析数学。必须指定新输出目录。

```bash
source /opt/ros/humble/setup.bash
OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 /usr/bin/python3 \
  experiments/fast_livo2_p0_closure/render_trial_video.py \
  --trial experiments/fast_livo2_p0_closure/runs/20261111_171817-liu6_7ya/trial_04_attempt_01 \
  --output outputs/p0-trial04-meeting-video-20261111
```

退出0、耗时187.285秒。独立预览目录`outputs/p0-trial04-video-preview-20261111`的起点/中途/终点预览退出0，脚本py_compile退出0。H.264编码器探测通过，无系统安装或升级。

PASS：COMMITTED/manifest及110份完整关键帧PCD SHA；最后记录快照与归档一致；两版3608/1914帧完整逐帧解码，尺寸正确；编码后1/30/65/94秒定位抽帧与人工检查通过；主要来源文件大小/mtime未变，数据库只读打开。视频SHA、参数、来源、命令、日志在输出目录`verification.json`、`source_file_identity.json`、`command.txt`、`render.log`。

PARTIAL边界：实录数据离线重绘，非当时屏幕录制；无独立真值，不宣传毫米级绝对精度；软件戳关联不证明物理同步；范围为FAST+B建图，不是C1/C2重定位或自主导航验收。

最少人工待办：组会直接播放2倍速MP4，按需使用README讲解及preview封面，无需重新采集。
