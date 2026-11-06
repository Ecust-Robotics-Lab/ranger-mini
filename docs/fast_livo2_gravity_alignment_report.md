# FAST-LIVO2 初始重力对齐验证

日期：2026-10-28。用户要求核对并开启重力对齐，使地图 Z 轴与竖直方向一致。本次仅修改前端试验配置与离线诊断，不启动设备。

## 结论与修改

此前 `config/fast_livo2/ranger_mid360s_hik_trial.yaml` 的 `uav.gravity_align_en=false`，因此既有 B/C1 地图不保证水平。现已开启为 `true`，新前端会话在 IMU 初始化后调用已有 `LIVMapper::gravityAlignment()`：将估计重力旋转到世界 −Z，同时旋转位置、姿态、速度和重力。世界 +Z 因而朝上，yaw 仍无绝对航向基准。保留完整三融合、重力估计和六自由度运动，不锁死 roll/pitch。

源码版本仍为 ROS 2 前端 `837b7bbc1431cb04cf936528e52c83c835efba8e` 加既有 `patches/fast_livo2_stage_a.patch`，无需修改算法补丁。离线 harness `scripts/fast_livo2/replay.cpp` 增加启用/完成标志、对齐次数、初始对齐和终态重力诊断。

## 实际验证

输入为原始 `bags/fast-livo-motion-w5phdosj/raw`，原始消息与时间戳不改写。为保留源码目录中既有 `Log/`，在 `outputs/fast-livo-gravity-20261028/source` 建立源码链接投影，并为该投影创建独立 Log 目录；仅回放 harness 指向本次源码。依赖仍复用项目内 Sophus/vikit 和现有系统库，无系统安装。

构建命令（退出码均 0）：

```bash
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
export CMAKE_PREFIX_PATH="$PWD/ws_fast_livo/deps_install:${CMAKE_PREFIX_PATH:-}"
cmake -S outputs/fast-livo-gravity-20261028/source \
  -B outputs/fast-livo-gravity-20261028/build \
  -DBUILD_TESTING=OFF -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON
cmake --build outputs/fast-livo-gravity-20261028/build --target ranger_livo_replay -j2
```

回放命令（退出码 0）：

```bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=199 OMP_NUM_THREADS=4
export LD_LIBRARY_PATH="$PWD/ws_fast_livo/deps_install/lib:${LD_LIBRARY_PATH:-}"
outputs/fast-livo-gravity-20261028/build/ranger_livo_replay --ros-args \
  --params-file config/fast_livo2/ranger_mid360s_hik_trial.yaml \
  -p offline.bag:="$PWD/bags/fast-livo-motion-w5phdosj/raw" \
  -p offline.output:="$PWD/outputs/fast-livo-gravity-20261028/aligned" --log-level warn
```

另运行相同程序与输入，增加 `-p uav.gravity_align_en:=false -p offline.max_frames:=50`，输出改为 `disabled_control`（退出码 0）。两次回放之间将本次投影 Log 移至 `aligned_debug_log` 保存，没有改动原源码 Log。

| 检查 | 结果 |
| --- | --- |
| 开启完整运动回放 | PASS：1362 对、7943 次视觉求解更新；对齐完成且仅执行一次 |
| 初始化对齐重力 | PASS：`[1.33e-15, 2.78e-17, -9.81]` m/s² |
| 终态估计重力 | `[-0.002191, 0.010799, -9.810271]` m/s²，与 −Z 夹角 0.06436° |
| 关闭参数对照 | PASS：50 对、不执行对齐；重力偏轴约 22.49° |
| 原有日志完整性 | PASS：回放前后全部既有 Log 文件 SHA256 相同 |
| 外部竖直/绝对精度 | PARTIAL：无独立竖直或姿态真值；上述夹角是估计内部一致性 |

小型证据在 `docs/fast_livo2_evidence/gravity_alignment_20261028/`，包含构建日志、配置快照、两次 summary、原日志 SHA 和 verification。完整回放日志、轨迹与新 native_frames 保存在 `outputs/fast-livo-gravity-20261028/`。

## 使用边界

本次配置对下一次新启动的 FAST 前端生效，不自动重启硬件。原始包、旧地图和旧 native_frames 原样保留；开启配置不会追溯旋转旧地图，也不能将新输出直接套入旧前端坐标约定。若要得到新的重力对齐 B 地图，应使用本次对齐后的 native_frames 另建新归档并绑定来源，而不是改写原档。地图的水平基准依赖初始化 IMU 重力估计，实际外部精度与动态初始化条件仍需专门验收。
