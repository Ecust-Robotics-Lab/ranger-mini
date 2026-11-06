# FAST-LIVO2 在线前端与 RViz 验证

> 后续整合说明：本报告保留当次证据；相机/标定现已迁入工程，新入口与路径见 [整合报告](consolidation_report.md)、[标定 README](../calibration/README.md)。

2026-10-26，阶段 A 的在线追加。用户明确要求启动已就位的雷达、相机及 RViz，随后反馈“看上去效果不错”，最后明确要求停止并准备讨论下一步。本报告不构成阶段 B 或底盘控制授权。

**PASS：本次真实传感器收流、三传感器前端输出、RViz 显示和受管停止。整体仍 PARTIAL：物理同步、运动精度、回环、优化地图保存与先验定位未验收。**

## 实际运行及结果

- 设备确认：Mid-360S `ARMCP4E0033801`，192.168.1.101；海康 USB3 相机 `DB1216907`，Line0 外触发、1280×1024 RGB8、曝光 20000 us。主机 enp2s0 原有 192.168.1.50/24，无网络设置修改。
- 使用本机真实五参数内参、联合外参试验配置；标定 PARTIAL 与所有零时间偏移的试验属性保留。
- 复用现有驱动，先雷达、再时间戳 writer、再外触发相机；收到三类输入后启动 FAST-LIVO2，成对输出就绪后启动 RViz。ROS domain 175、localhost；未启动底盘驱动、Nav2 或公共定位桥接。
- RViz 已实际看到去畸变当前云、累计点云、位姿/轨迹与 VIO 使用的带特征图像，用户现场确认画面。截图：[rviz_live.png](fast_livo2_evidence/online_20261026/rviz_live.png)。OpenGL 4.6 正常，无渲染失败。
- 停止前监测到 **3925 对**同刻前端云/位姿，累计显示云 **61807 点**。可视化按每 5 帧、0.08 m 体素累计，最多 300000 点、路径最多 5000 姿态；这是显示层累计，不是经过回环优化的地图。
- 独立 5 s 观测：点云 50 条，约 9.9992 Hz；IMU 1000 条，约 199.9684 Hz；同步图像 49 条，约 9.9999 Hz；前端位姿 50 条，约 10.0002 Hz。各话题本次观测无回退/重复戳，各有唯一发布者。图像与点云有限观测窗口交集 48 个相等 header，不据此计算完整采集丢帧率。
- 独立观测的 `/tf`、`/cmd_vel`、`/base/safe_cmd_vel` 发布者为空。本轮不发布 map→odom 或 map→base_link；RViz 用 `fast_livo_local` 显示，可能因无 TF 数据出现 Global Status Warn，不影响已经确认的同系点云/图像显示。
- 约运行一分钟时进程观测：前端约 144% CPU、RSS 430548 KiB；显示监测约 37.6% CPU、RSS 97728 KiB；RViz 约 9.6% CPU、RSS 211076 KiB。仅是一次在线样本，不是峰值或长期内存验收。

## 启动前的同步保护修复

先备份 `~/calibration/ws_calib/src/` 下两个实际文件，再作最小修改并重建现有包；未替换传感器驱动或采用外机标定：

1. writer 删除 `open()` 的 O_TRUNC 标志，取得生命周期排他 flock 后才调整文件大小，避免第二个 writer 的失败启动破坏活动映射。
2. writer 先写奇数“写入中”序号，再写时间值，最后写偶数提交序号；相机 reader 要求两次序号相同、非零且为偶数。两端必须配套使用 v2。
3. 此会话使用独立 `.runtime/.../sensors-mng4paqh/timeshare`，没有截断历史 `~/timeshare`。

补丁：[camera_timeshare_v2.patch](../patches/camera_timeshare_v2.patch)。原文件、前后 SHA256 与路径保存在 `docs/fast_livo2_evidence/online_20261026/sync_source_before/`、`sync_patch_manifest.json`。

离线启动真实 writer 并发布合成点云 header：第二 writer 非零退出且原文件不变，**5859 次已接受快照**均符合提交序号/时间对应关系，最后序号 202，通过。该测试没有打开设备。日志 `sync_offline_test.log`。camera/writer 两包分别 `cmake --build ... -j2`，均退出 0；安装处原有 symlink 指向新构建产物。

修复只解决文件所有权和一致性，不证明曝光与扫描的物理对应关系。

## 必须带入下一步讨论的问题

1. **传感器旧历元。** 实际 driver 日志 time_type=2，传感器 header 仍约为 2020 年，主机 ROS 时间约为 2026 年。当前局部可视化不依赖公共 TF 新鲜度；后续接公共定位桥接/Nav2 前，必须查清同步源与统一时间策略，不能直接把这组旧戳当当前全局定位采样时间。
2. **旧戳图像被主动拒绝。** 全程 camera CSV 3997 行，OK 3935、BACKWARD 62（约 1.55%），无硬件 FrameID 跳号；无 NO_MAP/NO_WRITER/UNSTABLE/STALE。所有 OK 序号均为偶数。writer 末段日志累计 59 次约 200 ms 扫描/到达间隙。这些现象提示要优先调查订阅丢帧、调度与触发/组帧相位，不能只凭 RViz 好看就认定同步通过，也不直接硬编码 ±100 ms。
3. **需要可重放的运动数据。** 本轮没有启动 rosbag 全量录制；此前也没有三传感器运动包。应先明确数据与成功判据，再由用户控制实际采集过程，保存原三话题、同步 CSV、配置/时钟来源，建立运动前端基线。
4. 上述基础确认后再讨论 B 的关键帧、回环验证、GTSAM PGO 与优化地图重建；C 的给定初值定位、无初值自动定位分别验收。本轮停止后没有自动继续这些实现。

## 受管入口与停止结果

```bash
cd /home/lab307/ranger_nav
bash scripts/run_fast_livo2_live.sh           # 只显示计划
bash scripts/run_fast_livo2_live.sh --execute # 传感器 + 前端 + RViz
bash scripts/run_fast_livo2_live.sh --status
bash scripts/run_fast_livo2_live.sh --stop
```

入口复用已有 session_manager，含设备占用检查、资源租约、依赖就绪、身份限定清理，任一关键节点退出或健康持续丢失则失败停止，不自动重启。GUI 关闭本身不等于停止全部传感器，应使用 --stop。

本次运行目录 `.runtime/sessions/fast_livo_live/run-b5ekdtik`，传感器 CSV 位于同目录树 `sensors-mng4paqh/`。停止命令退出 0；最终 **STOPPED**、`guardian_cleanup_complete=true`、`owned=[]`，复核无存活组件 PID；SDK Deinit、相机采集线程退出日志均已记录。未按进程名全局清理。

可归档证据根目录 `docs/fast_livo2_evidence/online_20261026/`：

- `live_validation_final.json`：独立频率/发布者/时间观测；首次临时观测回调签名错误保留为空 `live_validation.json`，没有算通过。
- `online_snapshot.json`、`stopped_summary.json`、`session_state_stopped.json`：运行与停止状态。
- `lidar_final.log`、`writer_final.log`、`camera_final.log`、`rviz_final.log`，前端完整日志仍在受管运行目录。
- `rviz_live.png` 与原始 XWD（原始捕获留磁盘、Git 忽略）。

新增入口、显示/健康监测、离线同步保护测试、独立观测工具位于 `scripts/fast_livo2/`；配置为 `camera_live.yaml`、`mid360s_live.json`、`live.rviz`。标准 LocalizationEstimate、生产 runtime、现有地图和导航安全链没有修改。

回滚同步保护时，应先确保所有相机/writer 进程已停止，再用本次备份按前后 SHA256 逐文件审阅恢复并重建两个包；不能混用旧 writer 与新 reader，也不能在进程运行时覆盖共享库。此回滚说明不是本轮已执行的回滚动作，修复当前保留。
