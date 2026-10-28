# 新上下文接手指南

最新2026-10-28阶段B验证：用户明确要求完成后端验证并提交Git。已有运动包的1362对输入已实际接入独立ROS后端，PCL/GICP均49关键帧、35候选、13条接受并优化的回环约束、3项歧义拒绝，严格配对/传播/快照逐点重建通过；构建和16项原验收命令及3项新测试退出0。见[运动后端报告](fast_livo2_stage_b_motion_report.md)，证据`docs/fast_livo2_evidence/stage_b_motion_20261028/run-PYV5azso/`，完整输出在`outputs/fast-livo-backend-validation-20261028-{pcl,gicp}/`。不再套用“该包后端未验证”；物理同步、真值精度、误环泛化、实时wheel与长时仍PARTIAL，C1/C2未实施，本次无硬件启动。

最新追加2026-10-28：用户新增两份有效录包，要求删除第二份180秒并将第一份制成展示视频。保留`bags/fast-livo-motion-w5phdosj/raw`（约136.6秒）；`ez09stbr`完整180秒已移入系统回收站，另一个`58obp75g`无raw失败目录保留。三融合离线重放1362对/10911次视觉更新，视频交付位于`outputs/fast-livo-demo-20261028-fHJTbGCU/`，见[演示报告](fast_livo2_demo_20261028.md)。本轮不启动硬件；现已具备运动采集数据，勿继续套用“没有三传感器运动包”的旧结论。视频是前端重建，不是该包回环或真值精度验收。

交接更新：2026-10-26，项目 `/home/lab307/ranger_nav`。当前已完成独立后端阶段 B 的代码、构建与隔离离线验证，真实设备保持停止。阶段 B 报告见 [交付报告](fast_livo2_stage_b_report.md)，实际接口见 [B 契约](fast_livo2_backend_interfaces.md)。下一阶段 C1/C2 未获自动实施授权。

2026-10-27追加：用户要求完整运动录包脚本、Foxglove远程预览，并授权先用RViz查看。已完成[录包入口](fast_livo2_recording.md)与[本次报告](fast_livo2_recording_report.md)。本次雷达收流，但海康Line0模式连接后没有图像，前端/GUI未启动；已受管停止，owned=[]。待确认外触发线/源与供电，不切自由运行绕过。用户本次未授权底盘、导航或C1/C2；无新的真实运动包。

随后用户明确“已经准备好传感器了”，本次受管重试`run-4e1fptkc`已RUNNING：Line0图像恢复、三融合云位姿输出、RViz与localhost:8765只读Foxglove正常。会话保留运行供查看；使用`run_fast_livo2_live.sh --status/--stop`管理，不重复启动。尚未录包，可用`record_fast_livo2.sh --execute --attach --seconds 60`接入。新证据在`docs/fast_livo2_evidence/preview_20261027/ready-f9fe0j37/`；物理同步/精度仍PARTIAL，传感器2020历元仍未与wheel对齐。

最新：用户确认成功并要求关闭原程序准备录包，已执行`run_fast_livo2_live.sh --stop`，退出0；STOPPED、live=false、owned=[]、所有组件身份均已退出，见上述证据目录`stopped_summary.json`。没有启动录包。下一次录包应使用`record_fast_livo2.sh --execute --start-sensors --seconds 60`，不再使用`--attach`接入已停止的预览。

随后用户在无DISPLAY远程终端录包遇到默认RViz检查失败；已将录包默认改为`--visualization auto`，远程自动选Foxglove、有桌面选RViz，显式rviz无桌面仍拒绝。修复过程仅离线测试，不启动真实设备；用户可原命令重试，或显式加`--visualization foxglove`。

最新：用户已确认Foxglove话题可见/连通，要求删除之前采集数据。本轮4份`bags/fast-livo-motion-{9sbycm9a,tzj_swlp,unfaf7sz,kp3yt18t}`已移入系统回收站，不再位于bags原路径，可恢复；合计约29.4GB，移入回收站不等于释放磁盘。删除前确认会话已停止且无打开文件；未启动新采集。操作清单与原manifest/result在`.runtime/capture_cleanup/20261027_231026_079043/manifest.json`，历史包/地图/标定仍保留。


随后用户明确要求清空系统回收站，已执行`gio trash --empty`退出0，并核实`trash:///`为空；上述4份数据现已永久删除，不能再从回收站恢复。当前磁盘可用约848GiB。

## 建议阅读顺序

1. [AGENTS](../AGENTS.md)：当前约束和逐次真机授权。
2. [STATUS](STATUS.md)、[根 README](../README.md)：当前能力、入口和验收边界。
3. [工程结构](PROJECT_LAYOUT.md)、[标定流程](../calibration/README.md)：整合后的路径。
4. [后端与重定位计划](FAST_LIVO2_BACKEND_PLAN.md)：下一阶段实施顺序与验收。
5. [前端接口](fast_livo2_interfaces.md)、[源码审查](fast_livo2_source_review.md)：后端接入前必须读源码核对。
6. [定位接口契约](localization_backend_contract.md)、[受管会话](managed_sessions.md)：复用已有桥接和安全链。

## 不要重做或误判的事实

- 实物 **Ranger Mini V2**，不是任务早期的 3.0；Ubuntu 22.04、Humble、x86_64。生产默认仍 Lightning，FAST-LIVO2 前端可通过配置切换。
- FAST-LIVO2 已能真实在线三融合并在 RViz 显示，用户反馈“看上去效果不错”后要求停止。不能重新从选型/安装建议开始，也不能把显示效果当作闭环/重定位通过。
- 同步 writer 的 O_TRUNC/flock 顺序与读写一致性已经修复成配套 v2；不要重复要求先修。曝光相位和时间历元问题仍未解决，不硬编码 ±100 ms。
- `~/calibration` 已原子迁入 `ranger_nav/calibration`，旧路径只是链接；当前同步包在 `ws_camera/src/`。新入口去掉兼容链接仍能离线运行；不要重新复制一个标定工程。
- 截至10月26日没有三传感器运动录包；10月28日已新增并保留`w5phdosj`，详见本页最新追加。`calibration/bags/stageB_sync_run2`仍为静态包，不能冒充运动验收。新包已用于前端视频，后端回环及定位精度还需单独验证。
- 本项目B已有关键帧/回环/PGO、非关键帧传播、几何图快照导出、标准适配器隔离联调，且单份真实三传感器运动包的后端闭环管线已验证。没有独立真值和实车泛化验收。C1统一加载/给定初值定位、C2无初值重定位未实施。

## 工作树、源码与入口

- 主仓库分支 `refactor/pluggable-navigation`，交接 HEAD `a76d83a767d5691ca40351f3a57f65b398572a52`；**大量阶段 A、同步修复和整合文件未提交**。开始先 `git status --short`，不可 reset/clean，不因 HEAD 相同就忽略工作树。
- 用户原有 `docs/03_mid360_validation.md` IP 修改和六个 `docs/03_evidence/driver-*` 未跟踪目录保留；原相机驱动有用户修改。不要混入无关提交或删除证据。
- 部署前端：`vendor/fast_livo2_review/ros2_frontend`，humble，`837b7bbc1431cb04cf936528e52c83c835efba8e` 加 `patches/fast_livo2_stage_a.patch`。构建入口 `scripts/build_fast_livo2.sh`，产物 `ws_fast_livo/`；15 仓库（含 B 依赖）固定 SHA 见 `config/fast_livo2/sources.lock.json`。
- 优先参考后端：`vendor/fast_livo2_review/fastlio_backend`，main，`7baa59f7339ba88ce3b646fe35b23f61031c5d3f`。pgo/localizer/interface 已审查；B 以独立包实现必要算法，未启动参考原版节点；许可证、缺陷和其他候选对照见源码审查。第一版不整体引入 OpenFleX/HoloAgent，也不修改前端滤波器。
- runtime 默认 `mapping_backend: lightning`、`localization_backend: lightning`、`odometry_backend: wheel`、`terrain_backend: disabled`。FAST profile 为 `config/profiles/fast_livo2_frontend.yaml`，只切前端，不切导航定位。
- SDK 复用原 `ws_camera/src/hik_camera_ros2_driver/hikSDK`，不随 Git 分发。FAST-Calib 新编译路径为 `calibration/lidar_camera_extrinsics/ws/{build_integrated,install_integrated}`，旧工作空间保留追溯。

安全接手命令，均不打开实物：

```bash
cd /home/lab307/ranger_nav
git status --short
python3 scripts/runtime_config.py
bash scripts/run_mapping.sh online
bash scripts/run_mapping.sh online --runtime-config config/profiles/fast_livo2_frontend.yaml
bash scripts/run_fast_livo2_live.sh --status
bash scripts/run_calibration.sh status
PYTHONDONTWRITEBYTECODE=1 python3 tests/test_project_consolidation.py
```

`--execute` online 会开真实设备，本次交接不运行。离线 ROS 测试先 source Humble / 对应 workspace，按原脚本使用隔离 domain。构建脚本默认 dry-run。

## 后端最关键的接口

输入必须用 `/fast_livo/odometry` 与 `/fast_livo/deskewed_imu`，同一 stamp、同一 VIO 后 IMU 参考点。点云是已去畸变的单图像区间 IMU 机体系云；里程计为 T_fast_livo_local_imu。不能拿 `/cloud_registered` 累计世界云或 VIO 前 `/aft_mapped_to_init` 重映射充数。

后端先得到 T_map_imu(t)，适配器用真实方向的 T_base_imu 转成 T_map_base(t)，发布 `LocalizationEstimate`；现有公共桥接按同一采样时刻插值轮速，唯一发布 map→odom、odom→base_link。FAST 局部系不是 wheel odom；参考节点的 TF 必须关闭。车体安装外参仍近似，不能将相机–雷达外参当作车体标定。

实时传感器曾使用 2020 年 header、主机/存储时间为 2026 年。当前隔离展示未接公共桥接；直接接轮速/Nav2 会产生跨历元问题。后端离线使用 header-clock 调度与 use_sim_time，保留原消息；真机接公共 TF 前必须解决实际时钟一致性，不重打 now 掩盖。

## 已有证据与清理结果

- 前端完整静态基线 829 对，828 帧 VIO，4222 次视觉迭代；DDS 55/55，定向 C++ 13/13。
- 本次在线约 10/200/10 Hz，最终显示 3925 对；没有公共 TF/速度输出，已停止。最新在线雷达 SN/IP 在在线报告中，与 9 月旧雷达不同，不能套用旧 `.145`；当前配置为 `.101`。
- 整合后配置/迁移 10/10、标定 11/11、时间对齐 7/7、进程 11/11、定位契约 11/11；新入口静态短回放 20 对 / 20 VIO / 121 迭代。Nav2 全量 37/37 等结果保留原 10 月 20 日日期。
- 保留 `bags/fast-livo2-baseline-126a5fAo`、`bags/fast-livo2-dds-smoke-final`、全部原始数据与地图。约 541 MiB 重复 DDS/XWD/缓存已清理，不要重新生成多份全量等价回放。
- [整合报告](consolidation_report.md)、`docs/consolidation_evidence/20261026/verification_summary.json`、[在线报告](fast_livo2_online_report.md)、[阶段 A](fast_livo2_stage_a_report.md)。
- 回退资产：`.runtime/backups/consolidation_20261026/`，含迁前 tracked diff、源码检查点、原标定 `.git` 及完整 bundle；另有本轮文档备份 `.runtime/backups/docs_handoff_20261026/`。不要整目录清空 `.runtime/` 或 build。

## 阶段 B 新入口与边界

- 源码 `ws_fast_livo/src/ranger_fast_livo_backend` 已纳入可跟踪路径；前端旧 build/install 保留。后端依赖在 `backend_deps`，构建在 `backend_build`，安装在 `backend_install`，不覆盖前端部署。
- `bash scripts/build_fast_livo2_backend.sh`、`bash scripts/run_fast_livo2_backend.sh` 默认 dry-run。后者只有显式 `--offline-bridge` 才启动公共适配器和 TF 桥接；本轮未启动硬件。
- PCL、small_gicp、small_vgicp 可配置；正常默认 PCL。图基准保留 FAST local 轴，当前未验证重力对齐，不可默认水平。
- 配对队列启动缺对造成批量释放的问题已修复；创新失败写入修正缓存的问题已修复。B 离线桥接首次创新失败锁定，不影响私有 PGO。
- GTSAM SHA 同时有 4.2 / 4.2.0 标签，源码显示字符串仍含 a9；记录事实，不更改上游版本字符串。完整依赖/参考审查和测试命令见 B 报告。

## 下一窗口应做什么

B 已完成本轮交付。后续按 [计划](FAST_LIVO2_BACKEND_PLAN.md) 的 C1→C2→D 顺序：先定义归档加载/完整性/标定兼容和给定初值定位，再做无初值搜索与系统验收。不要把局部 ICP 当全局重定位，不宣称没有数据的成功率或精度。

下一阶段获 C1 明确授权后，复查 B 归档与当前 localizer，先实现新进程加载及失败状态，不重做 B、不把局部 ICP 称为全局重定位。高程地图与坡道不在这个阶段。
