# 当前项目状态

更新：2026-10-28。FAST-LIVO2阶段B已完成代码、构建/故障回归及已有真实运动包的后端离线验证，本轮按用户指令提交相关源码与证据；PCL与GICP各确认1362对、49关键帧、13条接受并优化的回环约束。物理同步/真值精度/实车实时性仍PARTIAL。本次不启动硬件或进入C1/C2。

## 当前结论

| 项目 | 状态与边界 |
| --- | --- |
| 硬件/环境 | Ranger Mini V2、Mid-360S、海康相机；Ubuntu 22.04 / ROS 2 Humble / x86_64 |
| 生产默认 | Lightning 建图/定位，wheel 连续里程计，terrain disabled；Nav2 与安全链保持 |
| FAST-LIVO2 前端 | 已构建、静态三融合回放、真实在线与 RViz 运行；用户确认画面后要求停止 |
| FAST 后端能力 | 真实运动包PCL/GICP均1362/1362、49关键帧、13条回环约束；快照/传播/无公共输出通过；泛化与真值精度待验，C1/C2未实施 |
| 标定/时间 | 本机五参数内参、试验外参均保留；几何验收、曝光/扫描物理对应、旧历元问题仍 PARTIAL |
| 运动数据 | 已保留w5phdosj约136.6秒三传感器运动包；前端1362对/VIO参与，后端闭环离线管线已验证；无独立真值精度结论 |
| 运动采集入口 | 默认dry-run，三传感器完整录制/日志归档/录后审查；可自动启动或接入已有预览，见[录包指南](fast_livo2_recording.md) |
| 当前预览 | 用户确认显示成功后要求关闭；run-4e1fptkc已STOPPED、owned=[]、清理完成，设备可供新录包会话使用 |
| 系统验收 | 静态链路和离线接口通过，不代表真实运动精度、完整 Nav2、安全停车距离或坡道能力通过 |
| Git | refactor/pluggable-navigation；交接快照 HEAD a76d83a767d5691ca40351f3a57f65b398572a52；阶段 A/整合修改大量未提交，保留用户已有修改 |

当前路径与入口以 [工程结构](PROJECT_LAYOUT.md)、[根 README](../README.md)、[标定流程](../calibration/README.md) 为准。`calibration/` 已迁入，`~/calibration` 仅兼容链接；当前相机/同步包在 `ws_camera/src/`。

## 最近完成

- [阶段B真实运动后端报告](fast_livo2_stage_b_motion_report.md)：默认门槛下PCL/GICP各1362对、35候选、13条回环约束、3项歧义拒绝；151项快照SHA与49份原扫描核查、1313项传播验证通过。项目内构建与16项既有验收命令、3项新增验证器测试均退出0。首尾距离PCL约45.7→44.5cm，变化小且非绝对精度；已生成图表和24秒优化前后对比视频。

- 2026-10-28：[真实采集数据演示视频](fast_livo2_demo_20261028.md)，1080p完整142.15秒、720p两倍速分享版约71秒；实际三融合重放1362对、10911次视觉更新。原第一份包保留，第二份ez09stbr完整180秒按用户要求移入回收站；另一个无raw的失败启动目录保留。未启动硬件。

随后用户明确要求清空系统回收站，已执行`gio trash --empty`退出0，并核实`trash:///`为空；上述4份数据现已永久删除，不能再从回收站恢复。当前磁盘可用约848GiB。

- 用户确认Foxglove已连通后要求删除之前采集数据：本轮4份`fast-livo-motion-*`（9sbycm9a、tzj_swlp、unfaf7sz、kp3yt18t，合计29,436,799,874字节）已在确认会话停止、无文件占用后移入系统回收站，可恢复；未删除历史包/标定/地图。操作记录`.runtime/capture_cleanup/20261027_231026_079043/manifest.json`，未自动重新采集。

- 修复远程录包默认RViz导致无DISPLAY退出：录包默认auto，有桌面用RViz、普通SSH用只读Foxglove；不重启设备。操作见[录包指南](fast_livo2_recording.md)。

- 2026-10-27传感器就绪后重试预览：5秒观测点云/图像/前端约10Hz、IMU约200Hz、云位姿50对严格同戳，各输入唯一发布者且无回退；公共TF/速度发布者为空。设备继续运行供用户查看，未录包；实际传感器仍是2020年历元，不能直接接wheel公共桥接。见[采集报告追加](fast_livo2_recording_report.md)。

- 2026-10-27：[完整采集脚本与远程可视化](fast_livo2_recording_report.md)，录制/故障8/8、实际Foxglove WebSocket收云1/1、整合回归10/10。真实预览未就绪，不宣称完成真实运动采集。

- 阶段 B 独立 C++ 包、GTSAM/small_gicp 项目内依赖、PCL/GICP/VGICP 接口、不可变云分片/一致性导出及受管私有入口落地；公共桥接拒绝缓存缺陷已修复。见 [B 报告](fast_livo2_stage_b_report.md) 和 [B 接口](fast_livo2_backend_interfaces.md)。

- 清理两份经 SHA 核对的冗余 DDS 回放、XWD 与缓存，约 541 MiB；原始数据、正式基线、final 派生包、PNG、地图与失败证据保留。
- 标定资产原子迁移，1818 条迁移身份检查、1158 条原始资产保护检查通过。相机/同步包、FAST-Calib 与 ws_nav 构建通过；去掉旧路径链接的八项离线检查通过。
- 新增统一建图和标定入口，runtime/profile 可选择 Lightning 或 FAST-LIVO2 前端。全局定位仍独立选择 lightning/standard，解析器拒绝把局部前端冒充全局定位。
- 更新 AGENTS、README、交接与文档索引；删除重复初始化记录和已完成批次的旧总计划，下一步计划集中于 [后端与重定位计划](FAST_LIVO2_BACKEND_PLAN.md)。见 [本轮文档报告](documentation_refresh_report.md)。

## 验证事实

| 验证 | 已有结果 | 日期与证据 |
| --- | --- | --- |
| 阶段 B 离线 | C++ 21/21、ROS 6/6、适配器 3/3、桥接拒绝回归 2/2；公共桥接 14/14、契约 11/11、时间 7/7、门禁 12/12；三传感器→前端→后端 55/55 | 2026-10-26，[B 报告](fast_livo2_stage_b_report.md) |
| 完整静态三融合基线 | 829 对、828 帧 VIO、4222 次视觉迭代；DDS 55/55 对 | 2026-10-26，[阶段 A](fast_livo2_stage_a_report.md) |
| 真实在线前端/RViz | 约 10 Hz 点云、200 Hz IMU、10 Hz 图像/位姿；已受管停止 | 2026-10-26，[在线报告](fast_livo2_online_report.md) |
| 整合后验证 | 配置/迁移 10/10、标定接口 11/11、时间对齐 7/7、进程 11/11、定位契约 11/11、前端 C++ 13/13 | 2026-10-26，[整合报告](consolidation_report.md) |
| 整合后短回放 | 20 对、20 帧 VIO、121 次视觉迭代；真实 writer 合成测试 5736 快照通过 | 同上；不属于新增运动验收 |
| Nav2 完整离线回归 | 桥接 14/14、门禁单元 12/12、安全链 37/37、合成受管入口测试 | 2026-10-20，[批次 3](architecture_batch3_report.md)；本轮未全量复跑 |
| 底盘停车 | 2026-09-28 用户确认该次停车安全通过，保留此前异常反馈记录 | [底盘报告](02_ros_motion_test.md)；不代表本次新增实车验证或运动许可 |

## 下一步与主要缺口

B 已完成本轮可执行范围并停止。下一步为地图包加载与给定初值定位 C1，之后才是 C2 与 D；各阶段需明确实施指令，本轮不启动后续开发或实物。

B的配对、关键帧/图优化、拒绝条件、保存原子性及隔离桥接已实现并测试，单份真实运动包的独立后端闭环管线验证通过。剩余为更多环境/错误地点的误环泛化与独立真值、长时资源测试；获新授权后可实施C1地图加载契约。精确T_base_imu、统一实时历元、重复结构/丢失恢复、重启定位、真机导航与坡道仍待验。

## 历史报告索引

环境/网络：[00](00_environment_audit.md)、[连接处理](LIDAR_CONNECTION.md)；底盘：[02](02_base_interface.md)、[运动与退出记录](02_ros_motion_test.md)；雷达：[03](03_mid360_validation.md)；Lightning：[04 建图](04_mapping_report.md)、[05 定位](05_localization_interface.md)、[06 导航](06_flat_navigation_acceptance.md)。

架构已完成批次：[基线](architecture_baseline.md)、[接口解耦](architecture_batch2_report.md)、[时间/会话](architecture_batch3_report.md)。这些是各自日期快照，不能从旧文档的“尚未安装/尚未授权/下一步”恢复已过期任务。完整分类见 [文档索引](README.md)，当前交接见 [HANDOFF](HANDOFF.md)。
