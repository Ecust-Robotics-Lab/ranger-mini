# Ranger 导航工程约束

新上下文先读 [HANDOFF](docs/HANDOFF.md) 与 [STATUS](docs/STATUS.md)，涉及设计时再读 [总计划](docs/FAST_LIVO2_BACKEND_PLAN.md)。当前运行状态按实际 Git、配置、二进制、boot/PID 与会话归属确认。带日期的独立报告记录历史证据。

## 范围与授权

- 项目 `/home/lab307/ranger_nav`。可在已有软件授权内实施、构建及离线验证 [EXACT-MPPI 计划](docs/EXACT_MPPI_NAVIGATION_PLAN.md)。使用 SE(2)/EXACT-MPPI 及独立安全链，防止混入旧规划控制器。
- 启动设备（含 receive-only）、底盘/CAN 控制、实车目标、非零速度及重启需要该次明确授权。计划、文档及历史许可不自动继承。确认会话归属后只管理本次自有进程，不按进程名全局清理或自动重启。
- 软件验收及真实验收分开记录。离线 ROS 用隔离 domain 和 localhost，不连接底盘。真实验收结论使用现场证据，无外部真值时不宣称绝对精度、APE/RPE 或实测制动距离。阶段结束更新 STATUS 和报告，保存命令、退出码、证据与 PASS/PARTIAL。
- 提交/推送须有用户授权。避免 reset/clean 或覆盖未提交修改。原包、地图、标定、失败证据、旧安装和 `.runtime/backups/` 受保护。

## 有效配置与资产

- 通用 `config/runtime.yaml` 默认 FAST-LIVO2、wheel、terrain disabled。C1/C2 通过 standard 接口发布先验地图定位，前端与 B 建图职责独立。保持 LiDAR–IMU–Camera 三融合。Lightning 是显式选择的对比基线，使用 `config/profiles/lightning.yaml`。
- 正式 FAST 前端由 `config/fast_livo2/active_frontend.lock.json` 锁定。旧地图使用绑定 UUID、manifest 和前端来源的 `frontend_timing_upgrade` 声明，并满足原传感器/时间契约。不得用离线兼容声明绕过在线校验。
- 启动时读取项目导航 YAML。实时覆盖边界与内部未知格分别计数，全车预测及制动检查使用同一策略。已知障碍与越界直接拒绝。静态总裕量、实时点障碍净空和体素误差按各自配置执行。
- 参考源码位于 `vendor/fast_livo2_review/`，核对 `config/fast_livo2/sources.lock.json`、`config/exact_mppi/sources.lock.json`、完整 SHA、补丁及许可证。Git 只保存可版本化内容，部署可能依赖本地构建和软链接。
- `calibration/` 是正式标定目录，`~/calibration` 仅作兼容。相机同步维护在 `ws_camera/src/hik_camera_trigger_driver`、`timeshare_writer`。
- 依赖优先使用项目内前缀。系统、ROS、CUDA、Eigen、PCL、网络和驱动的变更需要相应授权及可回退方案。sudo 使用正常权限流程，不记录密码或绕过权限。

## 时间、坐标与控制

- `/fast_livo/odometry` 与 `/fast_livo/deskewed_imu` 严格整纳秒同戳，分别为 VIO 后 IMU 位姿及单区间 IMU 系去畸变云。LiDAR–IMU 外参只应用一次。前端世界 +Z 朝上，地图按各关键帧优化后的位姿重建。
- `T_A_B` 将 B 系点变换到 A 系。正式图像偏移 `img_time_offset=+0.1s`，保留原始 header，禁止重复补偿。扫描戳共享仅证明写戳关系，物理曝光相位须独立验证。Tag 时间单独处理。
- `base_link` 是轴间中点的地面投影，相机镜头朝向为车头，雷达原点距同端约 26 cm。工作外参取 `config/fast_livo2/navigation_prep.yaml` / FAST profile。V2 的 `v2_sdk_front_opposite_base=true` 同时处理指令与反馈方向。
- 唯一公共桥接发布 map→odom、odom→base_link。底盘原生 TF 与前端 native_tf 关闭。用同机体的原始采样时刻计算 `T_map_base=T_map_imu×inverse(T_base_imu)` 和 `T_map_odom(t)=T_map_base(t)×inverse(T_odom_base(t))`。wheel 只允许历史内插，禁止外推和 latest-TF 拼接。
- 传感器与 host/wheel 历元分开。离线用 header-clock/模拟时钟，在线用每会话 receipt/steady 映射。心跳不刷新样本时刻，跳时或偏差锁定需新会话。拒绝的定位与旧异步结果不得写入已接受缓存，失败撤销就绪。
- C2 自动初始化保留地点歧义拒绝和 C1 独立确认。手动 C1 保留几何/运动确认，LOST 后需要新初值。Tag 默认关闭，缺少匹配地图的有效地标或时间依据时不放行 Tag 恢复。
- 自主 `LeasedControlCommand` 使用固定 100ms 租约，驱动换会话需新的 enable=false→true 边沿。只用最终验证 ACK 的轨迹作预测初值，按实测进展推进。输入失效撤销输出，恢复必须重验。
- `record_fast_livo2.sh` 默认执行，多数其他入口默认 dry-run。录包复用新鲜 wheel 源，否则经授权受管启动 `ws_base/install_exact` 的 receive-only 模式。receive-only 不请求模式、不接收速度，退出不发零速。已有外部源不代停，设备操作始终按本次授权。
