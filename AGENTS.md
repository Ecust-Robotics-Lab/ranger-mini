# Ranger 导航工程约束

更新：2026-11-06。新上下文先读 [HANDOFF](docs/HANDOFF.md)、[STATUS](docs/STATUS.md)，再读 [总计划](docs/FAST_LIVO2_BACKEND_PLAN.md)。目录与操作见 `docs/PROJECT_LAYOUT.md`、根 README；历史事实以带日期报告为准，本文件只维护有效约束。

## 当前范围与授权

- AprilTag 增量主体软件/离线范围已完成，计划与证据见 `docs/FAST_LIVO2_APRILTAG_PLAN.md`、`docs/fast_livo2_apriltag_report.md` 及后续带日期报告。当前140 mm的tag36h11:1、实测黑色边界、跨两新包固定及传感器安装未变已由用户确认，不重复询问；旧155 mm实例分开保留。Tag 默认关闭、独立时间未确认，无真实 VALID 表，真实Tag恢复/回环未验。既有成果已按2026-11-05用户指令分组本地提交；2026-11-06用户另行授权分组保存并上传，记录见 `docs/git_checkpoint_20261106.md`；禁止从计划自动启动设备、运动、提交或推送。

- 2026-11-05早期清理按 `docs/handoff_cleanup_20261105.md` 删除3份早期试录、3空目录及6处缓存，约14.1GiB；保留的元数据不能恢复原包。在线预建图交付后的新一轮交接清理见 `docs/handoff_cleanup_online_mapping_20261105.md`，仅清理可再生重放视图和缓存，保留正式成果与失败证据。新上下文先核对/汇报；两次具体名单均不是未来继续删除其他原包、地图、失败证据或备份的授权。

- 在 `/home/lab307/ranger_nav` 核对实际代码、工作树和证据；优先复用，不推倒重建。
- FAST-LIVO2 阶段 A、独立后端 B 已完成本机构建与合成/离线测试；单份真实三传感器运动包的后端闭环管线已验证。见 `docs/fast_livo2_stage_b_motion_report.md`；物理同步、真值精度、误环泛化、实时接车和长时稳定性仍 PARTIAL。
- 阶段 B 三个提交已按用户指令推送；C1 与其他既有成果已分组本地提交，C1 为 `1212b43`。后续授权的 C2 源码/构建/合成及同包功能检查已交付，C2-4 跨会话与真实负样本验收仍 PARTIAL，D 未完成；C2、重力对齐及净化交接已有分组提交，纳入2026-11-06用户授权的上传范围（提交记录见HANDOFF和Git）。总计划未全部完成。
- 2026-10-29 接手前已完成文档交接，后续用户另行授权的 AprilTag 离线工作见上条。新 Agent 仍须先核对工作树和证据，再按用户新指令确定范围；历史计划不构成自动续跑、采集/重启设备、提交或推送许可。不启动真实硬件、底盘/导航运动，不实施在线前端恢复、高程/坡道。
- 下一阶段收到明确实施指令后，在授权范围内自主完成源码、编译与离线测试，不必逐步等待确认。缺少独立运动定位包不阻止可执行的离线开发，但对应真实性能不得报告通过。
- 阶段结束更新 STATUS 和对应报告，记录实际命令、退出码、文件、证据、PASS/PARTIAL/未实施及最少人工待办。计划文件不构成硬件操作许可。

## 本机、硬件与生产默认

- 已核实 Ubuntu 22.04、ROS 2 Humble、x86_64、Python 3.10；换机器/系统后重新核实。
- 实物为 **Ranger Mini V2、Livox Mid-360S、海康相机**，相机采用雷达外触发；历史 Mini 3/Mid-360 称呼不作实物依据。设备 IP/SN 读取当前配置和运行结果。
- Lightning-LM 仍为生产默认建图/定位，连续里程计默认 wheel，terrain disabled。保留 Nav2、Smac Hybrid-A*、MPPI 和安全链。
- `config/runtime.yaml` / `config/profiles/` 独立选择建图前端与全局定位；FAST 前端和 B 建图后端不提供先验地图定位，不能配置成已有地图定位器。
- 默认 C1 为独立离线定位器，人工初值通过私有服务进行几何确认，LOST 后需重新给初值；显式 C2 入口才提供全局确认初值和自动恢复，仍由同一个 C1 跟踪。现有缺完整来源身份的 B 归档须显式绑定离线兼容说明，不改原档。C1/C2 不接管生产 runtime，不把同包功能检查当作跨会话验收。
- 保持 LiDAR–IMU–Camera 融合；关闭图像不能称为 FAST-LIVO2 三融合。静态/无图像数据不能代替三融合运动验证；同包验证不代替跨会话泛化或真值验收。

## 组织、版本与资产

- 工作空间：ws_base、ws_livox、ws_camera、ws_slam（Lightning）、ws_fast_livo、ws_nav（公共定位/导航）。参考源码在 `vendor/fast_livo2_review/`，构建锁在 `config/fast_livo2/sources.lock.json`，补丁在 `patches/`。
- `calibration/` 是正式标定/同步目录，`~/calibration` 仅兼容链接，新入口不得依赖旧路径。同步包在 `ws_camera/src/hik_camera_trigger_driver` 与 `timeshare_writer`；原驱动/SDK及用户修改保留。
- 核对实际源码、参数、launch、完整 SHA/分支/DETACHED、本地补丁和逐组件许可证。新增第三方依赖优先项目内前缀；参考过不等于已集成，元数据不等于完整授权。
- 先检查 Git 状态；禁止 reset/clean 或覆盖未提交修改。采集/相机/标定与 C1 既有成果已分组本地提交；新阶段仍须保护工作树。用户原雷达文档与六份 `docs/03_evidence/driver-*` 已单独提交，保持原证据，不混入无关提交。
- 不删除工作空间、原始 bag、标定、地图、失败证据或 `.runtime/backups/`。以前明确授权的特定数据删除不构成继续删除许可。Lightning 从 build/bin 启动，部分部署是链接，清理可再生产物前先检查运行依赖。
- 不擅自升级系统、ROS、CUDA、Eigen/PCL、网络或驱动。需要改系统/网络时保存原值并给出回退；sudo 使用正常授权，不索取/记录密码或绕过权限。

## 传感器、时间、后端和 TF

- 前端契约见 `docs/fast_livo2_interfaces.md`，B/C1/C2 契约见 `docs/fast_livo2_backend_interfaces.md`、`docs/fast_livo2_localizer_interfaces.md`、`docs/fast_livo2_relocalizer_interfaces.md`。后端接 `/fast_livo/odometry` 和 `/fast_livo/deskewed_imu`：严格同戳、VIO 后 IMU 参考点，云是去畸变单区间机体系云；LiDAR–IMU 外参只应用一次。
- 统一 `T_A_B` 将 B 中点变到 A。禁止把累计世界云当 body 云；全局地图逐关键帧按其优化位姿重建，每点只做一次正确变换。单个全局刚体修正不等于历史图已优化。
- 图像复制扫描戳不证明物理同步。2026-11-04 已核实 LIV_handhold_2 锁定配置及作者的一帧差解释，按用户明确指令将正式前端试验 `img_time_offset` 设为 +0.1 s，见 `docs/fast_livo2_timestamp_phase_report.md`；原包/驱动戳不改，历史零偏移/+110 ms 实验不追认。Tag 独立时间配置仍未确认、默认关闭，不重复补偿，不把 +100 ms 参数当曝光中心实测。共享读写 v2 已配套，旧历元/剩余相位仍待验，不重复修复已完成事项。
- 当前前端试验配置已启用初始重力对齐（世界 +Z 朝上）。20261028的两份旧B/C1地图未对齐，20261104的新局部图由对齐后的+110ms轨迹生成；逐资产核对，不追认旧图、不混用坐标系，不默认平面化或锁死roll/pitch。私有PGO/定位结果不反馈前端滤波器或视觉地图。
- 公共桥接唯一发布 map→odom、odom→base_link；底盘原生 TF 和前端 native_tf 关闭。参考 PGO/localizer 不得争抢 TF。
- `T_map_base=T_map_imu×inverse(T_base_imu)`；`T_map_odom(t)=T_map_base(t)×inverse(T_odom_base(t))` 必须同一机体、同一原采样时刻。复用 wheel 历史插值，禁止外推、latest-TF 拼接或两个最新到达值相减。FAST local 不等于 wheel odom。
- 传感器旧历元与主机/wheel 尚不能直接拼接；离线使用 header-clock 视图和模拟时钟，原包不改写，不重打 now 掩盖问题。心跳不刷新位姿采样时间。
- 定位失败不发布未经验证的新修正，撤销就绪；创新拒绝不能污染已接受缓存。B 离线桥接首次创新失败锁定，须显式重开联调会话，私有 PGO 可继续运行。
- 给定初值定位与无初值全局搜索分别验收，初始化、退化、丢失和恢复必须可观测。跨会话或旧异步结果不得污染新估计。

## 真机与测试

- 2026-11-05后续已按用户指令实施FAST统一在线预建图，操作见`docs/fast_livo2_online_mapping.md`，证据见对应交付报告。`map_fast_livo2.sh`默认dry-run，显式执行才启动本次传感器/前端/B/录包；FAST profile的统一online入口转到它。使用`ws_fast_livo/online_frontend_source`的可追溯I/O派生构建及`online_install`后端，保留原安装。停止先封存B、排空并导出，再清理发布者；私有sensor-clock不等于wheel同步，不接公共TF/控制。单独录包也已补存去畸变云，但不启动B。软件/离线授权不自动扩展为实物测试许可。

- 自动入口通常默认dry-run；专用 `record_fast_livo2.sh` 已按2026-11-05用户后续指令改为默认执行、独立启动传感器、读取FAST profile，显式 `--dry-run` 才显示计划，接入预览须显式 `--attach`。此入口默认行为不构成Agent自动开设备的许可，其余入口仍保持原规则。早期曾查到field和 `ranger_nav_k_integration` 进程，属于当时快照，不能当成当前存活状态。FAST会话停止不能代表整机设备停止；开始先按boot/PID身份查会话与进程归属，不抢占或擅自停止既有会话。C1/C2 离线阶段不启动或重启设备。
- 启动可能接管底盘的节点、CAN 控制、非零速度或导航目标须有该次明确授权，先审查控制模式副作用；重启不继承运动许可，“没发速度”不代表没有副作用。
- 复用受管入口的依赖/占用检查、进程身份、日志和故障清理；不按进程名全局杀，不自动重启设备。
- 离线 ROS 测试隔离 domain/localhost，不连接底盘；只跑与改动相关检查。无真值不编造 APE/RPE、绝对精度或检测器排名；模拟/离线测试不表述为实车结果。
