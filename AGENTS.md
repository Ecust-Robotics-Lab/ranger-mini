# Ranger 导航工程约束

更新：2026-11-10。新上下文先读[HANDOFF](docs/HANDOFF.md)、[STATUS](docs/STATUS.md)，再读[总计划](docs/FAST_LIVO2_BACKEND_PLAN.md)。当前状态以这两份入口为准，带日期报告保留历史事实；历史计划、命令和设备许可不自动续用。

## 范围与授权

- 项目`/home/lab307/ranger_nav`。先核对Git、实际源码/参数/二进制、boot/PID与会话归属、原包/地图；保护未提交工作，优先复用。
- 用户已授权[EXACT-MPPI计划](docs/EXACT_MPPI_NAVIGATION_PLAN.md)的软件实施、构建与离线验证；新全局SE(2)/EXACT-MPPI及独立安全链已替换源码导航入口，当前验收见[报告](docs/exact_mppi_implementation_report_20261110.md)。不恢复旧Smac/Nav2 MPPI调参分支。最新定位优化已实现并上传，见[报告](docs/fast_livo2_initialization_optimization_report_20261109.md)、[Git检查点](docs/git_checkpoint_initialization_upload_20261109.md)。同一实施会话可继续已授权的软件工作；独立新接手先核对当前状态与剩余项。
- 明确授权实施后，在范围内自主完成源码、构建及离线验证，避免过多防御性设计；不用缺独立包或外参精测阻塞可完成的软件工作，缺的真实验收如实标PARTIAL。
- 设备启动、底盘/CAN控制、非零速度、导航目标及重启须该次明确授权。重启不继承运动许可，receive-only启动也属于设备操作；不自动开设备寻找初值。计划不构成许可。
- 当前实车目标已确认：图上方办公室出发，沿长走廊到端点掉头返回；用户要求准备并允许适当减小规划额外裕量。已新增窄通道参数、地图复核与录制入口，见[实车准备](docs/EXACT_MPPI_FIELD_TEST.md)。变化场景新包s10xmu6g已完成，新图UUID160f8572a77a61856cf87e43514be769。用户本次明确授权实车测试，后又确认相机朝向是车头、同侧雷达距离约26cm，并现场确认修正后的正速度确实前进。修正车头后的候选路线6/7静态通过，最终回办公室恢复朝向仍搜索超时；不能直接发完整路线。当前结果与授权历史见docs/exact_mppi_changed_scene_live_report_20261110.md，新独立接手仍须现查状态与当次许可。
- 不自动提交/推送；历史保存和上传指令已完成，不授权未来操作。阶段结束更新STATUS和报告，记录命令、退出码、文件、证据、PASS/PARTIAL及最少人工待办。

## 资产、依赖与组织

- 禁止reset/clean、覆盖未提交修改；不删除工作空间、原bag、标定、地图、失败证据或`.runtime/backups/`。历史两次清理的固定名单不授权继续删除；见`docs/handoff_cleanup_20261105.md`、`docs/handoff_cleanup_online_mapping_20261105.md`。最新仅文档去陈旧记录见`docs/handoff_cleanup_20261110.md`。
- Git不是原bag/地图/二进制的完整备份。Lightning从build/bin启动，部署可能是链接，清理前核对运行依赖。用户原雷达文档和`docs/03_evidence/driver-*`保持原证据，不混无关修改。
- 工作空间ws_base/ws_livox/ws_camera/ws_slam/ws_fast_livo/ws_nav；参考源码`vendor/fast_livo2_review/`，锁`config/fast_livo2/sources.lock.json`，补丁`patches/`。核对完整SHA/分支/DETACHED、实际补丁和逐组件许可证；参考不等于集成，元数据不等于授权。新增依赖优先项目内前缀。
- `calibration/`是正式目录，`~/calibration`仅兼容链接；新入口不依赖旧路径。相机同步维护于`ws_camera/src/hik_camera_trigger_driver`、`timeshare_writer`，原驱动/SDK及用户修改保留。
- 不擅自升级系统、ROS、CUDA、Eigen/PCL、网络或驱动。本轮用户额外授权RTX5060驱动/CUDA，用户已安装580-open并完成MOK；项目内JAX CUDA12已实测可用，不重复安装或自行重启。改系统/网络先保存原值和回退；sudo正常授权，不索取记录密码或绕过权限。

## 硬件、默认与当前能力

- 已核实Ubuntu22.04、ROS2 Humble、x86_64、Python3.10；换机重新核实。实物Ranger Mini V2、Livox Mid-360S、海康USB3相机，雷达外触发；IP/SN读当前配置/运行结果。
- 生产`config/runtime.yaml`仍Lightning建图/定位、wheel连续里程计、terrain disabled；源码导航入口已改为复用FAST公共输入的EXACT-MPPI，原Nav2安装资产保留但新入口不启动旧规划控制器。前端选择和全局定位选择独立，FAST前端/B建图后端不能冒充先验地图定位器。
- FAST定位默认自有v2检索→KISS完整6DoF→small_gicp→同一C1跟踪。显式C2可全局初始化/恢复；独立手动C1初值仍需几何/运动确认，LOST须新初值。缺完整来源身份的旧B归档须显式离线兼容声明，不改原档。
- 显式C2 overlay开启强几何静止初始化，保留地点歧义拒绝和C1独立确认；预处理仅本次查询复用。原速记录输入2.800/4.220秒不等于实机耗时；40核心/21后端/9ROS通过，真实负样本泛化、连续就绪、长时与D仍PARTIAL。
- 相机线程/有限队列/迟到检测已实现；在线定位用`ws_camera/latency_install`，原安装保留。C2短时局部恢复已实现（2秒/.75m/.35rad、两新帧/.1秒），软件和记录点云通过，实机恢复尚未触发验收。最新实机co968xa3：90.044秒、921同步OK、591不同有效确认、无LOST，3次60–120ms就绪撤销；USB旧停顿未复现不等于根因已唯一证实。
- 保持LiDAR–IMU–Camera三融合，关闭图像或静态验证不能代替三融合运动；同包功能不能代替跨会话泛化/真值验收。PGO/定位不反馈前端滤波器或视觉地图，不自动实施在线前端恢复、高程/坡道。
- Tag默认关闭、独立时间未确认、无真实VALID地标表，真实Tag恢复/回环未验；主体软件已完成。用户已确认tag36h11:1、实测黑边140mm、跨两新包固定且安装不变，不重复询问；旧155mm实例分开保留。计划见`docs/FAST_LIVO2_APRILTAG_PLAN.md`。

## 时间、坐标与接口

- 读`docs/fast_livo2_interfaces.md`及B/C1/C2接口。输入严格同戳`/fast_livo/odometry`与`/fast_livo/deskewed_imu`：VIO后IMU参考点、去畸变单区间机体系云；LiDAR–IMU外参只应用一次。
- 统一T_A_B将B点变到A；累计世界云不能当body云。地图逐关键帧按各自优化位姿重建，每点只变换一次；单一全局刚体修正不等于历史图优化。
- 正式`img_time_offset=+0.1s`是用户明确选择；不改原bag/驱动header，不追认历史0/+110ms结果，不重复补偿。复制scan戳不证明物理同步，+100ms不等于曝光中心实测；共享读写v2已实现，不重复修已完成项。Tag时间独立，不给修正后的位姿再加偏移。
- 当前前端初始重力对齐、世界+Z朝上；20261028旧图未对齐，20261104 +110ms新局部图及正式+100ms图逐资产区别，不混坐标系、不默认压平或锁roll/pitch。
- base_link为轴间中点地面投影。11月10日用户确认相机镜头朝向为车头、雷达距同端约26cm，原SDK前向与此相反。工作外参更新为约RPY=[1°,21°,0.9°]，保留倾角、按同一实测距离重算XYZ；驱动v2_sdk_front_opposite_base=true统一指令/反馈，正向9.3cm微测经用户现场确认。精确XYZ/RPY以`config/fast_livo2/navigation_prep.yaml`/FAST profile为准；是运动估计，不是实测标定，勿退回历史零旋转。FAST＋wheel同录包pwr5lx7u已取得且核验，不再按“无同录包”阻塞。
- 公共桥接唯一发布map→odom、odom→base_link，底盘原生TF与前端native_tf关闭。T_map_base=T_map_imu×inverse(T_base_imu)；T_map_odom(t)=T_map_base(t)×inverse(T_odom_base(t))，必须同机体同原采样时刻，wheel历史插值，不外推、不latest-TF拼接、不减两个最新到达值。
- 原sensor历元与host/wheel不能直接拼；离线用header-clock/模拟时钟，在线每会话receipt/steady映射含未知接收偏差，不当物理标定；不重打now，心跳不刷新位姿采样时间。跳时/偏差锁定须新会话。
- 失败撤销就绪，不发布未验证修正，创新拒绝不污染已接受缓存。B离线桥首次创新失败锁定，须显式重开；私有PGO可继续。初始化、退化、丢失、恢复可观测；旧异步/跨会话结果不能污染新估计。

## 操作与测试

- 自动入口通常默认dry-run。专用`record_fast_livo2.sh`默认执行、独立传感器/FAST profile、默认录wheel；`--dry-run`仅计划，`--attach`显式借用预览，`--no-wheel`关闭wheel。默认行为不等于Agent自动开设备许可。
- 录包wheel有新鲜源则复用；没有则受管自启`ws_base/install_exact`并显式receive_only=true（保留原install_recording_rx历史资产，避免旧坐标wheel混入），receive-only不请求模式、不订速度、不退出发零速。已授权固定免密CAN命令由正常权限调用；不擅改系统授权。已有源不代停，自启源随会话清理。
- 按boot/PID身份核对现有会话，禁止全局按进程名杀，不抢占、不自动重启。FAST会话结束不代表整机设备全停；旧PID/CAN故障/运行状态只属旧快照。
- 审查底盘控制模式副作用，“未发速度”不等于无副作用。底盘运动执行尚未随新规划方案验收；旧故障解码与V2反馈问题见历史报告，不按旧快照判断当前故障。
- 离线ROS隔离domain/localhost，不连接底盘；只跑相关检查。无真值不编APE/RPE、绝对精度或检测器排名；软件/模拟/记录回放不得称为实车结果。
