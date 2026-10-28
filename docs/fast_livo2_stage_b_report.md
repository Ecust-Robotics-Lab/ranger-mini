# FAST-LIVO2 阶段 B 实施与离线验收

日期：2026-10-26；工程 `/home/lab307/ranger_nav`。本轮按用户明确指令实际开发、构建和测试，仅到 B；无真实硬件、底盘控制或导航目标。

后续2026-10-28已新增真实运动数据验证，PCL/GICP均完整处理1362对、生成49关键帧与13条回环约束，并重跑本报告16项验收全部退出0。详见[真实运动后端报告](fast_livo2_stage_b_motion_report.md)。以下“没有运动包”等文字保留原报告日期语境，不能代替最新证据；物理同步和绝对精度仍PARTIAL。

## 结论与适用范围

**PASS（本机代码与离线边界）**：独立 ROS 2 C++17 后端、严格配对、关键帧、位置检索、PCL ICP / small_gicp GICP / VGICP、几何门控、独立大修正确认、Pose3 iSAM2、非关键帧传播、不可变分片/一致性地图快照、标准适配器和公共桥接隔离联调已实现并验证。生产默认仍为 Lightning / wheel / terrain disabled。

**PARTIAL（真实能力待验）**：没有三传感器运动包，不能报告实车闭环成功率、APE/RPE、定位精度、误环率、真实实时性或长期稳定性。物理同步、曝光相位、精确 T_base_imu、真实 wheel 历元一致性、重力对齐和描述子大漂移检索仍未验收。只有位置检索，漂移超出半径可能漏环。

**未实施**：C1 地图加载和给定初值定位、C2 无初值重定位、在线前端恢复、重力因子、PCM、KISS-Matcher 求解器、SC/STD/iBTC、HBA/LVBA 精修、高程/坡道与实车系统验收。未向前端 ESIKF/视觉地图回灌。

## B0 实际审查与保留

- 主仓库分支 `refactor/pluggable-navigation`，HEAD `a76d83a767d5691ca40351f3a57f65b398572a52`；阶段 A/整合、原雷达文档和证据等未提交修改均保留。开发前状态、二进制 tracked diff 和时间写入 `fast_livo2_evidence/stage_b_20261026/initial_*`，未提交、reset 或 clean。
- Ubuntu 22.04.5、x86_64、ROS Humble，系统 Eigen 3.4.0、PCL 1.12.1。目标包原本不存在；已有 `ws_fast_livo` 是前端构建/部署目录，新增其 `src/ranger_fast_livo_backend`，没有重复工作空间或覆盖旧产物。
- 实际复查前端的同步切分、IMU 去畸变、LIO 云保留、VIO 状态更新及专用发布调用链。云在图像区间末端 LiDAR 系去畸变，发布时应用一次 T_imu_lidar 转到 IMU 系；位姿在 VIO 后读取，两个输出共用一次构造的 stamp。未修改阶段 A 源码或补丁，补丁一致性重验通过。
- 前端内部仍经过 double 秒；后端严格整数匹配不等于原始传感器纳秒精度。物理同步也不由同戳证明。
- 当前 gravity_align_en=false，不能假定地图水平。第一帧图先验为 X0=U0，保留 FAST local 原点/轴/yaw。当前前端无运行中局部 reset 路径；B 以 DDS GID/ROS 回退区分会话，未来增加同进程 reset 必须另加明确事件。
- 只读重新解码既有 static native_frames：829 odom、829 cloud、829 个整数同戳对，未见重复、所检字段错误或存储时间/header 不符。该数据只证明接口兼容。

## 源码、依赖与算法取舍

| 来源 | 完整提交 | 本轮用途 |
| --- | --- | --- |
| liangheming/FASTLIO2_ROS2 | 7baa59f7339ba88ce3b646fe35b23f61031c5d3f | SimplePGO 算法组织参考；不使用原版 pgo_node 队列/配对/保存/TF |
| borglab/gtsam | 4f66a491ffc83cf092d0d818b11dc35135521612 | 项目内 Pose3 / Prior / Between / iSAM2 |
| koide3/small_gicp | fa0cfc983417c044b6036f71dfa448038bb9c820 | 项目内 GICP/VGICP，PCL 基线可切换 |
| MIT-SPARK/KISS-Matcher | 8b58b118e00c7a5a22448741f8d956d638a44ae2 | 审查 coarse/fine、内点与候选分层接口；未集成 SAM 或求解器 |
| gisbi-kim/SC-A-LOAM | 321cb4b07b7815c114237ce5d87014d2e46b972c | 描述子候选/航向和独立 ICP 接口参考，未集成 |
| hku-mars/STD | 6e5903cfa2aaed5ca3c9e7d21424eebcca7655bb | SearchLoop/candidate_selector 接口与许可审查，未集成 |
| hku-mars/iBTC | 9f3bf49e26ff254d28043e8e5d404f3348da8ef2 | 头文件/元数据/许可定向审查，未集成或给出性能排名 |

实际构建源码在 sources.lock.json 新增 GTSAM/small_gicp 两个 DETACHED 记录，原 13 个锁不替换。参考审查见 `reference_audit.json`；必要许可文本在自有包 `third_party/`，说明见 THIRD_PARTY_NOTICES.md。没有把不同参考项目当成可互换地图格式。

GTSAM 该提交同时有 **4.2、4.2.0** 标签；源码数字版本为 4.2.0，显示字符串的 PRERELEASE 仍为 a9，原样记录。显式 SYSTEM_EIGEN=ON、MARCH_NATIVE=OFF、UNSTABLE/Python/MATLAB/MKL=OFF，依赖安装到 `ws_fast_livo/backend_deps`。未安装全局库或升级系统。

构建审计读取三方与消费目标实际 compile_commands、CMakeCache 和 ldd：均使用 `/usr/include/eigen3`，未见 native/AVX/fast-math 或不一致的显式 EIGEN 对齐定义，运行链接到本项目的 GTSAM/small_gicp。数值测试实际跨 PCL/Eigen/GTSAM 调用。发行版 PCL 原始编译旗标未随系统安装，不能将这些检查宣称为任意平台的 ABI 保证。

PCL 保持默认：三种方法均可构建、可运行且用共同几何门控验证；600 点合成输入的有限对照不足以决定实车默认或证明普遍倍数优势。后续有代表性运动数据再选默认。

## 实现与接口

详细变换、参数及行为见 [后端契约](fast_livo2_backend_interfaces.md)。主要实现如下：

- 输入缓存按 int64 ns 精确有序配对，校验消息、布局、frame、四元数及有限值；允许窗口内乱序，明确缺对/重复/过期/容量计数。按工作队列可用槽位移交，避免启动缺对超时后批量释放误触发溢出。
- 位姿关键帧与 full/match 云分片分开；0.5 m/10°，2 s 条件还需最低运动量，限制 0.1 s 最小间隔。原地旋转可选帧，静止重复扫描不无限选帧。
- 位置检索与 Registration/CoarseRegistration 抽象分离。历史 target 在历史 i 系，source 在当前 j 系，返回总 T_i_j；输入初值只应用一次。子图排除时间/ID 邻域，并检查源目标 scan ID 不交叉。
- 共用对应数、重叠、RMSE、退化、歧义门控；歧义地点间隔独立于搜索半径。大修正要求不同关键帧的时间和前端运动基线，且几何配准本身也证实运动；静止扫描即使有表观 odom yaw 漂移也不能凑出独立证据。候选记录初值/总变换、质量、预算、支持检查与决定原因。
- 独立平移/旋转噪声、可配置鲁棒核；不用 fitness 或 H 逆直接赋协方差。单 worker 管理图，检查 session/图依赖，失败从接受因子重建，只有完整有效版本对外发布。
- 非关键帧使用同版本原始/优化参考关键帧传播。私有 BackendEstimate 原子承载 pose/state/stamp/session/version/loop event；另有原始/优化路径与诊断，不发布公共 TF。
- 完整云同步确认落盘，匹配云 LRU；缓存、工作队列、子图点数和关键帧数量都有上限。当前未实现额外扫描聚合或异步写盘队列；磁盘延迟导致落后时按原 stamp 撤销有效性。
- ExportSnapshot 冻结同一图版本和文件清单，校验 SHA256，流式按各关键帧优化位姿重建地图。保留原始/优化轨迹、云 CDR/PCD、因子唯一 ID、候选日志前缀、标定/时间/参数/来源。临时目录完整写入后原子 no-replace 提交，失败保留明确 partial/FAILED 证据。
- 适配器复用公共五态、T_base_imu 和原采样时间；非法/过期/未来或回退消息不污染接受水位。普通桥接无需导入 backend 消息；仅显式离线适配器需要 source backend_install。
- 公共桥接已修复“创新拒绝却覆盖修正缓存”，拒绝项不进参考历史；重复失败心跳不累加。B 联调开启首次创新失败锁定；私有 PGO 继续运行，不删除其回环，不自动放宽门槛。

新消息、服务均在独立包内；公共 LocalizationEstimate 未改 wire 格式或枚举。新入口复用 session_manager 的资源锁、进程身份、日志、健康失败清理，默认 dry-run，未改变现有 mapping/localization runtime 选择。

## 可复现构建与验收命令

本机已有 ROS/Eigen/PCL/OpenSSL/nlohmann_json 等依赖。获取新依赖时只补缺失锁定仓库，不重置已有仓库：

```bash
cd /home/lab307/ranger_nav
python3 scripts/fast_livo2/fetch_sources.py --backend --execute
bash scripts/build_fast_livo2_backend.sh --execute
source /opt/ros/humble/setup.bash
source ws_fast_livo/backend_install/setup.bash
cd ws_nav
PYTHONDONTWRITEBYTECODE=1 colcon build --packages-select ranger_localization_bridge --executor sequential
cd ..
source ws_nav/install/setup.bash
source ws_livox/install/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 scripts/fast_livo2/backend/run_checks.py --execute
```

构建最后一次依赖/后端完整脚本退出 0，后续针对几何支持与归档断言的增量构建退出 0；适配器构建退出 0。证据为 `acceptance_build_exit.json`、`support_geometry_build.log`、`geometry_archive_test_build.log`、`adapter_acceptance_build.log`。完整构建日志及先前失败不覆盖删除。

自动验收默认不带 --execute 只列命令；带 --execute 仅执行隔离离线测试。每次创建新 acceptance-* 目录，results.json 保存 argv、逐命令退出码和日志路径。复用现有 header-clock 视图并重新校验其 payload 摘要，不复制一份相同三传感器 bag。

普通运行入口同样默认 dry-run：

```bash
bash scripts/run_fast_livo2_backend.sh
bash scripts/run_fast_livo2_backend.sh --offline-bridge
bash scripts/run_fast_livo2_backend.sh --status
```

运行 --execute 需要先准备相应隔离输入；不会帮调用者开启传感器/底盘。离线桥接必须同域模拟时钟及合成 wheel，不能接当前跨历元实时 wheel。来源可用 --source-id 绑定录包，声明的前端配置与真实运行配置仍需核对。

## 针对性结果与证据

全部以下命令实际退出 **0**。统一证据：[results.json](fast_livo2_evidence/stage_b_20261026/acceptance-glzazt73/results.json)。

| 检查 | 结果 | 日志 |
| --- | --- | --- |
| C++ 数值/图/配准/归档 | 21/21 | [cpp.log](fast_livo2_evidence/stage_b_20261026/acceptance-glzazt73/cpp.log) |
| ROS 后端/桥接/受管故障 | 6/6 | [ros.log](fast_livo2_evidence/stage_b_20261026/acceptance-glzazt73/ros.log) |
| 适配器契约与接受水位 | 3/3 | [adapter_contract.log](fast_livo2_evidence/stage_b_20261026/acceptance-glzazt73/adapter_contract.log) |
| 公共桥接拒绝缓存/锁定 | 2/2 | [bridge_guard.log](fast_livo2_evidence/stage_b_20261026/acceptance-glzazt73/bridge_guard.log) |
| 生产默认/配置/三融合限制 | 10/10 | [project.log](fast_livo2_evidence/stage_b_20261026/acceptance-glzazt73/project.log) |
| 现有定位契约 | 11/11 | [contract.log](fast_livo2_evidence/stage_b_20261026/acceptance-glzazt73/contract.log) |
| 原采样时间/轮速插值 | 7/7 | [time.log](fast_livo2_evidence/stage_b_20261026/acceptance-glzazt73/time.log) |
| 既有桥接回归 | 14/14 | [bridge_legacy.log](fast_livo2_evidence/stage_b_20261026/acceptance-glzazt73/bridge_legacy.log) |
| 门禁纯函数 | 12/12 | [gate.log](fast_livo2_evidence/stage_b_20261026/acceptance-glzazt73/gate.log) |
| 现有前端解析/保护检查 | 13/13 | [frontend_cpp.log](fast_livo2_evidence/stage_b_20261026/acceptance-glzazt73/frontend_cpp.log) |
| 前端→后端三传感器静态 DDS | 55 对前端 / 55 后端有效采样；无 TF；两个节点退出 0 | [链路结果](fast_livo2_evidence/stage_b_20261026/acceptance-glzazt73/chain/result.json) |
| 依赖/链接/来源审计、前端补丁一致性、三个 dry-run | 全部通过；逐项退出 0 | 同目录 build_audit / frontend_patch / dry_*.log |

25 次、每云 600 点、单线程、同一初值的合成对照（包含各方法预处理与共同几何门控；不是实车性能）：

| 方法 | 中位总耗时 ms | p95 ms | 最大平移误差 m |
| --- | --- | --- |
| pcl_icp | 1.626 | 1.666 | 1.25147e-06 |
| small_gicp | 1.935 | 1.972 | 0.000149963 |
| small_vgicp | 1.879 | 1.901 | 0.00124642 |

这些有限条件下的数据不支持 small_gicp 普遍更快或更准的结论，也不是各回环检测器排名。

本轮新增/修改文件及保留核对见 [change_inventory.json](fast_livo2_evidence/stage_b_20261026/change_inventory.json)。全部失败和早期通过目录均保留，最终统计以本节指向的 acceptance-glzazt73 为准。

C++ 用例涵盖：整数乱序/缺对/重复/容量/字段与 NaN、非零 xyz+RPY 和增量乘法方向、三个求解器同输入、大修正一致/不一致/静止伪基线、独立描述子候选不再被位置半径过滤、歧义门槛与半径独立、图异常回滚、时序/session 依赖、回环后的非关键帧传播，以及真正优化图的逐帧地图重建。归档测试覆盖版本冻结、校验损坏、目标存在、不可写路径和 fork 子进程 RLIMIT_FSIZE 注入写失败，无成功标记泄漏。

ROS 用例涵盖：非零外参、原 stamp、同刻 wheel 插值/唯一 TF、wheel/传感器断流、错误 frame/NaN、时钟回退、进程异常退出/新会话、多输入发布者、私有/公共状态隔离及受管健康失败清理。SIGKILL 只用于本测试拥有的 backend PID，非全局按进程名清理。

旧 Nav2 全链 37/37 仍为 2026-10-20 证据，本轮没有重跑含全局 pgrep 清理的旧安全链脚本；本轮运行门禁纯函数 12/12、桥接相关回归及新的隔离 TF/就绪测试，不将其替代完整实车安全验收。

## 失败记录与修正

- 首轮 CMake plain/keyword 链接签名冲突，修正为与 ament 一致；首轮 C++ 编译中 Humble Time API 和同一行 GTest 宏标签问题已修正，失败日志保留。
- 初版静止测试把“最新收到帧”误当“最新关键帧”检查最小间隔，已按真实关键帧时刻修正断言，没有降低选帧门槛。
- ROS 重启发现启动缺对等待 2 s 后集中释放，造成工作队列误溢出。改为参数化 0.25 s 缺对等待并按可用槽位移交，完整后续帧保留；容量与字节预算仍有效。
- 断流测试须排空在途消息后再断言采样时间保持；NaN 注入测试曾因 Python Header 共享引用误改 odom frame，已改为独立 header，没有放宽校验。
- 完整短链路早期失败分别为缺 libvikit、相对 data_root、缺 Livox 消息库路径；启动环境明确项目内前端/后端及 Livox 消息库路径，data_root 规范为绝对路径。原始失败目录保留，后续实际链路通过。
- 适配器增加接收前契约/新鲜度检查，避免错误或未来消息先污染单调水位；单元测试中 100 ns 超前样本按既有 fresh 策略应拒绝，测试推进模拟接收时钟，未放宽未来时间规则。

## 交接与最少人工待办

阶段 B 到此停止。最终受管私有会话为故障注入预期的 FAILED 状态，`live=false`、`owned=[]`，guardian_cleanup_complete=true；进程核对没有遗留后端、前端或适配器。证据见 `final_process_check.json`。下一阶段需明确 C1 指令，先定义 B 归档加载/完整性/标定兼容和新进程初始化，再接粗初值 ICP；C2 再增加独立地点检索及无可靠初值配准，不能把位置候选或局部 ICP 改名为全局重定位。

真实验收前仍需：三传感器运动闭环/无环和重复结构数据；实际物理同步/曝光证据；T_base_imu 与重力方向；同一实时历元的 wheel；有代表性的参数、资源和失败恢复测试。有真值才报告 APE/RPE。上述缺口不被本轮合成或静态结果消除。
