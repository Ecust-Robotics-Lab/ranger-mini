# 新 Agent 接手指南

更新：2026-11-05。项目 `/home/lab307/ranger_nav`。本文件只维护当前事实、关键证据与接手顺序；过程记录见带日期报告。**总计划未全部验收，当前不能宣称真实 Tag 辅助重定位/回环已经可靠可用。**

## 1. 本轮边界与快速阅读

2026-11-05先完成交接和经核对的清理，随后完成只读接手核对，并按用户“先分段提交一下本地git”的指令分组本地提交已有成果，见[本地检查点](local_git_checkpoint_20261105.md)。没有继续算法实施、长时间重放、硬件操作、部署或推送。下一 Agent 先核对实际状态并汇报，按新对话指令继续；不要从历史报告的“下一步”自动启动设备。

依次阅读：

1. [AGENTS](../AGENTS.md)、[STATUS](STATUS.md)、[总计划](FAST_LIVO2_BACKEND_PLAN.md)。
2. [AprilTag 计划](FAST_LIVO2_APRILTAG_PLAN.md)、[前端契约](fast_livo2_interfaces.md)、[Tag 契约](fast_livo2_apriltag_interfaces.md)。
3. 当前最关键的三份报告：[扫描相位与正式 +100 ms](fast_livo2_timestamp_phase_report.md)、[C1 修正/GICP 对照](fast_livo2_c1_correction_report.md)、[两份新包/地图对照](fast_livo2_tag_two_capture_report.md)。
4. 涉及对应模块再读 [B](fast_livo2_backend_interfaces.md)、[C1](fast_livo2_localizer_interfaces.md)、[C2](fast_livo2_relocalizer_interfaces.md)；目录与入口见 [PROJECT_LAYOUT](PROJECT_LAYOUT.md)、[README](../README.md)。

可直接复制的开场提示词见 [NEXT_AGENT_PROMPT](NEXT_AGENT_PROMPT.md)。本次清理明细见 [交接/清理报告](handoff_cleanup_20261105.md)。

## 2. 当前最重要的事实

- **正式前端试验参数现为 `time_offset.img_time_offset: 0.1`，即 +100 ms。** 用户已明确指定，勿退回 0 或改成拟合的 +110 ms。只改前端有效采样时间，原始 bag、相机/雷达驱动 header 不改；`exposure_time_init` 与 IMU/LiDAR 偏移仍为 0。
- LIV_handhold_2 锁定配置也为 +0.1；作者解释过雷达收完整帧才发布、共享值仍是扫描起始戳造成一帧差。新两包各900扫描跨度约100 ms，1843相机会话帧均读当时最新 writer；这支持固定扫描相位机制，不等于曝光中心/剩余相位已测定。
- **+100 ms 尚未重跑完整三融合、建图及 C2/C1 性能对照。** 当前 +110 ms 的地图/轨迹/6.11 px 结果全部保留原条件，不追认为 +100 ms 结果。仅参数语义/实际加载、dry-run及11项既有时序几何回归已验。
- `config/fast_livo2/apriltag_camera.json` 仍为 `offset_ns=0, time_verified=false` 的既有诊断配置，Tag 功能默认关闭；不能把它当作与新前端匹配的已确认映射提交候选。以后原图分别进入前端与 Tag 时各自明确映射一次，不给已经修正的位姿戳再加一次。
- AprilTag 候选、双解、地标归档、旧图补录、C2/B 接入及监视的主体软件和合成/隔离检查已交付。**没有真实 VALID Tag 地标表；所有已报告真实包的 C2 成功均是 Tag 关闭时的纯 LiDAR 路径。**
- 既有 small_gicp 已在项目中实现，本轮仅通过离线 `--registration small_gicp` 选择它，影响 C1 粗/精与 C2 精配准；未换 FAST 前端、Tag PnP、KISS 全局粗配准或 B 建图默认，未部署新的生产版本。

用户已确认，不要重复询问：当前新部署为 **tag36h11、ID 01=1、打印后实测黑色检测边界140 mm**；Tag在两新包内及两包之间固定，传感器刚性安装未改变，组件在车上手动操控。新尺寸表 `config/fast_livo2/apriltag_sizes_capture_140mm.json`。旧155 mm实例及旧输出单独保留，不能按同一个ID合并。新包与早期建图包为同一场地；这不证明旧图完整覆盖或绝对精度。

## 3. 已验证能力及正确解释

| 内容 | 已有结果 | 不能据此推断 |
| --- | --- | --- |
| A/B/C1/C2 | 本机构建、合成/故障/接口、同包功能及部分不同采集包的全局搜索→C1跟踪 | 完整 C2-4、多地点负样本、真值、实车实时/长时、D 已验收 |
| Tag 软件 | 首轮123项软件/合成/隔离检查；后续有相关回归，数量有重叠 | 所有真实验收已完成或可相加成互不重复的总数 |
| +110 ms 新图直行/PCL | 895提交、894处理、1次合法超时取消，525有效，首VALID约37.000 s | 当前 +100 ms 的结果、实时吞吐、未来整层图恢复时延 |
| +110 ms 新图直行/GICP | 895提交、894处理、1取消、520有效、DEGRADED 1，无LOST/ERROR，首VALID约37.400 s | GICP启动更快、Tag辅助成功、位置绝对正确 |
| 冻结原Tag常量的跨访问对照 | 同516观测，像素RMS的P95约17.324→6.114 px，仅改查询使用的已有注册器 | Tag本身有6 px测量误差或机器人绝对定位误差已知 |

回投残差混合机器人定位、地图、Tag地图位姿、内外参、时间及角点误差。小残差非零正常；3 px RMS/6 px max是最初试验门槛，尚未完成整链误差预算标定，不能当普适物理标准，也不能为让当前数据通过而任意放宽。应验真实候选能否提高正确恢复、保持歧义并拒绝错误地点。

## 4. Git、源码和真实运行版本

当前分支 `refactor/pluggable-navigation`；本次分组提交前基线为 **`acffa879cc8d52330276089cd3df83ad6a43f112`**，当时相对本地upstream领先8提交。2026-11-05按用户指令新增7组本地提交，当前HEAD以 `git log -1` 为准，记录见[本地检查点](local_git_checkpoint_20261105.md)；未联网刷新远端，未推送。此前AprilTag、writer/录包、C1计算/评估、诊断和文档已归档提交；两份新增 `docs/03_evidence/driver-*` 单独成组。禁止reset/clean、覆盖后续未提交工作或把被忽略资产当垃圾；原包、地图及9份归档程序副本继续留在本机。

C1已提交基线 `1212b43`，C2 `2e0f06f`，重力 `4b49bb0`；此前B的三个提交已按历史指令推送。本次仅按新指令本地分组提交，没有推送。

| 位置 | 当前用途/注意事项 |
| --- | --- |
| `vendor/fast_livo2_review/ros2_frontend` + `patches/fast_livo2_stage_a.patch` | 锁定ROS2前端源码；版本见 `config/fast_livo2/sources.lock.json` |
| `ws_fast_livo/src/ranger_fast_livo_backend` | B、Tag核心/检测/地标工具；正式依赖项目内前缀 |
| `ws_fast_livo/src/ranger_fast_livo_localizer` | 同一个C1/C2，含Tag候选/复核；不是新定位后端 |
| `ws_fast_livo/backend_install/` | 既有受管默认安装；不能当作最新隔离C1版本 |
| `ws_fast_livo/install_c1_confirmation_20261104/` | C1等待确认计算优化的隔离安装，未替换上行默认 |
| `ws_fast_livo/independent_time_source_20261104/`、`build_independent_time_20261104/` | 最新两包重放的锁定副本/程序，原零gyro初始化；可作固定+100 ms离线实验的来源 |
| `ws_fast_livo/motion_diag_source_20261104/` | 旧诊断含初始化零偏实验改动，不可混为正式锁定来源 |
| `ws_fast_livo/build_c1_correction_probe_20261104/` | 单扫描四条件配准控制实验程序，不是生产节点 |

实际隔离节点为 `ws_fast_livo/install_c1_confirmation_20261104/ranger_fast_livo_localizer/lib/ranger_fast_livo_localizer/localizer_node`，SHA **`99453aca95043aba1beb78f11fb1a08fb0805140c964e74ec7ad44a32d0bb619`**。动态库与构建证据见 `docs/fast_livo2_evidence/tag_two_capture_20261104/optimized_live_linkage.json`；不要只凭目录存在认定装载版本正确。

原安装前端/生成harness与锁定源码曾发现不一致；后续实验需记录源SHA、补丁、二进制、参数和动态库。新两包录制时前端SHA `dfcf5d4d54f7b81f573a000589527afa36f4dc71c5fe44df1c94f5bff9ae7b27`，不是“任意本地构建都相同”的证明。

关键已完成修改勿重做：writer RELIABLE/depth5、DDS诊断与v2共享一致性；录包 PREPARING/WAITING/RECORDING及输入缺失报错；C1等待帧减少重复13初值搜索，但首支持/可提交帧仍全检；`scripts/fast_livo2/localizer/validate_motion.py`明确区分超时取消与同戳处理确认，取消不算处理成功。

## 5. 当前保留的数据、地图和展示

| 资产 | 路径及作用 |
| --- | --- |
| 转向90s，下一轮训练/建图来源 | `bags/fast-livo-tag140_turn_03-bk8_pjyy/raw` |
| 直行90s，独立采集查询 | `bags/fast-livo-tag140_straight_04-ryzykq2l/raw`；已经参与过方法评估，今后不能称全新盲测 |
| 120s同步诊断 | `bags/fast-livo-tag140_sync_diag_02-35f_m8nd/raw`；1200扫描，writer均覆盖 |
| 60s漏写证据 | `bags/fast-livo-tag140_check_01-dguhgx2l/raw`；7个漏写与重复共享戳对应，独立失败证据保留 |
| 早期136.6s三融合/旧图来源 | `bags/fast-livo-motion-w5phdosj/raw`；不是完整180s，也非真值 |
| +110 ms两包前端与新地图 | `outputs/fast-livo-tag140-two-capture-20261104/` |
| 新局部地图 | 上行 `new_local_map/snapshot`：UUID `4ec68a99c626e1c6e2652506f5ecd8d6`，graph41，31KF/10已接受LiDAR回环；**不是整层地图** |
| 旧B图 | `outputs/fast-livo-backend-validation-20261028-{pcl,gicp}/snapshot`；PCL UUID `6acd0a5763786389402bf72f397d9cbc`，49KF/13loop |
| 最新GICP控制与完整运行 | `outputs/fast-livo-c1-correction-20261104/`；证据 `docs/fast_livo2_evidence/c1_correction_20261104/` |
| 静态测试固定输入 | `bags/fast-livo2-baseline-126a5fAo`；现有ROS回归引用，不可删 |

转向/直行原DB SHA分别 **`23b3bdba0c4ac5497732eaae61f256bef74d57c52f0181efe58b7f5fe7a48e81`**、**`e6c7cd1b2d200b4629abfa7002458ace896e3c6a3b2268cc673145d21f400b6d`**。源包/配置、地图manifest与二进制完整性见两包及C1报告的最终核对。

重用实验脚本前先读参数与默认值，输出必须新目录：`docs/fast_livo2_evidence/tag_two_capture_20261104/replays.py`默认offset为0、只应显式传100000000；`validate_optimized_recovery.py`指向隔离新C1；`docs/fast_livo2_evidence/c1_correction_20261104/run_gicp_checks.sh`归档的是+110ms历史命令，带--execute且原输出已存在，**不要直接执行覆盖或当成新+100ms入口**。

可展示图片：`docs/fast_livo2_evidence/c1_correction_20261104/runtime_results.png`、`tag_reprojection_example.png`、`correction_controls.png`；地图/独立访问图在 `docs/fast_livo2_evidence/tag_two_capture_20261104/map_and_independent_visit.png`；中文图注与ZIP见 `outputs/fast-livo-apriltag-presentation-20261104/README.md`。黄角点是检测值，不是外部真值。

## 6. 时间、坐标和安全边界

- 保持三融合；Tag不注入ESIKF，不回灌前端状态/视觉地图，不新增在线Tag图因子。Tag给候选，原LiDAR配准/验证和合法提交决定是否接受。
- `T_A_B`将B点变到A；后端仅用同戳 `/fast_livo/odometry` 与 `/fast_livo/deskewed_imu`，VIO后IMU参考点、单区间机体系云，外参只应用一次。
- 本次前端重力对齐已开启；旧20261028 B图未对齐，新20261104局部图使用对齐后的+110ms轨迹。逐资产区别，不能一概称“所有地图未对齐”或默认锁roll/pitch。
- sensor虚拟UTC约2020、host/wheel约2026，不能直接拼接或重打now。离线用独立header-clock视图/sim_time；原包不改。
- 公共唯一桥接发布TF，按同采样时刻插值wheel，禁止外推/latest拼接。私有候选成功不绕过LOCKED，不删锁、不放宽运动/几何要求。
- Lightning/wheel/terrain disabled仍为生产默认；FAST前端/B不是先验地图定位器。硬件、底盘/CAN、导航与部署另按该次明确指令。

## 7. 下一步与首次只读核对

本轮交接后先停在接手理解。收到继续指令后，优先：

1. 固定 **+100 ms**，复用已有转向/直行原包与可追溯隔离源码，完整三融合另存新输出；使用同一比较集合/冻结规则与既有0/+110结果对照，不重新扫值后再宣称独立验证。
2. 需要新时偏地图时按新轨迹重建新版本/UUID，不改+110ms旧图。继续使用已有small_gicp作受控对照，核对实际节点/库和几何、时间、会话契约。
3. 区分定位/地图、标定、曝光相位和角点的剩余不一致，合理评估误差预算；有实证才更新Tag时间映射及其有效性，不为解锁候选直接写true。
4. 建立绑定正确地图版本的真实地标表，分离登记和评估访问；再验Tag C2候选/歧义/合法提交与真实B回环、负样本。当前两包已用于选择，后续盲测必要时提出新的针对性数据需求。

当前不要求用户重复录同类包；有需要时说明具体待测量和用途。全层图、实时资源/长时、独立真值、wheel与D仍待验收。

```bash
cd /home/lab307/ranger_nav
git status --short --untracked-files=all
git log -8 --oneline
git branch -vv
python3 scripts/runtime_config.py
bash scripts/run_fast_livo2_live.sh --status
bash scripts/record_fast_livo2.sh --status
bash scripts/run_fast_livo2_localizer.sh --status
bash scripts/run_fast_livo2_relocalizer.sh --status
```

2026-11-05接手实查：FAST live/record为STOPPED，B/C1/C2的历史故障测试状态为FAILED；五者均owned=[]且不存活，不能当线上故障自动重启。另一个受管field会话 `run-u0m5bw9a` 存活、状态DEGRADED，实际有Lightning、底盘驱动和公共桥接进程；旁边 `ranger_nav_k_integration` 工作区另有雷达时间归一化和Nav2相关进程，ROS_LOCALHOST_ONLY=0。没有据此验证车辆运动、就绪或新时间方案，也没有干预既有会话。新Agent仍须现查，不能把FAST会话停止写成整机停止。清理和交接前副本/完整diff在 `.runtime/backups/handoff_cleanup_20261105_171706/`；不是所有大文件备份，已删除原录包无法从这些元数据恢复。本次提交前清单和diff另存 `.runtime/backups/local_git_checkpoint_20261105_173931/`。
