# FAST-LIVO2 AprilTag 离线增量交付报告

日期：2026-10-29。基线 `acffa879cc8d52330276089cd3df83ad6a43f112`，分支 `refactor/pluggable-navigation`，开始时工作树干净。用户明确选择“计划落盘后继续离线编码与测试”，期间补充确认已有包中含 **tag36h11、ID 1、边长 15.5 cm**。本轮不启动硬件、不采集、不修改原包，不提交或推送 Git。

## 1 结果与边界

已在原 B/C1/C2 上增加可关闭的官方 AprilTag 观察器、双解/像素/时间核心、地标归档与旧图补录、C2 候选、B 回环初值、C1 监视。核心调用链和已有定位/桥接约束通过合成及隔离 ROS 检查；旧包真实图像完成只读抽样检测/PnP。

这不是本项目全部计划完成。真实 Tag 的固定部署、曝光时间关联、独立访问恢复/回环、误接受泛化、精度、实时性和长时稳定仍 PARTIAL。**没有生成冒充已验收的真实 VALID Tag 地图**。当前相机配置 time_verified=false、固定 Tag 白名单为空，生产 runtime 未启用 Tag。

计划见 [FAST_LIVO2_APRILTAG_PLAN](FAST_LIVO2_APRILTAG_PLAN.md)，公式、schema、参数与操作见 [AprilTag 接口](fast_livo2_apriltag_interfaces.md)。本报告记录实际完成范围，计划矩阵不等于每个场景均已有测试。

## 2 源码审查与实际接入

| 实际模块 | 发现 | 实际修改 |
| --- | --- | --- |
| B `Backend::detect_loops`、`Candidate` | 原候选只有 ID/距离，总用 Xi⁻¹Xj 作初值 | 可选总初值、证据、回投复核；Tag 可越位置半径，仍走原注册器、任务依赖、大修正支持和 `Graph::add_loops` |
| C2 `Relocalizer::process` | 无外部候选总线；query 是折算到最新 IMU 的多帧云 | 接收可选 Tag Prior，统一几何/歧义/独立支持，仍返回 SearchResult.confirmed |
| C1/C2 node worker | 已有 generation 和 C1 handoff | 有界观测队列，原上下文代际隔离，Tag handoff 首次 C1 VALID 再复核 |
| C1 | 原有地图/跟踪/初始化逻辑可复用 | 可选投影监视、连续三次可信段记录；不直接修改定位结果或运行中地标 |
| B schema / C1 Archive::load | 严格文件枚举，不能直接追加 landmarks.json | 新建独立不可变扩展，绑定源 UUID/manifest SHA/graph version，原读图器不放宽 |
| 相机驱动 | 原图+CameraInfo 用主机 stamp，image_sync 复制雷达 stamp | 不改驱动；绑定静态模型并显式区分曝光验证状态，不伪造同步 CameraInfo |
| 前端 vio.cpp | 实际 Rci=Rcl×Rli，Rli 为 extR 的逆 | 据使用代码推导 T_imu_camera，保留原标定来源，不根据变量名猜方向 |
| 公共 bridge | 实际锁为 `_innovation_latched`，要求重开显式 alignment session | 没有改解锁或门槛；新增 Tag 恢复后仍锁定的集成检查 |

共享数学/时间/地标实现放在既有后端包内，不增加一串重复 ROS 包。观察器只封装官方 libapriltag，不使用 apriltag_ros 第二条检测路径。新增接口复用 `std_msgs/String` 的 schema=1 JSON，并在消费者中做有界校验；未新增公共定位消息或 TF 发布者。

## 3 依赖、外参与像素

官方 apriltag v3.4.5，固定 SHA `94be783968e5091bcc9972c72c84fd63efce2935`，本地 DETACHED、无补丁；源在 `vendor/fast_livo2_review/apriltag`，前缀 `ws_fast_livo/apriltag_deps`。BSD-2-Clause 原文和来源锁纳入源码。OpenCV 使用原系统 4.5.4（Ubuntu 包 `4.5.4+dfsg-9ubuntu4`），没有升级库。既有 KISS→ROBIN→PMC 的 GPLv3+ 许可没有被改称 MIT。

已核对实际链接：仅可选观察器经 detector 库链接项目内 libapriltag；backend/localizer/tag_map_tool 不链接它。具体 ldd、二进制 SHA、源码 SHA 与原图资产校验见 `source_and_asset_audit.json`。运行环境必须 source 对应 install，并包含项目内 GTSAM/AprilTag 前缀；没有用 sudo 安装替代库。

原始 plumb_bob、校正像素和归一化射线有明确模型；normalized 残差换算到指定虚拟校正像素域。ROI/binning 不匹配拒绝。官方解码角点直接对应规范物点，官方渲染 4 个编码方向验证了语义，未按图像左上重排。所有分支检查有限性、SO(3)、正深度及实际像素域残差，不使用 object-space error 冒充像素 RMS。

`T_imu_camera` 来自 `T_C_I=T_C_L×inverse(T_I_L)` 的逆；仅对原六位小数旋转的微小舍入作 SVD 投影，原 YAML 不改写。相机模型来自正式 1280×1024 plumb_bob 五参数 CameraInfo。初始阈值与未实测标定项完整列在接口文档。

## 4 时间、地标与候选行为

- B 的 odom/deskewed cloud 仍按原整数 ns 严格配对。图像允许异步，只在同会话连续前端轨迹中包围插值，不外推；既检查采样时间，也检查回调接收到 worker 消费的 steady 时间，仿真时钟暂停不延长队列寿命。
- 原始/effective 时间和 offset 来源保留，重复补偿拒绝；没有硬编码历史 ±100 ms。真实曝光关联尚未验证，真实默认只观测、不提交。
- 新地图从各自冻结优化关键帧位姿重算地标；旧图补录从 C1 可信段及同会话局部轨迹传播到图像时刻。原图没有可编辑 PGO 也可补录。
- 地标按身份/姿态模式和独立视角聚合。静止连拍不增加支持或估计权重；移动/双峰标成 AMBIGUOUS/SUSPECT，不平均进 VALID。未知协方差不填零。
- 新扩展目录原子提交、SHA/版本绑定、不覆盖。加载失败禁用 Tag 分支，旧图继续纯 LiDAR。禁用地标也生成新版本，不热改运行中表。
- C2 候选逐支通过原 LiDAR 门槛与回投，空间不同的合格解不能强选“分数第一”；相同物理解不增加支持。未完整评估本次 Tag 分支不提交幸存者，后续通过 backoff 留出纯 LiDAR 尝试。
- B 仅使用显式固定身份声明；同身份双解最多四组合，仍排除自匹配、近邻和重复边，仅产生原 LiDAR 回环因子。Tag 不能补足走廊缺失的可观性。
- C1 监视只输出残差；连续矛盾形成待复核诊断，不移动地图或直接重置。记录失败只关闭记录，不使原跟踪失败。

## 5 实际构建与测试

证据根为 `docs/fast_livo2_evidence/apriltag_20261029/`。构建入口：

```bash
bash scripts/build_fast_livo2_tags.sh --execute
source /opt/ros/humble/setup.bash
source ws_fast_livo/backend_install/setup.bash
source ws_nav/install/setup.bash
python3 scripts/fast_livo2/tags/run_checks.py --execute
```

构建命令实际退出 0，最新日志 `build_delivery.log`；新增仲裁测试构建另见 `build_accepted.log`。最终 **16 条命令全部退出 0，共 123 项测试/检查通过**；其中新 Tag 为 22 项 C++ 与 5 项 ROS/工具检查。完整清单在 `acceptance-2m126s1v/results.json`，驱动日志为 `acceptance_delivery_driver.log`。地标 EACCES 写盘故障另行补验退出 0，见 `pgo_write_failure_accepted.log`，不重复计入 123。

| 检查组 | 覆盖 | 最终数量 |
| --- | --- | --- |
| Tag 核心 | 独立 Brown 投影、IPPE 六自由度、raw/rectified/normalized、模型错误、假质量/错尺寸、时间/杆臂、未知时间、容量、双峰、持久化 | 12 |
| 官方检测器 | 4 个旋转、非连续 stride、encoding、未知尺寸、另一 family、同图重复物理 ID | 3 |
| Tag 与 B/C1/C2 C++ | Tag-only 恢复再 C1、视觉冲突、旧地图/取消、重复图像、异解歧义、同解去重、越半径非单位回环 | 7 |
| Tag ROS/地图工具 | 图像→官方检测→C2/C1→监视/记录、观察器退出、坏地标/旧上下文、逐帧 PGO 重算/禁用版本、桥接锁、仿真暂停队列超龄 | 5 |
| 原 B / C1+C2 C++ | 原图/配准/故障/跟踪/检索/恢复 | 21 / 30 |
| 原 B / C2 / C1 ROS | 原节点、会话、故障清理、给初值/无初值恢复和桥接 | 6 / 4 / 9 |
| 公共桥接/创新锁/时间/适配器 | 原安全契约直接相关回归 | 14 / 2 / 7 / 3 |

独立几何夹具使用非零三轴旋转、平移、相机杆臂、不同曝光/扫描时刻；检测夹具用官方图案和独立投影。PGO 工具夹具的历史局部漂移与每帧优化位姿不同，检查不能套单一刚体校正。登记和查询使用不同姿态/帧，所有合成输出明确标识，无实车精度结论。

失败和修正均保留：首次缺项目内 GTSAM 路径、随后测试证据目录误用相对路径、一个时间夹具 int 溢出、C2 窗口夹具配置、ROS JSON 参数的 YAML 引号、B 测试驱动误改隔离域，以及创新锁测试未先建立 0.5 s 参考区间。均定位并修正测试/启动环境，不放宽算法或安全阈值。首次在旧阶段目录生成的本轮失败输出已移动到本轮证据，`baseline_evidence_relocation.json` 记录映射，历史证据没有覆盖或删除。

Tag ROS/工具 5 项合计 16.997 s，`time` 统计最大子进程 RSS 179928 KiB；这是整组测试（含 Python 渲染夹具）的资源观测，不是观察器单进程指标或实时吞吐认证。库内 ICP/KISS 不能硬中断，以点数/迭代/线程和返回后预算检查约束；长时、候选洪泛极限和真实尾延迟仍需专项验收。

## 6 已有真实包的只读检查

下文为最初 12 帧抽样快照。后续用户授权的 1367 帧全量处理已完成，最新结果见 [全包报告](fast_livo2_apriltag_fullbag_report.md)：14 次检测/28 支假设、完整 SHA 不变；不将全量检测升级为真实定位验收。

`audit_bag.py` 以 SQLite mode=ro 读取 `bags/fast-livo-motion-w5phdosj/raw/raw_0.db3`，抽样 12 帧覆盖全段，未播放到 ROS 图或启动设备。图像 RGB8、1280×1024，step=3840；原图和 CameraInfo 使用主机历元，image_sync 使用旧雷达历元。1367 条前端位姿无时间回退，中位间隔 99,840,164 ns，约 10 Hz。

用户已确认包内标记 tag36h11:1、0.155 m；第 745 帧检测到该身份，hamming=0、decision_margin≈41.986、面积≈10705.94 px²。按用户尺寸重算得到两支有效解：原始像素 RMSE≈0.47656 / 2.12671 px，最大残差≈0.50299 / 2.22473 px；保留两支。仅据 margin>50 会丢掉这个样本，也不能据 hamming=0 或较低残差宣布地点关联正确。

这只是 **12 帧抽样中的 1 个检测结果**，没有全包人工标签，不能计算召回率或假阳性率。已知尺寸不是固定部署或地图坐标真值。时间来源仍标 false，因此没有把这些观测提交为真实全局定位。

命令、原始消息 SHA、模型、角点、位姿与处理时间见 `sized_image_audit.json`；PNG 和完整抽样输出在 `outputs/fast-livo-apriltag-20261029-sized-audit/`。原 DB 大小/mtime 检查不变；旧 PCL/GICP 地图各 151 个文件 SHA 均通过，manifest 与交接基线完全一致。

## 7 验收分类和下一步

**PASS（软件/合成与明确范围的离线检查）**：构建与来源锁、官方检测器、双解/像素/坐标/异步时间、地标两条生成路径与加载/禁用、C2→同一 C1、B 回环初值、C1 监视/补录记录、无 Tag 回退、旧上下文拒绝、公共锁保持、旧图资产校验。实际数量和命令以最终 results.json 为准。

**PARTIAL / 未验证**：真实曝光/扫描相位与时间偏移不确定性、Tag 固定安装来源、真实地图地标登记、独立访问 Tag 恢复/回环、移动/相似地点/遮挡与动态负样本泛化、真实精度、极限负载/长时和实时接车。当前的多解/多峰/退化测试不是这些真实场景的全面替代。

**FAIL / 阻塞**：最终选定的软件验收无剩余失败；历史试验失败保留并归因。真实 VALID 地标登记被未验证时间/固定性证据限制，不影响已实现的软件/合成路径。

**未实施**：ESIKF 注入、Tag 图节点/因子/绝对先验、视觉–LiDAR 联合优化、激光平面 PnP 生产接入、前端恢复、设备采集、底盘/Nav2 运动、D 实车验收、Git 提交/推送。

最少后续人工输入为 ID 1 的固定安装记录及时间关联证据；之后才能在明确授权的数据范围内补录真实 Tag 表，并用独立访问验收恢复/回环。0.155 m 尺寸已保存，不需要再次提供。启用顺序、停止、禁用和回退命令见接口文档，本轮不执行这些硬件动作。
