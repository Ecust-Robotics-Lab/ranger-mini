# 当前项目状态

更新：2026-11-05。**软件主体已交付，真实Tag辅助定位/回环和总计划验收尚未完成。** 快速接手见 [HANDOFF](HANDOFF.md)，阶段/参考架构见 [总计划](FAST_LIVO2_BACKEND_PLAN.md)，Tag细化见 [增量计划](FAST_LIVO2_APRILTAG_PLAN.md)。日期报告只描述其当时条件。

## 当前配置与工作树

- 正式FAST-LIVO2试验 `img_time_offset=+0.1 s`，由用户明确要求，原包/驱动header不改；曝光中心/剩余相位未实测。**只完成参数加载/相关回归，尚未重跑完整+100ms建图/定位对照**。上游配置、作者解释与两包检查见 [扫描相位报告](fast_livo2_timestamp_phase_report.md)。
- Tag默认关闭；独立诊断配置仍 `offset_ns=0, time_verified=false`，没有真实VALID地标表。启用前明确其原图映射，与前端保持采样语义一致且不重复补偿。
- 生产仍为Lightning建图/定位、wheel连续里程计、terrain disabled；Nav2/Smac/MPPI与安全链保留。C1/C2是独立离线路线。
- 分支 `refactor/pluggable-navigation`；`acffa879cc8d52330276089cd3df83ad6a43f112` 是本次分组前基线。2026-11-05按用户指令将AprilTag、同步/采集、C1、诊断及交接分为7组本地提交，未推送；当前HEAD以Git现查，详情见[本地检查点](local_git_checkpoint_20261105.md)。原包、地图和被忽略资产继续保留，后续未提交修改仍须保护。
- 本轮完成交接/清理、只读接手核对及本地分组提交，没有运行新算法实验、部署或操作硬件。FAST live/record STOPPED，B/C1/C2历史故障测试FAILED，五者均不存活、owned=[]。另有field会话存活且DEGRADED，旁边 `ranger_nav_k_integration` 的雷达/Nav2相关进程也在运行；未干预或据此验收导航，新会话前重新现查。

## 能力与验收边界

| 项目 | 已有证据 | 当前限制 |
| --- | --- | --- |
| A 三融合前端 | 锁定源码/补丁、本机构建、静态/真实运动重放与历史在线预览 | 当前+100ms完整重放待做；物理同步、标定/真值精度待验 |
| B 独立回环PGO | 真实旧包PCL/GICP各1362对、49KF/13接受回环；两包转向另建31KF局部图 | 多环境误环、真值、实时/长时未验 |
| C1 先验定位 | 六自由度初值确认、持续跟踪、健康/失效/会话与桥接检查 | 新计算优化仅隔离安装，真实跨场景与D未验 |
| C2 全局重定位 | 无人工初值候选/粗细配准/多帧确认/恢复，已有不同采集包→C1功能试验 | 完整C2-4、真实相似地点/地图外负样本、绝对位置精度未验 |
| Tag增量 | 官方检测、双解/像素/时间契约、地标保存/补录、C2/B候选及C1监视，首轮123项软件检查 | 所有已报真实C2成功均Tag关闭；真实VALID表、Tag恢复/回环未验 |
| D 系统实车 | 原Nav2/公共链保留 | 本轮未实施；真实wheel历元、T_base_imu、接车/导航另验 |

## 最近证据与正确解读

- **正式+100ms修正**：[扫描相位报告](fast_livo2_timestamp_phase_report.md)。两新包各900扫描，单帧约99.83～100.33ms；1843相机会话帧均读最新writer，未复现旧漏写。正式配置由0改0.1；11项既有回归、实际参数加载和录包dry-run通过，不等于新性能结果。
- **+110ms C1/GICP对照**：[C1修正报告](fast_livo2_c1_correction_report.md)。525扫描×4控制；原PCL单初值524帧复现。冻结原Tag常量、同516跨访问观测，回投RMS P95约17.324→6.114px。完整直行895提交/894处理/1取消、520有效、DEGRADED 1、无LOST/ERROR，首VALID37.400s。仅显式选既有small_gicp，门槛与生产默认保持，新增6项独立/3项相关检查通过。
- **两包/地图对照**：[两包报告](fast_livo2_tag_two_capture_report.md)。转向训练冻结+110ms，直行独立采集验证；新图31KF/10LiDAR回环。直行PCL C2→C1首VALID37.000s、525有效；C1等待帧计算和评估取消记录已修复，74项相关检查通过，仅隔离安装。
- **早期归因证据**：[运动几何](fast_livo2_tag_motion_geometry_report.md)、[120秒诊断](fast_livo2_tag140_sync_diag02_report.md)、[旧60秒writer漏收](fast_livo2_tag140_consistency_report.md)保留，各自偏移/源码/数据条件不可混用。
- **本次交接/清理**：[20261105报告](handoff_cleanup_20261105.md)。删除3份未引用的过时早期试录、3空目录及6处缓存，约14.1GiB；实际名单/哈希留档。下一步需要的原包、地图、失败证据、源码和备份保留。

上述像素误差是整条定位/地图/标定/时间/Tag观测链的一致性，不能称Tag单独精度或绝对定位误差。初始3px RMS/6px max尚未完成误差预算标定，小幅残差本身不等于Tag不可用。各轮测试有重叠，123/74/9/11不相加为独立验收总数。

## 当前数据与运行版本

当前Tag为tag36h11:1、实测140mm黑色边界，Tag与相机/雷达安装在两新包内和之间固定，用户已确认，无需再问。旧155mm实例单独保留。

- 下一步数据：`bags/fast-livo-tag140_turn_03-bk8_pjyy/`、`bags/fast-livo-tag140_straight_04-ryzykq2l/`；另保留120秒诊断、60秒漏写原证据、旧建图包及测试固定基线，具体用途/SHA见HANDOFF。
- 当前新局部图：`outputs/fast-livo-tag140-two-capture-20261104/new_local_map/snapshot`，UUID `4ec68a99c626e1c6e2652506f5ecd8d6`，由+110ms转向轨迹生成，有重力对齐，非整层/真值地图。
- 既有受管默认为 `ws_fast_livo/backend_install/`；C1优化版本为 `ws_fast_livo/install_c1_confirmation_20261104/`。二者不同，不能通过读取源码便声称已部署；实际节点和库SHA见HANDOFF/报告。
- 后续可复用 `independent_time_source_20261104/`、`build_independent_time_20261104/` 的原零gyro初始化隔离来源；旧mean-bias诊断副本不混用。当前+100ms参数不自动更新任何已运行旧会话。

## 下一步与用户待办

本轮完成接手核对及本地分组提交；算法实施仍待新指令。继续时先用现有两包固定+100ms完整三融合重放、与0/+110ms同集合对照，必要时创建新地图版本。保留已有GICP对照和原安全/几何/时间门槛，核对版本、采样戳与地图身份；再处理剩余不确定性、真实地标登记、Tag C2/负样本及B回环。

当前无需重新录同类包。后续新盲测访问、特定曝光/触发测量、整层地图和真实wheel/系统测试如确有需要，须说明具体目的并遵循该次硬件/运动授权。未获独立真值时不报告APE/RPE或绝对精度。

旧阶段的完整命令、许可、失败和资产记录见 [文档索引](README.md) 及带日期报告。本次精简当前入口，历史报告仍有复核用途，不按日期一刀切删除。
