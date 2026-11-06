# FAST-LIVO2 在线定位与自动重定位首轮交付

日期：2026-11-06。用户要求检查新包后立即开发，优先快速复用既有实现。本轮完成私有在线入口、C1/C2实时调度、隔离构建、原速重放及相关回归。没有启动实物设备、底盘、Nav2或公共TF，没有提交/推送。操作见[在线定位指南](fast_livo2_online_localization.md)，授权与顺序见[计划](FAST_LIVO2_ONLINE_LOCALIZATION_PLAN.md)。

## 实现

新入口 `scripts/localize_fast_livo2.sh` 默认dry-run，显式execute支持auto/manual、已有地图、原速header-clock视图、新输出目录、状态/停止及RViz。复用受管设备/时钟/收流/录包机制和原C1/C2私有消息服务；C2确认后仍经C1独立验证，定位模式不启动B或新建地图。实物模式入口已经具备，但本轮未执行实物。

原localizer节点新增默认false的online_scheduling；新入口启用后只保留最新两对待处理完整扫描，不因新帧反复取消当前求解，控制任务保留原代际。旧几何确认结果只作内部支持，提交时超龄则保持无效，待新鲜扫描重新配准后才发布VALID；完整歧义和原多帧门槛保持。新诊断记录处理、跳帧、过期结果和采样年龄。原离线入口行为保持。

来源身份计算从建图入口提取为小型共用函数，既有身份哈希不变；由实际查询程序/配置生成期望身份，通过共用读取器加载正式地图。监视器复用既有固定雷达header/steady锚点和接收计数，增加私有定位结果观察；业务搜索/LOST与进程存活分开。另提供已有Initialize服务的人工初值helper，以及静态地图/已确认轨迹显示，无公共TF。

资源占用回归时发现：旧入口在资源锁拒绝之前就备份共享前端日志。本轮增加小型prepare_hook，日志备份在取得资源锁后执行，避免被拒绝的会话触碰正在运行的共享日志；所属清理和原资源锁不变。

最终完整性核对还发现原入口的finally使用session_live检查，而返回后的guardian正是仍存活的调用者，导致固定前端日志恢复被跳过。已改为检查guardian_cleanup_complete及所属子进程全部结束，再保存after快照并恢复before。保存本轮末次共享日志到备份后，按首次smoke_01的原文件SHA恢复三份原日志；final_cleanup_check再次完整原速EOF通过，existing_logs_restored=true。较早运行未生成逐次after固定日志快照，不能补造；其完整原始录包和session frontend.log仍保留。

## 构建与来源

源码基线HEAD `e8f1317a519cf08919691bf4c09643fd7e3700f7`，分支 `refactor/pluggable-navigation`；接手时工作树干净。本轮捕获核验和开发文件未提交。修改前节点及diff保存在 `.runtime/backups/online_localization_20261106_150344/`。

实际构建命令 `bash scripts/build_fast_livo2_online_localization.sh --execute`，退出0，约56.9秒。C1/C2从当前源码安装到 `ws_fast_livo/online_localization_install`；开启既有KISS，复用 `online_install` 后端和项目内GTSAM/small_gicp/可选全局配准依赖，不升级系统或替换旧安装。编译器提示backend使用既有online_install，符合本次隔离设计。

正式前端仍是 `online_frontend_install` I/O派生版本，正式+100ms、初始重力对齐开启、Tag关闭。实际进程二进制/库SHA见 `fast_livo2_evidence/online_localization_20261106/runtime_linkage.json`，确认载入新的localizer core与online backend core。查询三项来源身份与正式地图一致，归档305KF/1829386点由新进程成功校验。

## 原速真实数据结果

地图固定为 `outputs/fast-livo-demo-online-mapping-20261105/full_replay_12/map`，UUID `253f573d7a7eabbc363a225ab192cc14`、graph419。结果根目录为 `outputs/fast-livo-demo-online-localization-20261106/`；不覆盖地图、原包或旧失败结果。

| 项目 | 新282秒独立采集访问/PCL | 390秒建图同包/PCL |
| --- | ---: | ---: |
| 输出目录 | visit01_auto_pcl_01 | corridor_auto_pcl_01 |
| 会话/录制正常收尾 | PASS，退出0 | PASS，退出0 |
| 不同扫描的有效定位数 | 2309 | 2978 |
| 心跳状态 | INITIALIZING 2749、VALID 11552，DEGRADED/LOST/ERROR 0 | INITIALIZING 1750、VALID 14975、LOST 2998，DEGRADED/ERROR 0 |
| 有效心跳占比（包含初始化） | 80.78% | 75.93% |
| 有效位姿接收年龄P50/P95/max | 0.253/0.312/0.434秒 | 0.273/0.373/0.494秒 |
| 接收时有效位姿年龄超过0.5秒 | 0 | 0 |
| 有效配准耗时P50/P95/max | 74.63/100.07/134.95ms | 82.52/109.16/160.21ms |
| 待处理队列最大项数 | 2 | 2 |
| 已尝试处理/过载跳过/过期结果抑制 | 2791/26/1 | 3513/376/6 |

处理数统计worker扫描尝试，跳过数统计待处理队列被新扫描替换；会话重置还会清除旧任务，因此不能把这些数相加声称所有扫描都成功处理。心跳数也不代表独立定位测量。

新包无人工初值、Tag关闭，首次VALID在视图源时间+51.494秒，之后持续有效至结束。新包三路重录输入完整payload SHA与视图一致：2821雷达、56427 IMU、2821同步图像；前端重新生成2817对严格同戳位姿/云，区别于原录包旧前端的2821对。只重排派生bag调度时间，原消息不改。

用户追问冷启动后，进一步只读拆解见startup_latency_analysis.json：源时间约0.445秒已开始积累C2查询，约0.535秒出现首个无几何匹配结果；前端相对首帧累计平移0.05/0.15米分别在18.900/19.500秒达到（估计运动，不是真值）。C2确认初值的首次已发布状态在49.514秒，C1首次有效在51.494秒，交接后约1.980秒；两次较重初始化配准约998/929ms，最后新鲜跟踪复核约63ms。

首次VALID前可观察到53份不同查询戳的候选诊断，共265个候选，247个未通过粗配准、18个通过粗配准且通过精配准。单次查询记录的实际耗时多数约50～200ms、最大约323ms；首轮部分候选只有10～11个粗配准内点，低于既有30要求。诊断不是所有尝试的完整逐次trace，不能据此报告检索Recall或断定正确地点未进入top5。当前实现几何失败会清空支持并退避0.5秒；早期偶发匹配难以积累确认，直到后续位置/视角才完成C2。因此约51.5秒包含前段低运动与反复候选失败/支持不足，不能解释为一次求解耗时或仅由静止等待造成。优先待查候选排序、粗配准点云/特征及支持重置原因，不通过放宽几何门槛掩盖问题。

用户声明新访问从办公室出发，绕走廊后回到办公室，起终点与建图访问一致。地图XY叠加 `visit01_map_overlay.png` 的已确认轨迹从走廊上段开始，沿走廊访问并返回办公室区域，与该粗粒度描述基本一致。首次确认前没有伪造地图定位轨迹。图是估计结果，不能作为独立真值或绝对精度证明；约51.5秒的启动等待仍是需要后续优化的限制，不作普遍恢复时延承诺。

390秒检查期间出现新鲜度失效，随后多次完成C2重新搜索→C1验证→恢复跟踪。最长无效心跳跨度约37.82秒（含各类等待）；这项功能有原速证据，但连续定位仍PARTIAL。没有为消除LOST放宽0.5秒门槛或重打now。同包验证不计独立访问泛化。

实际整包命令：

```bash
bash scripts/localize_fast_livo2.sh --execute \
  --archive outputs/fast-livo-demo-online-mapping-20261105/full_replay_12/map \
  --replay-view outputs/fast-livo-demo-online-localization-20261106/inputs/visit01/view \
  --output outputs/fast-livo-demo-online-localization-20261106/visit01_auto_pcl_01 \
  --visualization none --seconds 400 --min-free-gb 10 --domain 215

bash scripts/localize_fast_livo2.sh --execute \
  --archive outputs/fast-livo-demo-online-mapping-20261105/full_replay_12/map \
  --replay-view outputs/fast-livo-demo-online-localization-20261106/inputs/corridor/view \
  --output outputs/fast-livo-demo-online-localization-20261106/corridor_auto_pcl_01 \
  --visualization none --seconds 500 --min-free-gb 10 --domain 215
```

这些目录已存在，命令是历史执行记录，复现须使用新的输出。完整replay_command.json及冻结配置在各结果目录。

## 回归、故障和剩余验收

证据目录 `docs/fast_livo2_evidence/online_localization_20261106/`：32项现有C1/C2核心检查、3项新增在线ROS检查、6项相关C1/C2 ROS回归、11项会话管理检查通过；资源锁prepare_hook用例另重跑一次，不累加为新独立用例。新增ROS检查覆盖过载完整配对替换、过期撤销与人工重初始化、慢初始化期间旧位姿不能发布；原ROS回归覆盖取消/恢复、切图旧支持、时钟/多发布者和非法输入。

8秒三融合短包端到端正常EOF退出0；独立domain217的显示检查收到182939个显示点，只接纳确认位姿生成轨迹，无公共TF/速度发布。显示采样不修改源地图。390秒过程一次资源快照：localizer VmHWM约405MiB、前端约1259MiB，属于进程当时高水位，不是整机总内存或长期峰值认证。

人工模式另用新访问前70秒视图，在源时间约30秒通过原Initialize服务给初值，确认后以统一入口--stop正常收尾；9个不同有效扫描、0个接收年龄违规。该初值由此前自动结果与同次访问前端相对运动推导，明确仅验人工接口/调度功能，不计额外独立全局初始化证据。原始请求、确认质量与命令见manual_check.py、manual_seed.json、manual_result.json和manual_summary.json。共用建图入口待定位会话结束后，另一个8秒三融合/B/封存回归在mapping_regression_02正常退出0，保留原建图默认行为。

开发失败保留：首轮在线测试复用了不同载荷的初始化request_id，被既有规则正确拒绝，修正测试ID后3项通过；首个入口dry-run因同名Python模块导入冲突失败；首轮短包监视器误将rclpy的nanoseconds属性当函数，进程退出，修正后新目录完整EOF通过；建图短回归与定位同时运行时被共享资源锁拒绝，没有绕锁。上述不追认为旧轮通过。

旧建图45个LOST心跳只读分析：reason均为pose_or_stream_stale，最近前序记录时钟对应位姿年龄约0.535～0.886秒、附近后端队列最大6项。结合既有原速输入到达证据，支持处理/发布积压导致过龄的判断，但时钟/健康并非原子采样，不能证明唯一根因或推断传感器停发。

PASS：首版代码/隔离构建、来源与归档加载、原速软件流程、新独立访问无初值C2→C1功能、过期拒绝与相关回归。PARTIAL：冷启动速度、390秒连续新鲜度、独立地点/真实负样本/长期泛化、物理同步、绝对精度、实物运行、wheel/公共桥接及D。未实施：真实Tag、原地图更新、前端自动恢复、导航运动、生产部署和Git提交推送。

收尾检查见证据目录final_checks.json和command_results.json：Python AST、JSON、shell语法/文档链接及git diff --check通过；正式地图919文件SHA和COMMITTED匹配，17,363个旧保留资产属性未变，原前端/C1二进制和初始共享前端日志保持，新原包大小/mtime保持。建图/录包/在线定位会话均STOPPED、live=false、owned=[]。这些是完整性检查，不增加算法验收样本。
