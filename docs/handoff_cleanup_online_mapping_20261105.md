# 在线预建图交付后的交接与清理

本轮请求始于2026-11-05，2026-11-06完成。用户要求清理过时记录/成果并准备新上下文。只整理当前交接入口和删除固定名单中的可再生中间视图、Python缓存；未继续算法实施、编译、ROS重放、硬件操作、提交或推送。

## 当前接手结论

在线预建图软件流程已打通，正式结果是`outputs/fast-livo-demo-online-mapping-20261105/full_replay_12/`：390秒真实包原速DDS重放、3900对输出、305关键帧、114接受回环，完整归档与共用读取器加载PASS。地图UUID为`253f573d7a7eabbc363a225ab192cc14`，地图及轨迹图完整保留。

这不等于真机/导航系统验收。此前确有接收延迟，最终整包仍有45个LOST后端心跳；连续新鲜度、真机同时运行、长时、误回环泛化、绝对精度、独立访问C1/C2和wheel/D待验。正式+100ms、Tag默认关闭、无真实VALID地标表的约束不变。

## 删除范围

| 路径 | 删除原因 |
| --- | --- |
| `outputs/fast-livo-demo-online-mapping-20261105/replay_view/` | 已完成联调的三路header-clock中间视图，可从完整原包重建；不属于正式地图或失败证据 |
| `scripts/__pycache__/` | 未跟踪的Python字节码缓存 |
| `scripts/fast_livo2/__pycache__/` | 未跟踪的Python字节码缓存 |
| `scripts/fast_livo2/backend/__pycache__/` | 未跟踪的Python字节码缓存 |
| `tests/__pycache__/` | 未跟踪的Python字节码缓存 |

删除前按文件核对SHA、大小/mtime、无符号链接、无Git跟踪文件及当前同用户进程的可读文件描述符占用。全部删除文件的路径、大小和SHA见[固定清单](fast_livo2_evidence/handoff_online_mapping_20261105/cleanup_plan.json)；实际删除结果见[执行结果](fast_livo2_evidence/handoff_online_mapping_20261105/cleanup_result.json)。清单是本次记录，不是未来通用清理入口。

共删除13个文件、17,432,300,913字节，约16.235GiB文件内容。磁盘可用空间变化以执行结果为准，不把逻辑文件大小冒充精确释放块数。

## 保留与核验

- 所有原始包，包括390秒走廊/教室包、转向/直行两90秒包和故障诊断包。
- `full_replay_12`的地图、完整去畸变云/位姿录包、来源身份、冻结配置、日志、轨迹与图片。
- 旧前端全量PCD、历史+110ms地图、原B失败快照、所有开发失败/中停结果；新结果不追认旧结果。
- 所有源码、未提交修改、标定、隔离构建/安装、依赖前缀及`.runtime/backups/`。

删除前重新读取390秒原DB，SHA为`b6f7902c1b5fbffbaea69bf241efff540d48c0a85358d1267a4f98d468496137`，与交付证据一致。删除后核对保留的bags/outputs共17,363个文件的大小、mtime、模式和链接目标未变；详见[保留资产清单](fast_livo2_evidence/handoff_online_mapping_20261105/protected_inventory_before.json)。正式`MAPPING_COMPLETE`与`mapping_result.success=true`仍在。

被删视图的metadata和manifest另存本清理证据目录，原始[视图来源证据](fast_livo2_evidence/online_mapping_20261105/view_manifest.json)也保留。删除文件内容未做整份备份；仅凭元数据不能恢复SQLite原文件，但完整原包保留，可重新生成相同三路序列化消息内容。

## 重建视图

下面仅供后续明确授权的离线实验使用，本轮没有执行。目标目录须不存在，约需17.4GB空间。再次建图另选新输出目录，不覆盖`full_replay_12`。

```bash
cd /home/lab307/ranger_nav
source /opt/ros/humble/setup.bash
python3 scripts/fast_livo2/make_replay_view.py \
  bags/fast-livo-corridor_classroom_map01-182fdzyj/raw \
  outputs/fast-livo-demo-online-mapping-20261105/replay_view \
  --seconds 400
```

当前脚本只选三路融合数据。预期雷达3903、IMU78068、同步图像3903，逐话题序列化SHA应匹配上述旧manifest；当前manifest可省略旧清单中计数为0的两个话题。SQLite封装字节不要求相同，原始消息header/点时间不得改写。领先于首雷达的一张图像不在视图内，完整原始六话题仍在原包。

## Git与文档

HEAD仍为`4f4977992438dcca6e84c5d2a904d7dea94d1f70`，分支`refactor/pluggable-navigation`，相对本地upstream领先15提交；未联网刷新远端。录包默认行为、走廊包展示、统一在线建图和本次交接的改动均未提交、未推送。

更新AGENTS、HANDOFF、STATUS、NEXT_AGENT_PROMPT和文档索引，删除过时的当前任务优先级及将历史进程快照当现状的措辞。旧带日期报告继续留作证据；在线报告追加视图已清理和须重建的说明，不改历史结果。

修改前文档、完整tracked diff及状态清单保存在`.runtime/backups/handoff_online_mapping_20261105_214554`。这是文档/差异备份，不是全部大资产副本。

最近会话指针指向6秒`eof_case/final_control`回归，状态STOPPED、live=false、owned=[]；不能将其误当正式390秒结果或全机停止证明。2026-11-06清理前只读进程检查未发现相关前端/B/收流/录包/传感器/导航运行进程；新上下文仍须重新核对。

可直接复制的完整提示词见[NEXT_AGENT_PROMPT](NEXT_AGENT_PROMPT.md)。新Agent先接手汇报，再由用户明确下一阶段，不自动续跑旧Tag计划或启动真机。

最终检查：固定白名单清理通过`python3 -`执行，退出0；文档本地链接、代码围栏和非交接源码差异保护检查通过，`git diff --check`退出0。详见[最终核对](fast_livo2_evidence/handoff_online_mapping_20261105/final_checks.json)。本轮是清理与文档修改，没有新增算法测试。
