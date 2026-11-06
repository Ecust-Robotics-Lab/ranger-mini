# 2026-11-05 交接与过时资产清理

用户要求：“编辑交接文件并给我指导新的Agent快速接手项目的提示词。还有，记得把所有过时的文件，记录和录制包删除。”本轮只完成交接、过时记录整理和具体名单的清理；没有继续算法开发、编译/长时回放、传感器/底盘/导航启动、部署、Git提交或推送。

## 当前交接

已重写 [HANDOFF](HANDOFF.md)、[STATUS](STATUS.md)，删除当前入口中堆叠的旧进展和被后续事实取代的状态，保留到日期报告的引用；新增 [NEXT_AGENT_PROMPT](NEXT_AGENT_PROMPT.md)。同步更新AGENTS、根README、文档索引、工程结构和两份总计划的当前摘要。录包指南删去已经完成的120s/两份90s动作命令与“下一次待录”重复说明，改为现有数据/报告入口；采集方案的独立日期记录保留。

新Agent须首先注意：

- 正式前端试验现在是用户指定的 **+100ms**，完整新配置性能对照尚未重跑。旧0/+110ms轨迹、地图和GICP结果保留各自条件。
- Tag默认关闭，独立配置仍offset0/time_verified=false，没有真实VALID表。当前真实C2成功均Tag关闭，不能写成Tag辅助成功。
- 17.32→6.11px是冻结Tag、改变已有查询配准器后的整链回投一致性，不是Tag单独测量精度；初始像素门槛未完成完整误差预算标定。
- 既有受管 `backend_install` 和隔离优化 `install_c1_confirmation_20261104` 不是同一部署；检查实际节点/库。旧图未重力对齐，新局部图使用已对齐+110ms轨迹，逐资产区别。
- 用户已确认当前Tag为tag36h11:1、实测140mm，跨两包固定、传感器安装未变；已有两份90s包，不必重复提问或先要求重录。

本次HEAD仍为 `acffa879cc8d52330276089cd3df83ad6a43f112`，分支 `refactor/pluggable-navigation`，领先本地upstream8提交，未刷新远端/未推送。此前未提交/未跟踪的代码、配置、报告及用户driver证据保留。

## 实际删除

先核对大小、采集来源、代码/报告/配置/地图引用、是否被打开、是否Git跟踪；删除前冻结逐文件字节数、mtime和SHA清单。以用户本次明确授权，将以下三份早期60s试录归为过时数据：当前没有代码/验收报告/地图依赖，后续已由140mm专门诊断和两份分离动作包替代。

| 已删除目录 | 删除前文件总字节 | 判断 |
| --- | ---: | --- |
| `bags/fast-livo-tag_check_01-xxl8kg83` | 见 `delete_plan.json` | 早期未引用试录，非当前评估来源 |
| `bags/fast-livo-tag_check_01-pcnds53t` | 见 `delete_plan.json` | 早期未引用试录，非当前评估来源 |
| `bags/fast-livo-tag_check_01-gbglapyw` | 见 `delete_plan.json` | 早期未引用试录，非当前评估来源 |

**它们是不同时间的原始实采，不是逐字节重复副本。原始图像/点云数据库已删除，没有做完整内容备份，不能靠Git、文件哈希或保留下来的少量元数据恢复。** 保留的仅是本次删除记录、原采集manifest/result/audit、bag metadata及同步CSV，用于解释数据去向，不能据此重放这三包。

另删3个空测试目录：`bags/fast-livo2-stage-b-chain`、`bags/fast-livo2-stage-b-chain-final`、`bags/stage-b-acceptance-i5zzwvdx`；它们没有任何文件内容。另清理 `scripts/`、其fast_livo2子目录和 `tests/` 下共6处未跟踪的 `__pycache__`。

合计删除文件payload **15,139,703,674字节，约14.100 GiB**（包含缓存，非精确磁盘分配块释放量）。`bags/`目录实占约55GiB降至41GiB，实际可用空间变化另见最终核对。删除脚本采用固定白名单，默认dry-run；本次名单已经执行，不能将此历史脚本作为未来通用清理入口。

## 保留的资产与理由

- 两新包turn03/straight04：下一轮固定+100ms的训练/独立采集查询输入，原始数据不可替代。
- 120s同步诊断与60s漏写包：分别保存正常writer/动态诊断和7次漏写/重复戳证据；失败与不完美数据仍有独立复核价值。
- `fast-livo-motion-w5phdosj`：旧地图和A/B/C1/C2历史成果来源；它质量不完美并不使来源失去价值。
- `fast-livo2-baseline-126a5fAo`、DDS视图及静态/手动试验包：有代码回归、标定或阶段报告依赖。其他baseline结果存在不同视觉求解/数值结果，哈希不同，没有假称为重复文件删除。
- 旧两份B地图、新31KF局部图、全部源码/版本锁/许可证、校准资产、失败/对照输出、实际安装/隔离构建与 `.runtime/backups/`：仍用于运行、核对或复现。未清空build/install，也未按日期删除独立历史报告。

本轮删除的过时文字来自当前入口的重复/矛盾状态；独立日期报告不是当前操作指令，也不因时间较早就变成无用文件。原版当前文档、Git状态与完整tracked diff保存在 `.runtime/backups/handoff_cleanup_20261105_171706/`。

## 命令、证据与核对

证据目录：`docs/fast_livo2_evidence/handoff_cleanup_20261105/`。

| 实际命令/检查 | 结果 | 证据 |
| --- | --- | --- |
| `PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 docs/fast_livo2_evidence/handoff_cleanup_20261105/cleanup.py --prepare` | 退出0；冻结3包完整SHA/文件表、3空目录、6缓存 | `prepare.log`、`delete_plan.json` |
| 同脚本 `--execute` | 退出0；固定名单已删除，元数据另存 | `deletion.log`、`deletion_result.json`、`removed_capture_metadata/` |
| `PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 docs/fast_livo2_evidence/handoff_cleanup_20261105/verify.py` | 退出0：239份代码/配置源SHA、405份地图文件SHA、16份保留DB大小/mtime、二进制/配置/会话、当前文档本地链接与Git核对通过 | `verification.log`、`verification.json` |
| `git diff --check -- AGENTS.md README.md docs/HANDOFF.md docs/STATUS.md docs/README.md docs/PROJECT_LAYOUT.md docs/FAST_LIVO2_BACKEND_PLAN.md docs/FAST_LIVO2_APRILTAG_PLAN.md docs/fast_livo2_recording.md` | 退出0 | `whitespace.log` |

本轮不修改算法，无需重复已有123/74/9/11项实验或构建；核对命令不冒充新的算法性能验收。

PASS：交接和提示词已落盘、具体过时试录/空目录/缓存已删除、当前入口过时状态已整理。文件完整性/链接/会话的最终结论见 `verification.json`。

PARTIAL保持：物理曝光参考点/剩余相位、完整+100ms新评估、真实VALID Tag表与Tag辅助恢复/回环、跨场景负样本/真值/实时及D。

未实施：新算法实验、硬件动作、二进制部署、Git提交和推送。此次清理只适用于清单，不自动授权新Agent继续删除其他原包、地图、失败证据或备份。
