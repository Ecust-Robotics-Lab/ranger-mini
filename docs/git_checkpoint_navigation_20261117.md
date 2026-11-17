# 2026-11-17 导航工程分段本地Git检查点

用户明确要求“先将当前更改以git形式分段保存”。本次仅本地提交，未推送、未改写历史、未启动设备或运动。分支`refactor/pluggable-navigation`，起始HEAD`e8245fa80310a4e91b5d2eb303f33883bb965216`。中断后核对已有8个提交，再补文档检查点；不会重复提交。

| 提交 | 内容 |
| --- | --- |
| `51d4011c4dea1528742b24c609e0a7ebd1dfff25` | chore(repo): keep generated replay and experiment assets outside Git |
| `0680555971475f8e8871532df380f4fa7560483a` | feat(mapping): save P0 closure experiments and backend path filtering |
| `49918f3342b8342d7ca6b8ec78af176c90e49dd1` | perf(localization): save online scheduling and stationary initialization refinements |
| `b9310aac0aa80bcbc4c1822c6ed212990617cc60` | feat(navigation): integrate EXACT-MPPI and Ranger V2 motion modes |
| `1902b3a90c2382956e02df53099176fa7a704d03` | perf(navigation): avoid redundant input preparation and visibility reprojection |
| `2a2d51841ed73d9cc891dfaf5d6d7fb93e32c148` | perf(frontend): finish pending VIO without waiting for the next scan |
| `046d707906a28666c72b9f9ad57c3996429dae1e` | test(navigation): retain offline, field and latency validation evidence |
| `7fd7db74eb60d7e5b96e8a177acf96a27f71cfec` | chore(evidence): record prior user-authorized bag cleanup |

最后一个文档提交保存本页、当前报告与STATUS/HANDOFF/AGENTS等入口，其SHA以`git log`现查，避免自引用。本轮共9个提交。

## 分段方式

导航替换与本次延迟优化分开：先将保存的延迟优化前源码放入Git索引，工作文件保持原样；下一提交再保存最终协调器、覆盖处理及测试。因此可直接对比b9310aa与1902b3a。前端等待下一扫描的问题另存2a2d518，仍是隔离候选，未切换正式定位入口。

历史录包清理已有用户授权，本轮只提交已有删除与清理记录，没有再次删除文件。大体积数据库、点云、安装目录、视频、原始冗长日志和profiles继续本地保存；新增忽略规则避免将其混入Git。约80.8MB未跟踪源码/配置/小型证据经分类保存，未新增ELF或SQLite数据库。Git不是原包、地图与二进制完整备份；本机资产统计见`git_checkpoint_evidence_20261117/local_assets_inventory.json`，没有将该统计冒充完整内容校验。

## 底盘补丁完整性

ws_base两个嵌套仓库继续保留实际修改，由主仓库固定HEAD、前置补丁及EXACT增量补丁保存，不额外改写其历史。复核发现ranger_ros2补丁落后于当前ranger_messenger.cpp；旧补丁备份到`.runtime/backups/git_checkpoint_20261117/`，已按原preimage重新生成增量并更新sources.lock中的SHA。SDK增量核对后无需修改。

临时独立Git目录从锁定HEAD导出文件，顺序处理前置补丁（已存在的累计补丁做reverse-check识别），检查所有preimage SHA后应用EXACT补丁，结果逐文件与实际源码相等。见`base_patch_verification.json`和核对脚本。补丁刷新只保存已有实现，未改变底盘工作源码/安装。

## 验证与状态

保存前相关53项检查通过（含物理车头、输入调度、覆盖记忆、延迟审计及P0实验）；原有107项回归、前端原生/回放证据随阶段保存。分组staged whitespace检查通过；CSV的CRLF、原始日志和上游源码空白通过.gitattributes保持。算法构建/长回放未因纯保存操作重跑。

全部源码、配置、小型验证记录与交接入口保存后检查主仓库工作树、提交范围和新增对象。最终检查结果存于`.runtime/backups/git_checkpoint_20261117/final_verification.json`。实车保持暂停，完整出门/坡道/走廊任务仍PARTIAL；来源兼容和真实端到端延迟未通过本轮保存变为已验收。
