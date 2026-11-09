# 2026-11-09 重定位优化本地保存与上传

用户明确要求“先提交本地git，然后上传”。本次分支`refactor/pluggable-navigation`，远端`origin`为`https://github.com/Ecust-Robotics-Lab/ranger-mini.git`。

起始HEAD为694b8b6d463f07a340b2c66b6b5cab4dd2938668。`git fetch origin`退出0，核对相对远端为落后0/领先8，无分叉。本轮上传包含这8个先前本地提交及以下新增提交，不改写历史。

| 提交 | 内容 |
| --- | --- |
| `509a1d4649edc5fd6b2f6a8cf8b5de552eb24d50` | 修复后90秒实机短测及首次定位耗时分析 |
| `0acec1bd348aa1b60616fdbdb87effaffc3014a3` | C2/C1静止强几何确认、配准预处理复用及测试/契约 |
| `176c31dec1e8fd9b21146d8cdf9417ba1305ff10` | 初始化原速对照、成功/失败记录与回归证据 |

最后另以docs提交保存STATUS/HANDOFF/AGENTS、NEXT_AGENT_PROMPT及本检查点；其SHA以Git现查为准，避免自引用。

## 范围与核对

起始状态、完整tracked diff、起始HEAD保存在`.runtime/backups/git_upload_initialization_20261109/`。125份新文件约37.7MB，经文件头核验无ELF或SQLite数据库；源码、参数、实际命令、日志、报告及失败对照均保存。原始bags、正式地图、标定、本地二进制和`.runtime/backups`保持本机现有忽略规则，Git上传不等于完整资产备份。

沿用本轮已通过的40项C1/C2核心、21项后端及9项ROS检查；保存/上传未重跑算法或启动设备。分组staged whitespace检查通过，原始日志按既有属性保留原文。

上传使用`git push origin HEAD:refs/heads/refactor/pluggable-navigation`，无force。完成后核对本地HEAD、远端分支SHA和工作树，结果写入本轮最终回复及`.runtime/backups/git_upload_initialization_20261109/upload_verification.json`；此文件在提交前不预先宣称网络上传成功。

最新功能与限制见[首次定位优化报告](fast_livo2_initialization_optimization_report_20261109.md)。本轮只保存与上传，不启动设备、运动或删除资产。
