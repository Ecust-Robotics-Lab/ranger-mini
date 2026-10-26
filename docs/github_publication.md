# GitHub 发布、作者归属与 README

日期：2026-10-26。仓库：[Ecust-Robotics-Lab/ranger-mini](https://github.com/Ecust-Robotics-Lab/ranger-mini)。范围：文档和仓库身份配置，不开展新算法部署或真机测试。

## 作者显示原因与处理

发布时 `main` 和 `refactor/pluggable-navigation` 均指向 `15335452c4c351050e3b3941d69ca39e549f8e9c`。原有 29 个提交的主作者均为 `lab307 <lab307@lab307-Thunderobot-MIX-G2>`；其中 25 个提交消息含 `Co-Authored-By: Claude Code <noreply@anthropic.com>`。

GitHub 的提交 API 对上述最新提交返回 `author: null`，说明本机作者邮箱未关联到 GitHub 账号。经登录账号 API 确认，当前用户为 `ujd66`，ID 为 `193844208`。GitHub 按提交邮箱关联账号，而不按谁执行 push 关联；参见 [GitHub 作者邮箱说明](https://docs.github.com/en/account-and-profile/how-tos/email-preferences/setting-your-commit-email-address)和 [贡献者统计说明](https://docs.github.com/en/repositories/viewing-activity-and-data-for-your-repository/viewing-a-projects-contributors)。

首次处理时仅调整本仓库的配置（后续用户邮箱更正在文末）：

```bash
git config --local user.name ujd66
git config --local user.email 193844208+ujd66@users.noreply.github.com
```

原值为 `user.name=lab307`、`user.email=lab307@lab307-Thunderobot-MIX-G2`，需要时可使用相同命令恢复。未修改全局 Git 身份、其他工作空间或认证凭据。

新增 `.mailmap`，将上述工作站别名映射到 ujd66，供 Git 支持 mailmap 的历史展示使用。它不改变原提交 SHA，不把 Claude 的共同作者身份映射给 ujd66，也不保证 GitHub 会重新计算旧提交的账号归属。新 README 提交使用正确身份，推送后以 GitHub 提交 API 的 `author.login` 核验；贡献者侧栏可能有缓存，旧共同作者仍可能显示。

## 本次文件

- `README.md`：项目用途、硬件/软件、架构与公共接口、目录、源码恢复边界、集成包构建、dry-run 入口、测试与文档导航。
- `.mailmap`：原工作站作者别名。
- `docs/STATUS.md`、本文件：记录本次维护范围及作者归属处理。
- `docs/repository_evidence/20261026/verification.json`：文档链接、shell 语法、默认 dry-run 和作者配置核验。

README 不将克隆等同于完整环境安装：第三方源码、原始 rosbag、构建产物与运行状态不在本仓库中。旧构建脚本的固定版本约束、未完成的硬件同步/标定和真机验收状态均明确保留。

## 验证与结果

- `git check-mailmap 'lab307 <lab307@lab307-Thunderobot-MIX-G2>'`：exit 0，返回 ujd66 的 noreply 身份。
- `git shortlog -sne HEAD`：exit 0，原 29 条主作者记录在 mailmap 展示中归为 ujd66。
- README 本地链接存在性、bash 代码块语法、`git diff --check`：通过，命令和结果见证据。
- `python3 scripts/runtime_config.py`、`bash scripts/field_session.sh`、`bash scripts/run_navigation.sh`：exit 0；后两者返回 `DRY-RUN` 计划，未创建会话。

**PASS：文档、身份配置及默认入口只读核验。** 不复跑与本次文档修改无关的导航回归；已有测试及其日期在 README 和原报告中保留。推送仅作正常快进更新，不改写历史。

本次无需用户人工操作。已有 `docs/03_mid360_validation.md` 的独立 IP 修改及六个未跟踪雷达日志目录保留，不纳入本次提交。

## 用户邮箱更正（2026-10-26）

README 提交 `b0dc898ef0f98012c259fffc4aa3b73e56756652` 发布后，GitHub API 已核验 `author.login=ujd66`、`committer.login=ujd66`。随后用户明确提供邮箱 `qzy154846@163.com`，因此将本仓库 `user.email` 改为该值，`user.name` 保持 ujd66；`.mailmap` 同时将原工作站邮箱和此前 noreply 邮箱映射到此身份。原始提交内容和 SHA 不变，上方首次核验记录保持原时间语义。

```bash
git config --local user.email qzy154846@163.com
git check-mailmap 'lab307 <lab307@lab307-Thunderobot-MIX-G2>'
git check-mailmap 'ujd66 <193844208+ujd66@users.noreply.github.com>'
git diff --check
```

上述配置、别名与差异核验均 exit 0。邮箱关联查询 `gh api user/emails` 返回 `Not Found`（exit 1），当前凭据不能据此核验邮箱关联状态，不能将它解释为邮箱未关联。新提交推送后另核对 GitHub 提交作者；若仍未关联，需用户在 GitHub 的邮箱设置中添加并验证该地址。

## 按用户指令移除 Claude 共同作者（2026-10-26）

用户随后明确要求删除 Claude 共同作者标记，因此本节执行历史改写；上文“不改写历史”“保留共同作者”是此前操作的历史记录，此后以本节为准。

- 完整改写前历史已保存于本机 `.runtime/backups/coauthor-removal-20261026T035430Z/before.bundle`，`git bundle verify` exit 0；此备份不推送到 GitHub。
- 遍历 31 个提交，仅删除提交消息中的 25 条 Claude 共同作者尾注，并更新后代的 parent 引用。逐个验证 tree SHA 完全相等，其余原始提交头（含 author、committer 及日期）保持原值。
- 当前工作分支、旧本地 main 分支均更新到对应的新历史；代码、地图、录包、嵌套仓库与无关工作区修改不变。
- 改写前顶端为 `18aa1f3be689ba814ba35589ae9418384e714407`，对应的新顶端为 `ee80fe2adee00f68893e67203adb370c01a398e3`；之后另加本次维护记录提交。
- 全量新旧 SHA 映射、备份哈希、树一致性和无关工作区文件哈希见 [改写证据](repository_evidence/20261026/attribution_rewrite.json)。已有阶段报告中的 SHA 保留为当时事实，查询当前历史时需使用此映射；本次不改写旧证据内容。
- 远程 main 与 refactor/pluggable-navigation 使用原顶端的显式 `--force-with-lease=<ref>:<old-sha>` 和 `--atomic` 一并更新；若远端已被其他人更新则拒绝覆盖。

检查命令包括 `git cat-file commit`、`git hash-object -w -t commit --stdin`、逐提交 `git rev-parse <sha>^{tree}`、`git diff --check` 及提交消息尾注扫描，均通过。**PASS：共同作者尾注移除及文件树一致性核验**；没有修改作者邮箱的历史记录，没有重跑无关导航测试或启动硬件。其他克隆需要同步改写后的历史，GitHub 贡献者侧栏可能稍后刷新。

用户提供的邮箱提交在 GitHub 上返回 `author: null`、`committer: null`；后续仍使用用户指定的 `qzy154846@163.com`。用户应在 GitHub 邮箱设置中添加并验证该地址，才能让这些提交关联到账号；删除共同作者与邮箱关联是两个独立操作。
