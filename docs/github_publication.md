# GitHub 发布、作者归属与 README

日期：2026-10-26。仓库：[Ecust-Robotics-Lab/ranger-mini](https://github.com/Ecust-Robotics-Lab/ranger-mini)。范围：文档和仓库身份配置，不开展新算法部署或真机测试。

## 作者显示原因与处理

发布时 `main` 和 `refactor/pluggable-navigation` 均指向 `15335452c4c351050e3b3941d69ca39e549f8e9c`。原有 29 个提交的主作者均为 `lab307 <lab307@lab307-Thunderobot-MIX-G2>`；其中 25 个提交消息含 `Co-Authored-By: Claude Code <noreply@anthropic.com>`。

GitHub 的提交 API 对上述最新提交返回 `author: null`，说明本机作者邮箱未关联到 GitHub 账号。经登录账号 API 确认，当前用户为 `ujd66`，ID 为 `193844208`。GitHub 按提交邮箱关联账号，而不按谁执行 push 关联；参见 [GitHub 作者邮箱说明](https://docs.github.com/en/account-and-profile/how-tos/email-preferences/setting-your-commit-email-address)和 [贡献者统计说明](https://docs.github.com/en/repositories/viewing-activity-and-data-for-your-repository/viewing-a-projects-contributors)。

仅调整本仓库的配置：

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
