# 2026-11-05 本地分组提交检查点

用户在只读接手汇报后要求“先分段提交一下本地git”。本轮只保存既有源码、配置和证据，并同步当前交接元数据；没有继续算法实施、构建、ROS重放、设备操作、部署或推送。

## 提交范围

分支 `refactor/pluggable-navigation`，分组前HEAD为 `acffa879cc8d52330276089cd3df83ad6a43f112`，当时领先本地upstream 8提交。原暂存区为空，工作树有39个已跟踪文件修改、2957个未跟踪文件。本次新增7组本地提交，未联网刷新远端；当前完整HEAD用 `git log -1 --format=%H` 查询，避免文档自引用其自身提交SHA。

| 组 | 提交 | 内容 | 文件数 |
| --- | --- | --- | ---: |
| 1 | `d1af7000bea8a2cefa405beff1f34947d537fa53` | 两份新增原始雷达driver证据，单独保留 | 12 |
| 2 | `9725bd585d9b4a36dc232d465ee31a18f4a126ce` | writer可靠订阅/DDS诊断、录包就绪/报错、既有回归证据与产物忽略规则 | 1143 |
| 3 | `7773982d8587afd3eed201c4d30f86faaa5eb395` | AprilTag检测/双解/时间/地标、C2/B候选与C1监视、既有软件验收 | 1168 |
| 4 | `d1b7f2f123d00bce7aac4c19c798ed4fd61e4044` | 140mm检查、同步/运动几何、Tag关闭的跨包C2及离线评估诊断 | 252 |
| 5 | `30494ad17c0270f6af3035bf5cabea60ca8501ce` | C1等待确认计算优化、两包地图/C1修正及既有GICP对照 | 342 |
| 6 | `71034a087fbf2b405ca394a616ea13f5c863bc25` | 正式前端+100ms参数、接口与扫描相位证据 | 19 |
| 7 | 本报告所在提交 | 交接/状态/计划/索引、历史清理记录和本地提交记录 | 52 |

共2988个文件路径归档。历史报告中的旧HEAD、未提交状态、失败结果与当时参数保留原义，没有改写为当前配置的验收结果。正式前端 `img_time_offset=0.1`；完整+100ms建图/定位仍未重跑。历史+110ms地图/6.11px结果不追认；Tag仍默认关闭、独立时间未确认、没有真实VALID地标表。

## 资产和会话保护

- 2996份原改动/未跟踪文件全部保留；仅对 `.gitignore` 和8份当前入口文档作本次元数据更新，其余2987份文件SHA256保持。另新增本报告。
- 检出9份证据目录内的ELF归档副本，合计5,901,648字节；保留本机文件并按三种明确程序名加入忽略规则，没有删除。证据manifest及其来源SHA照常提交。原包、点云/CDR/数据库、工作空间构建安装、SDK、地图和 `.runtime/backups/` 保持原有资产边界。
- 接手与提交阶段均未启停会话。只读接手时FAST live/record为STOPPED，B/C1/C2历史状态FAILED，五者均不存活且owned为空；另一个field会话 `run-u0m5bw9a` 存活、DEGRADED，并有Lightning、底盘驱动及公共桥接进程。旁边 `ranger_nav_k_integration` 有雷达时间归一化和Nav2相关进程，ROS_LOCALHOST_ONLY=0。该观察不等于车辆运动、导航就绪或新时间方案已经验收。
- 提交前备份在 `.runtime/backups/local_git_checkpoint_20261105_173931/`：`baseline.json`冻结2996份文件SHA/大小；`worktree_before.patch`保留tracked diff；`original_files/`保存当前入口原文；`commit_plan.json`及逐组日志记录具体路径和实际操作。它不是所有被忽略大资产的完整内容备份。

## 实际检查与结果

| 实际命令/检查 | 退出码与结果 | 本轮备份内证据 |
| --- | --- | --- |
| `git status --short --untracked-files=all`、`git diff --binary HEAD` | 0；冻结分组前状态，无既有暂存内容 | `git_status_before.txt`、`worktree_before.patch` |
| 对2996份候选文件进行凭据模式和文件魔数检查 | 0；未发现所检查类型的凭据，9份ELF留在本机 | `commit_plan.json`中的排除清单 |
| `ast.parse`检查28份当前scripts/tests Python文件，`bash -n`检查新增shell入口，解析4份新增JSON配置 | 0；静态检查通过，未执行脚本业务或启动ROS | 提交前工具执行记录 |
| 每组 `git add --pathspec-from-file=- --pathspec-file-nul`、精确比对暂存路径并检查无暂存后变动 | 0；按固定名单分组，未使用 `git add .` 或强制加入被忽略资产 | `commit_plan.json`、逐组提交记录 |
| `git diff --cached --check -- . ':(exclude)docs/fast_livo2_evidence/**' ':(exclude)docs/03_evidence/**'` | 各组0；当前源码/配置/文档检查通过，历史原始日志不重排或改空白 | `group_*_whitespace.log` |
| 各组 `git commit -m TITLE -m BODY` | 各组0；原作者配置保持，无推送 | `group_*_commit.log`、`commits.jsonl` |
| 提交前完整性核对 | 0；239份源码/配置、405份地图文件SHA保持；2份主程序SHA保持；16份保留数据库大小/mtime保持；原2996份文件无缺失 | `integrity_before_final_commit.json` |
| 收尾 `git status --porcelain=v1 --untracked-files=all`、核对7组路径全集与本地upstream差异 | 0；工作树/暂存区干净，本地upstream差异0/15；9份ELF仍在且被忽略 | `final_verification.json` |

PASS：本地分组保存、静态检查、路径与内容完整性及交接状态同步。PARTIAL保持：物理同步、完整+100ms评估、真实Tag地标/恢复/回环、真实负样本/真值/实时长时及D。未实施：新算法实验、硬件/导航动作、部署和推送。

下一步仍须收到实施指令后，复用已有turn03/straight04两份90秒原包，固定+100ms完整三融合另存对照，按新轨迹需要创建新地图版本，再处理Tag时间与真实地标/恢复/回环验收；当前无需重录。
