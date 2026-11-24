# 代码库精简首批检查

基线 `a7b5ccb5c7b1a7a5825aa5880ca52d664018c963`，分支 `refactor/pluggable-navigation`。首批删除 B 后端消融、P0 回环实验和独立检索对照脚本，共 34 个跟踪文件，包含实验配置和检索来源锁。根说明和文档索引集中展示工作台、构建入口与配套工具。历史报告保留当时的实验路径和结果。本机实验构建、输出和录包沿用此前的 Git 忽略规则。

| 检查命令 | 退出码与结果 | 本机日志 |
| --- | --- | --- |
| `python3 tests/test_project_consolidation.py -q` | 0，10 PASS | `.runtime/cleanup-first-pass-fjsziC.log` |
| `.runtime/navigation_workbench_venv/bin/python -m unittest tests.test_navigation_workbench tests.test_navigation_workbench_reset -q` | 0，62 PASS | 同上 |
| `bash scripts/run_navigation_workbench.sh`；`bash scripts/map_fast_livo2.sh --dry-run`；`bash scripts/record_fast_livo2.sh --dry-run --seconds 60`；`bash scripts/run_calibration.sh sync`；`git diff --check` | 均为 0，入口和差异检查 PASS | 同上 |
| `bash scripts/localize_fast_livo2.sh --archive outputs/fast-livo-demo-changed-scene-20261110/mapping_replay_01/map --navigation-map outputs/fast-livo-demo-changed-scene-20261110/navigation_map_05 --public-tf --continuous --record-navigation` | 0，当前归档及占据图的定位 dry-run PASS | `.runtime/cleanup-localization-plan-IkOCg7.log` |
| `.runtime/navigation_workbench_venv/bin/python -m unittest tests.test_navigation_workbench_ros -q` | 0，合成 ROS 传输 7 PASS | `.runtime/cleanup-workbench-ros-eWCi3a.log` |

合成 ROS 检查加载 Humble、Livox、FAST、base/nav exact 安装前缀，使用 domain 228 及 localhost。实车验收未执行。通用 runtime 的 Lightning 地图包校验仍是共享依赖，旧入口迁移属于下一批。

## 默认配置迁移

同一基线继续实施。通用配置选择 FAST-LIVO2、wheel 与 standard 定位接口。C1/C2 在线会话仍校验所选 FAST 归档与导航地图。Lightning 使用显式 profile，保留自己的外参、初值、地图包和来源校验。桥接包构建和安装到隔离路径，工控机现有安装保留。

| 检查命令 | 退出码与结果 | 本机日志 |
| --- | --- | --- |
| `python3 -m unittest tests.test_project_consolidation tests.test_session_manager -q`；`.runtime/navigation_workbench_venv/bin/python -m unittest tests.test_navigation_workbench tests.test_navigation_workbench_reset -q`；FAST 和 Lightning 建图、工作台、定位、底盘、导航、录包和标定 dry-run | 0，84 项 PASS，入口检查通过 | `.runtime/cleanup-fast-defaults-SE4HAV.log` |
| `colcon build --base-paths ws_nav/src/ranger_localization_bridge --packages-select ranger_localization_bridge`，独立 build/install/log；`ros2 launch ranger_localization_bridge loc_bridge.launch.py --show-args`；`python3 -m unittest tests.test_architecture_contract -q`；`python3 tests/test_stage5_bridge.py` | 0，默认 standard，11 项接口测试与 14 项 Lightning 合成检查 PASS | `.runtime/cleanup-bridge-goIVDL/test.log` |
| `.runtime/navigation_workbench_venv/bin/python -m unittest tests.test_navigation_workbench_ros -q`，domain 228 及 localhost | 0，7 PASS。测试结束时有一条 ROS publisher context 已失效的清理日志 | `.runtime/cleanup-fast-workbench-ros-aWD9uM.log` |

## 空包清理

从 `bags/` 移出 31 个已经删除录制数据的目录和 3 个未产出录制数据的历史失败目录。元数据与日志归档到 `.runtime/backups/obsolete-empty-bags-20261124-193650/`。清单记录原路径、归档路径和文件字节数。仍有代码、配置或回归测试引用的 6 个目录保留原址。工作台定位日志保留以追溯运行记录。

移出后复查退出码 0，配置与校准入口 11 项 PASS，工作台和 FAST 定位 dry-run PASS，现用办公室录包仍有原始 DB 文件。日志为 `.runtime/cleanup-empty-bags-JWD9Q8.log`。

用户随后明确要求删除 `fast-livo-corridor_lab_terrain01-9654ngg5` 与全部 28 个 `workbench-localization-*` 目录。检查 Git 跟踪与进程占用后直接删除，退出码 0，删除前占用约 21.1 GiB。复核目标均已移除，`bags/` 余 24 个目录。清单为 `.runtime/bags-removal-explicit-20261124-iy89la_i.json`。其余 13 个 `fast-livo-localization-*` 录包的元数据起始时间均在北京时间 2026-11-16 晚间。

## 运行目录缓存清理

核对 Git 与进程引用后移除 `.runtime/` 内可重新生成的 Playwright 浏览器、三份临时浏览器配置、测试访问密钥目录和 Python 字节码，共 6 个目录，约 738 MiB。清单为 `.runtime/runtime-cache-cleanup-20261124-i0t21flx.json`。同一来源的旧回放导入包含不同的快照采样，保留历次回放及会话证据。

`bash scripts/run_navigation_workbench.sh` 预演退出 0，两个 Python 环境入口均存在。`.runtime/navigation_workbench_venv/bin/python -m unittest tests.test_navigation_workbench tests.test_navigation_workbench_reset -q` 退出 0，62 PASS，日志为 `.runtime/runtime-cache-verification-KPOiot.log`。

## 文档清理

按用户清单删除 82 个顶层文件和 9 个证据目录，原路径计约 347 MiB。清单为 `.runtime/docs-cleanup-20261124.json`。保留工作台、FAST 与现场验收的现行说明，独立保留 `exact_mppi_evidence/input_latency_20261117/` 及正式前端来源锁所需的对照数据。仍使用的回归输入移至 `tests/fixtures/`，Lightning 构建清单移至 `config/lightning/`。新测试及诊断输出进入 `.runtime/validation/`。

| 检查 | 退出码与结果 | 日志 |
| --- | --- | --- |
| `python -m pytest -q tests/test_exact_coverage_policy.py tests/test_exact_endpoint_diagnostics.py`；`python3 scripts/tests/test_post_exit_feedback.py` | 0，12 项导航回归及停车反馈检查 PASS | `.runtime/docs-cleanup-offline-Ci4NrL.log` |
| `python3 scripts/prepare_lightning_deps.py`；`python3 -m unittest tests.test_project_consolidation -q`；`python -m unittest tests.test_navigation_workbench tests.test_navigation_workbench_reset -q` | 0，Lightning 依赖预演 PASS，73 项配置与工作台检查 PASS | 同上 |
| 工作台、Lightning 建图及 FAST 建图、定位、录包入口预演 | 0，PASS | 同上 |
| `python3 -m unittest tests.test_fast_livo_mapping_startup_clock tests.test_fast_livo_global_evaluation -q` | 0，9 PASS | `.runtime/docs-cleanup-fast-tests-Ww3kKp.log` |

现行入口文档的本地链接、变更 Python 语法与 Shell 语法检查通过。未运行浏览器自动化或实车。

## 历史报告与延迟数据清理

按第二批用户清单删除旧架构、迁移与 FAST 历史报告 29 份，以及四个证据目录和 `exact_mppi_evidence/` 中其余文件。共移除 1113 个文件，释放约 7.18 GiB。`exact_mppi_evidence/` 仅保留正式前端来源锁引用的三份 JSON，原路径与 SHA-256 校验通过。清单为 `.runtime/docs-cleanup-final-20261124.json`。

离线可视化从本机 `.runtime/exact_recorded_inputs.json` 读取已有 NPZ 的最小身份和查询时间戳，119 个输入索引及 SHA-256 核对通过。用已有查询和仿真结果重渲染成功，退出 0，日志为 `.runtime/docs-visualization-check-EfK99t/render.log`。未重跑实录优化和仿真。

配置、工作台与重置共 73 项检查通过。工作台、Lightning 建图和 FAST 建图、定位、录包 dry-run 均退出 0，日志为 `.runtime/docs-final-check-TZ8mOV.log`。
