# 工作台路径前段覆盖与2.5cm统一口径

软件、保存输入反事实计算、GPU、独立几何、隔离ROS和浏览器PASS；新实车导航、坡道与办公室走廊完整往返仍PARTIAL。没有启动设备、操作CAN、发送实车目标或非零指令，没有提交/推送。

## 原因与改动

用户遇到`guidance_unobserved`，认为覆盖足够，要求放宽少量未知格，并指出总外扩应为2.5cm。此前仅静态地图更新为1cm净空+1.5cm扫掠界；实时覆盖仍用`body.clearance=.04`加`.015`，实际5.5cm，运行口径没有统一。

工作台记录`.runtime/navigation_workbench/records/live-1c390df5530f49a1bb7d1956ee06ee4f/events.sqlite3`显示多个目标覆盖失败。保存全局请求/结果来自`.runtime/exact_navigation/run-xxxnlzlh/`，id=4、generation=18、nonce=c9efbf108ef24bf5b88402ea6af9a6e5，起点约(0.329,-0.059,0.021)、目标(1.709,0.196,0.349)。当时定位有效、GPU就绪，但未提交局部控制请求；80×80/5cm观测网格仅因车体外侧4个未知单元，在旧5.5cm包络下被拒绝。不是这份输入的点数或GPU运行时延造成。

匹配请求/结果复制到[证据目录](workbench_evidence/20261117/coverage_margin/)，原文件保留；`baseline.json`记录任务/会话，`original_request.npz`/`original_result.npz`可复现。[覆盖对照图](workbench_evidence/20261117/coverage_margin/coverage_comparison.png)从该输入绘制，绿线是离线候选轨迹，不是实际执行轨迹。

新增`CoveragePolicy`，正式`exact_navigation.yaml`的`coverage`由launch同时传给协调器和独立门禁，工作进程使用显式CLI参数，并在READY状态回报实际策略。

| 检查项 | 当前行为 |
| --- | --- |
| 静态占据图总外扩 | 2.5cm：1cm静态净空+1.5cm扫掠界，本轮保持 |
| 实时观测覆盖总外扩 | **2.5cm，已含扫掠界**，不再叠加4cm或1.5cm |
| 必须观测范围 | 完整0.74×0.52m车体+1.5cm扫掠保护范围 |
| 少量未知格 | 每个位姿最多8格，仅在保护范围外、总2.5cm内的外侧1cm带；按完整单元相交判断 |
| 已知障碍、越界 | 不受未知格额度豁免；保护范围未知/整车未知仍拒绝 |
| 点障碍裕量 | 本轮保持：standard净空4cm+4cm体素误差+1.5cm扫掠界=9.5cm；narrow为8.5cm，与覆盖分开 |

路径前段、JAX预测、独立NumPy全量障碍/扫掠/制动复核、末端门禁使用同一覆盖策略。静态未知仍禁止穿越，不将未知格改写成自由格；当前车体缺观测也不能跳到前方有观测位姿。覆盖记忆2秒、输入期限、实际外参、100ms租约及模式反馈保持。

GPU观察预留软代价改为超出额度的未知格比例，避免几个边缘格触发整条候选固定高罚分。标准10cm/窄通道4cm预留是软代价范围，不是额外硬外扩。网页显示具体拒绝原因、保护范围未知数、边缘未知数/额度及25mm，终态保留明细，IDLE隐藏旧诊断。

## 验证

以下命令在项目根执行；构建在`ws_nav`。最终退出码均0。输出均在证据目录，旧失败日志保留。

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .runtime/exact_mppi_venv/bin/python -m pytest tests/test_exact_coverage_policy.py tests/test_exact_navigation_core.py tests/test_exact_temporal_visibility.py tests/test_exact_visibility_precision.py -q
# core_tests_final.log：61通过
source /opt/ros/humble/setup.bash
source ws_base/install_exact/setup.bash
source ws_nav/install_exact/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .runtime/exact_mppi_venv/bin/python -m pytest tests/test_exact_coverage_critic.py tests/test_exact_optimizer.py tests/test_exact_warm_start.py tests/test_exact_control_recovery.py tests/test_exact_sampling_ros.py tests/test_exact_field_preparation.py -q -rs
# gpu_tests_final.log：58通过，4个launch检查因JAX环境缺lark跳过
/usr/bin/python3 -m pytest tests/test_exact_field_preparation.py -q
# field_tests_final.log：系统ROS环境完整18通过，包含上述4个launch检查
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .runtime/exact_mppi_venv/bin/python -m pytest tests/test_exact_sampling_ros.py tests/test_exact_navigation_reset.py tests/test_exact_endpoint_diagnostics.py -q
# related_tests.log：20通过，部分与前一组重叠
.runtime/navigation_workbench_venv/bin/python -m unittest tests/test_navigation_workbench.py tests/test_navigation_workbench_reset.py tests/test_fast_livo_frontend_upgrade.py -v
# workbench_tests.log：65通过
XLA_PYTHON_CLIENT_PREALLOCATE=false JAX_PLATFORMS=cuda .runtime/exact_mppi_venv/bin/python docs/workbench_evidence/20261117/coverage_margin/verify_recorded_control.py
/usr/bin/python3 tests/check_exact_coverage_ros.py --evidence-directory docs/workbench_evidence/20261117/coverage_margin/ros
PLAYWRIGHT_BROWSERS_PATH="$PWD/.runtime/workbench_browsers" .runtime/navigation_workbench_venv/bin/python tests/check_workbench_coverage_browser.py
/usr/bin/python3 docs/workbench_evidence/20261117/coverage_margin/plot_coverage_comparison.py
```

GPU与独立网格SAT在64个随机旋转位姿及边缘/越界输入交叉核对；实际5cm网格4格边缘容忍通过，已知障碍不豁免；9格、保护范围未知、越界、全未知均拒绝。全局非零旋转及正式非零外参输入保持坐标一致。

保存输入旧检查起点即失败；新检查允许约0.739m路径前段进入优化。256候选GPU在零初速假设下给出首周期约(0.01471,0.00240,0)m/s，独立全量保留障碍和制动轨迹复核通过，最小轮廓距离0.165m。零初速来自该任务静止wheel/零命令背景，是反事实假设；该命令是第一周期加速值，不是巡航速度。单次预热后优化15.64ms，不是P95、三融合竞争或整链实时性结论。

真实安装的协调器+独立GPU+门禁在ROS_DOMAIN_ID=231、localhost、无驱动模拟植物中通过5项：实际策略25mm；未使能零输出；GPU候选通过末端门禁；观测断流停车；取消清除旧输出。仅5个有效计算样本端到端约29–52ms，不足以证明性能验收通过。自有测试进程已清理，`ros/result.json`及trace/log保留。浏览器4项检查通过，无控制请求/控制台错误。

首次GPU测试2项仍引用旧策略（critic半尺寸、路径检查旧`free`测试替身），已更新为当前策略；系统ROS补跑发现历史standard静态reserve断言仍是4cm，改为正式0cm并增加覆盖/静态值传递断言。浏览器首次缺默认缓存路径、第二次CSP禁止测试字符串eval，改为项目浏览器目录/原生文本断言，页面CSP保持。旧失败日志均保留。

构建前按boot/start_ticks确认所有受管owner/guardian不存活，正式安装无在用导航进程，见`before_build.json`。

```bash
# cwd=/home/lab307/ranger_nav/ws_nav；先source ROS及ws_base/install_exact
colcon --log-base log_exact build --build-base build_exact --install-base install_exact --packages-select ranger_navigation --executor sequential
```

`build.log`：1包成功、退出0，正式`ws_nav/install_exact`已更新。源码/安装身份及最终会话检查见`final_identity.json`。FAST正式低延迟锁、地图/manifest和原依赖保持，源码基点bb418321ea139503242bcc5d975906896c808714，保护此前及本轮未提交工作。

用户正常启动/刷新工作台，再网页显式启动自有导航会话加载新策略；没有代启设备或恢复旧目标。新实车单点、零速占比、门口覆盖变化、坡道和完整走廊往返仍PARTIAL，不能用离线候选替代。


## 后续：项目 YAML 直接调节（2026-11-17）

用户进一步要求外扩和未知格数通过YAML调节。既有coverage参数已经贯通；本轮补齐中文注释/范围说明，并修复launch仍读安装目录构建快照的问题。默认每次新导航会话直接读取项目`ws_nav/src/ranger_navigation/config/exact_navigation.yaml`；可选`navigation_config`完整文件覆盖。启动前用CoveragePolicy检查原值，避免非整数未知格被int静默截断；0表示严格未知格检查。默认2.5cm/1.5cm/8保持，不热改运行参数，worker reset不重新读取YAML。

`coverage_yaml/tests.log`：系统ROS下30项通过（含非默认3.5cm/2cm/4格、零额度、4种非法输入、两节点同值及worker CLI传递），退出0。独立前缀colcon构建1包成功，见`coverage_yaml/build.log`。当时用户会话正运行，因此在独立前缀构建，仅将已验证的launch及相同数值的YAML以原子替换部署到正式share，旧文件保存至`.runtime/backups/coverage-yaml-ueb3fbjv`；实际运行模块未改，未中断/重启会话。文件SHA和所见会话见`coverage_yaml/deployment.json`，本节部署身份覆盖前一阶段同名文件旧SHA。

用户随后要求全部停止；现查工作台run-_41wsega、base run-gf6qftvv、FAST定位run-yfri94h1及旧navigation均已STOPPED，无存活owner/guardian/owned及相关残留节点，因此无需再发信号。新快照见`coverage_yaml/final_check.json`；没有新设备启动、CAN操作、实车目标或运动，未提交推送。实车验收仍PARTIAL。
