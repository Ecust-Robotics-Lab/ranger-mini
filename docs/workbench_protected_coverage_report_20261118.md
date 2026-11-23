# 保护范围未知格独立额度

新增项目 `exact_navigation.yaml` 的 `coverage.protected_unknown_cell_budget`，默认0，取0至16之间的整数。边界未知格继续使用原 `unknown_cell_budget`。两类分别计数，任意一类超额就拒绝。当前项目文件是边界16、内部0，采用用户原有总外扩和保护范围各0.015m。已知占据和越界任一项都直接拒绝，未知格保留原值。

`allow_boundary_unknowns: true` 按完整单元是否落在保护范围内分类。关闭时，触及保护范围的格子也纳入内部计数。内部额度默认0，既有 YAML 未指定时沿用默认值。网页分别显示两类未知格的实数和额度，旧记录缺内部额度时显示0。工作台和独立导航入口新建导航会话时读取项目 YAML，规划重置沿用会话已加载的值。

正式 `ws_nav/install_exact` 已构建。实际安装的 launch 读出边界16、内部0并传给协调器和门禁。合成5cm网格放置2个内部格和7个边界格。CPU 网格检查、GPU 代价和独立门禁在两种额度各自不足时拒绝，在内部2且边界7时通过。将内部格标成已知障碍后仍拒绝。隔离 ROS 用内部额度2启动，GPU 回报2，模拟运动下断流和取消停车均通过。浏览器核对两列额度及旧记录默认显示。

以下命令执行和证据均在 `docs/workbench_evidence/20261118/protected_coverage/`，退出码均为0。ROS 环境先 source Humble、base 和 nav 安装。

```bash
/usr/bin/python3 -m pytest tests/test_exact_field_preparation.py tests/test_exact_coverage_policy.py tests/test_exact_temporal_visibility.py tests/test_exact_visibility_precision.py -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 JAX_PLATFORMS=cuda XLA_PYTHON_CLIENT_PREALLOCATE=false .runtime/exact_mppi_venv/bin/python -m pytest tests/test_exact_coverage_critic.py tests/test_exact_optimizer.py tests/test_exact_sampling_ros.py -q
PLAYWRIGHT_BROWSERS_PATH="$PWD/.runtime/workbench_browsers" .runtime/navigation_workbench_venv/bin/python tests/check_workbench_coverage_browser.py --evidence-directory docs/workbench_evidence/20261118/protected_coverage/browser
/usr/bin/python3 tests/check_exact_coverage_ros.py --evidence-directory docs/workbench_evidence/20261118/protected_coverage/ros
# 在 ws_nav 下构建
colcon --log-base log_exact build --build-base build_exact --install-base install_exact --packages-select ranger_navigation --executor sequential
```

49项核心与参数检查、29项GPU及控制相关回归、隔离ROS和浏览器通过。源码基点 `bb418321ea139503242bcc5d975906896c808714`。最终部署校验见 `identity.json`。软件 PASS，实车 PARTIAL，未操作设备或发送实车目标。
