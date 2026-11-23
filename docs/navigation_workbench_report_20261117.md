# 导航工作台 v1 实施报告

日期：2026-11-17。用户明确要求实施已确认的 [工作台计划](NAVIGATION_WORKBENCH_PLAN.md)。本轮完成软件、构建、浏览器和离线/隔离 ROS 验证；没有启动真实传感器、底盘/CAN或发送实车目标。实车继续暂停，整段办公室出门/坡道/走廊往返仍 PARTIAL。没有提交或推送。

## 交付

- [使用入口](../workbench/README.md)：`bash scripts/run_navigation_workbench.sh --demo --execute` 查看真实地图与已有包回放；正式入口去掉 `--demo`，加 `--lan` 供远程浏览器访问。两个入口都只启动服务，设备必须由网页显式启动。默认命令仍为 dry-run。
- 本地原生 JS/Canvas 界面（可重复 --map 显式注册新正式地图）：正式预建图、真实车体及相机侧车头、公共重定位位姿、手动/预设单点、全局/局部/执行估计轨迹、模式、速度、输入年龄、停止原因、任务事件、回放时间轴。桌面、平板、手机检查通过。
- Python HTTP/WebSocket 与 ROS 任务所有者独立进程，Unix socket 权限 0600；访问密钥登录、Origin 检查、仅白名单动作。刷新、断网及 HTTP 重连不重复下目标。任务由工控机端持有，WebSocket 关闭后仍能完成。
- 自有会话管理复用原 guardian/boot/PID 身份。底盘→三融合/C1/C2→公共桥接→导航未使能，执行按钮才启用；取消/终态撤销使能。拒绝占用的外部会话，不按进程名清理。独立 watchdog 在所有者死亡/心跳超过 0.5s 时只发布 false；此门限不改变原 100ms 指令租约或 250ms 输入门限。
- 连续定位新增显式 `--continuous`，只移除有限 recorder，其余组件和默认 CLI 有限采集行为保留。可选原始包独立录制，必要话题已写入后才标为 RECORDING，定时/手动结束不停止导航；封存与来源快照保留。
- 轻量记录后台有界写入、zlib 压缩、十分钟分 SQLite，保存任务/位姿/模式/路径/环境和事件。磁盘不足只停止记录并提示，不删除资产。原包离线读取 CDR、按记录顺序及同戳 TF 索引，不发布 ROS 数据；兼容原旧控制类型话题。缺失信息显示为空，不推断历史目标或模式。
- 新增 `/navigation/route` 只读 JSON 分段元数据，原 `/plan` 保持。按 history_generation 匹配显示行驶/平移/旋转；不参与控制决策。车体尺寸读取正式 geometry，同现有规划/防撞来源一致。

## 实际验证与证据

所有命令均在项目根目录；成功命令退出码 0。

| 检查 | 命令/证据 | 结果 |
| --- | --- | --- |
| 工作台单元/HTTP/生命周期 | `.runtime/navigation_workbench_venv/bin/python -m unittest tests/test_navigation_workbench.py -v`；[日志](workbench_evidence/20261117/unit.log) | 37 PASS |
| 真 ROS 传输/Action、取消、独立 watchdog、关闭浏览器后单次目标完成 | source 当前 ROS/Livox/FAST/nav/base exact 前缀，`ROS_DOMAIN_ID=228 ROS_LOCALHOST_ONLY=1 .runtime/navigation_workbench_venv/bin/python -m unittest tests/test_navigation_workbench_ros.py -v`；[日志](workbench_evidence/20261117/ros_transport.log) | 4 PASS；合成节点，无底盘 |
| 相关导航恢复/GPU/采样回归 | source ROS/nav/base exact，`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=227 ROS_LOCALHOST_ONLY=1`，使用 exact venv 执行 `pytest -q tests/test_exact_control_recovery.py tests/test_exact_warm_start.py tests/test_exact_sampling_ros.py`，保留 sourced PYTHONPATH；[日志](workbench_evidence/20261117/navigation_regression.log) | 31 PASS |
| nav exact 构建 | `bash scripts/build_exact_navigation.sh --execute`；[日志](workbench_evidence/20261117/build.log) | 4 包 PASS |
| 有限/连续生命周期 | 两次 localize dry-run，比较只差 capture；[组件比较](workbench_evidence/20261117/lifetime.json) | PASS，无硬件 |
| 浏览器交互 | `PLAYWRIGHT_BROWSERS_PATH="$PWD/.runtime/workbench_browsers" .runtime/navigation_workbench_venv/bin/python tests/check_navigation_workbench_browser.py`；[结果](workbench_evidence/20261117/browser.json) | PASS，控制台错误 0 |
| 真实包索引 | `/usr/bin/python3 -m workbench.import_bag --folder bags/fast-livo-localization-u622df4n --output .runtime/navigation_workbench/records/recorded-u622df4n-v4`（source 当前 ROS/FAST/nav/base）；[结果](workbench_evidence/20261117/bag_import.json) | 356541 条消息，队列丢帧 0；原包不改 |
| GPU/网页竞争 | `.runtime/exact_mppi_venv/bin/python tests/benchmark_navigation_workbench.py`；[数据](workbench_evidence/20261117/performance.json) | 3 个真实包回放客户端、共 30 次请求，无 HTTP 错误 |
| 语法/空白 | compileall、node --check、bash -n、git diff --check | PASS |

性能使用相同的合成 256×60、512 点 GPU 输入，前后各一次无客户端基线，夹一次三个 5Hz 回放客户端负载。基线 P95 分别 11.41/11.90ms，有客户端 11.89ms，较前后均值增量 0.24ms，满足本次有限负载 ≤5ms 目标。这不是 FAST 三融合+定位+控制真实整链数据，不能覆盖之前端到端 P95 43.29ms 的结论。最初两次性能脚本在负载启动/本地代理设置上未正确建立请求，已修正后重新运行，不将零请求结果作为验收。

浏览器检查包括真实地图、点击拖动目标、旋转地图原点坐标往返、静态 0.5m 规划及独立扫掠、桌面/平板/手机无横向溢出、真实记录跳转、回放只读、播放暂停、刷新重连。未支持的旋转占据图搜索保留原规划器拒绝行为；显示坐标可以旋转，不以显示通过冒充规划能力。

截图：[实时/静态预览](workbench_evidence/20261117/desktop.png)、[真实历史回放](workbench_evidence/20261117/replay.png)、[平板](workbench_evidence/20261117/tablet.png)、[手机](workbench_evidence/20261117/mobile.png)。回放 1:40 处展示历史任务无进展失败，是原实车记录，不是本轮实车测试。

## 来源、保护及剩余项

基点 `bb418321ea139503242bcc5d975906896c808714`，本轮开始已存在恢复优化未提交修改；[基线](workbench_evidence/20261117/baseline.json)。运行地图由原 load_map 完整核验，UUID `160f8572a77a61856cf87e43514be769`，source manifest SHA `93628bcf34742e8fbdb1f08afa9501f44aaa1f9dab4d9ef2c80914b7a62fc105`，没有覆盖地图、原包、标定或失败证据。

新代码集中在 `workbench/`。原源码只追加连续定位选项、录制 route 话题、只读 route 发布；接口文档追加说明。其余原未提交恢复优化经 SHA 核对保留。依赖及 Chromium 均位于项目 `.runtime`，没有安装系统包或修改驱动/CUDA/网络/DDS 配置；运行入口的 ROS localhost/domain 仅进程环境。

以下仍 PARTIAL：真实局域网远程控制器访问、设备完整启停及原始录包实机验证、实车目标/取消/故障/驾驶接管、三融合整链性能竞争及长时使用。合成 DDS/Action/录包器模拟不能替代这些项目。网页实现不修复或宣称通过坡道/完整巡航；首版仅单点，候选走廊点不自动串行发送。


## 追加：用户实际点击启动后的 CAN 前置修复

用户在正式服务点击启动，任务服务收到两次请求，base 均退出 2；嵌套 launch 实际错误为 `can0 must already be UP/ERROR-ACTIVE with known bitrate.`。只读检查显示 can0 不含 UP、operstate DOWN、CAN state STOPPED。原因是新网页启动入口没有接入现有录制入口的 CAN 初始化，不是 SWB 档位直接导致 Linux 接口 DOWN。V2 官方说明书的 SWB 最上方是 CAN 指令控制档，仍需独立核对实际控制反馈。

已补显式启动时复用原 `wheel_capture.check_other_drivers / can_state / prepare_can`，只用已审查的固定非交互命令，保存原状态及回退命令。未知/被占用/UP参数异常不重置；启动取消后不继续开底盘。新增阶段/组件状态、底层错误透传及持久失败提示，点击后使缓存失效。48 项检查（43 工作台、5 原 CAN）及浏览器故障模拟通过；后者使用模拟传输，没有 session.start 或硬件请求。

本次没有执行真实 CAN 启用、重启旧工作台或再次启动设备，旧进程需用户重启后加载。原失败现场及新检查：[失败快照](workbench_evidence/20261117/startup_fix/failure.json)、[测试](workbench_evidence/20261117/startup_fix/tests.log)、[浏览器](workbench_evidence/20261117/startup_fix/browser.json)、[模拟失败提示截图](workbench_evidence/20261117/startup_fix/failure_ui.png)。设备/遥控档位/实际启动恢复尚未验收，不把模拟通过算作实机通过。


## 追加：遥控器关闭时由电脑请求 CAN 模式

用户要求不再每次开遥控器/拨SWB。V2[官方协议](https://new.agilex.ai/raw/upload/20230816/RANGER%20mini%202.0%E7%94%A8%E6%88%B7%E6%89%8B%E5%86%8CV1.0-20230316%28%E4%B8%AD%E8%8B%B1%E5%8F%8C%E8%AF%AD%29_71720.pdf)规定未连接遥控器时上电待机，电脑通过0x421模式帧请求CAN；开启遥控器后遥控器具有更高优先级。此前“每次必须开启遥控器”的表述已更正。当前源码 `ranger_messenger.cpp:47` 对非receive-only调用 `EnableCommandedMode()`，SDK agilex_base.hpp将其编码为CAN模式请求，驱动仍验证实际反馈mode1、vehicle0、error0及各原始反馈新鲜度；此次没有修改SDK/底盘固件/receive-only路径。

工作台新增CAN控制反馈确认阶段，原采样与接收年龄均不超过250ms且系统正常、control_mode=1才继续；待机/遥控/故障时明确报告，不强制覆盖或重复抢占。接受目标也核对此条件，页面显示“底盘控制档”。没有加入遥控器在线或SWB拨档软件条件。

53 项检查（48 工作台、5 CAN）和4项隔离ROS通过，浏览器以模拟CAN/RC/待机验证按钮及显示，无硬件请求。见[检查](workbench_evidence/20261117/computer_control/tests.log)、[ROS](workbench_evidence/20261117/computer_control/ros.log)、[浏览器](workbench_evidence/20261117/computer_control/browser.json)。独立浏览器检查可运行 `PLAYWRIGHT_BROWSERS_PATH="$PWD/.runtime/workbench_browsers" .runtime/navigation_workbench_venv/bin/python tests/check_workbench_computer_control_browser.py`，测试自行建立只读本地HTTP服务，不连接任务所有者。

只读现查原工作台已由用户停止，base/定位/导航会话均不存活；没有代重启或代发接管请求。遥控器保持关闭的真实启动/控制确认仍须现场验证，不把协议支持或模拟结果当作固件实测结论。
