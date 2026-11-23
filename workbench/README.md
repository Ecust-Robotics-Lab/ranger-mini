# Ranger 导航工作台

单点失败后可点击“重置规划控制”：先结束自有目标，清除旧路线/优化历史并重新初始化规划控制；定位、传感器与记录继续运行。等待就绪后再执行目标。该按钮在失败或定位暂时无效时也可用；“取消并停车”结束活动目标，“停止自有会话”停止整套自有设备。见[重置报告](../docs/workbench_navigation_reset_report_20261117.md)。

当前正式入口默认加载低延迟三融合前端，无需额外优化开关；页面会在启动就绪后显示已加载。定位原采样期限400ms、接收期限250ms，静态总裕量2.5cm与实时避障净空独立。旧地图通过精确版本迁移校验，原资产不改；详见[本轮报告](../docs/workbench_goal_latency_frontend_report_20261117.md)。软件更新后需重新运行工作台并在网页显式启动加载，单独启动网页不启设备。

本地服务，局域网浏览器访问。EXACT-MPPI/定位算法及安全门禁沿用现有实现；前端不是坡道、出门或巡航验收的替代。

## 启动

```bash
# 项目内依赖（当前机器已安装），不更改 ROS、驱动或系统 Python
bash scripts/setup_navigation_workbench.sh --execute
# 仅网页 / 无 ROS 演示，显示真实预建图及已有历史记录
bash scripts/run_navigation_workbench.sh --demo --execute
# 正式 ROS 观察及网页控制入口；本命令不启动设备
bash scripts/run_navigation_workbench.sh --execute --lan
```

本机打开 http://127.0.0.1:8080，远程浏览器打开 `http://工控机局域网IP:8080`。`--lan` 只绑定 HTTP，不改网络、防火墙或 DDS。浏览器输入 `.runtime/navigation_workbench/access.key` 中的密钥（权限 0600）；可在工控机本地查看该文件。部署限可信局域网，不配置公网转发。

默认入口 dry-run。网页“启动底盘、传感器与导航”会请求 CAN 控制及启动设备，须由操作者确认当次现场条件。点击启动后先检查 USB-CAN：复用已有 500kbit/s、ERROR-ACTIVE 接口；若 can0 尚未启用，复用项目已授权的固定 `sudo -n /usr/sbin/ip link set can0 up type can bitrate 500000` 命令，记录启动前后状态及回退命令。已有接口参数异常时拒绝自动重置；权限不足时显示具体错误，不在网页索取 sudo 密码。普通网页服务启动和演示模式不执行此操作。纯电脑控制时可从上电起保持遥控器关闭：原驱动启动会发送 CAN 模式请求，工作台等待新鲜的正常系统反馈 `control_mode=1` 后继续。无需每次开遥控器或拨 SWB。遥控器开启时优先级仍更高；要同时保留遥控器开启，SWB 最上方才是 CAN 指令档，工作台不会反复请求覆盖遥控接管。网页“底盘控制档”显示实际反馈，不能以软件请求已发出代替确认。按钮执行单点时才使能运动。默认速度和净空来自原导航配置，不由网页暗中放宽。

```bash
bash scripts/run_navigation_workbench.sh --status
bash scripts/run_navigation_workbench.sh --stop
# HTTP 进程异常退出，而原任务所有者仍存活时单独恢复网页
bash scripts/run_navigation_workbench.sh --execute --attach --lan
```

正常关闭终端会停止整个工作台及自有设备；关闭浏览器、刷新网页或浏览器断网不会停止当前目标。服务不自动恢复旧任务。任务所有者故障由独立 watchdog 撤销使能及 guardian 清理自有进程。界面“取消并停车”显示取消状态，并独立显示反馈是否停稳；不能替代物理急停。

## 使用

1. 选择正式地图/净空配置，启动会话，等待有效定位。
2. 点击拖动地图设置位置和朝向，或选择预设单点。预设中“候选”属于历史路线位置，不是当前实车位姿，也不承诺现场可达。
3. 静态预览支持显式离线起点；扫掠通过只证明静态可行性。会话启动先等待新的重定位结果，再启动导航；等待时显示查询窗口进度，最多等120秒，不沿用上一会话位姿。
4. 执行单点导航、替换当前目标或取消。执行时从当前有效定位重新规划。
5. 回放页选择轻量记录；选原包会先离线建立索引。回放不发布 ROS 数据，真实后台任务不会因切页停止。

轻量记录：`.runtime/navigation_workbench/records/`；预设：同目录 `presets.json`；会话日志：`logs/`。轻量记录按 10 分钟分 SQLite 文件，压缩快照、有界后台队列，队列丢帧计数和磁盘故障可观察。原始录制：`bags/workbench-raw-*`，默认 900 秒，可设 1–3600 秒，20GB 空间下限；录制结束不停止导航，不删除旧包。所有时间轴使用原记录顺序，位姿/轨迹保留原采样时刻。

当前预注册地图 UUID `160f8572a77a61856cf87e43514be769`。要新增地图，启动时使用 `--map /绝对路径/正式导航投影目录`，可重复该参数注册多张地图；该目录必须包含 map.yaml、map.pgm、manifest.json、COMMITTED 且通过原 manifest 完整性检查；不扫描并信任任意浏览器文件路径。预设导入/导出 JSON，支持单个目标，不要求往返闭环。

## 接口

`/api/v1/login`、`maps`、`maps/{id}/image`、`presets`、`presets/import`、`presets/export`、`snapshot`、`events` (WebSocket)、`previews`、`jobs/{id}`、`session/start`、`session/stop`、`goals`、`goals/cancel`、`navigation/reset`、`recordings`、`recordings/start`、`recordings/stop`、`replays/import`、`replays/{id}?time=秒`。

所有 API 需要 cookie 认证（登录除外）；POST 使用 JSON 和相同 Origin。设备/任务写请求带 `view:live`、`session_id`、唯一 `request_id`，目标还带 `map_id`、`pose:[x,y,yaw弧度]`。替换显式 `replace:true`；取消带 `task_id`。会话启动带 `map_id`、`profile:standard|narrow`。回放以只读 API 返回，不连接 ROS graph。

HTTP↔任务所有者使用权限 0600 的本地 Unix socket。查看代码的入口为 core.py、server.py、daemon.py、ros_bridge.py、web/app.js。没有任意 Shell/ROS 发布接口。

## 验证

```bash
.runtime/navigation_workbench_venv/bin/python -m unittest tests/test_navigation_workbench.py -v
# 浏览器测试依赖仅用于开发；浏览器也安装到项目内
.runtime/navigation_workbench_venv/bin/pip install -r workbench/requirements.dev.lock.txt
PLAYWRIGHT_BROWSERS_PATH="$PWD/.runtime/workbench_browsers" .runtime/navigation_workbench_venv/bin/python -m playwright install chromium
```

ROS 合成检查需要 source Humble、Livox、FAST、nav/base exact 前缀，设置 `ROS_DOMAIN_ID=228 ROS_LOCALHOST_ONLY=1` 后运行 `.runtime/navigation_workbench_venv/bin/python -m unittest tests/test_navigation_workbench_ros.py -v`。测试不连接底盘，不启动传感器，Action 由合成节点提供。报告与截图见 `docs/navigation_workbench_report_20261117.md`。


停稳参数统一位于 `ws_nav/src/ranger_navigation/config/exact_navigation.yaml` 的 motion：线速度范数≤0.05m/s、角速度≤0.025rad/s，连续0.5秒。导航、独立门禁、V2底盘模式切换及网页使用同一组参数；没有将原始轮速/里程计强制归零。网页优先使用原始 `/wheel/odom` 及 host 采样时刻显示停稳，不依赖定位/传感器时钟；过期反馈显示等待新数据。历史轻量记录没有参数字段时保留原记录语义。


## 观测覆盖的 YAML 调节

编辑项目文件 [exact_navigation.yaml](../ws_nav/src/ranger_navigation/config/exact_navigation.yaml) 的 `coverage` 段。当前配置见 [STATUS](../docs/STATUS.md)。

`total_margin`可在`protected_margin`到0.10m之间调节，`protected_margin`至少为0.015m。两种未知格额度独立计算，分别接受0–16的整数。`allow_boundary_unknowns: true` 时，完整落入保护范围的未知格计入内部额度，其他与覆盖轮廓相交的未知格计入边界额度。设为 `false` 时，所有与保护范围相交的未知格都计入内部额度，因此内部额度为0时会拒绝任意保护范围相交。已知障碍和越界会拒绝本次检查。静态地图净空和实时障碍点碰撞裕量各自配置。

默认入口直接读取上述项目文件，不依赖构建时复制的YAML。改完后重新启动导航会话生效，无需重新构建；运行中不热改。网页“重置规划控制”仅重建计算子进程，不会重读这些参数。协调器、GPU和独立门禁使用同一组值；非法值在启动节点前拒绝。独立ROS入口可用`navigation_config:=/绝对路径/配置.yaml`选择另一份完整配置，工作台默认仍用上述项目文件。


## 正式速度上限

正式工作台与导航默认直接读取上述项目YAML的`motion.max_speed` 和 `motion.max_yaw_rate`。`motion.acceleration`/`yaw_acceleration`仍控制加速度。修改后新启动导航会话加载，无需重新构建，运行中不热改。网页“实际导航上限”显示当前导航进程回报的值，历史记录缺字段时显示等待参数。

显式ROS参数`low_speed:=true`可采用`real_vehicle_first_test`档（当前0.10m/s、0.20rad/s）；若显式指定CLI `max_speed`，不得超过YAML motion上限。上限是规划与安全约束，不是保证每时刻恒速；接近目标、障碍、模式/轮角切换和实际反馈可能降低速度。设备及实车运动仍按当次授权。
