# 工作台重定位 7/8 显示与静止确认恢复 — 2026-11-17

用户报告重定位长期显示 `searching_waiting_query_window，7/8帧`，要求继续排查。已完成软件修复、构建和隔离验证。没有启动/重启真实设备、操作 CAN、发送导航目标或非零速度，没有提交推送。

## 现场记录能证明什么

最新用户会话为工作台 `run-jf8gf38n`、底盘 `run-b8ie_p7u`、定位 `run-wzus6nsa`，boot `6c018ad9-9443-435b-aa93-3a471f85d7de`；数据目录 `bags/workbench-localization-cdc074e3d4da`。源码修改前只读检查时，这些会话均已 STOPPED，且没有存活的该安装 localizer_node，因此此次构建没有覆盖正在运行的定位库。

该记录收到1205对前端输入，传感器、前端、时钟和定位后台均健康，定位未就绪。日志在收集窗口与 `verifying_insufficient_time_or_motion_baseline` 间循环；最终完整查询为8帧，匹配约1012ms，最优RMSE约0.1161m、覆盖0.9843，`stationary_strong_geometry=true`，没有 VALID。不是始终收不到第8帧。匹配期间旧程序没有中间阶段更新，网页会保留上一条7/8，造成停在收集阶段的印象。

源码另有可复现的确认锁定缺陷：第一次候选不满足静止强几何时，support.stationary_strong为false；后续每次与当前strong做AND，即使新匹配足够强，也不能恢复静止确认。强锚点中途遇到一次弱匹配同样会永久失去静止资格。原现场逐样本日志没有保存强/弱及锚点字段，不能据此断言该缺陷是这次失败的唯一原因；新增诊断用于下一次现场区分质量不足、时间跨度与位姿一致性。

## 修改与约束

- 早期弱候选后首次获得强匹配时，丢弃弱锚点及其计数/scan ID，从当前强窗口的1组重新累计；若已有真实 odom 与地图运动基线，保留原运动确认路径。
- 已有强锚点遇到弱查询时，弱查询不计数，也不永久污染强锚点；后续只有与原强锚点一致且scan ID不重叠的强窗口才计数。
- 静止要求仍为至少3组不重叠强几何窗口、原采样跨度至少1秒，FAST/地图位姿差≤3cm/0.03rad、修正差≤5cm/0.03rad。原RMSE、覆盖、退化、地点歧义门限和C1独立确认不变。一直弱、别名、错误几何、里程计漂移都不能通过等待得到就绪。
- 完整查询开始检索/配准前发布 `searching_geometric_registration` 及8/8进度；这只是无效位姿状态的遥测，取消/换会话代际检查仍生效，不刷新已确认采样时间。
- 工作台显示收集、匹配、独立窗口等待、弱锚点重建和确认组数；弱匹配显示RMSE与静止上限。Monitor逐样本保存简短的强度、支持组数、跨度及odom/map/修正差，避免只留下最终health快照；不逐帧重复保存全部候选大数组。
- 已构建到正式 `ws_fast_livo/online_localization_install`。此前用户指定0.05m/s停稳阈值保持，250ms输入期限和100ms指令租约不变；正式三融合、+100ms、地图/外参与前端来源不改。

## 验证与证据

目录：[static_confirmation](workbench_evidence/20261117/static_confirmation/)。执行命令和退出码见 commands.json，源码/安装与结束会话身份见 final_identity.json。

| 项目 | 最终结果 |
| --- | --- |
| 定位构建，localizer_build_final.log | PASS，退出0 |
| localizer_tests，localizer_tests.log/xml | 45 PASS，包含弱首帧恢复、弱中间查询不计数、持续弱拒绝、匹配回调取消及原别名/漂移负例 |
| 工作台与CAN工具离线单测，workbench_tests.log | 59 PASS，退出0，无真实CAN调用 |
| 在线队列/旧结果/过期与新增静止C2→C1，localization_ros.log | 4 PASS，localhost隔离domain229，真实节点、合成输入，无设备 |
| 新增静止确认及实际消息日志落盘，static_ros_monitor.log | 1 PASS；与上行新增项重叠，另增加Monitor断言 |
| 浏览器，browser.json/ui.png | PASS；7/8→8/8匹配、弱锚点重建、质量原因、未就绪禁止目标；独立模拟HTTP，无设备请求 |

ROS静止场景观察到3组支持、原采样跨度1.6秒，并通过C1独立确认；这是合成静止输入的软件证据，不是实车耗时或精度。首轮ROS测试漏设项目GTSAM库路径，节点未加载；补上正式启动同样使用的backend_deps/relocalization_deps后全部通过，初始日志保留。首轮C++新增测试误把后续滚动强窗口复用最后一个弱扫描也视为禁止，修正测试为拒绝弱窗口本身计数，保留初始失败日志；生产几何条件没有因此更改。

## 下一步

本轮软件完成，现场静止初始化/重复启停仍 PARTIAL。用户下一次运行 `bash scripts/run_navigation_workbench.sh --execute --lan` 后，在网页显式启动自有会话加载新版；单独运行网页命令不启动设备。无需为了这一软件缺陷先移动车辆。若仍未就绪，依据新增支持/位姿差诊断继续处理，不能宣称已证明现场唯一根因或保证任意场景静止成功。完整出门、坡道与走廊往返的PARTIAL状态不变。
