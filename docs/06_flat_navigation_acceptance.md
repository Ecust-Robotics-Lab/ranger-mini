# 阶段 6：Nav2 平地单点导航集成与安全链验收

更新：2026-09-28。范围：**配置与离线（dry-run）安全验证**；真机导航测试未执行。
**本文不声称"导航已成功"——当前结论为"配置/离线测试完成，真机待验"。**

实物：Ranger Mini V2（can0 / /base/safe_cmd_vel）+ Mid-360S；定位 = Lightning-LM（阶段 5 桥接）；
地图 = `maps/lightning-offline-suej7qsz`（已验收三维 + g2p5 二维）+ 导航副本（见 4.3）。

## 1. 架构与速度链（实现要点）

速度链（用户指定顺序，Collision Monitor 最后、其后无任何再平滑节点）：

```
Nav2 controller_server(/cmd_vel_nav) ─┐
键盘遥控(/teleop/cmd_vel) ────────────┤
                                      ▼
                          twist_mux 4.3.0（仲裁：遥控 90 > Nav2 10）
                                      ▼ /cmd_vel_mux
                          nav2_velocity_smoother（OPEN_LOOP，20 Hz）
                                      ▼ /cmd_vel_gated_in
                          ranger_motion_gate（本项目新增）
                            · 运动许可 /ranger/motion_enable（VOLATILE 粘滞，
                              进程启动恒 False，重启不自动恢复）
                            · 软急停 /ranger/soft_estop（True 锁存至显式清除）
                            · navigation_ready 新鲜度（未定位不放行）
                            · 障碍点云新鲜度（补偿 CM fail-open）
                            · 限速 + 阿克曼运动学强制（见 §2）
                                      ▼ /cmd_vel_after_gate
                          nav2_collision_monitor 1.1.20（最后一环）
                                      ▼ /base/safe_cmd_vel（唯一发布者=CM）
                          Ranger 底盘驱动（launch 已重映射的专用入口）
```

- 消息类型全链 **Twist**（1.1.20 各组件实际接口，未跟随新文档的 TwistStamped）。
- `twist_mux 4.3.0` 输出话题实测为 `cmd_vel_out`（launch 重映射到 /cmd_vel_mux）。
- CM 停车语义（源码核实）：进入零速后仅继续发布 `stop_pub_timeout`（1 s）零速，
  随后静默；底盘侧"CAN 指令丢失后归零"由用户 2026-09-28 确认（阶段 2 结论更新）。
- CM fail-open 行为（源码核实）：TF 查询失败/点云过期时该帧静默无点（不减速），
  由 motion_gate 的点云新鲜度门禁补偿（测试 T4 验证）。
- 生命周期交互（实测）：CM 进程死亡 → bond 断 → lifecycle_manager CRITICAL
  FAILURE → reset(true) 全栈停机，且无同名节点重生时不自动恢复（测试 T9 验证）。
  这构成"CM 死亡→系统停机"的防线；注意若有人以同名节点重启 CM，manager 会把
  全栈自动拉起（测试中曾因此产生级联伪影，现场不要手工重启 CM 单节点）。

## 2. 底盘运动学一致性与运动模式约束

Ranger Mini V2（ugv_sdk `ranger_params.hpp` + `ranger_messenger.cpp` 源码核实）：

- `min_turn_radius = 0.4764 m`；**驱动 TwistCmdCallback 在 |vx|/|wz| < 0.4764 时
  自动切换 SPINNING（原地旋转）模式、linear.y≠0 时切换横移模式**——这正是
  motion_gate 运动学强制要拦截的行为。
- 三层转弯半径一致性：底盘 0.4764 < **门禁 0.55** < **Smac/MPPI 0.60**（静态与
  运行时双重核验，测试 T12/T14）。
- 禁倒车三重：MPPI `vx_min: 0.0` + smoother `min_velocity[0]: 0.0` + Smac
  `DUBIN` + 门禁负 vx 钳零。
- 门禁运动学强制（`enforce_kinematics`，**tests/test_stage6_gate_unit.py 单元
  12/12** + 集成测试；顺序：先限幅后裁半径——审查确认的回归用例在内）：
  - vy≠0 → 整条指令归零（防横移模式切换）；
  - |vx| < 0.05 且 wz≠0 → wz 归零（防低速 SPINNING 切换）；
  - 半径 < 0.55 → wz 裁剪到 |vx|/0.55（用限幅后的 vx 计算）；
  - 限幅 vx∈[0,0.20]、|wz|≤0.60；非有限值归零；
  - 输出不变量：任意输入下 R ≥ 0.55 或 wz=0（网格扫描验证）。

## 3. 障碍物输入与未知区

- `ranger_obstacle_filter`（新增）：/livox/points_rviz → /livox/obstacles，
  只删不可能是障碍的点（距离门限 0.30–12.0 m、传感器水平 0.35 m 内自身点）；
  **不删地面/动态物体**——地面点由下游在 base 系高度带正确剔除，行人等动态
  障碍始终留在实时防撞输入里。字段防御：缺 x/y/z 的异常点云丢弃并告警（曾因
  异常点云崩溃导致 CM 失明，已修复）。
- collision_monitor 点云源：`min_height 0.10 / max_height 0.80`（base 系，与
  g2p5 建图障碍带一致）；Stop 多边形 = footprint+0.08 m（半轴 0.45×0.34），
  `max_points: 0`（1.1.20 语义为严格大于，即多边形内出现任意 1 点即停）；
  Approach 多边形取局部代价地图 footprint、碰撞时间 2 s 平滑降速。
- 局部代价地图 obstacle layer：同一点云 + 同一高度带（运行时参数核验 T14）。
- 未知区不当可行驶：全局 costmap `track_unknown_space: true` + Smac
  `allow_unknown: false`（运行时核验）。

## 4. 地图与 footprint

### 4.1 footprint（PROVISIONAL）
- 用户确认外包络 **0.74×0.52 m**（半轴 0.37×0.26，base_link 按几何中心——
  原点定义未核对，base_link 外参精测后应复核）。
- 公开来源尺寸不一致（720×590 / 738×500），长边取外包络；真机前建议卷尺实测。

### 4.2 本场地走廊净空（实测地图数据）
- 起点周边净空紧张：起点最近占据 0.48 m、直线走廊多处 0.08–0.4 m；可用目标
  区在起点 107–120° 方向约 2 m 处（净空 ~0.55–0.6 m）。
- 含义：footprint 半宽 0.26 + 内切环模型下，本场地属于**窄走廊边缘工况**；
  真机目标点应选开阔区，紧走廊段是后续（坡道/重映射）阶段要处理的现实。

### 4.3 导航副本地图（make_nav_map.py）
- 已验收原图 `data/new_map/map.pgm|yaml` **未改动**。副本 `data/new_map/nav/`
  清除了建图起点 0.177 m 处 1 个不可能的占据像元（建图时车体覆盖该位置，
  属近体/地面泄漏点；经 costmap 内切环膨胀后使起点致死、Smac 拒规划——
  证据 safety-15ege02f "Starting point in lethal space"）。
- 副本仅清除起点 0.30 m 内（≤ footprint 半宽+0.04）占据像元，共 1 个；
  生成与复核命令、清除明细留档于 scripts/make_nav_map.py 输出。

## 5. 安全验证（dry-run，六项要求逐条）

补充说明（审查后如实声明）：**键盘遥控同样要过 smoother→门禁→CM 全链**——
即遥控也会被门禁的运动学强制与急停/许可拦截（这是设计选择：急停语义单一）；
"人工接管"指遥控在 twist_mux 处以优先级 90 压制 Nav2 的指令源，而非绕过链路。

测试：`tests/test_stage6_safety.py`（隔离 ROS_DOMAIN_ID=180，跨进程全栈，
与 run_navigation.sh 同一 launch/参数；证据目录 docs/06_evidence/safety-*）。

**最终结果（2026-09-28，safety-ych6w74e）：集成 37/37 全部通过（exit 0）**；
单元 tests/test_stage6_gate_unit.py **12/12**。代表性数字：未定位归零 0.05 s、
点云过期归零 1.02 s、指令过期 0.01 s、软急停 0.03 s、障碍点入多边形停车 0.06 s、
地面点不误刹恢复 ✓、门禁重启后 2 s 内 20 条零值消息且无非零、BT 目标下
/plan 生成+控制器发令而最终输出恒零、CM 死后最终话题发布者数=0（3 s 与 11 s
复核，全栈复位停机无自动恢复）、运行时参数四节点全部与 YAML 一致。
历轮失败证据目录全部保留（rczmq9w3…ych6w74e 共 14 个，记录了每个缺陷的
发现与修复过程）。

| 要求 | 实现 | 测试 |
|---|---|---|
| 未定位不放行 | gate 检查 /navigation_ready 新鲜度 | T3 |
| 传感器过期不放行 | gate 检查点云新鲜度（补偿 CM fail-open） | T4 |
| 指令过期停车 | gate cmd_timeout 0.5 s + smoother velocity_timeout | T5 |
| 急停与人工接管 | /ranger/soft_estop 锁存；遥控优先级 90 压制 Nav2 | T6/T7（遥控优先为配置项，真机复验） |
| CM 退出不执行旧指令 | CM 死→输出静默；manager 全栈停机不自动恢复 | T9 |
| 重启不自动恢复许可 | gate VOLATILE 粘滞许可，重启恒 False | T10 |
| 录包回放不重放控制 | 全部录制脚本不含 cmd_vel 链（静态核验）；最终话题仅 CM 发布、无旁路 | T12/T11 |
| 防撞有效 | Stop 多边形 1 点即停；地面点不误刹 | T8 |
| 参数真实落地 | 运行时 param dump 核验（MPPI/Smac/costmap/CM） | T14 |
| 离线规划可用 | 已验收地图上 BT 目标→/plan 生成、控制器发令、许可关时最终输出恒零 | T13 |

**测试工程教训（防空转/防污染，均已固化为护栏）**：
- 断言必须防自我污染：模拟器停止自发的 /cmd_vel_nav 后，cmd_seen 才能作为
  "控制器在发令"的证据（此前两版为空转检查）；
- 零速断言接受"零值消息或静默"两种形态（CM 停车后 1 s 静默是设计行为）；
- 进程清场必须杀进程组：`ros2 run` 包装器被 SIGINT 后子进程会孤儿化，泄漏的
  带许可门禁曾持续发布 0.15 污染后续轮次（表现为"幽灵 80 Hz 发布者"）；
- Fast DDS SHM 残留：SIGKILL 的进程在 /dev/shm 留下 fastrtps_* 段并出现
  永不超时的幽灵端点，跨轮次干扰图状态；清场应用 INT→TERM→KILL 并在无人
  交互时清理 /dev/shm/fastrtps_*（现场脚本见 scripts/clean_dds_shm.sh）。

## 5.1 对抗性审查（工作流：6 维度找茬 → 每条 3 视角验证，≥2 票保留）

156 代理中 100 完成、56 因账户限流（429）失败——确认清单偏保守，"驳回"清单
中若干条目实为漏验，已人工复核并处理。**确认 15 条**，其中 major 全部修复：

| 确认发现（major） | 修复 |
|---|---|
| 门禁云新鲜度监视原始 /livox/points_rviz，而 CM/代价地图消费过滤器输出 /livox/obstacles——**过滤器死亡=防撞全链静默失明且门禁仍报 fresh** | 门禁改订 /livox/obstacles（与被补偿对象同源）；过滤器改为**空云也发布**（新鲜度心跳） |
| enforce_kinematics 的 vx 截幅在半径裁剪之后——遥控形态输入（vx=0.25,wz=0.5）输出半径 0.44 < 底盘 0.4764，**触发驱动 SPINNING 原地旋转切换** | 重排为"先限幅后裁半径"+防御性复截；新增单元测试 tests/test_stage6_gate_unit.py（含回归用例与全网格不变量扫描，12/12） |
| send_nav_goal 缺省 --yaw-deg 发送全零四元数 → Nav2 解析为 map 系 0° 朝向，与帮助文本相反 | 缺省改用 TF 查询当前车头朝向；TF 不可得则报错要求显式 --yaw-deg |
| send_nav_goal --cancel 跨进程恒空操作（BasicNavigator.cancelTask 只对本进程目标有效）仍报成功 | 改为直接调用动作 CancelGoal 服务（空 goal_id=取消全部，rcl_action 语义）；集成测试 T13 增加该路径验证 |
| --timeout 不覆盖 action server 不可用阶段（goToPose 无限等待） | 发送前服务预检，不可用即报错退出 |

minor/note 级修复：门禁再武装斜坡（阻断→放行不再 0→0.15 阶跃，0.30 m/s² 爬升）；
velocity_smoother 加速度比值 ≥ 底盘半径（θ 加速度 1.00→0.60）；motion_gate.yaml
注释与实现一致性（VOLATILE）；MPPI critics 显式列出（代码默认不可依赖，类名
对照 libmppi_critics.so 符号表核实）；goal_checker 容差显式放宽（0.30 m / 0.40 rad，
窄走廊内只能前进弧线调头）；Approach 多边形 max_points 5→0（严格大于语义）；
run_navigation.sh --stop 模式改为可执行路径锚定（防误伤编辑器/日志进程）并补
map_server；nav_safety.sh 发布命令加 timeout、clear 语义说明；launch 默认地图
指向导航副本（原默认指向原版会复现起点致死）。

**测试侧修复**（防空转/防污染）：T11 主判定改 rclpy 实时图计数（本机 ros2
daemon 图不可靠：实测 `_NODE_NAME_UNKNOWN_`、param dump 全空）；T14 弃用 CLI
dump 改 get_parameters 服务直读；T10 增加"活跃零值消息 ≥8"防死门禁空转满足；
T9 kill 窗口 0.5 s 在途消息不计；阶段异常时仍输出汇总与证据；目标选取净空对
"占据+未知"像元计算；录像静态核验扩展到 shell 脚本；进程清场 SIGINT→宽限→
KILL 并扫尾 static TF。

**运维发现**（属实录）：`ros2 run` 包装器被 SIGINT 后子进程孤儿化（曾致 6 个带
许可门禁泄漏、以 ~80 Hz 向安全链发布 0.15）；SIGKILL 进程在 /dev/shm 留下
fastrtps_* 残段与永不超时的幽灵端点，跨轮次污染 DDS 图——现场清理脚本
scripts/clean_dds_shm.sh（确认无活 ROS 进程才执行）。

## 6. 现场操作（真机，待用户逐次授权后执行）

前置：field_session.sh --execute 全绿（navigation_ready=true），
ENABLE_MAP_PUB=0（避免与 map_server 双发布 /map），can0 已配置。

```bash
bash scripts/run_navigation.sh          # 干跑
bash scripts/run_navigation.sh --execute
bash scripts/nav_safety.sh status       # 查看门禁判定
bash scripts/nav_safety.sh enable       # 授权本次运动（重启后需重新授权）
python3 scripts/send_nav_goal.py --x -0.6 --y 1.9 --yaw-deg 108
bash scripts/nav_safety.sh estop        # 任何时候软急停
bash scripts/nav_safety.sh disable      # 撤销许可
bash scripts/run_navigation.sh --stop
```

真机测试顺序（按用户七：隔离平坦空旷区、确认后的很低速度起步）：
1. 0.15 m/s 限速下 1–2 m 短距直行目标（改 config/motion_gate.yaml max_vx 收紧）；
2. 缓弯目标；3. 取消目标（观察停车）；4. 静态软障碍（纸箱）接近停车；
5. 遥控接管压制；6. 按制动响应逐步放宽限速（不直接用底盘最大速度）。
**不允许让人突然进入车前测试防撞。**

## 7. 命令与退出记录

| 命令 | 退出 | 说明 |
|---|---|---|
| `sudo apt install ros-humble-navigation2 …`（用户执行） | 0 | nav2 全套 1.1.20 + twist-mux 4.3.0 |
| `colcon build --packages-select ranger_navigation`（ws_nav，多次） | 0 | 含审查修复后重建；**改 yaml 后必须重建**（曾因漏重建致 max_points 旧值假失败） |
| `python3 tests/test_stage6_gate_unit.py` | 0（12/12） | 门禁运动学纯函数 |
| `python3 tests/test_stage6_safety.py`（EVIDENCE_DIR=docs/06_evidence/…） | 0（37/37） | 最终轮 safety-ych6w74e；此前 13 个失败/中断证据目录保留 |
| `python3 scripts/make_nav_map.py` | 0 | 清除起点 1 个不可能占据像元，独立复核 START_CELL_OK |
| `bash scripts/run_navigation.sh`（干跑） | 0 | 打印将启动组件与速度链 |

## 7.1 已知边界与如实声明

- **FootprintApproach（平滑减速）v1 禁用**：1.1.20 实测在空点云+动态 footprint
  组合下误触发"即刻碰撞"（速度×0，证据 safety-3zlp6hrr CM 日志 "Robot to
  approach" 交替）；机制待上游核查。防撞核心=Stop 多边形（1 点即停，T8 验证
  0.06 s）+门禁+局部代价地图，不依赖 approach。
- 迟到消息伪影：订阅端 executor 批量投递会让 kill 前消息在 1 s 后才入队——
  T9 判定改为实时图发布者数（对时序免疫）；真机底盘按线上时序收包，不受此影响。
- MPPI 目标判定容差放宽（0.30 m/0.40 rad）：窄走廊内只能前进弧线调头；真机
  验证后应收紧。

## 8. 修改文件清单

- 新增 `ws_nav/src/ranger_navigation/`（motion_gate.py、obstacle_filter.py、
  nav2_params.yaml、collision_monitor.yaml、twist_mux.yaml、motion_gate.yaml、
  obstacle_filter.yaml、behavior_trees/single_point.xml、navigation.launch.py）
- 新增 `scripts/run_navigation.sh`、`scripts/nav_safety.sh`、`scripts/send_nav_goal.py`、
  `scripts/make_nav_map.py`、`scripts/clean_dds_shm.sh`、`tests/test_stage6_safety.py`
- 新增 `maps/lightning-offline-suej7qsz/data/new_map/nav/`（导航副本，原图未动）
- 修改 `scripts/field_aux.sh`（ENABLE_MAP_PUB 开关）、`docs/STATUS.md`、
  `docs/02_ros_motion_test.md`（阶段 2 停车安全结论更新，用户确认）
- 依赖安装：ros-humble navigation2/nav2-bringup/collision-monitor/mppi-controller/
  smac-planner/velocity-smoother/twist-mux/simple-commander 全部 1.1.20（twist-mux 4.3.0）

## 9. 遗留与待验（PENDING）

- **真机导航全项未执行**（§6 顺序）；本阶段状态 = 配置/离线测试完成，真机待验。
- FootprintApproach 禁用（§7.1），重新启用需上游核查。
- footprint 为公开尺寸外包络（PROVISIONAL）；base_link 原点未核对。
- 阶段 5 遗留不变：独立数据验证、T_base_imu 精测、time_type=0 时间质量。
- 遥控优先级压制（twist_mux 90>10）为配置项，真机复验在 §6-5。
- `/ranger/motion_enable` 的发布依赖人工命令；如需常驻使能器需另行评审。
