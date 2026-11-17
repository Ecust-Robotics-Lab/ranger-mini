# 办公室—走廊往返实车准备

最新实机（2026-11-16）：门内60cm接近点成功，但出门仍未通过。摆轮指令缺陷已修并分项复验，实时覆盖与短段衔接仍待解决；最后45秒门外目标正常取消、输出/wheel零，设备均停。见[本轮报告](exact_mppi_office_corridor_live_20261116.md)。下方8/8仅为静态路线成绩。

最新（2026-11-16）：新候选增加门外转向后倒车/平移入室，8/8段、三初始模式24/24静态检查通过，约40.39m。见[当前准备报告](exact_mppi_office_corridor_preparation_20261116.md)、[路线图](../outputs/exact-office-corridor-preparation-20261116/route_review.png)和[交互复核](../outputs/exact-office-corridor-preparation-20261116/index.html)。本轮仅离线准备，无设备/目标/速度；门口流畅性、持续0.30m/s及整圈仍PARTIAL，需当前位姿重规划。原7/7属于修正车头前历史候选，修正后旧候选为6/7；下方旧地图、旧命令和未启动说明保留为准备阶段历史，当前地图以最新报告为准。

更新：2026-11-10。用户已确认办公室在预建图上方，任务是沿走廊到端点掉头返回。本文件记录准备工作和分项验收顺序，不构成本次设备启动或运动许可。当前没有启动设备、启用 CAN、发目标或实车速度。

## 路线与地图

正式三维归档 UUID `253f573d7a7eabbc363a225ab192cc14`，manifest SHA `e81d4a05a8da6cf3c1badfc0a6171295020bec59e5615408796c92a85b49b247`。导航投影仍为 `outputs/fast-livo-demo-navigation-prep-20261106/navigation_map_final`，5cm / 453×835；原图和归档没有修改。

候选路线：[点位配置](../config/exact_mppi/office_corridor_route.yaml)、[交互复核](../outputs/exact-field-preparation-narrow-final-20261110/index.html)、[路线图与门口放大](../outputs/exact-field-preparation-narrow-final-20261110/route_review.png)。办公室候选起点 `(0.6,0,0)` 不是当前实车定位。端点候选 `(3,-20)` 位于地图下方弯口前，原地掉头的完整车体扫掠已做静态检查，现场仍需核对具体折返范围。返程终点与候选起点相同。

标准/窄通道两档，在所有三个初始模式下分别检查各路段：走廊主体、端部掉头和走廊返程共 **5/7 路段通过**；办公室出门及最后进门两段在每次5秒的正式搜索预算内未找到路径，整条路线为 **PARTIAL**。延长候选出门搜索至30秒的对照也未找到路线；另10秒的三档裕量诊断均超时。超时不是“几何上一定无路”的证明，不能以红色直线替代路径。

交互页可修改候选点位及朝向，修改起点会同步返回点；编辑后旧规划立即失效。导出 JSON 再离线复核，使用新的输出目录，保留每次失败对照：

```bash
python3 scripts/prepare_exact_field_test.py \
  --clearance-profile narrow \
  --route /绝对路径/office_corridor_route.json \
  --output outputs/exact-field-preparation-review-NEW
```

退出码0仅表示准备流程完成，路线是否通过读 `preparation.json` 的 `all_static_legs_passed`；`field_route_accepted` 在离线准备中始终为false。下一步需结合真实停车位复核门口点位和地图投影，必要时从原三维证据导出新的候选图并独立审查，不覆盖原图，也不按目标路线擦除障碍。

## 窄通道参数

同一算法支持 `clearance_profile:=standard|narrow`，受管入口通过 `RANGER_CLEARANCE_PROFILE` 选择，默认仍standard。所有几何消费者读取同一个轮廓和相同净空；窄通道采用原地停稳切换模式，不产生第二套导航算法。

| 参数 | standard | narrow |
| --- | --- | --- |
| 车体长×宽 | 0.74×0.52m | 0.74×0.52m |
| 物理净空参数 | 0.04m | 0.03m |
| 静态地图额外工程余量 | 0.04m | 0.01m |
| 扫掠离散保守界限 | 0.015m | 0.015m |
| 静态图每侧合计裕量 | 0.095m | 0.055m |
| 局部路径额外观察空间预留 | 0.10m | 0.04m |
| 段间朝向到达容差 | 0.06rad | 0.03rad |
| 窄处固定转向平移偏好 | 关闭 | 开启，狭窄静态区行驶代价×3 |

静态地图的名义直行横向包络从0.71m减到0.63m，这是栅格检查参数，不是实车63cm门洞通行承诺。实时体素中心检查仍另加0.04m误差项及0.015m扫掠项；地图工程余量单独调整，不削减点云误差。优化器障碍代价/验证器同时纳入0.015m扫掠保守界限，与最终点障碍检查一致。未知格、未观测轨迹、原戳/TF缺失、超时、定位失效和独立停车规则继续生效。末端防撞使用同一物理净空和完整保留障碍。

新增65cm栅格通道测试证明：原标准裕量拒绝、窄通道完整矩形直行通过；空间不足的90°旋转仍拒绝；未知/点障碍/覆盖失效仍拦截。局部JAX候选与独立覆盖几何也交叉核对。较小净空下发现段间朝向误差会使后续绕障停滞：平移/行驶段若实际停稳朝向或必要起点约束与已规划段不符，则从实际反馈重新规划；旋转直接预测并独立验证实际旋转中心，避免短平移误差导致重规划循环。窄处行驶模型微小偏航会用尽侧向净空，故提高其搜索代价，优先固定转向平移，保留需要行驶弧线时的可达性。最终选用3cm净空、较紧朝向判据；2cm尝试及衔接/模式选择失败对照全部保留，最终闭环成绩以本轮证据汇总为准。

## 分阶段现场顺序

| 阶段 | 操作与记录 | 进入下一阶段的条件 |
| --- | --- | --- |
| S1 静止定位与观察 | 当次授权后启动三融合、C2/C1、wheel映射和公共TF；只接收底盘可使用原receive-only安装；观察门口原始云、地图、真实车体位置 | 同图身份、新采样连续就绪、TF唯一；不得靠放宽定位条件继续 |
| S2 底盘与停车 | 另获底盘控制和运动授权；停掉自启receive-only会话后启用全模式驱动，避免重复CAN拥有者；空旷区逐项核对前后/左右/斜行/转向、真实模式确认、停稳切换、取消/急停/遥控接管 | 低速0.10m/s、0.20rad/s，实测反馈及地面标记/视频记录制动；没有未解释的轮向或模式错误 |
| S3 门口单段 | 从实际定位设置出发位；先到门内等待位，再逐段穿门和返回；完整保留失败地图及轨迹 | 门口实际净宽、全车扫掠、当前观测覆盖和单段往返通过 |
| S4 走廊分段 | 先短距离走廊往返，再延长至端部；静态障碍、临时堵路、取消和定位撤销停车分项验证 | 各段实际执行无碰撞，定位撤销会停车，掉头空间与实际模式确认通过 |
| S5 全程往返 | 依次发送单点目标，上一点成功且反馈停稳后发下一点；取消、失败、接管时停止后续目标，不盲目继续返回 | 办公室出发—走廊端部掉头—办公室返回真实完成；≥30分钟重复任务另记成功率/超时/切换失败/定位撤销/接管 |

全程几何往返距离约40余米，0.10m/s下仅平移就需要数分钟，模式停稳、重规划和等待会更久；本次准备录制窗口1800秒。以现场实际路径长度和耗时报告，不预设完成时间。没有外部真值时，到点估计误差不写成绝对停车精度。

## 已准备的入口与证据采集

新增 `--record-navigation`，必须伴随 `--public-tf`，只扩展现有受管录包的话题和RViz视图，不启动导航或底盘控制。共43个录制话题，含原三融合、wheel/时钟、定位/TF、地图、全局/预测路径、原子控制、最终安全速度、底盘模式/系统反馈、仲裁/门禁、诊断及action状态/反馈。action隐藏话题显式纳入；不把可选导航话题当成传感器录包启动前提。目标请求本身仍须记录目标文件、实际客户端命令和action结果，不能仅凭状态话题复原目标。

实车RViz配置：[field_navigation.rviz](../config/exact_mppi/field_navigation.rviz)，只带查看工具，不带点击发目标工具。新增导航录制会记录导航配置/源码与实际全模式底盘二进制哈希。

以下命令均为已检查的 **dry-run**，不会启用CAN或启动设备；未来执行必须先有当次授权，且先核对当前boot/PID/会话归属。全模式底盘、定位、导航分别管理，不要直接使用旧Lightning `field`入口：

```bash
# 统一DDS域；这些环境设置只影响当前命令。
ROS_DOMAIN_ID=215 ROS_LOCALHOST_ONLY=1 python3 scripts/robot_session.py base --plan

bash scripts/localize_fast_livo2.sh \
  --archive outputs/fast-livo-demo-online-mapping-20261105/full_replay_12/map \
  --public-tf \
  --navigation-map outputs/fast-livo-demo-navigation-prep-20261106/navigation_map_final \
  --record-navigation --visualization rviz --seconds 1800 --domain 215

ROS_DOMAIN_ID=215 ROS_LOCALHOST_ONLY=1 RANGER_CLEARANCE_PROFILE=narrow \
  bash scripts/run_navigation.sh
```

正式组合定位前需要同域新鲜wheel；上述命令未使用 `--auto-wheel`，为后续复用全模式驱动准备。S1若仅启动receive-only，要在该阶段单独明确授权并加入 `--auto-wheel`；退出后按实际归属清理，不能假设可直接升级为全模式。导航启动默认未使能，许可、目标和任何非零运动仍由当次阶段授权决定。

准备时只读快照：boot `31de1c53-fc87-45d0-9b02-966b8a628ec2`；can0 DOWN/STOPPED；未发现相关设备/导航节点；RTX5060 Laptop /580.178.04可查询，磁盘空闲约505GiB。快照不能充当下一次启动时的现状。

本轮小证据位于 [field_preparation_20261110](exact_mppi_evidence/field_preparation_20261110/)，大地图复核页在 `outputs/`。原软件验收、失败日志、原包/地图/安装及备份保持原位，未提交或推送。

## 本轮最终软件验证

| 检查 | 最终结果与证据 |
| --- | --- |
| 新底盘/导航隔离安装构建 | PASS，4个base包+4个nav包，`build_final.log`，退出0；源码Python/launch/config与新安装逐文件一致 |
| 几何、优化器、安全、任务及工程回归 | `regressions_final.log`：109通过、25 subtests通过；JAX环境缺少仅供launch解析的lark，2个launch用例跳过。系统ROS环境 `field_launch_final.log`：16通过，补齐这2例，其余14为重复验证，不累计成新增用例；共111独立用例覆盖通过 |
| 标准参数GPU闭环 | `closed_loop_standard_final.json`：9/9 PASS，完整模型车体无碰撞 |
| 窄通道GPU闭环 | `closed_loop_narrow_final.json`：10/10 PASS，含65cm通道；该模型通道采用固定转向平移，8.70模拟秒完成1.5m目标；不等于实车成绩 |
| 窄通道ROS任务/故障链 | `ros_narrow/ros_safety.json`：19/19 PASS；localhost/domain231、合成反馈，无CAN/传感器；动态堵路绕行、取消/替换、模式失败、断流、定位撤销、急停/接管和计算阻塞/退出均覆盖 |
| 窄档ROS整链耗时 | 同上记录P50 26.02ms / P95 35.58ms / P99 40.71ms；包含云处理到独立验证，不替代真实三融合并发与实机时延验收 |
| 正式地图候选路线 | `outputs/exact-field-preparation-narrow-final-20261110/preparation.json`：5/7段静态通过，门内外两段仍PARTIAL；所有三种起始反馈模式分别验证，无自由区擦除 |
| 图像及启动准备 | Firefox无头截图/JS语法、桌面打开、底盘/定位/导航dry-run、`git diff --check`通过；导航录制列出43话题，RViz配置已切换 |

现场全任务仍PARTIAL。标准档1.5m绕障模型耗时94.95秒，窄档56.85秒，包含多次停稳/重规划，说明效率仍有提升空间；不把纯平移速度当全程效率。版本、文件身份及实际检查命令见同目录 `delivery_identity.json`、`command_ledger.json`；先前的2cm净空、段间重规划循环及窄通道行驶停滞均保留为失败对照，旧验收记录不覆盖。
