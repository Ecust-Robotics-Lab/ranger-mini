# FAST 定位接入导航与首轮闭环计划

日期：2026-11-06。用户要求保存当前Git，并询问成熟导航系统下一步及是否开始规划控制。本文件是后续路线建议，本次只写计划；未实施公共桥接改造、硬件操作、CAN控制或导航运动。后续源码/离线实施及实物联调按用户实际指令推进。

## 当前判断

下一阶段应进入“定位接入导航与受控闭环联调”。现有Nav2、Smac Hybrid-A*、MPPI、motion_gate和Collision Monitor可复用，首先补齐定位/地图/车体接口，不重新开发整套规划控制算法。C1/C2目前只验证了私有定位与部分跨访问功能，新默认v2/KISS/GICP在新282秒包首次VALID22.315秒；真实wheel历元、公共TF、车体外参和长期泛化仍未验。

现有配置事实：

- `config/runtime.yaml`生产仍为Lightning与wheel；`T_base_imu`的平移`[0,0,0.556]`仍标记APPROXIMATE。
- `ws_nav/src/ranger_navigation/config/nav2_params.yaml`已选择Smac Hybrid/DUBIN、MPPI/Ackermann，规划/控制转弯半径0.60米，当前MPPI上限0.10m/s、禁倒车。
- 同文件两个costmap的实际`robot_radius`均为0.10米；其旧注释仍出现0.20米，Collision Monitor另外使用0.74×0.52米真实车体外形加边距的矩形。这是历史窄口试验配置，不能直接当作规划几何与真实外形已一致。下一阶段核对原点与包络，统一规划/控制/防撞的几何和版本，不在本次自动改写旧现场决定。
- FAST先验地图为三维定位归档；现有Nav2 map_server还需匹配同一地图坐标系的占据图及来源绑定。

## 实施顺序与每步交付

| 顺序 | 工作 | 交付与验收重点 |
| --- | --- | --- |
| 1 定位接车基础 | 核实雷达/前端与主机/wheel的真实时间关系；校核T_base_imu；复用已有私有定位适配器和唯一公共桥接 | 原采样时刻的map位姿与wheel历史可插值对应；map→odom→base_link及传感器静态TF完整、无争抢；断流/过期/重定位跳变状态可见。不能重打now或用latest-TF代替同步 |
| 2 导航地图与障碍输入 | 从正式优化地图生成平地占据图与yaml，记录分辨率/原点/高度语义/源manifest；接入实时障碍过滤与costmap | 定位与规划处于同一map；墙、门、桌椅等障碍可核验，未知区保留未知，不能把无点区域一律当空地；地图可加载、障碍可标记和合理清除 |
| 3 几何与运动约束 | 核实车体外形相对base_link原点、实际转向模式、速度反馈、转弯半径及制动行为；统一costmap footprint、Smac和MPPI约束 | 规划路径的车体扫掠包络可通行，控制器可跟随；保持现有Ackermann和前进模式先完成最小闭环。半径、速度参数按本机Humble实际版本核对 |
| 4 离线/只读联调 | 回放或模拟wheel/TF检查只作为软件验证；获对应授权后做真实只读定位、局部costmap和规划显示 | 输出路径及速度建议可检查，先不让算法接管底盘；初始化、取消、LOST、恢复与控制就绪语义一致。模拟wheel不得记作实车同步通过 |
| 5 低速单点闭环 | 获本次明确运动授权后，在开阔平地依次测试短直线、缓弯、停车和单目标到达 | 复用Smac＋MPPI＋门禁＋Collision Monitor；记录路径跟踪误差、目标误差、速度/转向响应、停车距离与定位年龄；障碍、失效与取消能停止，恢复后按明确策略重新允许任务 |
| 6 场景稳定性 | 再逐步加入窄门、走廊、动态行人、重启/重定位恢复及重复路线 | 多访问/不同起点结果与失败原因可复查，长期运行无持续积压，资源稳定；再决定是否增加倒车、精确终点朝向、多点任务等能力 |

第1与第2步可并行开发软件，第3步复用已有底盘/安全配置进行一致性审查；通过后再集中调Smac/MPPI参数。第一轮不加入新的控制器、复杂恢复树、坡道/高程导航、在线前端恢复或语义导航。

## 最小闭环和范围

建议下一次软件阶段的目标是“FAST定位适配→同刻wheel公共TF→占据图/实时障碍→Nav2规划与控制输出的隔离联调”。不把“能在RViz看到路径”当作底盘已能可靠跟踪。已有参考安全链保留，定位失效时撤销运动就绪；重定位恢复的位姿变化先经桥接和就绪流程处理，不直接带着旧控制任务继续运动。

C2当前要求独立运动支持，初始化流程需明确人工辅助初值/受控采样与何时允许导航目标；本计划不自动驱动车辆寻找初始定位。长期负样本、独立真值与盲测访问逐步补齐，不要求先无限扩展检索算法才推进接口联调，也不以私有单包PASS代替这些验收。

## 参考与本机版本

[Nav2定位与地图官方说明](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/sensors/mapping_localization/)明确map→odom是导航基本要求；[Collision Monitor官方说明](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/)说明独立传感器防撞监视与规划器/costmap职责。这里仅参考架构，配置参数继续以项目安装的ROS 2 Humble及本机源码为准，不照搬Rolling接口。
