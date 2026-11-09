# 在线每会话时间映射与公共TF入口

日期：2026-11-09。用户要求开始实施前两项计划。本轮完成软件和隔离检查，没有启动设备、底盘、真实在线定位或导航运动，没有提交/推送。沿用现有C2/C1默认组合及用户授权工作外参。

## 实现与语义

新增`live_clock.py`与`wheel_clock_node.py`。复用已有receipt_probe的雷达首帧头戳/单调接收时刻，与mapping_monitor的私有`/clock`使用同一锚点。节点在当前进程采样主机时钟与单调时钟差，冻结本会话转换：

    t_sensor = t_original_wheel_host - (t_host - t_steady)_anchor
               + first_lidar_header - first_lidar_receipt_steady

每次新启动重新建立，不读取上一包拟合偏移。所有计算保留整数纳秒，转换的是原wheel采样戳，不用回调now替换采样时间。原`/wheel/odom`和位姿/twist不改；派生数据发布到`/fast_livo/navigation/wheel/odom`，只对公共桥接重映射此输入。定位器、原前端、原包不改时间。

至少10帧、1秒后允许转发；receipt超过0.5秒不新鲜则暂停转发，新鲜数据恢复后可继续。主机时钟步变超过20ms、雷达与固定锚点偏差超过100ms、锚点变化、计数/传感器或wheel采样时刻回退会锁定本次转换，需新开会话；不会通过重建锚点让旧定位继续就绪。wheel原采样时间必须在主机当前时间之前且年龄≤0.5秒，frame必须是odom/base_link，wheel和receipt各需唯一发布者。暂停转发后，公共桥接按已有wheel过期规则撤销就绪；不会凭心跳更新位姿。

**此方法建立软件时间轴的一致关系，首帧接收延迟造成的固定偏差尚未测得，因此不代表物理同步。** 100ms是停止转发门限，不是精度保证。长期漂移暂采用检测后停止，没有增加自适应拟合框架。源进程或设备重启后重新开会话。

在线入口`localize_fast_livo2.sh`增加显式`--public-tf --navigation-map PATH`；复用现有地图服务、静态TF、C1适配器与公共桥接，不再建另一套TF实现。核验占据图与定位归档的来源关系，以及C1/静态TF/适配器工作外参一致；启动前检查公共输出占用。额外进程纳入原受管会话清理，新增话题进入同次录包，`wheel_clock.json`记录状态与锚点，配置和关键源码SHA进入provenance。

现有默认仍为私有定位，入口默认dry-run。公共模式消费同一隔离domain内已存在的真实host-clock wheel，不自行启动底盘。`--replay-view`与公共在线模式互斥；旧包完整公共链路使用既有专用离线回放入口。

## 验证结果

证据目录：[live_clock_20261109](fast_livo2_evidence/live_clock_20261109/)。

- `PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -p test_fast_livo_live_clock.py -v`：退出0，4项通过。覆盖纳秒采样转换、建锚/断流恢复、跨context拒绝、主机跳时、传感器偏差、锚点/计数回退、未来wheel拒绝。
- `source /opt/ros/humble/setup.bash`后，`ROS_DOMAIN_ID=227 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 python3 tests/test_fast_livo_live_clock_ros.py`：退出0，1项真实ROS转发检查通过。收到转换后的原采样戳和原位姿，receipt断流后停止新增wheel，TF/速度发布者均为0。
- 只读`audit_recorded_clock.py`：退出0，1801条雷达/9005条wheel；以bag记录时间近似单调接收时刻，8947条wheel满足转发条件、启动建锚期间58条暂缓；雷达残差约−3.31至20.52ms，无锁定。这是实际包的接收时间代理审查，不是该包真实在线receipt重放或物理同步证明。
- 在线公共/原私有两种dry-run退出0；依赖图、唯一组件名、公共桥接wheel重映射、原私有默认检查通过。Python AST、shell语法检查通过。无C++改动，不需要重编定位器或底盘。

前一轮180秒实际C2/C1→公共TF全链路仍见[报告](fast_livo2_offline_wheel_chain_report_20261109.md)。本轮没有把代理时钟检查冒充一次新的完整真实公共TF试验，也没有重新报告原速/绝对定位精度。

## 后续实测入口

预览以下命令不会启动设备；需要同domain215的wheel源，真实执行前按受管入口检查其归属、TF关闭及修复驱动版本：

```bash
bash scripts/localize_fast_livo2.sh \
  --archive outputs/fast-livo-demo-online-mapping-20261105/full_replay_12/map \
  --public-tf \
  --navigation-map outputs/fast-livo-demo-navigation-prep-20261106/navigation_map_final \
  --domain 215 --seconds 90 --visualization none --dry-run
```

显式`--execute`才会启动传感器/定位/公共TF，属于后续实物操作；Agent本轮未执行。该次短测先静止，再由用户遥控直行、转弯、停车，核对修复驱动真实wheel符号、时间残差、车体朝向、定位/公共TF及断流撤销。现场每会话映射、接收偏差、真实定位精度和长时稳定性仍PARTIAL；规划控制后续按用户新设计开展。
