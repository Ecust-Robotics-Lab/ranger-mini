# 同录包C2/C1—wheel—公共TF离线联调

日期：2026-11-09。用户要求立即实施时间映射与180秒包的完整离线链路。沿用已授权工作外参、自有v2＋KISS＋small_gicp；不启动硬件/底盘/控制器，不发送速度/导航目标，原包不改。本轮所有结果属于离线功能检查，物理同步、绝对精度和实车D不据此验收。

## 实现

新增scripts/replay_fast_livo2_wheel.sh，默认dry-run；显式execute才运行隔离ROS链路。消费原包已有1801对FAST三融合位姿/IMU区间云，进入当前实际C2/C1节点，经过现有C1适配器和公共桥接，发布唯一map→odom→base_link。没有用FAST局部里程计冒充地图定位；全局初值由C2确认，没有人工初值。

新增capture_clock.py提供整数纳秒时间转换，绑定本包路径、metadata SHA及分析SHA，保存clock_mapping.json。约203782752743349211ns的传感器→主机偏移只适用于这份采集，轮速头戳在离线派生消息中转换到sensor-clock，原host头戳仍在原包/派生输入数组中。前后窗口lag相差20ms写入元数据，physical_sync_verified=false；没有把这个值写入生产全局时间常量，也没有用now重打原采样时间。

原包wheel来自反馈符号修复前的版本。默认使用已明确标记的诊断重建wheel（平移/航向积分修正，spin方向保留）；raw选项作为负对照。原始位姿/点云内容不改，wheel派生和时钟变换均标明来源。本轮不追认为录得了新版驱动真实输出。

查询采集与地图的外参、相机/雷达配置和时间参数逐项相符，运行入口核验归档文件SHA。查询原前端与地图在线I/O派生前端的实际二进制不同，单独记录两个SHA；采用已核对的离线契约兼容说明，不宣称二进制相同或物理时间已验证。地图仍是UUID253f573d7a7eabbc363a225ab192cc14、graph419。

## 首轮完整结果

首轮chain_01处理1801对，退出0，C2首次VALID为源时间28.800秒，1502个有效扫描、1494个就绪采样；C1状态为初始化288、VALID1502、DEGRADED11，无LOST/ERROR输入采样。首次定位时间包含用户起步前静止时间，且本工具按每扫描确认推进模拟时钟，不作为原速吞吐或冷启动墙钟性能排名。

公共TF检查1493次，代数平移一致性最大约7.4e-16m；这是同采样时刻公式的浮点一致性，**不是机器人定位精度**。145秒处注入0.8秒wheel断流，过门限后就绪撤销，恢复数据后重新就绪。末尾模拟时钟前进0.8秒而不再供数据，旧位姿过期，navigation_ready=false。仅一个动态TF发布者、零速度发布者，三个子进程正常退出。

## 复验、反例与检查器修正

封装入口复跑chain_final中，定位/断流恢复仍正常，但检查器误把新到定位与上一条缓存TF拼成“两个最新值”，一处误报0.02415m，严格检查因此退出2。保留该失败。改为依据TF发布戳，只与不晚于TF、年龄≤0.5秒的已确认定位及对应wheel历史比较，记录匹配源戳、平移和旋转残差，不放宽2cm检查门限。

修复后的完整chain_verified退出0：1801对、1502个有效定位、首VALID源时间28.800秒、1494个就绪采样；1494次TF公式检查最大平移残差7.36e-16m、最大旋转残差0rad，匹配源样本比TF早25.8–85.9ms，均在缓存时效范围内。断流成熟窗口两次就绪均false，恢复后true，尾部旧数据过期false。唯一动态TF发布者、零速度发布者、三个子进程均退出0。墙钟292.77秒，只作确认式回放功能检查，不是原速性能测试。

运行期间代码新增了旋转最大值的结果字段，因此该次原result没有此聚合字段；从已保存的逐条TF trace重新聚合，按当前平移<2cm、旋转<1e-6rad和全部状态条件严格复核通过。原result及实际执行脚本均未改写。详见[汇总](fast_livo2_evidence/offline_wheel_chain_20261109/summary.json)、[严格复核](fast_livo2_evidence/offline_wheel_chain_20261109/checks.json)。

未修正wheel的反例chain_raw_control也处理完整1801对，C1仍有1502个有效扫描，但桥接在约50.2秒因航向创新20.6°超过原20°门限锁定；之后持续撤销就绪，恢复wheel也不擅自解锁。全程只有215个就绪采样，post_gap_ready_recovered=false。退出2是预期反例拒绝，不是需要放宽门限的理由，证明私有定位成功不能覆盖公共桥接的创新拒绝。

## 复现

输出必须使用新目录：

    bash scripts/replay_fast_livo2_wheel.sh \
      --capture bags/fast-livo-wheel_alignment_visit01-pwr5lx7u \
      --archive outputs/fast-livo-demo-online-mapping-20261105/full_replay_12/map \
      --analysis outputs/fast-livo-demo-wheel-alignment-20261106/analysis_20261109_split \
      --identity-file outputs/fast-livo-demo-online-localization-20261106/visit01_auto_pcl_01/source_identity.json \
      --output outputs/fast-livo-demo-wheel-alignment-20261106/NEW_CHAIN \
      --execute

默认隔离domain221/localhost，use_sim_time=true。--domain可选择另一空闲域；--wheel-mode raw用于原始wheel反例。默认145秒/0.8秒的断流注入可通过--wheel-gap-start、--wheel-gap-seconds调整。该入口不消费生产wheel、不启动传感器或前端，不复用真实控制域。

证据：docs/fast_livo2_evidence/offline_wheel_chain_20261109/；大结果在outputs/fast-livo-demo-wheel-alignment-20261106/chain_*。每次保存实际命令、三个进程日志、扫描状态、桥接健康、时钟映射和执行脚本副本。修正检查器的原失败和首轮脚本均保留。

## 检查与边界

- 时间转换单元检查通过：精确整数纳秒、跨包拒绝、metadata变化拒绝、窗口lag差记录。
- 回放验收条件3项检查通过：断流/过期撤销、恢复、TF唯一、速度发布者和创新锁定；私有VALID不能使锁定桥接判通过。
- SIGTERM中断测试通过：退出2，保存中断原因并等待/清理全部三个子进程；子进程启动期的非零退出记录保留。没有全局按进程名杀进程。
- 完整数据重放、严格数值关联复验和raw负对照的最终退出码/统计见summary.json和checks.json。所有ROS测试隔离、无硬件接入。

当前已交付本包的可重复时间桥接与定位—wheel—公共TF离线入口。实际位置正确性没有独立真值；候选时钟/旧wheel诊断重建也不等于实车验收。新版驱动真实短测、在线每会话时间方案及实车公共TF仍需对应硬件授权。既有原包/地图/失败证据/备份保留，生产默认与规划控制未改；本轮尚未提交或推送。
