# FAST-LIVO2 在线预建图交付报告

日期：2026-11-05。统一在线预建图的软件与离线范围已交付：正式+100ms、重力对齐开启、Tag关闭，在约390秒原速DDS重放中完成三融合、B回环、同步录制、地图封存和新进程归档加载。实物设备没有启动，物理同步、绝对精度、连续新鲜度和长时稳定性仍为PARTIAL。使用方法见[在线预建图](fast_livo2_online_mapping.md)。

## 交付行为

`map_fast_livo2.sh`在一个受管会话中运行独立传感器、前端、B后端、C++序列化收流计数、录包和可选RViz/Foxglove。默认dry-run，显式`--execute`才执行；FAST profile的统一online入口也转到它。默认900秒，配置、二进制与来源在启动前冻结。生产Lightning、wheel和公共TF/导航入口保持原选择。

前后端继续使用同戳VIO后IMU位姿与去畸变IMU区间云，不改变融合数学、不反馈PGO到滤波器。新I/O派生前端复用执行器，将空闲轮询5000Hz改为500Hz，成对输出历史为32；锁定vendor工作树及原安装保留。后端在独立显示线程按每个关键帧优化位姿重建采样地图，显示有点数与时间预算。

定时、Ctrl+C和`--stop`先按预期session封存后端输入，补齐边界已有配对，排空已入队任务，原子导出并检查，再清理发布者。后端失败可降级保留采集，失败不生成整次建图成功标记。独立录包入口也新增去畸变云；此前用户原包没有该话题，原包保持原样。

## 来源与执行

输入原包：`bags/fast-livo-corridor_classroom_map01-182fdzyj/raw`，原DB SHA为`b6f7902c1b5fbffbaea69bf241efff540d48c0a85358d1267a4f98d468496137`。派生header-clock视图逐字节保存选中的三路融合消息；它含3903雷达、78068 IMU、3903同步图像。原包的一张领先于首雷达的图像不在视图窗口，使用主机历元的原图和camera_info仍保留在原包；不把视图说成原包所有话题的完整复制。

前端隔离来源在`ws_fast_livo/online_frontend_source`，基于锁定ROS2源码、既有stage-A补丁和新增I/O补丁；程序在`online_frontend_install`。B及收流节点在`online_install`，依赖复用项目内GTSAM/small_gicp，未升级系统库。逐源文件、实际程序、加载库和DDS共享段记录见[来源清单](fast_livo2_evidence/online_mapping_20261105/online_source_manifest.json)、[实际进程与库](fast_livo2_evidence/online_mapping_20261105/final_runtime_linkage.json)。

实际构建与最终整包命令：

```bash
bash scripts/build_fast_livo2_online.sh --execute
bash scripts/map_fast_livo2.sh --execute \
  --replay-view outputs/fast-livo-demo-online-mapping-20261105/replay_view \
  --domain 214 --visualization none --seconds 900 --min-free-gb 10 \
  --label corridor_replay \
  --output outputs/fast-livo-demo-online-mapping-20261105/full_replay_12
```

输入结束后正常封存，`session_exit=0`、`success=true`、`MAPPING_COMPLETE`存在。地图UUID为`253f573d7a7eabbc363a225ab192cc14`，graph419；结果目录不覆盖此前前端图、B失败快照或其他开发失败记录。

交付后交接清理已删除上述可再生`replay_view`，原包与完整结果保留，详情及重建命令见[最新清理报告](handoff_cleanup_online_mapping_20261105.md)。上方是历史实际命令，不能直接覆盖既有`full_replay_12`；再次实验须重建视图并使用新的结果目录。

## 缺陷定位与修复

旧图事务使用截断增量估计和相对1%目标容差，原验收只允许绝对`1e-6`增加。相同历史因子序列在旧库重放出现3次接受后的目标增加，其中含原3→299回环；改用完整回代、最多三次必要的完整求解及严格提交后，414次事务全部接受且增加次数为0。几何、歧义、大修正确认及原验收门槛未放宽。对照数据见[旧库](fast_livo2_evidence/online_mapping_20261105/graph_legacy.json)、[修复后](fast_livo2_evidence/online_mapping_20261105/graph_fixed.json)。原失败快照不追认为通过。

集成早期的启动依赖和参数类型错误已修复。收流监视由C++直接统计序列化接收，Python只处理低频计数与单调时间；绑定本次目录身份，避免大图像转换与高频回调影响监视。回放成功EOF有独立握手，正常发布者退出不会冒充设备故障，未知退出和实物断流仍按原规则失败。

接收停顿不能仅凭连续header排除。失败轮尾段，实际录包器的IMU最大接收记录间隔为1.1116秒，而对应header只前进约4.25毫秒；前端位姿也出现过长间隔。这是接收延迟/批量到达证据，不证明传感器本体停发，也不能一概称为监视误报，见[到达间隔诊断](fast_livo2_evidence/online_mapping_20261105/arrival_gap_diagnosis.json)。

本机Fast DDS为2.6.12。默认512KiB共享段小于单帧约3.9MB图像；依据[官方共享内存说明](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/transport/shared_memory/shared_memory.html)配置本会话256MiB段和8192项端口队列，保留同步发布、RELIABLE数据与原2秒断流门槛。UDP只允许回环接口，未修改网卡或内核参数。最终整包用domain214，此前失败轮多为210；这是交付配置的整体验证，不宣称单变量证明缓冲是唯一原因。

整包收尾时新控制participant曾报SHM注册错误，但UDP回退成功完成服务。随后为低带宽封存RPC显式采用仅回环UDP配置，6秒完整EOF回归通过且无该SHM错误；数据面和算法未再改变。原整包日志的警告保留。

## 验证结果

汇总及逐项数值见[最终核验](fast_livo2_evidence/online_mapping_20261105/delivery_summary.json)。

| 检查 | 实际结果 |
| --- | --- |
| 构建与来源 | 隔离前端、后端及收流节点构建退出0；锁定源码、原补丁与加载路径核对 |
| 固定因子数值重放 | 414次事务，0次接受目标增加 |
| 原速整包 | 3900对前端输出全部录制、后端处理3900对，未报告非法输入或配对容量损失 |
| B图与回环 | 305关键帧、419因子、114条接受回环；0条接受目标增加；48条歧义地点候选拒绝 |
| 原始输入保真 | 3903雷达、78068 IMU、3903图像，逐话题完整序列化payload SHA与视图一致 |
| 实际接收记录间隔 | 完成轮全程最大：IMU12.51ms、雷达101.38ms、图像106.78ms；不是物理同步误差 |
| 地图归档 | 919个文件SHA核对；1829386点；按各关键帧X逐点重建一致性最大差约1.91e-6m，非定位精度 |
| 来源配对 | 3900对整数同戳；305关键帧的原始位姿、云字段/数据与重录包逐份一致 |
| C1/C2共用读取器 | 新进程通过来源身份、文件及结构校验，加载305关键帧/1829386点；不代表独立访问定位成功 |
| 时钟与封存 | 8对真实前端输出检查：旧历元、停流后时钟继续推进、过期位姿撤销、错误session拒绝、封存后发布者退出不重置图，PASS |
| 既有相关回归 | 8项后端输入/图/归档、11项会话、10项统一入口、21项录包检查通过；未新增单元测试用例 |
| 正常EOF与控制面 | 短包完整收尾PASS；最终仅UDP控制面短包回归PASS |

整包心跳含66个INITIALIZING、7763个VALID、45个LOST样本，LOST按原新鲜度规则撤销有效位姿。因此地图/录制/封存功能PASS，本机原速吞吐有证据，但不能把连续实时新鲜度或导航接车标为全面通过。各心跳不是独立位姿测量或新增真值样本。

本轮没有写新算法测试矩阵；重点复用了现有检查、固定失败因子与真实原包。开发中失败记录及中途停止结果全部保留，其成功/失败与正式整包结果分开，不由新代码追认旧结果。

## 交付资产与剩余事项

最终目录：`outputs/fast-livo-demo-online-mapping-20261105/full_replay_12/`。

- `map/map.pcd`：完整关键帧三维地图；每个前端区间的完整云另存`raw/`。
- `map_trajectory.png`：优化关键帧轨迹与回环的XY展示，PCD不裁剪、不混入装饰点。
- `map/manifest.json`、`COMMITTED`、关键帧、候选日志及原始/优化轨迹。
- `source_identity.json`、冻结配置、来源快照、收流计数、会话日志及`mapping_result.json`。

真机在线网络/外触发、物理同步、绝对精度、误环泛化、长时运行和wheel/公共TF接入尚未验收。B仍为位置邻近候选搜索，不冒充C2全局检索。原两90秒包固定+100ms对照与C1/C2独立访问评估仍待进行；已有包足够继续软件验证，无需重录同类包。Tag独立时间与真实VALID表未确认，继续默认关闭。

原包、地图、失败证据、备份和用户原改动保留。原安装前端及vendor源码保持，旧共享固定日志经备份恢复并校验；本阶段没有设备/底盘/导航操作，没有提交或推送。
