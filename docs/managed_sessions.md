# 受管会话与时间对齐（批次 3）

更新：2026-10-20。适用于 `refactor/pluggable-navigation` 分支。此文档描述已完成的代码和离线验证，不代表已执行真机验收。

## 1. 定位与轮速如何按采样时刻组合

公共桥接缓存最近 2 s 轮速位姿，最多 1000 条，使用整数纳秒索引。收到采样时刻为 t 的标准定位消息后：

1. 找到覆盖 t 的轮速记录，精确命中时使用实际记录，否则平移线性插值、四元数按最短弧 SLERP。
2. 只允许缓存内部插值，邻接轮速记录间隔不得大于 0.1 s。不在缓存范围内或出现大间隙时不外推。
3. 计算 `T_map_odom = T_map_base(t) × inverse(T_odom_base(t))`。创新检查也比较两组同刻配对的相对运动。
4. 每次新轮速回调，在实际轮速时间戳发布当前 odom→base_link 和缓存的 map→odom 修正；合成得到的 map→base_link 因而随最新轮速更新。

原始定位采样时间仍单独用于新鲜度判定，传播修正不会伪造新测量。同一采样时间的定位心跳不重复计算、不重复评分创新检查，也不刷新采样年龄。

正常情况下，新定位消息可能比最新轮速早到一帧：先等待下一条轮速形成插值区间。等待期间只允许继续使用仍新鲜的已对齐修正；缺少修正或等待超出最大轮速间隙时就绪为 false。过旧缓存、大间隙、位姿时间回退和 ROS 时钟回退均有明确处理，禁止用最新轮速冒充历史值。

轮速时间回退会重建缓存并撤销当前对齐；定位时间回退拒绝该消息并撤销修正，等待有效新样本；ROS 时钟回退清除全部位姿/轮速/修正缓存。重复轮速时间戳不替换已保存的位姿。

参数位于 `config/runtime.yaml`：`odom_history_s=2.0`、`odom_max_gap_s=0.1`，位姿年龄仍为 0.5 s；缓存时长不得小于最大定位年龄和插值间隙。对应环境覆盖为 ODOM_HISTORY_S、ODOM_MAX_GAP_S。诊断新增 time_aligned、alignment_reason、aligned_sample_time。

本机制假设位姿和轮速使用相同时间基准且原始时间戳正确，**不会修复雷达主机接收时间、逐点时间回退或相机硬件同步问题**。

## 2. 公共 TF 与算法私有位姿

受管现场入口采用：

```text
map → odom → base_link → imu_link → livox_frame
  └→ lightning_imu_pose（Lightning 原生算法位姿，仅供适配器读取）
```

公共桥接拥有 map→odom 与 odom→base_link；sensor_mount 发布 base_link→imu_link，sensor_tf 发布 imu_link→livox_frame。安装外参仍是原先用户近似值，未通过本批变成实测标定。

Lightning 的 `--tf_child` 与适配器 `loc_child_frame` 由 runtime 的 `native_pose_frame=lightning_imu_pose` 统一选择。该帧不是物理 imu_link，避免给同一个物理帧同时指定 map 与 base_link 两个父节点。公共物理传感器链因而也可服务 standard 后端。

LOC_CHILD_FRAME（旧 TF_CHILD 兼容名）可选其他私有帧，但不允许 map、odom、base_link、imu_link 或 livox_frame。旧 BRIDGE_EXTRA_ARGS 中的 loc_child_frame 同样经过校验。直接运行 Lightning 的旧外部脚本也需要相应改用私有帧；不应与已存在的旧 map→imu_link 发布者同时运行。

## 3. 操作入口

入口统一为只读计划默认值：

```bash
bash scripts/field_session.sh
bash scripts/run_navigation.sh
bash scripts/field_aux.sh
bash scripts/start_base_only.sh
bash scripts/start_lidar_only.sh
```

输出 JSON，包含组件 argv、依赖、就绪检查和所需资源；不会启动 ROS 或创建会话目录。

实际执行现在是**前台受管会话**，不再创建多个终端标签页或静默 nohup。含底盘的现场会话和导航会话要求该次测试的明确授权，并显式传入 --control-authorized；此参数不存成下次启动许可，也不触发 motion_enable。以下命令仅供已经获得该次真机测试授权的操作员使用：

```bash
bash scripts/field_session.sh --execute --control-authorized
# 在另一终端，定位及传感链健康后：
bash scripts/run_navigation.sh --execute --control-authorized
```

新导航进程的门禁默认无运动许可；后续使能与目标操作仍遵守原逐次授权流程。程序不发送运动使能，也不自动恢复进程。雷达/可视化等不含底盘控制的会话只需 --execute；本批自动测试仅在隔离 domain 中使用合成传感器及不带底盘的配置。

查看/停止本次会话：

```bash
bash scripts/field_session.sh --status
bash scripts/run_navigation.sh --status
bash scripts/run_navigation.sh --stop
bash scripts/field_session.sh --stop
```

Ctrl-C 也会清理该会话。--stop/--status 直接读取本地状态，不要求地图配置仍可解析。它们只操作本管理器建立的相应会话，不寻找或停止旧版 nohup、其他工程或人工启动的进程。

组件开关仍在 field_session.env；初始位姿、阈值和地图在 runtime/地图清单中。standalone base/lidar/aux 使用同一资源租约，不能抢占仍由 field 会话持有的硬件或静态 TF。启动已有资源会明确拒绝；需要先停止旧会话，再在新授权范围内启动目标会话，不支持本批内部组件自动补起。

## 4. 启动、降级和退出

- 每个会话先启动只读 ROS 探针。启动前检查关键接口是否已被发布者占用；已受本管理器管理的组件还通过资源文件锁互斥，禁止重复控制底盘。
- 按就绪条件启动组件，而不是固定等待：雷达点云/IMU 流、轮速采样年龄、私有定位 TF、公共 navigation_ready、完整物理传感器 TF 链。
- 导航启动还要求外部定位、雷达和传感器 TF 健康；就绪依赖真实 GetState 服务确认六个生命周期节点 ACTIVE、地图和最终速度发布者唯一、门禁和过滤点云心跳存活。
- 核心顶层进程退出、探针退出或启动超时，停止整个本会话，不自动重启。可视化等非核心组件退出报告 DEGRADED。
- 现场会话数据暂失时保持进程但报告 DEGRADED，由公共就绪与既有门禁阻断；数据恢复后现场会话可以重新成为 RUNNING。导航会话关键健康持续丢失 1 s 则停止导航栈并释放其门禁进程，需要人工新启动。短暂输入波动的门禁阻断/恢复保留原实现，未宣称每次瞬态都重新授权。

资源锁只协调同一个会话状态根目录内的受管会话。ROS 发布者检查不能替代对其他无 ROS 控制程序的人工核查；不会自动杀掉外部程序或更改 CAN/网卡配置。

## 5. 进程身份、日志与故障边界

默认状态根目录 `.runtime/sessions/` 已加入 Git 忽略；每次生成新的 run-* 目录，不覆盖历史日志。current.json 指向本次目录，state.json 保存 guardian、调度进程、组件 PID、Linux start_ticks、boot_id、运行状态和已归属子进程，events.jsonl 保存启动/失败/退出事件；组件及 ROS 日志也在同一运行目录。

管理器使用 Linux subreaper 追踪子孙进程，即使子进程调用 setsid 也归属本会话。独立 guardian 在调度进程意外死亡后收养并清理其子孙；guardian 意外死亡会通过父进程死亡信号要求调度进程退出。发送信号前核对 PID/start_ticks/boot_id，陈旧 PID 不作为当前进程处理。

正常退出只向顶层启动器发送一次 SIGINT，留出 16 s 正常收尾，再对剩余的本会话后代逐步 TERM/KILL、回收。不会按进程名字匹配或向其他会话广播信号。状态中的精确进程身份可用于 stop 的补救判断。

同时 SIGKILL 两个监督进程、掉电或内核故障不能由这一用户态管理器完全处理；若两者都死，只允许清理状态中已记录且身份仍匹配的进程，不声称能找回最后一次记录之前产生的所有孤儿。这种更强保证需要后续 systemd/cgroup 部署，不能由本批离线测试代替。RANGER_SESSION_ROOT 仅用于明确的独立运行/测试目录；隔离测试还必须使用独立 ROS_DOMAIN_ID，不能仅靠换状态目录隔离硬件。

## 6. 验证范围

测试包含已知延迟的加速/转弯 6DoF 轨迹、插值数值、四元数跨 ±180°、缓存边界/容量、时间回退和原采样时间冻结。进程测试覆盖异常退出、setsid 子进程、任一监督进程被 SIGKILL、重复会话/资源租约、陈旧 PID、依赖等待和无关进程保留。

真实入口的 ROS 联调使用合成点云/IMU/轮速/定位，未启动物理驱动。它验证 field→Nav2 生命周期就绪、最终输出保持零、输入中断导致现场降级和导航停机、清理无残留；不证明真机导航、停车距离、精确标定、坡道或传感器同步已通过。
