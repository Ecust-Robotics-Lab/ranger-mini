# 同戳、扫描相位与正式 +100 ms 配置核查

日期：2026-11-04。基线 HEAD `acffa879cc8d52330276089cd3df83ad6a43f112`，未提交工作树保持。范围为上游/本机源码审查、两份已有包只读分析及用户明确要求的正式前端试验配置修正；没有启动设备、底盘、ROS 回放或导航，没有提交/推送。

## 结论与本次修改

**最有证据支持的主因是：相机复制到的是刚发布的雷达包的起始戳，而雷达已经花约 100 ms 收完该包。硬件触发可以对齐采集节拍，却不会自动修正这条软件赋戳链的扫描相位。** 当前新包没有复现偶发漏写共享戳；传感器与主机的历元差确实存在，但当前三融合前端的输入/位姿使用传感器时间域，不支持把该历元差直接归因为内部约 100 ms 的相位误差。

用户在本次调查中明确要求“以后我们正式试验配置也要是 0.1（100 ms）”。已将 `config/fast_livo2/ranger_mid360s_hik_trial.yaml` 的 `time_offset.img_time_offset` 从 `0.0` 改为 **`0.1`**，修正顶部注释。其他参数语义不变。受管采集/预览默认读取这份文件，未来新建会话生效；本次没有启动或重启会话，不声称活动节点已动态更新。

仅在前端图像回调执行 `t_image_effective = t_image_header + 0.1 s`；原始 `/front/image_sync`、点云/IMU header、原 bag 和逐点时刻均不改。`exposure_time_init`、LiDAR/IMU 偏移仍为 0。相机驱动不再额外加 100 ms。历史零偏移和 +110 ms 实验、原地图及失败证据保留，不能把它们重新标记为新 +100 ms 结果。

`config/fast_livo2/apriltag_camera.json` 保持 `offset_ns=0, time_verified=false` 的既有诊断配置，Tag 仍默认关闭；它不能被当作与新前端匹配的已确认时间映射来提交候选。以后启用 Tag 时，若观察器读的是原始 image header，须为该输入单独明确 +100 ms 映射、版本与剩余不确定性；原图分别进入前端和 Tag 时各自映射一次，不在已修正的位姿戳上再加一次。历史标定来源 hash 保留为历史来源，不因只修改时偏而改写历史观测。

## 上游直接证据

本机已锁定的 LIV_handhold_2 参考源码 commit 为 `30507028a7c00c99ac992322507701e0efe72a2a`。核对其本地文件与在线来源：

- [锁定版本 mid360.yaml](https://github.com/hku-mars/LIV_handhold_2/blob/30507028a7c00c99ac992322507701e0efe72a2a/FAST-LIVO2-Mid360-Config/config/mid360.yaml)：`img_time_offset: 0.1`。此前本项目正式试验值为 0，二者确实不同。
- [相机 grab_trigger.cpp](https://github.com/hku-mars/LIV_handhold_2/blob/30507028a7c00c99ac992322507701e0efe72a2a/mvs_ros_driver/src/grab_trigger.cpp)：成功取图后读取共享 `pointt->low` 赋给 ROS header；不是直接将相机曝光事件的设备时刻映射到雷达域。
- [雷达 lddc.cpp](https://github.com/hku-mars/LIV_handhold_2/blob/30507028a7c00c99ac992322507701e0efe72a2a/livox_ros_driver2/src/lddc.cpp)：组帧 `base_time` 写入点云 header 和共享值。上游将共享写入放在雷达驱动内，本工程由独立 writer 订阅点云完成，不能将两者调度路径视为完全相同。
- [FAST-LIVO #96](https://github.com/hku-mars/FAST-LIVO/issues/96)：作者在 2024-06-01 的解释正是“雷达整帧采集后发布，短曝光相机收到时复制上一扫描起始戳”的机制；该历史设备建议相机加 0.1 s。它是机制证据，不是本机曝光相位的实测证明。
- [FAST-LIVO2 #163](https://github.com/hku-mars/FAST-LIVO2/issues/163)：作者确认共享块持续保存最新雷达时间；“最新收到”不等于“当前曝光采样时刻”。
- [LIV_handhold_2 #13](https://github.com/hku-mars/LIV_handhold_2/issues/13)：维护者说明同步设备使用虚拟 UTC，和系统时间/其他设备协同时可能不在同一时间基准。

## 本机实际调用链

| 环节 | 实际文件/实现 | 含义 |
| --- | --- | --- |
| 雷达组帧 | `ws_livox/src/livox_ros_driver2/src/comm/pub_handler.cpp`，`CheckTimer`、`GetLidarBaseTime` | 发布频率 10 Hz；收齐区间后发布，base_time 取首点 |
| 雷达 header/逐点时间 | 同包 `src/lddc.cpp`，`InitPointcloud2Msg` | header 是 base_time；逐点 timestamp 为绝对纳秒 |
| 共享文件 | `ws_camera/src/timeshare_writer/src/timeshare_writer_node.cpp`，`onScan` | 收到 ROS 点云后将 header 作为 latest 写入，v2 seqlock 保障完整快照；不提供曝光/触发序号匹配 |
| 取图及原图 header | `ws_camera/src/hik_camera_trigger_driver/src/hik_camera_node.cpp` | SDK 取图、颜色转换后 `/front/image` 与 CameraInfo 使用主机 now；设备 tick 另存 CSV |
| 同步图 header | 同文件 `readTimeshareConsistent`、`processSyncFrame` | 校验共享值新鲜/单调后，复制到 `/front/image_sync`；没有重建曝光时间 |
| 前端图像有效时间 | 锁定前端加项目补丁 `src/LIVMapper.cpp`，`img_cbk`、`sync_packages` | `header + img_time_offset`，再加 `exposure_time_init`；按这一时刻切分点云和 IMU、先 LIO 后 VIO |
| 前端位姿/后端点云 | `publishBackendFrame` | 使用同一 `last_lio_update_time`，两话题严格同戳；不是处理完成的主机 now |

已复核当前隔离源码 `ws_fast_livo/independent_time_source_20261104/src/LIVMapper.cpp` 的加法方向；对正式配置只改参数，没有替换安装前端或将隔离实验副本部署。

## 两份新包的只读证据

输入为转向 `fast-livo-tag140_turn_03-bk8_pjyy` 和直行 `fast-livo-tag140_straight_04-ryzykq2l`。脚本 `fast_livo2_evidence/timestamp_phase_20261104/investigate.py` 以 SQLite `mode=ro`、`query_only=ON` 读取，每包完整 SHA 在分析前后核对；结果 `turn_timing.json`、`straight_timing.json`、`timing_summary.json`。

| 指标 | 转向包 | 直行包 |
| --- | --- | --- |
| 实际录入点云帧 | 900 | 900 |
| 末点减 header，中位数 | 99.830060 ms | 99.830034 ms |
| 末点减 header，范围 | 99.829838～100.325242 ms | 99.829838～100.325194 ms |
| 相机会话 CSV 帧 | 924，全 OK | 919，全 OK |
| 取图到达时消费最新 writer 序号 | 924/924 | 919/919 |
| writer 写入到 SDK 取图到达，中位数 | 22.036889 ms | 22.144270 ms |
| 已录扫描在 writer 中缺失 | 0 | 0 |
| 会话曝光设置/实际日志 | 20000 us | 20000 us |

点云 timestamp 是 epoch 纳秒 FLOAT64，约 256 ns 数值间距；上述微小小数变化不用于宣称亚微秒物理精度。SDK 设备 tick 单调及帧号连续的复核见上一轮 `tag_two_capture_20261104/*_delivery.json`。

本次还将每个话题的 `record_time-header` 减去同包 IMU 的中位数，消除大历元常量，以观察打包/传输时序。雷达相对到达中位数约 102.60/102.57 ms；由同一主机 writer 的 steady/system 对应估计，SDK 取图到达约 124.34/124.32 ms（`sdk_arrival_relative_clock.json`）。图像录入 bag 的对应值反而约 140.51/140.78 ms，直行单帧最高 545.34 ms。这些还含读出、传输、颜色处理、序列化、队列和 IMU 自身到达延迟，**都不是曝光相位**，不能把 124/140/545 ms 再硬编码成时间补偿。

两包 LiDAR/IMU 日志均为 `time_type=2`，本机枚举是 GPS 同步时间类型；传感器 header 为 2020 年历元、主机/录包时间为 2026 年。GPS 类型不证明正在接收真实 GNSS UTC，也不证明触发与曝光相位；虚拟 GPRMC 可以产生这种类型。

## 对三个原因的判断

1. **扫描区间与曝光区间：事实成立，但区间长短本身不是错误。** 雷达逐点时刻与去畸变可以处理约 100 ms 的扫描；关键是图像被标成哪个参考时刻。若扫描起点是 t、整包在 t+100 ms 左右发布、相机此时触发并曝光 20 ms，复制出来仍是 t。该条件下曝光中心可能约 t+110 ms，能解释既有 +110 ms 最优区域；尚未实测触发沿、TriggerDelay 和 device tick 的事件语义，不能宣布“恰好 100+10”已证实。
2. **读取上一帧戳：固定相位错配最有支持；偶发旧值不是这两新包的主要证据。** 相机读到最新 writer 值和取到物理上前一个扫描区间的起始戳可以同时成立。v2 协议保障不撕裂/不重复消费，RELIABLE 订阅缓解漏写，均不能解决固定整帧语义差。旧 60 秒包确有 7 个漏写扫描/重复共享戳，对应另一类已经留证的偶发问题，不能与新包的固定约 100 ms 混为一谈。
3. **不同时间基准：对系统/公共 wheel/导航接入是已知事实；当前内部约 100 ms 不优先归因于此。** 前端实际输入是传感器域的 `/front/image_sync`、LiDAR 和内置 IMU；输出也沿用有效采样时刻。`/front/image`、CameraInfo、bag 存储时间和主机/wheel 不能直接与之按数值混配。C1/C2 必须保留前端采样戳；不能用新到达的机器人位姿或 now 与旧曝光图拼接。

因此，不应将残差全部归给 Tag 或全部归给时间。上一轮固定 Tag 常量、只改已有 C1 注册器后明显改善，已说明定位本身也贡献不一致；时间修正、地图/配准和像素/标定误差需要分别记录。

## 核验命令与边界

所有命令在仓库根目录执行；完整结果位于 `docs/fast_livo2_evidence/timestamp_phase_20261104/`。

| 命令/检查 | 结果 | 日志/证据 |
| --- | --- | --- |
| `/usr/bin/python3 docs/fast_livo2_evidence/timestamp_phase_20261104/investigate.py --execute` | 退出 0，2 包/1800 点云；原包 SHA/大小/mtime 保持 | `investigation.log`、`timing_summary.json` |
| `bash scripts/record_fast_livo2.sh --seconds 60 --visualization foxglove` | 退出 0，DRY-RUN；正式配置正确选入，未开设备 | `record_dry_run.log` |
| `PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m unittest discover -s tests -p test_tag_imu_timing.py -v` | 退出 0，6/6 | `timing_tests.log` |
| `PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m unittest discover -s tests -p test_fast_livo_tag_consistency.py -v` | 退出 0，5/5 | `association_tests.log` |
| `PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 docs/fast_livo2_evidence/timestamp_phase_20261104/check_config.py` | 退出 0；YAML 语义对照、domain 211/localhost 隔离参数节点实际读到 0.1；节点已销毁，只改一项参数语义 | `configuration_check.log`、`configuration_check.json` |

核验过程中曾尝试导入新版 rclpy 的 `parameter_dict_from_yaml_file`，本机 Humble 无此 helper，ImportError 退出 1，尚未执行配置写入或启动节点；随后改用 Humble 实际支持的隔离参数节点确认成功。该工具 API 不兼容不是前端运行失败，也未因此升级依赖，记录为 `configuration_api_probe.json`。

PASS：上游与本机赋戳机制审查、两包扫描区间及最新 writer 对应、用户要求的正式 +100 ms 参数、11 项既有时序/几何回归、原包完整性。

PARTIAL：曝光参考点/触发延迟和剩余不确定性、不同曝光/启动后的稳定性、真实 Tag 地标/辅助恢复/回环、绝对精度。没有新 FAIL/阻塞。

未实施：用新 +100 ms 正式配置重新生成整套地图/定位性能结果、修改 Tag 时间有效标志、硬件测量/驱动启动、公共桥接/导航、Git 提交推送。已有 +110 ms 和 GICP 结果仅属于各自归档配置。后续应先用已有两包做固定 +100 ms 的独立对照，再决定剩余约 10 ms 是否值得做物理测量；不要求先重录同类包。若需物理定值，核验同步器触发沿与相机 ExposureActive/曝光事件及雷达参考时刻，记录 TriggerActivation/TriggerDelay、tick 频率和事件语义；SDK 字段存在不代表本机已启用或已有有效值。

回退仅需从本轮 `.runtime/backups/timestamp_phase_20261104_222024/` 找到原试验 YAML，按需要恢复 `img_time_offset: 0.0`；不自动回退用户选定的 +100 ms。启动已有旧会话不会热加载 YAML，须在下一次获授权的新建会话中核验实际参数。本轮仅修改配置文件，没有主动重启。
