# FAST LIVO2 在线预建图

更新：2026-11-05。`map_fast_livo2.sh` 将三融合前端、B 回环后端、原始与去畸变数据录制、优化地图显示及停止封存放在同一个受管会话中。正式参数为 `img_time_offset=+0.1 s`、初始重力对齐开启、Tag 关闭。验证结果见[交付报告](fast_livo2_online_mapping_report.md)。

## 构建与启动

项目根目录执行：

```bash
# 本机构建，复用已锁定的项目内依赖，不打开设备
bash scripts/build_fast_livo2_online.sh --execute

# 显示完整计划，不启动设备
bash scripts/map_fast_livo2.sh --dry-run

# 操作员执行：启动雷达、同步 writer、相机、前端、后端与录包
bash scripts/map_fast_livo2.sh --execute \
  --seconds 900 --label corridor_classroom_map02 --min-free-gb 100
```

入口默认读取 `config/profiles/fast_livo2_frontend.yaml`，独立启动本次传感器。默认900秒，`--seconds` 从录包器就绪开始计时。执行仍须显式 `--execute`；只有专用 `record_fast_livo2.sh` 按用户既有指令默认执行。启动不包含底盘、CAN、Nav2、公共定位桥接或公共 TF；采集运动由操作员控制。

统一入口选择 FAST profile 后，在线模式也转到本入口：

```bash
bash scripts/run_mapping.sh online --execute \
  --runtime-config config/profiles/fast_livo2_frontend.yaml \
  --seconds 900 --label corridor_classroom_map02 --min-free-gb 100
```

`MAP_DIR` 是本次全部资产目录，默认位于 `bags/fast-livo-map-<label>-<随机串>/`。`--output /绝对路径/新目录` 可指定位置，父目录须存在，已有目标拒绝覆盖。默认预留20GB磁盘，可用 `--min-free-gb` 修改。该预算包含原始图像、前端输出、后端分片及最终地图；未启用压缩和分卷。

出现 `MAP MAPPING` 及“在线建图已就绪”后，先静止15秒，再进行人工采集。回环需要满足候选、几何、一致性与优化提交检查，不保证每次回到旧位置都会接受回环。终端显示前端配对数量、图版本、回环事件数量、队列及处理延迟；短时不新鲜会显示 `DEGRADED`，不会刷新旧位姿的采样戳来冒充有效定位。

建图模式在输入预检期间就启动录包器的发现/写入，尽量保留前端初始化阶段的原始数据；全部必需话题收流且录包器订阅就绪后才开始计算请求时长。因此文件的实际时长可以包含额外的启动与收尾片段。

建图收流由轻量C++节点对序列化消息计数，Python仅接收`/fast_livo/capture/input_receipts`的低频计数与单调接收时间，不再复制高频IMU和大图像来判断断流。该计数绑定本次输出目录身份；节点退出或心跳失效会撤销会话就绪，原2秒断流门槛保持。

## 显示与停止

`--visualization auto` 在有 `DISPLAY` 时启动 RViz，否则启动只绑定 `127.0.0.1:8765` 的只读 Foxglove Bridge。可选 `rviz|foxglove|both|none`，显式RViz而没有显示环境时在启动设备前拒绝。

RViz 使用 `config/fast_livo2/mapping.rviz`；Foxglove 3D面板固定坐标系选 `map`，订阅：

| 话题 | 内容 |
| --- | --- |
| `/fast_livo/backend/view/map` | 按关键帧优化位姿重建的显示点云，最多20万点 |
| `/fast_livo/backend/optimized_path` | 优化轨迹，显示点数有界 |
| `/fast_livo/backend/view/loops` | 已接受的回环约束连线 |
| `/fast_livo/backend/view/status` | 显示图的会话、图版本、采样点数及生成结果 |
| `/fast_livo/backend/estimate`、`health` | 后端原采样位姿、状态及队列/错误计数 |

显示点云约每3秒更新，使用独立线程与有界缓存；显示采样不修改最终归档。原始前端轨迹位于 `fast_livo_local`，不应在没有正确变换时叠加到 `map`；本显示配置只直接叠加同在 `map` 中的结果。Foxglove 的客户端发布、服务和参数访问均关闭。

```bash
bash scripts/map_fast_livo2.sh --status
bash scripts/map_fast_livo2.sh --stop
```

Ctrl+C、定时结束和 `--stop` 都先进入 `FINALIZING`：封存本会话的新输入，补齐已接收的同戳配对，排空后端队列，导出并检查地图，再结束组件。前端发布者必须在封存期间保持存活，避免旧逻辑将其消失解释成新会话。终端显示收尾进度，最终出现 `MAP COMPLETE` 才表示本次流程完成。后端失败时会保留原始/部分数据与失败原因；成功的原子快照不单独代表整次录制或实时验收通过。

## 记录与地图

在线实物模式录制原有六话题，并增加 `/fast_livo/deskewed_imu`；后端估计、健康、优化轨迹、显示状态和私有 `/clock` 同时保存。单独 `record_fast_livo2.sh` 也已增加去畸变云，但仍只启动采集与前端；需要在线回环时使用本建图入口。

| 文件 | 用途 |
| --- | --- |
| `raw/` | 原始输入、同戳前端位姿与去畸变云、后端状态 |
| `map/map.pcd` | 完整关键帧点云，逐帧应用各自的优化位姿 |
| `map/keyframes/` | IMU机体系完整/匹配点云及后端输入CDR |
| `map/original.tum`、`optimized.tum` | 关键帧原始与优化轨迹；权威整数戳在manifest中 |
| `map/manifest.json`、`COMMITTED` | 地图UUID、会话、图版本、全部因子及文件校验 |
| `map/candidates.jsonl` | 回环候选、拒绝原因、配准与图目标记录 |
| `config/`、`sources/`、`provenance.json` | 冻结参数、来源快照、实际二进制及来源哈希 |
| `source_identity.json` | 传感器配置、时间契约、前端来源身份的明确构成与摘要 |
| `mapping_result.json`、`audit.json`、`session_logs/` | 封存边界、处理数量、数据核验及进程日志 |
| `frontend_logs/` | 本次前端固定路径日志的运行前备份与运行后快照 |
| `MAPPING_COMPLETE` | 本次封存、录制和数据检查成功标志 |

`map.pcd` 是后端关键帧地图，包含每个已保存关键帧的全部点；它不是每个前端区间点的无重复并集。所有区间的去畸变云和位姿另保存在 `raw/`，后续可直接导出完整前端点云或重新运行后端，不必重新执行三融合。地图文件不混入轨迹装饰点。

私有后端继续按整数纳秒严格配对 `/fast_livo/odometry` 与 `/fast_livo/deskewed_imu`。云已经在对应时刻的IMU机体系，后端不再应用雷达—IMU外参或重复去畸变；回环后逐关键帧重建历史地图。PGO结果不回灌前端滤波器及视觉局部地图。

## 时间与后续定位

本机传感器戳与主机/wheel仍处于不同历元。实物建图模式由首个雷达header建立固定的单调时钟锚点，持续推进隔离域内的 `/clock`；不跟随最新处理位姿反复重设时钟，传感器停止后时钟仍继续推进。原始header和逐点时间保持原样，主机单调时钟另行检查断流和积压。这提供纯建图的时间基准，不等于物理触发、曝光中心或主机/wheel同步已经校准。

会话显式加载冻结的`config/mapping_fastdds.xml`：256MiB共享段、8192项端口队列，以及仅回环地址的UDP发现/后备传输；继续使用同步发布和原RELIABLE数据契约。配置只作用于本会话进程，不修改系统网卡、内核缓冲或全局ROS设置，退出即回到调用者原有环境。本机Fast DDS 2.6默认共享段为512KiB，官方提示段大小接近或小于消息时有数据丢失风险；本项目RGB图像单帧约3.9MB，依据见[官方共享内存说明](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/transport/shared_memory/shared_memory.html)。联调曾观察到真实接收记录的长间隔，即使header连续也不能认定收流及时；本配置的整包结果与剩余限制见交付报告。

新地图带有 C1/C2 已支持的三项来源身份。C1/C2 入口新增 `--identity-file`，传入与查询前端实际配置/版本对应的身份文件；加载器比较身份及文件校验，物理精度仍单独验收。`--identity-file` 与旧图的 `--compatibility` 离线兼容声明互斥，不应把任意地图自身的身份盲目当作当前查询前端的身份。

既有C1/C2仍是独立定位模式；本次在线预建图不会启动它们或把 B 当作已有地图定位器，也不会改动生产 Lightning 默认。

## 无设备重放

可在新目录创建三路融合输入的header-clock调度视图。原序列化消息逐字节保留，仅派生bag的存储调度时间变化；原包不修改。`/front/image` 与 `camera_info` 使用主机历元，不进入该三路调度视图，仍保留在绑定的原始包中。

```bash
source /opt/ros/humble/setup.bash
python3 scripts/fast_livo2/make_replay_view.py \
  /绝对路径/原包/raw /已存在父目录/新视图 --seconds 400

bash scripts/map_fast_livo2.sh --execute \
  --replay-view /已存在父目录/新视图 \
  --domain 210 --visualization none --seconds 900 \
  --output /已存在父目录/新建图结果
```

视图目录必须有核验后的 `view_manifest.json`。重放只播放原始融合输入，忽略原包中的旧前端里程计，重新运行实际前端与后端，并按原速自动在输入结束后封存。此模式不启动驱动、同步writer、相机、wheel或控制节点。离线DDS重放证明软件链路和本机负载条件下的行为；实物网络、外触发与长时稳定性仍须真机验收。
