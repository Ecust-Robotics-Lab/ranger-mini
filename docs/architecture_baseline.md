# 架构改造批次 1：基线整理报告

日期：2026-10-19。分支：`refactor/pluggable-navigation`。

**PASS（限磁盘基线整理与只读核验）；导航完整验收仍为 PARTIAL。** 本批没有安装依赖、调整运行参数、构建、启动 ROS/底盘或执行运动。基线源码起点为 `1963cc84d9e1c7d448f5be95c9599ead52c13455`；采集时主仓库 HEAD 为计划提交 `e5b2029d67475a15cf8516f7a4069a4206474346`。

## 1. 实际完成与证据

- 核对主仓库及七个嵌套仓库的完整 SHA、分支、origin 和本地修改。
- 保存配置、脚本、补丁、地图产物的 SHA256，核对导航与桥接源文件和安装副本。
- 建立当前默认地图关联清单，核对输入包哈希、生成关系和关键帧数值。
- 对照 Git 历史补齐 9 月 28 日晚至 9 月 30 日的变更事实，明确现场实验参数与文档缺口。
- 审查启动入口后仅运行默认 dry-run，记录输出和退出结果。

证据目录：[`architecture_evidence/batch1_20261019/`](architecture_evidence/batch1_20261019/)。

| 文件 | 用途 |
| --- | --- |
| `baseline.json` | 实际采集快照；含命令 argv、cwd、退出码、输出、仓库版本、配置/补丁哈希、地图清单和比较结果 |
| `collect_baseline.py` | 可复用的只读采集程序；只向 stdout 输出 JSON，不启动 ROS，不修改输入文件 |
| `validation.txt` | 本批静态一致性核验结果 |

快照描述磁盘配置与入口默认值，不代表正在运行进程的参数。复采必须输出到新文件，不能覆盖本次历史证据。

## 2. 源码与安装基线

| 仓库 | 分支/状态 | 完整 commit SHA |
| --- | --- | --- |
| ranger_ros2 | `ranger-local`，干净 | `7b75bc2443a1d6cbd42cc3ef3059cf4897242072` |
| ugv_sdk | `ranger-local`，干净 | `e2f5764acc00a2e556cf3bdc3ea3b206fa326e5b` |
| livox_ros_driver2 | `ranger-local`，干净 | `cc34f0a18f8c43adbf304c71ebd9f8af69440f41` |
| Livox-SDK2 | **detached HEAD**，干净 | `08f523c930b2f0ba1e98a6afaa8d7476bf479908` |
| Lightning（运行源码） | `ranger-local`，干净 | `b973e7fcfef147959dcfa6d78dd7a07a59384c52` |
| Lightning（审查副本） | **detached HEAD**，干净 | `1325fed8fa97a2506878360fc23427ea00a767da` |
| hik_camera_ros2_driver | `main`，两处已有修改 | `4e351d0b096e124cc076b2edf9f6abb7bfb23eda` |

完整路径和 origin 见快照 `repositories`。主仓库没有 origin（查询退出 2），不是采集失败；未添加远程或执行推送。

相机已有修改：`config/camera_params.yaml` 的设备、分辨率、帧率、曝光及增益；`src/hik_camera_node.cpp` 的 camera_info_manager 头文件后缀。其原有 diff 与当前文件哈希已记录，未修改相机仓库。

`config/base_sources.repos` 与 `config/livox_sources.repos` 仍登记官方基点，不能替代上述带本地修改的实际 commit。`patches/` 全部补丁哈希见快照。本次未重新验证每个旧补丁都可独立应用；复现本地源码时以实际本地 commit 为准，不重复叠加补丁。

已检查的 13 个 `ranger_navigation` / `ranger_localization_bridge` 配置、launch、行为树和 Python 模块与安装副本逐字节相同。此结论不扩展为全部 C++ 二进制构建一致性。dpkg 记录 Nav2 `1.1.20`、twist_mux `4.3.0`，完整发行包版本在快照命令输出中。

FAST-LIVO2 与 elevation_mapping_cupy 在本批未获取、未选择部署版本，不列入已验收软件基线。

## 3. 当前默认地图关联清单

本清单是配置审计记录，尚不是可被运行程序加载的新地图包格式。

| 角色 | 实际入口 | 当前默认资产 |
| --- | --- | --- |
| 全局定位 | `field_session.env` 的 `LOC_CONFIG` → `ranger_mid360s_loc_v2.yaml` 的 `system.map_path` | `maps/lightning-offline-2lx5kkho/data/new_map/` |
| Nav2 地图 | `run_navigation.sh` 的 `MAP_YAML`；navigation launch 默认值一致 | `maps/lightning-offline-8cw1afzl/data/new_map/map.yaml` |
| 辅助可视化地图 | `field_session.env`、`field_aux.sh` 与 `publish_loc_map.py` 的默认目录 | `maps/lightning-offline-suej7qsz/data/new_map/`（旧图） |

`ENABLE_MAP_PUB` 在现场会话中默认为 0，因此旧图默认不随现场会话发布；单独运行 `field_aux.sh` 则默认启用辅助地图发布。后者可能显示旧图，并在导航同时运行时形成 `/map` 双发布。本批只登记缺口，不更改开关或资产路径。

当前定位/导航关联的核验结果：

- 两者 manifest 均引用 `bags/mid360-manual-loop-p42bk0yi/raw`、Lightning `b973e7fcfef147959dcfa6d78dd7a07a59384c52`；导航图的前置验收引用明确指向 `2lx5kkho`。
- 两份 manifest 的输入文件哈希完全相同；本批重新计算 `metadata.yaml` 与 `raw_0.db3` 哈希，均与记录一致。
- 外参文件逐字节相同，仍为用户近似值而非精测标定。配置差异仅为启用 g2p5 及地面/障碍带参数：地面 -0.556 m、雷达高度字段 0.556 m、障碍带 0.10–0.80 m。
- 25 个关键帧 ID 和时间戳一致；优化位姿最大平移差 `0.0000713513 m`，最大旋转差 `0.00416482°`。原生点云、索引和轨迹文件并不逐字节相同。
- 导航栅格为 400×496，分辨率约 0.05 m，origin 为 `[-13.6500005722, -7.5, 0]`；发布坐标系由 map_server 配置指定为 `map`。
- 以上支持两次同包运行的坐标数值一致性，**不是独立几何精度、定位或导航验收**，也不证明旧图与新图可以互换。

验收记录缺口必须保留：`2lx5kkho/acceptance.json` 的三维为 PASS（历史用户图审），尺度为 WAIVED，其余部分仍 PENDING；`8cw1afzl/acceptance.json` 全部为 PENDING。旧图 `suej7qsz` 的二维通过记录不能转移给新图。本批未修改任何地图的 acceptance 文件。

## 4. 配置实值与现场实验边界

下表按当前磁盘实值登记，旧注释不能覆盖参数值。

| 项目 | 当前值/事实 | 分类 |
| --- | --- | --- |
| 车体外包络 | 0.74×0.52 m；base_link 原点和精确外参待核对 | 机器人近似基础信息 |
| 全局/局部规划模型 | 圆形半径 0.10 m；padding 0 | **窄口实验配置，不是实际车体轮廓** |
| CM 停车区域 | 四角 ±0.39×±0.28 m；即记录外包络加 0.02 m 边距 | 当前独立防撞配置，非规划可通过性证明 |
| MPPI / 门禁速度上限 | 0.10 / 0.10 m/s | 现场限速配置 |
| 膨胀半径 / 代价衰减 | 0.35 m / 8.0 | 现场调整 |
| 目标位置/朝向容差 | 0.30 m / 3.15 rad | 现场宽松目标判定，不能视作精确到姿 |
| MPPI | batch_size=400、time_steps=32、Ackermann 最小半径 0.60 m | 现场减载参数 |
| 门禁最小转弯半径 | 0.55 m | 保留现有运动约束 |
| 障碍输入 | 直订 Livox CustomMsg，默认 every_n=3 | 源码默认；避免可视化中转负载 |
| 初始定位种子 | x=-0.85、y=-0.31、z=0；qz=-0.951、qw=0.309 | 上次现场摆位配置，不适用于任意冷启动位置 |
| 雷达运行时限 | 3600 s | 到时退出仍需健康门禁响应 |

当前只有一套已选中的导航参数文件；不能把 9 月 28 日文档当成另一套已经可启动的“常规配置”。后续批次再将现场实验策略抽成显式 profile，本批不复制或回退运行参数。

源码核验还发现：

1. `BRIDGE_EXTRA_ARGS="tf_max_age_s:=1.5"` 虽进入命令，但桥接 launch 只转交外参和 min_confidence；源码与安装副本均未转交 tf_max_age_s。按本入口的静态调用链，节点仍使用默认 0.5 s；不是已查询到的运行时参数结论。
2. `field_session.env` 声称全部变量可被环境覆盖，实际 LOC_CONFIG、INIT_X/Y/Z、INIT_QZ/QW、LIDAR_SECONDS、BRIDGE_EXTRA_ARGS 使用直接赋值。统一配置时需明确优先级。
3. 桥接里程计回调对状态新鲜度使用默认 10 s 参数，看门狗却硬编码 1 s；同一状态可能因回调路径不同得到不同 ready 判定。留待公共健康检查改造。
4. 桥接读取最新定位 TF，与当前轮速位姿组合计算 map→odom，缺少采样时刻对齐。留待批次 3 数值测试与修复。
5. 导航注释仍有旧地图名、半径 0.20/0.14、速度 0.20 和 CM 边距 0.08 等描述，与当前值不符；后续配置重构需同步更正。

## 5. 补齐后的历史事实

| 日期/来源 | 可据此认定的内容 | 不能扩大为 |
| --- | --- | --- |
| 9 月 28 日阶段 6 报告与 `safety-ych6w74e/console.log` | 当时隔离测试记录 37/37；runtime_params.json 当时 MPPI vx_max=0.20 | 当前 9 月 30 日配置已经重新通过相同测试 |
| 9 月 28 日晚提交 `1939e6fb96d158e7ef7de214bc984681c3b0ee4a` | 新地图切换；生成关联由本批 manifest/哈希再次核对 | 新导航地图 acceptance 已补齐 |
| 9 月 30 日提交 `6ff318ba99e53f64db4a08f01a71d72720717904` | 直订原始点云、关闭默认可视化中转、MPPI 减载；代码可核对 | 提交说明中 1.5 s 参数已通过 launch 生效 |
| 9 月 30 日提交 `6c72c3063645fbb14e80f268ee07b87687f2b80c` | 撤回此前 UDP-only 强制，记录传输积压问题 | 当前机器无 DDS 故障 |
| 9 月 30 日提交 `7b687db10fa052d3ba20e7e2b1b2530aa7433fed` | 提交说明记录首次自主行驶约 5 s 后重规划失败 | 独立真机复测或完整导航验收 |
| 后续至基线 HEAD | 规划半径最终缩至 0.10 m、CM 边距 0.02 m、膨胀 0.35 m | 常规车体几何已变更、窄口安全性通过 |

因此原阶段 6 顶部“真机测试全部未执行”只能作为 9 月 28 日早期快照；现应表述为已有后续现场尝试记录，完整真机验收仍未齐备。本批没有重新执行运动测试，也没有从提交说明推导新的 PASS。

## 6. 命令、验证与交付

| 关键命令 | 退出 | 结果 |
| --- | --- | --- |
| `python3 docs/architecture_evidence/batch1_20261019/collect_baseline.py > docs/architecture_evidence/batch1_20261019/baseline.json` | 0 | 完成磁盘快照和只读比较 |
| 采集器内各仓库 `git rev-parse / branch / status / diff` | 0 | 完整版本和本地修改已记录 |
| 主仓库 `git remote get-url origin` | 2 | 无 origin，如实记录；其他仓库该命令均 0 |
| `dpkg-query -W`（具体 argv 见快照） | 0 | 获取已安装 Nav2/twist_mux 版本 |
| `bash scripts/field_session.sh`（采集器调用） | 0 | 仅输出计划，无节点启动 |
| `bash scripts/run_navigation.sh`（采集器调用） | 0 | 仅输出计划，无节点启动 |
| 静态快照核验（见 validation.txt） | 0 | 原始包、安装副本、地图关联、配置哈希核对通过 |
| `git diff --check` | 0 | 文档与证据补丁格式检查通过 |

修改文件：`ARCHITECTURE_PLAN.md`、`docs/STATUS.md`；新增本报告及证据目录中的采集器、JSON 和验证记录。不修改运行配置、源码、安装副本、历史报告原文或地图资产。

本批为文档/基线整理，未重跑会启动节点的阶段 5/6 集成测试，不据此声明当前参数全栈通过。

下一步最少人工操作：本批无需人工操作。后续接口重构可使用现有离线数据；新图独立定位验证、精确标定和任何真机测试仍需对应阶段安排及逐次授权。下一批优先处理统一配置、地图引用与 Lightning 适配边界，不自动进入 FAST-LIVO2 安装或坡道测试。
