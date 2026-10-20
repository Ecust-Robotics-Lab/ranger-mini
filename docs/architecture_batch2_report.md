# 架构改造批次 2：定位接口与配置解耦

开始：2026-10-19；完成：2026-10-20。分支：`refactor/pluggable-navigation`。批次前提交：`1420575dc6436f60b8ad7b4277193fc6d3edd1db`。

**PASS：本批接口实现、构建与隔离离线验证。整体真机导航验收仍为 PARTIAL。** 真实底盘、雷达和相机均未启动，没有真机运动测试；未安装 FAST-LIVO2 或高程地图依赖。

## 1. 实际完成

- 新增实际可构建的 `ranger_localization_msgs` 包，以一条 LocalizationEstimate 原子承载地图系车体位姿、原采样时间、状态和原因。
- 将 Lightning 的 GOOD 枚举、置信度、输入过期判断及传感器外参换算移入独立适配器；公共桥接只消费标准消息和轮速里程计。
- 公共桥接保留 TF、创新检查、`/navigation_ready` 与 `/localization_health`。轮速回调和看门狗复用同一健康判据；对错误坐标系、未知状态、非有限位姿、无效四元数及未来/过期 VALID 位姿拒绝放行。
- 实现 `standard` 外部后端入口；离线模拟发布者无需 Lightning TF/状态即可驱动公共桥接，退化与断流能够撤销 ready。
- 引入 `config/runtime.yaml` 和 `config/maps/field_20260928.yaml`。现场、导航、辅助可视化默认引用同一地图清单；旧环境变量产生地图冲突时启动前报错。
- 修复阈值 launch 未转交的问题，默认仍采用此前实际生效的 0.5 s。Lightning 事件状态有效期统一为 10 s，置信度有效期仍为 1 s。
- 单独 aux 默认地图发布改为关闭，显式开启时使用当前导航图；保留旧启动入口。旧 BRIDGE_EXTRA_ARGS 转为白名单、类型校验后的覆盖，未知项拒绝执行。

详细消息语义、QoS、配置优先级、兼容入口及回退方式见 [定位后端接口说明](localization_backend_contract.md)。

## 2. 兼容性与本批边界

- 速度链、底盘入口、真实外包络记录、规划半径、Nav2 参数和地图文件没有调整。公共门禁 Bool 话题保持不变。
- `/localization_health` 中算法专属 status/confidence 键改为通用 backend_valid/backend_alive，并附后端状态、原因和原采样时间；消费诊断键的外部工具需要按接口说明迁移。
- 旧 `LOC_CONFIG` / `MAP_YAML` / `MAP_DIR` 只允许与所选清单一致；改地图改用 MAP_BUNDLE。底层 launch 显式 map_yaml 与发布脚本显式 --map-dir 保留，调用者负责一致性。
- 本批测试使用新的 0.10 m 实验规划半径配置，但它仍小于真实车体半宽。CM 外形覆盖和链路测试通过不构成窄口通行验收；没有缩车体或修改障碍数据以通过测试。
- **同刻里程计插值尚未实现**：本批保留最新定位位姿与当前轮速位姿组合，源码与接口文档均明确说明，留待批次 3。
- 完整进程监督、外参/公共 TF 树整理、在线后端热切换与独立定位数据验证尚未完成。原生 Lightning TF 继续存在。
- 当前地图原有 acceptance.json 内容、独立尺度/定位验证及精确标定缺口原样保留。地图清单验证关联与兼容性，不颁发地图验收结论。

## 3. 验证证据与命令

证据目录：[`architecture_evidence/batch2_20261019/`](architecture_evidence/batch2_20261019/)，目录按开始日期命名，包含跨日完成结果。

以下命令在项目根目录运行。构建先 source Humble 与 ws_livox 安装环境；ROS 测试先 source Humble 与 ws_nav 安装环境，安全链测试还 source ws_livox。测试强制 localhost 和独立 domain，不启动任何底盘驱动。

| 关键命令 | 退出/结果 | 证据 |
| --- | --- | --- |
| `colcon build --packages-up-to ranger_navigation --executor sequential`（ws_nav 内） | 0，三个包构建完成 | `build_complete.log` |
| `PYTHONDONTWRITEBYTECODE=1 python3 tests/test_architecture_contract.py` | 0，10/10 unittest；包含多个故障子场景 | `contract_complete.log` |
| `PYTHONDONTWRITEBYTECODE=1 python3 tests/test_stage5_bridge.py` | 0，14/14 | `stage5_verified.log` |
| `python3 tests/test_stage6_gate_unit.py` | 0，12/12 | `gate_unit.log` |
| `EVIDENCE_DIR=docs/architecture_evidence/batch2_20261019/safety python3 tests/test_stage6_safety.py` | 0，37/37 | `safety_console.log`、`safety/` |
| `python3 scripts/runtime_config.py` | 0 | `runtime_resolved.json` |
| `bash scripts/field_session.sh` / `bash scripts/run_navigation.sh` | 均 0，只打印 dry-run | `field_dry_run.log`、`navigation_dry_run.log` |
| `bash -n`（四个变更 shell 入口）与 Python AST 检查 | 0 | `verification.json` |
| `git diff --check`、源文件/安装副本及保留资产核验 | 0 | `verification.json` |

构建使用 PYTHONDONTWRITEBYTECODE=1 时出现 setuptools 的“byte-compiling is disabled”提示，构建成功；不是缺失依赖或编译失败。原始 ROS 日志包含终端回车和尾部空行，保持原字节，通过 .gitattributes 仅对本批原始 log/txt 证据关闭空白格式检查，源码与文档仍正常检查。

测试覆盖：

- 标准后端：VALID 基线、TF 数值、INITIALIZING/DEGRADED/LOST/ERROR、错误坐标系/车体参考点、NaN、零/非单位四元数、未知枚举、旧/未来时间戳、持续心跳但采样时间冻结、后端心跳断流、轮速断流及恢复。domain=181。
- Lightning 适配器：非零平移和旋转外参后的车体位姿数值，低置信度退化、NaN 错误、状态过期及恢复。domain=182。
- 实际 launch：参数服务直读证明 tf_max_age_s 等传入公共节点和适配器；事件状态超过 1 s 仍正确有效；原创新、TF 断流和轮速看门狗测试保留。domain=179。
- 安全链：当前地图产生真实规划与上游控制消息，许可关闭时最终输出保持零；限速、障碍停车、地面点过滤、软急停、断流、单一最终发布者、重启后许可失效均通过。CM 死亡后链路停机，无自动恢复；cleanup_report 记录无残留。domain=180。

## 4. 测试修正与失败记录

所有失败记录保留，没有改写成通过结果。

1. `contract_attempt1.log`：测试在冻结采样阶段读到了上一阶段诊断，导致错误归因；改为等待对应诊断判据，再验证持续阻断与恢复。最终增加更多无效位姿场景并通过。
2. `safety_attempt1_console.log`：旧测试读取已不存在的 footprint 键，静态检查阶段退出 1；没有启动导航链。当前实际配置自 9 月 30 日已采用 robot_radius。
3. `safety_attempt2_console.log` 与 `safety_attempt2/`：35/37。旧速度断言写死 0.20，而实际为 0.10；目标选择仍读取旧地图、模拟朝向也对应旧图，导致新地图规划失败。修正为当前 0.10 断言、当前地图上全路径净空检查的直线目标及相应模拟朝向；不修改生产参数或地图。最终仍要求 planner 和 controller 真正发出消息，避免全零假通过。
4. 原“规划 footprint 外包络”断言更新为“独立 CM 停车区域覆盖记录车体外包络”，同时输出小规划半径的实验边界；不再声称规划圆形模型覆盖真实车体。
5. 早期测试清理向整个 launch 进程组发 SIGINT，launch 又向子进程转发，曾产生重复中断的退出日志。最终正常清理只通知 launch，由它转发；超时才对本测试进程组兜底强停。`stage5_verified.log` 为最终结果。

## 5. 修改文件与交付

按子系统分组，精确文件和哈希见 verification.json：

- 新消息包：`ws_nav/src/ranger_localization_msgs/` 的消息、CMake 和 package.xml。
- 公共桥接包：拆分 geometry、health、lightning_adapter、runtime_config；更新 bridge_node、launch、依赖和入口声明。
- 集成入口：新增 runtime 配置和地图清单、只读解析 CLI 与 shell 加载器；更新 field_session.env、现场/辅助/导航脚本、地图发布默认值及 navigation launch/包依赖。
- 测试：新增架构契约测试，扩展阶段 5 测试，修正阶段 6 的历史地图/几何/限速测试夹具。
- 文档：更新计划与 STATUS，新增接口说明、本报告及证据。六个未跟踪雷达日志目录保持原位；相机与第三方仓库没有修改。

最少人工操作：本批无需人工操作。下一批为采样时刻对齐与进程监督；实际传感器/底盘验证继续按 AGENTS.md 取得该次测试授权，不从本批离线 PASS 推导真机许可。
