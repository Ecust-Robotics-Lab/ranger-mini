# 定位后端接口与统一配置

更新：2026-10-26。公共契约沿用批次 3，保持轮速里程计、Lightning 全局定位和既有速度链。FAST-LIVO2 前端已部署并可配置选择，但独立先验全局定位尚未实现；阶段 B 已实现私有 PGO 和仅显式离线启用的标准适配器，见 [B 接口](fast_livo2_backend_interfaces.md)。生产入口不自动启用它。受管入口、时间对齐和 TF 细节见 [操作说明](managed_sessions.md)，后续接入见 [后端计划](FAST_LIVO2_BACKEND_PLAN.md)。

## 1. 模块边界

```text
Lightning TF / 状态 / 置信度 → lightning_adapter ─┐
                                              ├→ /localization/estimate
其他后端适配器或离线模拟发布者 ──────────────────┘
    → 公共 LocalizationBridge + /wheel/odom
    → map→odom、odom→base_link、/navigation_ready、/localization_health
```

launch 参数 `localization_backend:=lightning` 启动独立 Lightning 适配器和公共桥接进程；`localization_backend:=standard` 仅启动公共桥接，调用者提供标准消息生产者。两种后端互斥，公共话题应只有一个后端发布者；本批不实现在线热切换或自动择优。

公共桥接不知道 Lightning 的枚举和分数门槛。Lightning 适配器处理外参和原生状态；TF 计算、创新检查与就绪判定属于公共桥接。适配器不发布公共导航 TF。

## 2. 标准消息

话题 `/localization/estimate`，类型 `ranger_localization_msgs/msg/LocalizationEstimate`，默认 QoS 为 RELIABLE、VOLATILE、KEEP_LAST depth=10。

| 字段 | 约定 |
| --- | --- |
| `header.frame_id` | 公共地图坐标系，当前为 `map` |
| `header.stamp` | 原始位姿采样时刻；心跳重发不得改成当前时间 |
| `base_frame` | 位姿参考车体，当前为 `base_link`；传感器位姿必须先经外参转换 |
| `pose` | 完整 6DoF 车体位姿；VALID 时必须有限，四元数为单位四元数（模长绝对容差 0.001） |
| `state` | INITIALIZING=0、VALID=1、DEGRADED=2、LOST=3、ERROR=4 |
| `reason` | 状态原因，供诊断使用，不参与算法分数比较 |

消息以一次发布原子地携带位姿、采样时间与状态，不要求消费者在独立位姿和状态话题之间配对。初始化期间可以没有有效位姿并使用零时间戳，但不得标成 VALID。后端必须持续发布心跳；本批 Lightning 适配器为 20 Hz。传感器停止时保留原采样时间，报告 LOST 或等待状态，不能制造新位姿时间。

标准消息不提供控制速度或声称具有协方差。控制器速度仍来自 `/wheel/odom`；不对带全局修正的定位位姿差分生成速度。

公共桥接拒绝错误坐标系、未知状态、无效四元数、非有限数以及标为 VALID 的过期/未来位姿。`/navigation_ready` 必须同时满足：位姿及后端心跳新鲜、后端 VALID、轮速新鲜、公共地图变换已生成且新鲜、创新检查通过。轮速回调与 0.2 s 看门狗复用同一健康策略。

`/localization_health` 保持 DiagnosticArray 类型，包含各项判定、`backend_state`、`backend_reason` 和可用时的 `sample_time`。旧的 Lightning 专属 `status`/`confidence` 诊断键由通用 `backend_valid`/`backend_alive` 替代；这不改变门禁消费的 Bool 话题。

**批次 3 已按定位采样时刻插值轮速位姿。** 缓存默认 2 s，最大相邻轮速间隙 0.1 s，不外推；平移线性插值、旋转 SLERP。创新检查也使用同刻配对。缓存的 map→odom 修正随新轮速传播，但原采样时间仍用于新鲜度判定，重复定位心跳不刷新它。

## 3. Lightning 适配与兼容入口

受管启动的原生输入为 `map→lightning_imu_pose`、`/lightning/loc_status`、`/lightning/loc_confidence`。转换为 `T_map_base = T_map_imu × inverse(T_base_imu)`；GOOD=2 与 min_confidence=1.0 仅在此适配器解释。私有算法帧名由 runtime 的 native_pose_frame 统一传给 Lightning 与适配器；直接运行适配器可执行文件仍默认读 imu_link，以兼容旧外部调用。

状态事件有效期默认 10 s，置信度有效期 1 s，位姿最大年龄 0.5 s。10 s 状态有效期现在只有一套实现；修复了旧看门狗硬编码 1 s 的不一致。默认 0.5 s 保留此前实际生效值，不自动扩大到历史命令里未生效的 1.5 s。

原 `ros2 launch ranger_localization_bridge loc_bridge.launch.py` 继续可用，阈值显式转交到两个进程。直接调用 `ranger_localization_bridge` 可执行文件或 `python -m ...bridge_node` 时，默认在同一进程组合 Lightning 适配器以兼容旧用法；公共桥接的独立调用使用 ROS 参数 `adapter_backend:=external`。

受管入口使用固定 base_link→imu_link→livox_frame 物理链；Lightning 改用私有算法帧，避免 imu_link 有两个父节点。固定外参仍为原近似值，不构成精确标定通过。旧外部启动流程不得继续把 map→imu_link 与新的物理链同时发布。

## 4. 运行配置与地图清单

- `config/runtime.yaml`：后端选择、阈值、外参近似值、初始位姿和雷达时限。
- `config/maps/field_20260928.yaml`：定位原生地图、定位配置、导航栅格、标定来源和源包的关联。
- `config/field_session.env`：组件开关，及旧参数兼容入口。

`scripts/runtime_config.py` 只解析配置并输出 JSON；`--shell` 输出经 shell 引号编码的白名单变量。解析器不 source ROS、不启动节点、不修改地图。项目位置从祖先目录发现，另支持 `RANGER_PROJECT_ROOT`。

选择不同配置用 `RANGER_RUNTIME_CONFIG`，选择不同地图清单用 `MAP_BUNDLE`，相对路径统一以项目根目录为基准。清单检查后端兼容性、资产存在、Lightning 配置引用的原生地图及两次生成记录的源包/源码关系。当前清单保留实验目录布局；不是通用原生地图互转工具，也不代替全量资产哈希校验。

优先级为 runtime 默认值 → 支持的环境变量 → 旧 `BRIDGE_EXTRA_ARGS` 中受支持的阈值/外参覆盖。数字、外参和初始四元数经过校验和规范化；未知额外参数明确报错，不再静默丢弃。数组参数在 BRIDGE_EXTRA_ARGS 中含空格时需作为一个带引号的 token。

`LOC_CONFIG`、`MAP_YAML`、`MAP_DIR` 旧变量只能指向清单选定资产；冲突时在启动前报错，应改选地图清单。直接调用导航 launch 的显式 `map_yaml` 和地图发布脚本的显式 `--map-dir` 仍作为底层兼容接口保留，调用者负责其一致性；正常现场入口使用统一解析器。

现场、导航与辅助可视化入口现在默认选择相同导航栅格 `8cw1afzl`，定位原生地图为关联的 `2lx5kkho`。单独 aux 的地图发布默认改为关闭，需明确 `ENABLE_MAP_PUB=1` 才启用，避免与 Nav2 map_server 重复发布。地图清单只登记关联，新导航地图 acceptance 中的 PENDING 不被改变或视为 PASS。

## 5. 构建、回退与后续边界

新增消息包需要先构建，再 source 安装环境：

```bash
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
cd ws_nav
colcon build --packages-up-to ranger_navigation --executor sequential
source install/setup.bash
```

阶段 B 修复了创新拒绝后仍覆盖接受修正缓存的问题：失败候选不进入缓存/参考历史，重复失败心跳不累加。新增 `latch_on_innovation_failure` 默认 false；B 离线入口显式 true，首次拒绝锁定未就绪并停止地图修正，重启会话才能解除。后端接入验收需覆盖坐标参考点、非零外参、状态退化、原采样时间停止、心跳停止和轮速停止；不能只检查消息存在。标准后端的离线测试没有 Lightning 原生输入或 TF，验证桥接无需算法专属输入即可正常工作。

回退应在进程停止后回到批次前提交 `1420575dc6436f60b8ad7b4277193fc6d3edd1db` 的独立 checkout 并重建 ws_nav；不能仅切源代码而继续使用本批安装副本。保留本分支及所有地图、bag、历史证据，不执行删除或硬重置；回退也不继承任何运动许可。

现有受管会话使用 PID/start_ticks/boot_id 和 Linux subreaper 管理所属子孙进程，异常不自动重启。FAST-LIVO2 前端与相机同步包已构建并完成一次在线展示，现已停止；独立后端/先验定位、物理同步及运动精度仍待实现或验收。高程地图仍禁用，坡道能力未验收。
