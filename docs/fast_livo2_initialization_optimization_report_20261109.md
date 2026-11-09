# 首次重定位优化交付

日期2026-11-09。用户“就按你说的做吧，先优化重定位系统”授权源码、构建及离线验证。本轮未启动传感器、底盘或导航、未发速度/目标、未提交/推送。开始HEAD为694b8b6，保护已有实机短测报告与未提交交接文件，未重写原包/地图/标定。

## 改动

1. 保留自有v2→KISS全6DoF→small_gicp→C1。显式C2 overlay启用`global_stationary_initialization: true`；核心/节点裸默认false，手动C1行为保持。C2最多检查16个检索候选并保留原地点歧义拒绝；额外要求更强重叠、残差、非退化性及不同地点竞争余量，至少3个不重叠查询窗口、1秒跨度、FAST和地图都近静止、修正一致，才传递内部静止证据。具体阈值见重定位接口，不把重复静止观测当独立视角证明。
2. C1只对内部C2证据免运动基线，仍独立执行13初值歧义检查、2帧/0.2秒及修正一致性确认。手动服务不能设置该内部标记；切图、取消、重置仍清除支持。Tag参与不走本纯LiDAR静止分支。
3. 单次C1 match复用粗/精small_gicp的降采样、协方差/KD-tree，以及独立几何检查的目标法向量/KD-tree。预处理仅在本次source/target内存活，没有跨扫描缓存、没有删除初值或放宽原配准门槛。两轮初始化的总耗时包含预处理，旧Registration::align保留单次路径；共用几何公式保持。B/生产runtime未切换算法。

## 构建和检查

证据目录`docs/fast_livo2_evidence/initialization_optimization_20261109/`。先将旧节点/核心库备份到`.runtime/backups/initialization_20261109/`，哈希见`hashes.json`，旧依赖与新依赖分别加载。

构建命令（exit0）：

```bash
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH="$PWD/ws_fast_livo/backend_deps:${CMAKE_PREFIX_PATH:-}"
cd ws_fast_livo
colcon --log-base online_log build --base-paths src --packages-select ranger_fast_livo_backend --build-base online_build --install-base online_install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
cd ..
bash scripts/build_fast_livo2_online_localization.sh --execute
```

最终日志backend_build_final/localizer_build_final均exit0。已构建既有隔离online_install后端共用库及online_localization_install定位器，backend_install/原相机/底盘安装保留。CMake警告backend不在本次localizer安装前缀中构建，实际明确使用online_install，非构建失败。

- 最终40项C1/C2核心PASS，9.612秒；含新增强静止C2→C1、重复地点拒绝、错误粗解、静止云/漂移odom拒绝、重置丢弃支持，以及预处理复用前后三初值结果一致。历史手动初始化/退化/超时/取消/6DoF回归保留。
- 共用后端21项PASS，0.528秒。
- 新增实际ROS静止交接与手动C1仍需运动2项PASS，4.681秒；已有短时恢复、长时回退、队列/过期、旧证据、取消和切图7项PASS，16.375秒。见stationary_ros/regression_ros.log；全部隔离localhost/domain196，无硬件。
- `git diff --check`通过；正式runtime及地图manifest SHA与本轮前一致。

## 原速真实记录输入对照

`realtime_replay.py`只读SQLite，发布原采样戳的FAST位姿/去畸变云对及模拟clock，按原header间隔、wall-clock 1x发送，无逐帧ack降速；隔离domain223/localhost，不运行原始传感器或FAST前端。因此衡量已就绪前端到C2/C1的首次有效输出，不含设备启动/地图加载时间，也不是完整实车负载测试。

统一预加载同一正式地图、v2/small_gicp，旧二进制与旧库单独加载；新版本使用本次最终构建。两次均回放co968xa3前45秒。

| 版本 | 首VALID墙钟 | 首VALID源采样时间 | 不同有效采样 |
| --- | --- | --- | --- |
| 旧版本原速重放 | 29.820秒 | 29.600秒 | 153 |
| 本轮最终版本 | 2.800秒 | 2.400秒 | 422 |

另一份独立访问包trpozzyj前45秒最终版本首VALID墙钟4.220秒、源采样3.800秒、409个不同有效采样，输入期间无LOST；两轮完整初始化165.148/164.502ms。该包曾参与早期v2选择，不是新的盲测集，其历史22.315秒与本轮不同执行条件不做严格同条件倍速比较。

首VALID墙钟减少约90.6%。第一轮仅small_gicp预处理复用的中间版本为4.320秒；加独立几何检查预处理复用后为2.800秒。旧两轮完整C1初始化1000.989/994.699ms，中间879.152/871.351ms，最终163.627/162.680ms（约降低84%）；保留所有原始日志，未用中间数据冒充最终版本。

旧/新153个共同有效采样输出，最大平移差2.45e-8m、旋转差8.04e-9rad；这是相同实现结果一致性，不是绝对真值精度。两者输入期间均无LOST；新版本仍有短暂not_converged，未用本优化宣称连续就绪问题已解决。发送结束继续推进clock的尾部过期为预期撤销，单列于samples而不算输入期间丢失。

首轮baseline误用默认offline scheduling导致pair队列满、45秒零VALID，保留baseline/及其argv，判为测试入口无效，不作为性能基线；修为与在线入口一致的online_scheduling=true后，baseline_online与final才用于同条件比较。没有放宽sample_max_age或重打原消息时间。

复现（先source ROS及online_localization_install，LD_LIBRARY_PATH含backend_deps/relocalization_deps）：

```bash
python3 docs/fast_livo2_evidence/initialization_optimization_20261109/realtime_replay.py --baseline --output /tmp/ranger-init-baseline-new
python3 docs/fast_livo2_evidence/initialization_optimization_20261109/realtime_replay.py --output /tmp/ranger-init-final-new
python3 docs/fast_livo2_evidence/initialization_optimization_20261109/realtime_replay.py --capture bags/fast-livo-relocalization_visit01-trpozzyj --output /tmp/ranger-init-independent-new
```

所有输出要求新目录；原袋、地图与原时间不改。新版本参数可显式`-p global_stationary_initialization:=false`回到运动确认；若回退整个二进制必须配套旧核心/后端库，不能混用。

## 验收边界

软件/合成与所列真实记录功能通过。top16候选非全图穷举，多帧同视角无法消除所有几何混淆；当前实际错误地点负样本/更广泛场景与绝对精度不足，不能宣称误定位率为零。实车首次耗时、真实负载、连续就绪仍PARTIAL，未自动开设备。本次未实施上次位置/停靠点先验、Tag或规划控制改造。
