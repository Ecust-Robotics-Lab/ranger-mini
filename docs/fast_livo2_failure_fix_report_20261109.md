# 迟到图像检查与C2短时局部恢复实施

日期：2026-11-09。用户明确要求按研究方案实施。本轮完成两项优先软件修正、隔离构建、原始CSV/真实点云检查及合成ROS断流测试。没有启动相机/雷达/底盘或发运动指令，未提交/推送。原包、地图、失败记录和原有工作树修改保留。

## 已实施

### 相机交付与关联

源码`ws_camera/src/hik_camera_trigger_driver/src/hik_camera_node.cpp`及新增`frame_time_guard.hpp`；相对于本轮前源码的补丁为`patches/hik_camera_live_latency.patch`。源码已纳入根Git工作树；补丁另外保留本轮相对接手状态的改动。

- 取帧/转换与ROS发布/同步CSV分到两个线程。队列最多保留两帧，积压丢最旧待发布帧，summary记录`publish_queue_dropped`。SDK源缓存仍按既有规则及时释放。
- 在采集线程完成转换后保存共享scan快照，发布线程使用这份快照，避免ROS发布等待后再次读取更新的scan。raw时间仍来自实际采集处理时刻，未在发布线程重打now。
- `FrameTimeGuard`以设备tick和scan时间的相对进度校验配对，前5个规则帧建立比例，之后使用最多15个已通过比例的中位数。当前FAST 10Hz配置的启动间隔范围70–130ms，容许相对进度残差35ms。它不是曝光标定或通用任意帧率模型。迟到帧拒绝不更新基准，之后对应进度追上可再次通过；设备重连另建段。
- 新拒绝原因`DEVICE_SCAN_MISMATCH`独立计数，不再把第一张迟到旧图当OK。原+100ms前端相位保持原语义。
- CSV追加SDK取帧调用开始、转换结束、raw发布开始/结束；已有取帧返回和sync发布返回字段保留，能区分SDK等待、转换、发布队列和发布耗时。逐帧flush改为每50条flush、正常结束close排空。

构建入口`bash scripts/build_fast_livo2_camera_latency.sh --execute`，默认dry-run。安装到`ws_camera/latency_install`。在线定位入口显式引用该节点并优先加载对应动态库，归档实际节点/库/guard来源。普通录包/建图原相机安装未替换。本轮没有切换SDK抓取策略或QoS，避免把多个未测配置同时加入；分段日志和队列先提供下一次实测依据。

### C2短时局部恢复

修改同一C1核心和节点，不另建定位器：

- 确认样本过期仍立即撤销有效性、递增作业代数并取消旧异步结果。
- 仅全局搜索已启用且仍允许的C2模式入队Recover，而非直接清掉已接受地点；独立C1和人工初值覆盖搜索的行为保持原有语义。
- 同一地图/前端会话内保留已接受`X(t0),U(t0)`，新扫描用`X(t0)*inverse(U(t0))*U(t)`预测，在既有子图以small_gicp检查。绝不将旧候选当新有效输出。
- 本轮采用固定2秒源采样范围，运动范围复用0.75m/0.35rad门槛；保留残差、重叠、退化和创新检查。至少两个不同新采样、间隔至少0.1秒、地图修正一致才恢复；允许静止。
- 超范围、错误几何、创新或一致性失败退回全局搜索。新鲜度仍0.5秒；节点提交时再次确认当前代数和时间。切图、前端身份变化、手动初始化和取消不复用旧恢复状态。

独立C1默认不自动恢复，生产runtime与规划控制未切换。在线前端滤波器/视觉地图未实施恢复或重启。

## 验证与证据

证据目录`docs/fast_livo2_evidence/failure_fix_20261109/`。

| 检查 | 实际结果 |
| --- | --- |
| 两次相机隔离源码构建及入口脚本重验 | exit0；仅既有camera_info_manager弃用提示 |
| C1/C2隔离构建 | exit0，最终6.58秒，实际耗时见构建日志 |
| 核心gtest | 36项PASS，10.393秒；含新增静止恢复、超时/错几何、重置、过大运动和取消 |
| 实际C2节点新增隔离ROS检查 | 2项PASS，5.139秒；0.3/0.6/1秒断流、静止恢复不换全局request、长间隔回到搜索 |
| 既有在线ROS回归 | 3项PASS，5.034秒；过载、人工C1失效/重给初值、旧结果不得发布 |
| C2取消/错误几何恢复、切图旧请求回归 | 2项PASS，7.425秒 |
| 原始88k5eclm相机CSV经实际C++guard | 918接受、4拒绝：拒绝540–543，544通过；修正原来540错误接受/544被拒的模式 |
| 原始88k5eclm点云局部恢复 | PASS，实际两帧1590193864700110197/1590193864800430297确认；见下方限定 |
| 在线入口dry-run | hardware_started=false，实际argv指向latency_install |
| 相机补丁reverse-check、Python语法、git diff --check | PASS |

原包点云实验使用现场失效前已接受地图位姿作为先验，配合对应原始FAST本地位姿；先用此前两帧几何确认先验，再进入局部恢复。相隔约0.9秒的新扫描提供第一支持，下一实际采样约0.1秒后确认。点云来自未改写的原SQLite，另存派生PCD，程序为`test/recovery_replay.cpp`，输入和结果在`recorded_geometry/`。**它验证真实记录几何可局部恢复，不是34秒全链路原速复跑、独立真值或实车恢复耗时。** ROS断流测试提供独立的状态/时间逻辑覆盖。

首次直接启动核心测试缺`libgtsam.so.4`，exit127，保留`core_tests.log`；补项目内依赖LD_LIBRARY_PATH后36项通过，`core_tests_02.log`。未升级或安装系统依赖。

可复现命令：

```bash
bash scripts/build_fast_livo2_online_localization.sh --execute
bash scripts/build_fast_livo2_camera_latency.sh --execute
source /opt/ros/humble/setup.bash
source ws_nav/install/setup.bash
source ws_fast_livo/online_localization_install/setup.bash
export LD_LIBRARY_PATH="$PWD/ws_fast_livo/backend_deps/lib:$PWD/ws_fast_livo/relocalization_deps/lib:${LD_LIBRARY_PATH:-}"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
ws_fast_livo/online_localization_build/ranger_fast_livo_localizer/localizer_tests
python3 tests/test_fast_livo_local_recovery_ros.py
g++ -std=c++14 -Wall -Werror -I ws_camera/src/hik_camera_trigger_driver/src tests/test_hik_frame_time_guard.cpp -o /tmp/ranger_frame_guard_test
/tmp/ranger_frame_guard_test bags/fast-livo-localization-88k5eclm/sensor_assets/camera_sync.csv
ws_fast_livo/online_localization_build/ranger_fast_livo_localizer/recovery_replay docs/fast_livo2_evidence/failure_fix_20261109/recorded_geometry/input.json
```

最终二进制/库完整SHA见`binary_hashes.json`。节点a573ba…、C1 core 5dbe3a…；隔离相机节点45d120…、库1525e6…；原相机安装节点3829e9…/库242f2d…保留。runtime SHA仍7f434e1d7e9cced047070051a4aa342672e4351cf045b29e02a6f21e40fdabf6。

## 尚未验收和后续

本轮软件/离线针对性验证PASS，现场连续就绪仍PARTIAL。没有证据声称SDK/USB最初停顿已根治、曝光物理同步已确认，或实车恢复已降至1–2秒。新双线程队列的实际背压、丢帧及图像关联需一次带新分段日志的设备短测；该次设备操作须按具体授权执行。

全局top5召回/空间去重、长查询窗口、KISS低内点候选策略、C1多初值预处理缓存暂未修改。局部恢复已经在失效处原始点云通过，应先取得新现场日志，按实际剩余瓶颈决定是否实施这些后续优化；不为凑齐改动同时修改全局门槛。
