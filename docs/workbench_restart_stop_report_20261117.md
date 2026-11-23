# 重复启动与停稳判据修复 — 2026-11-17

用户报告停止自有会话后再次启动失败、车辆未被导航驱动时停稳显示反复；随后明确要求线速度停稳阈值改为0.05m/s。本轮仅软件/构建/离线及隔离ROS，未重启真实设备、未发实车目标或非零速度，未提交推送。

## 现场证据与结论

导航会话run-wcsv33wp的health为localization=false、fast_navigation_inputs=true，navigation组件保持WAITING，没有实际启动；对应定位run-gb4lze7a的后台存活、输入正常、processed_scans=343/skipped_scans=262，始终searching_waiting_query_window。另两次失败有相同现象。不是已证明的导航器崩溃或旧进程残留；新C2未就绪被通用导航等待超时掩盖。

源码确有可复现的查询饥饿缺陷：online调度对任何阶段都只留两帧待处理，成批到达时可在worker获取前丢弃组内帧；C2要求原时间窗1秒内8帧，因此可一直不足。新增成批4帧输入的独立C++对照：旧策略未形成查询，新策略完成相同8帧查询及强几何/C1交接要求。现场没有原始查询云录制，不能把合成对照宣称为该场景修复后的实车定位成绩。

原轻量记录live-1fd39...中1132个带wheel的快照，绝对线速度P50约0.007/P95约0.021/最大0.023m/s；851个在旧0.015阈值内、1132个都在新0.05阈值内。SDK非零快照为0。另3次失败的全部wheel速度为0。证据保留在recorded_diagnosis.json，说明网页读到了底盘小反馈值，不能据此称为小车被任务驱动，也没有外部真值用于轮速标定。

## 修改

- C2收集查询时按所需帧数保留待处理扫描（当前8），C1跟踪保持最新2帧；保留32MB输入队列保护。8帧、1秒、12000点、几何/地点歧义/C1独立验证及输出有效期全部不放宽。等待结果新增实际窗口帧数/需求/跨度诊断。
- 工作台启动先等待本次新定位和navigation_ready，再启动导航。显式等待最多120秒，显示C2原原因及窗口进度；不以延长navigation超时冒充定位就绪，不复用旧位姿。停止清理已退出自有进程引用，新启动清旧任务观察；模拟连续两轮启停验证会话身份不同。
- 按用户要求，线速度停稳阈值统一0.05m/s，角速度0.025rad/s及0.5秒连续停稳不变。motion配置传给协调器、worker、门禁、V2驱动，网页从同源配置读取。没有把轮速归零或放宽碰撞/250ms输入/100ms租约条件。
- 网页停稳优先使用原始host-clock `/wheel/odom`，在传感器时钟未启动/落后于relay帧时仍能正确显示；过期原始样本不能被迟到relay刷新。保留旧记录mapped wheel兼容。
- 重新构建online_localization_install、ws_base/install_exact、ws_nav/install_exact。按原SHA和累计前置补丁重新生成驱动增量，原补丁备份到.runtime/backups/stop005_20261117，preimage/逐文件重建/锁SHA通过。

## 验证（最终成功命令退出码0）

| 检查 | 结果 |
| --- | --- |
| workbench + wheel_capture unittest，workbench_tests.log | 58 PASS |
| localizer_tests，localizer_tests.log | 41 PASS，包括旧2帧/新8帧成批对照及原负例 |
| test_exact_control_recovery/warm_start/sampling_ros，navigation_tests.log | 31 PASS |
| test_exact_navigation_core/pause_resume/progress_persistence/stage6_gate_unit，task_safety_tests.log | 69 PASS |
| 实际SDK mock/receive-only，lifecycle.log、receive_only.log | PASS，无硬件I/O |
| 真实ROS消息/Action/取消/网页断连，ros_workbench.log | 4 PASS，隔离domain228 |
| 在线定位队列/原时刻过期/确认，online_localization_ros.log | 3 PASS，隔离domain229 |
| 页面C2进度、0.023原值保留/0.05内停稳、0.051运动、过期未知，browser.json | PASS，独立HTTP模拟无设备请求 |
| 基础/导航与定位构建，base_nav_build_final.log、localizer_build.log | PASS |
| patch_rebuild.json | 两仓库补丁逐文件重建PASS |

首轮同进程HTTP/ROS合成测试出现HTTP400，原日志ros_workbench_initial.log保留；测试准备HTTP后等待新鲜合成输入再提交目标，4项通过，生产250ms门限没有改变。原真实失败证据、原包、地图、标定均保留。受管设备当前状态另见final_identity.json；Agent没有停止/重启用户工作台。

## 尚待实车

结束只读检查时，用户已停止原工作台，全部设备受管会话均不存活；可直接运行 `bash scripts/run_navigation_workbench.sh --execute --lan` 加载本次更新。此服务命令不启设备；之后由用户点击启动。重复启动定位、真实0.05停稳/模式切换及完整巡航仍PARTIAL，不能用以上合成结果代替。所有已授权软件项完成，不自动提交或推送。
