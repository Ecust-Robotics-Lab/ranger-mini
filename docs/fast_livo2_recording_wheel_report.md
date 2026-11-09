# 录包自动wheel与遥控器控制

日期：2026-11-06。用户明确要求录制脚本能自动启动底盘驱动，仍由遥控器控制采集动作。本轮完成源码、隔离构建与合成录包验证；没有运行真实传感器或新启动/重启底盘，没有发布速度、模式切换或导航目标。此前获授权的base会话仍运行，真实只读检查确认新录包入口会复用它。

## 最终行为

record_fast_livo2.sh默认执行并记录8话题：三传感器、前端位姿/去畸变云、wheel。旧--record-wheel仍有效，已不必再写；--no-wheel才关闭底盘访问及wheel录制。其余profile、就绪、时长与停止规则保留。

1. 所选FAST域有唯一、RELIABLE、odom/base_link、最近0.5秒以内的wheel反馈时，复用已有发布者；不改变其模式、不另开驱动、不代停已有会话。
2. 没有wheel时，检查其他ROS域是否有ranger_base_node进程；有则拒绝重复启动，要求统一ROS域，不接管或停止它。
3. 没有其他底盘驱动时，can0已为UP/ERROR-ACTIVE/500k则直接使用；DOWN时执行固定免密sudo命令启用。没有接口/权限、已UP但速率或状态不符时明确报错，不自动重置。配置前值和回退命令进入manifest，结束保持CAN启用。
4. 以receive_only=true、publish_odom_tf=false启动隔离构建，纳入原会话base_control租约和boot/PID清理。录包器仍须收到全部8话题才显示RECORDING，wheel缺失/重复/断流不会伪造数据。
5. 自启wheel随录包结束或失败清理；复用的已有wheel/预览进程保留。没有Nav2、公共桥接或速度发布者。

直接使用：

    bash scripts/record_fast_livo2.sh --seconds 180 --label wheel_alignment_visit01 --visualization none --min-free-gb 100

纯传感器采集：

    bash scripts/record_fast_livo2.sh --no-wheel --seconds 60

只看计划：

    bash scripts/record_fast_livo2.sh --dry-run --seconds 180 --visualization none

can0启用的固定命令为sudo -n /usr/sbin/ip link set can0 up type can bitrate 500000。用户此前已自行配置对应免密规则。只看计划不会启用CAN或启动节点。

操作者确认遥控器手动接管，等CAPTURE RECORDING，静止15秒再人工采集，结束前静止15秒。当前旧base会话是外部复用路径，不会被录包结束自动停止；需要时用start_base_only.sh --stop停止。Agent不会因入口默认执行而自动开始新一轮实物录制。

## 源码、构建和保护

Python策略在scripts/fast_livo2/wheel_capture.py，由record_session.py调用。dry-run只输出计划，不做ROS发现/CAN配置；计划中的wheel组件表示无外部wheel时将启动。包装器补充source既有ws_base环境。manifest.wheel保留disabled/external/owned_receive_only、发布者GID/样本或自启命令/CAN前后状态；自启路径另保存实际二进制、驱动源码、配置和增量补丁字节及SHA。

复用ranger_base，新增receive_only参数，普通控制入口默认false不变。true时不调用EnableCommandedMode、不创建Twist订阅，析构不发送零速；Connect/Run只接收反馈并发布状态/wheel。SDK Connect和析构已检查，只有端口开关，没有隐式版本查询或控制命令。receive_only与publish_odom_tf=true冲突时拒绝。原控制分支保留原模式请求、命令订阅和退出零速。

补丁patches/ranger_ros2_recording_receive_only.patch作用于已有阶段2源码之上。ws_base被项目忽略，新增代码由增量补丁版本化，不覆盖此前补丁或原安装。

实际本地HEAD：ranger_ros2为7b75bc2443a1d6cbd42cc3ef3059cf4897242072，基于上游b6ea21a275ca5e7168130cc6470e61474681d679的阶段2累计提交；ugv_sdk为e2f5764acc00a2e556cf3bdc3ea3b206fa326e5b，基于f2704eacdc90357078cd93ec60aae08bb4baab35。本轮不修改SDK协议。

执行bash scripts/build_base_recording.sh --execute，只构建ranger_base到ws_base/build_recording_rx、install_recording_rx、log_recording_rx。原ws_base/install和当前运行节点不覆盖，未升级系统/依赖。构建13.1秒、退出0；保留旧控制回调的unused/maybe-uninitialized警告，receive-only不订阅该回调，不据此追认控制算法已修复。ROS/base环境内ldd无缺失依赖；二进制/源码SHA见build_identity.json。

## 验证

证据目录：docs/fast_livo2_evidence/record_wheel_20261106/。

| 实际检查 | 结果 |
| --- | --- |
| tests/test_fast_livo_recording.py CaptureUnit | 13项通过，4.406s，退出0；默认行为、可视化、缺流及设备前置检查 |
| tests/test_fast_livo_recording.py CaptureROS | domain189/localhost合成输入，10项通过，48.039s，退出0；定时/Ctrl+C/缺图/重复/断流/崩溃/磁盘、拥有和复用生命周期，两个新增wheel路径通过 |
| tests/test_fast_livo_wheel_capture.py | 5项通过，退出0；已有源/错误源、固定免密命令、已有CAN不改、权限失败、复用不碰CAN |
| check_base_receive_only.py | 退出0；实际构造/参数/订阅/析构方法编译到无I/O SDK shim，控制默认保留、接收模式无模式请求/命令订阅/退出命令、冲突TF拒绝 |
| check_base_model_selection.py | 退出0；固定真正上游b6ea基线复现旧V2错误，当前四机型参数通过 |
| owned路径增加源码归档后复跑 | 退出0，4.158s，不重复计入上述数量 |
| 当前domain175的wheel_capture.prepare只读检查 | 退出0，选external，真实wheel及原base会话未停止 |
| 默认/无wheel dry-run、bash语法、AST、补丁重放及Git空白检查 | PASS，未打开真实设备 |

28项Python检查不重复计入C++ shim/机型检查或单项重跑。拥有路径使用合成wheel进程验证收尾；C++策略用实际源码无I/O shim核验，没有声称本轮已跑真实receive-only CAN或遥控运动。

三个失败记录保留：初次构建脚本误用上游SHA匹配本地阶段2提交，立即退出1，随后按实际HEAD修正；首版测试shim不接受std::string话题，修正后通过；旧机型测试把已修复的本地HEAD当上游，改为锁定b6ea后通过。均没有因此启动设备。

本轮基线及源码副本在.runtime/backups/record_wheel_20261106_215121/。本轮和此前导航前置工作仍未提交/推送，没有删除原包、地图、失败证据或备份。

## 尚未验收

只接收驱动不接管遥控器，也不以控制心跳消除待机/通信保护；操作者须确认遥控器和底盘正常。此前V2故障32位、旧ROS16位截断及模式枚举差异仍未修复，见[真实wheel启动检查](wheel_start_report_20261106.md)，不能以旧error_code=0自动放行导航。

当前FAST三传感器＋wheel同录包仍待采集；真实receive-only采集、物理时间映射、转弯里程计精度及D未验。原始header/逐点时间保留，不重打now，不把同录本身当成物理同步通过。
