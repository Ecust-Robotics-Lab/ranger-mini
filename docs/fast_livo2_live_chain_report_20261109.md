# 在线公共TF联调与遥控短测准备

日期：2026-11-09。用户要求继续。本轮完成前一轮时间转换之后的实际ROS适配/公共桥接联调，以及单命令短测的软件准备；未启动真实设备、CAN或底盘，不发送速度/导航目标，未提交/推送。原包、地图及既有工作树保留。

## 新增结果

`tests/test_fast_livo_live_chain_ros.py`在domain228/localhost运行真实时间转换节点、现有C1适配器与公共桥接。输入为合成已确认C1消息、host-clock wheel、receipt和sensor-clock；没有把合成输入称为重新运行了真实C2。

以下完整序列通过：正常就绪 → wheel断流0.9秒后撤销 → wheel恢复后就绪 → receipt断流1.1秒后撤销 → receipt恢复后就绪 → 锚点变化后锁定未就绪 → 原锚点返回也不擅自恢复。确认map→odom平移2m与合成预期一致，只有一个TF发布者，没有速度发布者。测试退出0，用时7.914秒，子进程已等待退出。

在线入口新增显式`--auto-wheel`，只能与`--public-tf`组合。复用现有wheel_capture.prepare：已有新鲜唯一wheel源则借用，没有则检查跨域驱动占用、固定CAN状态和receive-only安装，再由原受管会话启动只接收底盘。驱动参数、`base_control`资源锁和录包入口一致；不请求控制模式、不订阅速度、不在退出时发零速。自启子进程随会话清理，已有源不代停。默认仍不自启底盘，dry-run不运行prepare/CAN/发现操作；真正执行属于后续实物操作。

增加录后`audit_live_capture.py`，只读扫描同次bag的原/派生wheel、clock状态、C1估计、navigation_ready、公共TF和localization_health。输出`navigation_audit.json`：有效采样数、就绪变化、时间错误、创新锁定、wheel戳回退及TF边。缺输入、无就绪、时间错误或创新锁定均不能报告功能已观察。入口原workflow_success继续只说明流程；公共模式还须functional_result=OBSERVED才退出0，否则退出2。OBSERVED仍不代表物理同步、真实精度、长期稳定性、故障恢复验收或TF发布者唯一。

本轮还补录`/localization_health`，并在终端输出wheel时间状态、残差和转发数。

## 证据与检查

证据：[live_chain_20261109](fast_livo2_evidence/live_chain_20261109/)。

- 在ROS、ws_nav、online_localization环境下，`ROS_DOMAIN_ID=228 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 python3 tests/test_fast_livo_live_chain_ros.py`：退出0，1项完整ROS链路检查通过。
- `python3 -m unittest discover -s tests -p test_fast_livo_live_audit.py -v`：退出0，3项摘要判定检查通过，验证私有有效定位不能覆盖无就绪/时间错误/创新锁定。
- `test_fast_livo_wheel_capture.py`：退出0，5项既有wheel复用/只接收准备检查通过，未访问真实CAN。
- 合成真实CDR消息写入临时SQLite，再用实际只读bag读取函数审查：OBSERVED，源文件大小/mtime未改。不是新真实采集结果。
- 默认私有、显式公共、公共auto-wheel三种dry-run和组件依赖检查见证据；auto-wheel计划仅包含一个receive-only底盘组件，所有实际硬件执行均未实施。

## 已准备的现场命令

先预览：

```bash
bash scripts/localize_fast_livo2.sh \
  --archive outputs/fast-livo-demo-online-mapping-20261105/full_replay_12/map \
  --public-tf --auto-wheel \
  --navigation-map outputs/fast-livo-demo-navigation-prep-20261106/navigation_map_final \
  --domain 215 --seconds 90 --visualization none --dry-run
```

经本次实物操作授权后，将末尾改为`--execute`才启动真实传感器、定位和所需只接收wheel；本轮未执行。设备或原会话占用时按原机制停止，不抢占既有源。用户全程遥控：先静止约15秒，再短直行、左右转弯、停车。无自主运动、无规划控制器。运动后再看首次定位和持续就绪，不能要求静止阶段必定完成C2运动确认。

重点验收：修复驱动真实wheel符号、车体/地图朝向、时间残差、公共TF、定位有效与就绪状态。录后摘要会自动生成；需要时可手动只读重查：

```bash
# 先加载ROS、ws_nav和online_localization安装环境
python3 scripts/fast_livo2/navigation/audit_live_capture.py --capture bags/实际本次目录
```

后续仍需现场测得接收偏差及真实同步关系；首帧receipt锚点不是物理同步证明。本轮工作截至可执行且经过隔离联调的软件准备，实物短测与导航运动尚未实施。
