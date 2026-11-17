# P0 实验在线 B 启动时钟修复

日期：2026-11-11。软件/离线验证 PASS；修复后的实车就绪和闭环实验 PARTIAL。

## 原因与现场事实

用户运行独立实验入口，创建 `experiments/fast_livo2_p0_closure/runs/20261111_170356-pjs0bdu3`。首轮尚未到出发阶段，前端已输出数百对同戳位姿/点云，雷达、相机和录包正常；B 始终 `input epoch/age mismatch`，图版本0、处理对数0。监视器报 `sensor timestamp diverged from fixed steady-clock anchor by >2s`。

真实receipt记录显示：初始锚点与后续稳定header/steady关系相差约2.902秒。原监视器只给定位模式启用 StartupAnchor，B 模式仍把首帧header/receipt直接锁定。实验入口复用B时继承了这条旧分支，导致模拟时钟一直落后于实际输入，等待不能自行修复。

用户随后明确要求删除无用旧逻辑。该轮在既有启动期限后自动受管退出，guardian_cleanup_complete=true、owned=[]；Agent没有额外发送设备停止或重启指令。原始bag约7.5GiB、传感器CSV、失败日志、会话和实验来源快照保持原路径；没有删除失败证据。该轮不计为有效正式实验。

## 实现

- `scripts/fast_livo2/mapping_monitor.py`：删除B首帧直接锁锚点分支。所有在线模式统一复用已有StartupAnchor：至少5个新计数、稳定跨度至少1秒、启动阶段50ms一致性检查、最多等待10秒；运行中不重新锚定，之后异常仍失败。
- 监视器统一使用传感器相对 `/clock`。新增 `clock_ready`；B额外核对实际后端诊断采样戳，确保消费者已收到同一时钟。稳态/超时定时器仍使用steady clock。
- `scripts/fast_livo2/mapping_session.py`：在线前端等待 `backend_listening` 和 `clock_ready`，显式use_sim_time=true；该条件不进入启动前占用检查，避免循环依赖。离线replay入口保持原有调度。
- `scripts/fast_livo2/localizer/online_session.py`：删除因公共入口已统一而重复的前端use_sim_time替换。原有未提交导航录制等修改保持。
- 实验 `suite.py`：等待时显示时钟/相机/配对数/后端原因/录包状态；出现时钟失败或非法输入时立即解释并受管收尾。

未改变原bag/header、几何阈值、新鲜度阈值、实际二进制或生产runtime，没有通过放宽过期检查来接纳错误历元。旧实现备份 `.runtime/backups/p0_clock_fix_20261111_170712/`；源代码活动路径不再保留该旧分支。

## 验证与证据

证据：[p0_closure_clock_fix_20261111](fast_livo2_evidence/p0_closure_clock_fix_20261111/)。

| 检查 | 命令/条件 | 退出码与结果 |
|---|---|---|
| 实验回归 | `python3 -m unittest discover -s experiments/fast_livo2_p0_closure/tests -v` | 0，16项通过 |
| 共享时钟单元 | `python3 -m unittest discover -s tests -p test_fast_livo_live_clock.py -v` | 0，6项通过 |
| 真实启动receipt复验 | `python3 -m unittest discover -s tests -p test_fast_livo_mapping_startup_clock.py -v` | 0，稳定取得锚点；后续3秒跳变仍拒绝且锚点不变 |
| 实际B节点隔离传输 | `ROS_DOMAIN_ID=219 ROS_LOCALHOST_ONLY=1 python3 -m unittest discover -s tests -p test_fast_livo_mapping_startup_clock_ros.py -v` | 0，合成启动2.9秒突发、10对发送/8对已处理、0非法消息、后端VALID |
| C1/C2公共链回归 | `ROS_DOMAIN_ID=220 ROS_LOCALHOST_ONLY=1 python3 -m unittest discover -s tests -p test_fast_livo_live_chain_ros.py -v`（Humble、online_localization_install、ws_nav/install_exact环境） | 0，定位就绪、wheel/receipt丢失撤销及恢复、锚点身份改变锁定均通过 |
| 两层计划 | 实验run.sh --dry-run及mapping_entry.py --dry-run | 0，新增clock_ready与前端模拟时间；不启动设备 |

首次隔离B测试的环境夹具遗漏RANGER_LIVO_TIMESHARE，报KeyError；已补充临时假路径。首次失败日志与最终通过日志分别保留，没有用硬件补测。

首次公共链回归没有加载ws_nav/install_exact，子进程缺ranger_localization_msgs；补全原安装环境后通过。两次日志分别保留。上述数值/实验/时钟/传输/定位链合计25项检查通过。

隔离ROS仅启动合成消息发布者、监视器和当前实际B节点，没有连接传感器、底盘或CAN。测试临时图位于系统临时目录，原实车地图未改。

## 用户重试

源码身份已经变化，保留旧失败组，创建新的五次实验组：

```bash
bash /home/lab307/ranger_nav/experiments/fast_livo2_p0_closure/run.sh
```

同样按提示操作；不对旧组使用--resume。本次修复后的实车启动尚未执行，没有提交或推送。
