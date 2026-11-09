# 当前USB-CAN与wheel启动检查

日期：2026-11-06。用户接入USB后明确要求尝试启动，随后自行配置固定CAN启停命令的免密sudo并回复完成。本次只尝试CAN/底盘驱动与wheel反馈，没有启动传感器、前端、录包、Nav2或发布速度/导航目标；原导航前置报告的软件阶段边界不因此追认为实车系统验收通过。

## 实际结果

- 免密规则查询`sudo -n -l /usr/sbin/ip link set can0 up type can bitrate 500000`退出0；检查时用户已将can0启用为500000bit/s、UP/ERROR-ACTIVE，Agent没有重复配置或切换接口。原配置快照在`.runtime/backups/wheel_start_20261106_214559/can_before.json`。
- 启动前没有底盘进程或已存活base会话；domain175/localhost内wheel和速度话题均无发布者。
- 执行`ROS_DOMAIN_ID=175 ROS_LOCALHOST_ONLY=1 bash scripts/start_base_only.sh --execute --control-authorized --startup-timeout 20`。复用原受管入口、资源锁和Mini V2配置；会话`.runtime/sessions/base/run-qqesz8kq`进入RUNNING、base/probe READY，检查结束时仍运行，供用户后续采集使用。
- 实际节点PID20584；二进制SHA`36c56486b018d87a785260710b770da25d91af613807d71d23bc7d33007f23f9`。参数服务确认Mini V2/can0、`/wheel/odom`、50Hz、`use_sim_time=false`、`publish_odom_tf=false`。
- 7秒ROS观察收到351条wheel，按接收首尾估计50.684Hz（含启动积压的影响）；header严格递增，odom/base_link正确，最大样本年龄95.48ms、最后约0.80ms。观察到首尾位置/速度为零。wheel唯一发布者为ranger_base_node。
- `/cmd_vel`和`/base/safe_cmd_vel`无发布者。驱动创建了TF广播端点，但后续3秒没有收到任何TF消息，不能将“图中存在端点”写成“仍在发布odom TF”。
- CAN总线错误/错误被动/bus-off计数为0；检查后TX只有1帧，来自驱动`EnableCommandedMode()`的模式请求，不是运动控制。启动前已有RX dropped累计值，不能把其历史值当作本次丢包率；接收套接字工作后计数未继续增加。

## 不能将启动成功等同于运动就绪

ROS消息显示`vehicle_state=2, control_mode=0, error_code=0`。进一步只接收原始0x211帧96条，得到例如`0200022700000200`：vehicle_state=2，control_mode=0，电压55.1V，错误字段为`0x00000200`。

对照[厂家V2手册](https://static.generation-robots.com/media/ranger-mini-user-manual-en.pdf)第19–23页：vehicle_state=2表示异常；控制模式0是待机、1是命令、3是遥控；byte[6] bit[1]为上位机通信状态故障位。现有SDK仅读取byte[4:6]的16位error_code，ROS消息枚举还把control_mode=0叫RC，与该V2手册不符。因此本次早期“ROS错误码0”不能作为底盘健康结论，原始帧已保存，不能忽略被截掉的低16位。

本次**CAN/wheel启动PASS，运动就绪未通过**。驱动发送了模式请求，但实际反馈仍待机，不宣称CAN控制切换已成功。不发送周期控制心跳或运动命令去清除该状态；后续运动采集前，先核对遥控器/底盘显示及手动接管状态。V2完整故障字段和模式语义需在后续底盘接口工作中修正，不能依赖当前`error_code=0`自动放行导航。

小证据在`docs/fast_livo2_evidence/wheel_start_20261106/`，包括真实参数、wheel统计、原始帧、TF观察、CAN前后计数和会话boot/PID身份。无新Git提交或推送。

状态/停止仍用受管入口：

```bash
bash scripts/start_base_only.sh --status
bash scripts/start_base_only.sh --stop
```

录包入口仍不启动底盘；确认手动运动条件后复用`record_fast_livo2.sh --seconds 180 --label wheel_alignment_visit01 --visualization none --min-free-gb 100 --record-wheel`。本次没有替用户执行录制。当前FAST与wheel真实时间映射仍待同录包核验。
