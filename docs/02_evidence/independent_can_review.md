# 独立 Agent：USB-CAN 与历史配置只读核对

时间：2026-09-14 21:51–21:54 +08:00。底盘型号 **Ranger Mini V2** 来自本轮用户确认；本次没有读取铭牌、SN 或固件。范围仅独立只读核对，不配置 CAN、不启动 ROS/SDK 节点、不发送控制。

## 结果与适用范围

- **PASS：USB-CAN 枚举和 Linux 接口映射。** `lsusb -d 1d50:606f` 实际返回 Bus 001 Device 005。sysfs `/sys/bus/usb/devices/1-11/{idVendor,idProduct,busnum,devnum}` 分别为 `1d50 / 606f / 1 / 5`；`can0` 的设备路径指向 `1-11:1.0`，driver 为 `gs_usb`。udev 同时给出 `ID_MODEL=candleLight_USB_to_CAN_adapter`、`ID_NET_DRIVER=gs_usb`、`INTERFACE=can0`，完成 USB 到接口的交叉核对。
- **BLOCKED：实际底盘 CAN 接收，未通过通信验收。** 前后两次读取均为 `can0 DOWN`、CAN `STOPPED`，无 `bittiming/bitrate` 字段。10 秒上限的 `candump` 立即返回 `read: Network is down`，退出 1，没有反馈帧。不能据此判断底盘供电、线缆、终端电阻或固件是否正常，也不能由 USB 枚举推导底盘已经通信。
- **PARTIAL：错误状态核对。** 两次读取 RX/TX 的字节、包、错误、丢包均为 0，CAN 累计 restart/bus-error/arbitration-lost/error-warning/error-passive/bus-off 均为 0，差值为 0。接口未运行，这些零值不构成健康总线证明；当前输出未提供控制器实时 TEC/REC，不能声称实时错误计数已验收。
- **PARTIAL：历史成功配置追溯。** 已搜索用户 shell 历史、常用工程/文档/下载目录、系统网络与服务配置、近期 journal，没有找到此前成功的 `candump` 命令或 bitrate。搜索无匹配不代表历史从未成功；本次也没有访问旧聊天或凭据存储。
- **未验证：运动安全。** 没有底盘状态帧、控制测试、超时停车、遥控接管或急停证据，不能放行自主导航。

## 官方 500000 依据（未执行）

本地官方 `ranger_ros2` Humble 对应完整 SHA 为 `b6ea21a275ca5e7168130cc6470e61474681d679`，当前检出为 **detached HEAD**；工作树现有修改为 `ranger_base/src/ranger_messenger.cpp`，由主 Agent 处理，本 Agent 未改源码。

- [bringup_can2usb.bash](/home/lab307/ranger_nav/ws_base/src/ranger_ros2/ranger_bringup/scripts/bringup_can2usb.bash:4) 第 4 行为 `sudo ip link set can0 up type can bitrate 500000`。
- [setup_can2usb.bash](/home/lab307/ranger_nav/ws_base/src/ranger_ros2/ranger_bringup/scripts/setup_can2usb.bash:7) 第 7 行同样配置 `500000`；第 4 行加载 `gs_usb`，第 10 行安装 can-utils。仅查看，未执行这些脚本。
- V2 launch 的实际设备默认值是 [ranger_mini_v2.launch.py](/home/lab307/ranger_nav/ws_base/src/ranger_ros2/ranger_bringup/launch/ranger_mini_v2.launch.py:16) 中的 `can0`；型号声明在第 31 行为 `ranger_mini_v2`。注意这个 launch 的节点参数表第 38–46 行未传入 `robot_model`，不能仅凭文件名认定节点得到 V2 型号参数；主 Agent 正处理项目配置。

因此 **500000 是固定版本官方 CAN 脚本的默认值，不是本机旧成功配置的已找回值，也不是本轮实测成功的 bitrate**。

## 命令、退出结果与证据

| 检查 | 退出结果 | 证据 |
| --- | --- | --- |
| `lsusb -d 1d50:606f`、`udevadm info --query=property --path=/sys/class/net/can0`、sysfs 读取、`ip -details -statistics link show can0` | 均 0 | [independent_can_host.log](independent_can_host.log) |
| `python3 scripts/base_readonly_diagnostics.py --execute --seconds 10` | 脚本 2；四次状态查询均 0；candump 1 | [commands.json](can-9h13lxmt/commands.json)、[candump.log](can-9h13lxmt/candump.log)、[before-json.log](can-9h13lxmt/before-json.log)、[after-json.log](can-9h13lxmt/after-json.log) |
| `rg -n -i ... candump\|can0\|bitrate\|gs_usb\|slcan` | 1，无匹配且无错误输出 | [independent_can_history_search.log](independent_can_history_search.log)，记录完整搜索范围 |
| `journalctl --no-pager --since 2026-09-01 -g 'can0\|gs_usb\|candump\|bitrate' -n 100` | 0，仅找到 gs_usb 注册和 can0 新设备记录，无 bitrate/成功接收记录 | [independent_can_system_search.log](independent_can_system_search.log) |
| `nmcli -t -f NAME,TYPE,DEVICE connection show` | 0，列出 Wi-Fi 与两项以太网连接，无 CAN 连接 | 同上 |
| 搜索 `/run/systemd/network` 和 `/usr/lib/systemd/network` | 2；前者不存在，后者无匹配 | 同上；不是成功搜索全部路径 |
| 官方脚本带行号读取、Git HEAD/分支/修改查询 | 均 0；分支名空说明 detached HEAD | [independent_can_host.log](independent_can_host.log) |

本次新增文件仅本报告、三个 `independent_can_*.log` 及独立诊断目录 `can-9h13lxmt/`。未修改 STATUS 或阶段主报告。

## 最少后续人工项

本次独立核对已完成。要继续通信验收，需补齐或决定 CAN 配置依据并恢复接口接收条件；当前接口原状态已保存为 DOWN/STOPPED、无 bitrate。本轮子任务明确只读，因此未执行启用接口或设定 bitrate。若采用固定官方 500000 候选，应先由主 Agent 按用户授权边界明确下一步配置及回滚，再只接收限时 CAN，核对反馈 ID、周期与错误增量。任何底盘接管、控制、运动测试仍须当次单独授权。
