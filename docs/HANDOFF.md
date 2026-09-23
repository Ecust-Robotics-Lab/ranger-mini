# 新窗口接手指南

> 2026-09-15 阶段 3：实物已更正并协议确认为 **Mid-360S**，SN ARMCP870038745。静止 SQLite3 包与真实收流通过，逐点时间回退仍为 PARTIAL；网络持久化配置/索引权限修复已完成，实际重启待验，当前采集后网卡无载波。雷达已停，底盘未启动。以 [STATUS.md](STATUS.md) 和 [03_mid360_validation.md](03_mid360_validation.md) 为准，不自动进入 SLAM/导航。

> 2026-09-14 后续更新：用户已授权并开展阶段 2，确认底盘为 Ranger Mini V2。以下交接正文保留为此前快照；最新源码、构建、CAN、ROS 与测试结果以 [STATUS.md](STATUS.md)、[02_ros_motion_test.md](02_ros_motion_test.md) 为准。22:11、22:16 两次另获授权的低速测试已执行，用户确认最后一次实物运动；主动发零后反馈归零，但退出后负速度异常复现，停车安全未通过。驱动已退出，不自动重启。

交接日期：2026-09-14，Asia/Shanghai。用户已结束上一窗口任务，下一阶段尚未指定。只读接手，不自动安装、改网、运行驱动或进入下一阶段。

## 先读这些文件

1. `/home/lab307/ranger_nav/AGENTS.md`：项目约束和真机许可边界。
2. `docs/STATUS.md`：最近一次已验证状态。
3. `docs/00_environment_audit.md`：2026-09-13 阶段 0 的历史快照，不是网卡修复后的状态。
4. `docs/LIDAR_CONNECTION.md`：按时间追加的连接处理记录。以最后“2026-09-14 21:10 最终连接结果”章节为准，早期 BLOCKED 是历史状态。
5. `docs/mid360_ip_connected_20260914.txt`：实际成功命令、退出码和输出。

## 已完成与边界

- 项目根目录 `/home/lab307/ranger_nav`，不要与其他工程混用。
- 本机已核实为原生 Ubuntu 22.04.5、x86_64、内核 6.8.0-40-generic，已安装 ROS 2 Humble。当前工具 shell 可能未 source ROS；`.bashrc` 中的 source 对非交互 shell 不一定执行。
- 阶段 0 审计 PARTIAL。用户之后明确追加“先连接上雷达”，这项已完成到 **IP 通信 PASS**。
- 用户确认 Mid-360 序列号尾号 **8745**，接板载以太网口。默认规则推导地址 `192.168.1.145` 已响应，尚未通过 Livox 协议读取完整 SN。
- 板载 PCI `0000:02:00.0` / `1f0a:6801`，驱动 `yt6801` 1.0.34，正式接口 **enp2s0**，电脑地址 **192.168.1.50/24**。
- 2026-09-14 21:10，指定 enp2s0 ping `192.168.1.145`：5 发 5 收、0% 丢包、平均 1.829 ms。邻居 MAC `ec:72:f7:da:cf:c8`。物理协商 100 Mbit/s、full duplex。
- 上网 `wlo1`：192.168.3.18/24，默认 via 192.168.3.1 metric 600。雷达网段与它不重叠，默认路由未变。
- **没有验收 UDP 点云、IMU、ROS 话题、录包、建图或定位。** 不将 ping 成功称为雷达数据链路全通过。
- Ranger Mini 实物具体版本尚未确认；USB-CAN `gs_usb` / `can0` 最近为 DOWN，没有当前 bitrate 或底盘收帧验收。不得启用底盘或发送控制。

## 当前配置是临时的

- 签名驱动仅 insmod 加载，**未安装到 `/lib/modules`**，不保证重启后自动加载。
- NM 连接 `ranger-mid360-8745` 已创建并激活，UUID `7d3735fd-229f-4461-bebd-421146eb06d2`，地址 192.168.1.50/24，`ipv4.never-default=yes`，无 gateway，IPv6 disabled，**autoconnect=no**。
- `有线连接 1` 是 NM 自动创建的旧 DHCP 配置；不要误改它或删除 Wi-Fi。
- 新窗口先重新读取实际状态。若机器重启或雷达重新接线，不能照抄过去的 PASS 当成当前结果。持久化尚未实施，也未被指定为下一阶段。

## 驱动来源与 Secure Boot

- 原因：系统缺少板载 YT6801 驱动；为现有内核补装 GCC 12，并编译厂商驱动，未更换内核或系统。
- 厂商归档：`vendor/yt6801/yt6801-linux-driver-1.0.34.zip`。
- URL：`https://www.motor-comm.com/Cn/Skippower/downloadFile.html?id=1817`。
- 归档 SHA256：`877e6953fc6eb47232a74e47b0daf33de0e7f9e9a395d3bc10d500d3c4150af8`。
- 源码目录：`vendor/yt6801/source/yt6801-1.0.34`，源码未改，含构建产物。厂商归档不是 Git 仓库，分支/commit SHA 不适用，不能虚构。
- 构建模块未签名版本 SHA256：`aed6951c8e9656d0443c2a52b07ce0be8061a56a46d83309b9d8bebba3c83918`。这是未签名文件，不能拿它替代 root 目录的签名副本。
- Secure Boot 保持开启。用户已现场完成 MOK 登记；证书 CN `ranger-nav YT6801 module signing`。
- 证书 SHA256：`84:B2:A7:E5:EA:31:84:56:1D:A7:B9:B2:0E:A6:A7:A7:EB:00:99:C0:D1:C5:4F:99:BA:68:05:33:CB:87:8A:98`。
- root-only 状态目录 `/var/lib/ranger-nav-yt6801`：含签名模块 `yt6801.ko`、`signed-module.sha256`、公共证书 `MOK.der` 和私钥。**不读取或输出私钥，不把它复制到项目。**
- 官方 ROS/SLAM 五个候选仓库尚未克隆。不要误以为已经建立 ROS 工作空间或装好了 Lightning-LM。

## 脚本与已修复问题

- `scripts/prepare_yt6801_secureboot.sh`：只用于首次签名/登记准备；已执行完，不能重新运行或要求用户再登记。
- `scripts/connect_mid360.sh`：默认 dry-run；新建雷达临时连接用。当前连接已存在，**不要反复 --apply**；先读实际状态，它会拒绝已有项目连接。
- `mokutil 0.6.0 --test-key` 对“已登记”返回 1。已改成捕获状态并严格验证肯定输出，不可仅凭非零退出判断登记失败。`--ignore-keyring` 仅影响该次只读查询，内核签名强制检查未关闭。
- 内核最初注册 eth0，udev 随后改为 enp2s0；已加入等待最终名称逻辑。
- 本机 nmcli 不接受创建时改写 connection.uuid；应让 NM 生成。详情查询用 `-g connection.uuid`，摘要列表才用 `UUID`。
- 早期 dry-run/语法通过没有覆盖全部真机路径，实际执行时发现上述兼容问题；已修复，不能宣称早期脚本全流程一次通过。

## 回滚与证据

最近一次有效状态/日志目录：`/var/lib/ranger-nav-yt6801/connect-jQpZBTzH/`。

```bash
sudo bash /var/lib/ranger-nav-yt6801/connect-jQpZBTzH/rollback.sh
```

此命令是**备用回滚，不是接手步骤**：按 UUID 删除项目连接，并按记录恢复先前有线连接；保留本次操作前已加载的驱动。若需卸载驱动，必须先确认 enp2s0 已不再使用。不要运行早期失败尝试目录的旧回滚脚本。

该目录包含 `activation.log`、激活前后默认路由、原地址和修复后的回滚脚本。普通用户可能不可读。项目 docs 中另有完整只读审计和成功验证证据。

## 接手可执行的只读检查

```bash
uname -r
ip -brief address
ip route
ip route get 192.168.1.145
ip neigh show dev enp2s0
nmcli -f connection.id,connection.uuid,connection.interface-name,connection.autoconnect,ipv4.addresses,ipv4.never-default connection show ranger-mid360-8745
```

这些命令不加载模块、不激活连接、不启动 ROS。若沙箱阻止 netlink/D-Bus，应通过正常工具授权读取，不将沙箱限制误判为主机网络故障。

## 接手工作方式

- 阅读并概述当前状态后，等待用户指定下一阶段；不自动继续雷达驱动、持久化或 SLAM。
- 后续技术路线仍为 Lightning-LM 建图/定位；先通信、录包、建图、定位和平地导航，坡道后续。Nav2/Smac Hybrid-A*/MPPI 是后续候选。
- 用户偏好少来回确认，已授权范围内自主实施；必要修改先保存原值并提供回滚。所有自动测试默认 dry-run。
- 没有任何底盘运动许可。启动可能切换底盘控制模式的驱动、发送 CAN 控制、非零速度或导航目标，必须取得用户对该次测试的明确许可；进程重启后旧运动许可失效。
- 用户曾要求代理自行处理正常 sudo 认证；**本交接不包含密码，也不要求新窗口从旧聊天查找密码**。以新窗口实际工具权限和可用的正常认证方式为准，不写入密码、不绕过权限。
- 每阶段更新 STATUS.md 和对应报告；注明实测、用户提供信息、推断及待验项。
