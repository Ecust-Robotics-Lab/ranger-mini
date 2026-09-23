# 阶段 0 环境与现有工程审计

2026-09-13，Asia/Shanghai。结果：PARTIAL。用户随后明确要求“先连接上雷达”，雷达连接处理另记，不将其冒充阶段 0 只读工作。

## 已确认事实
- Ubuntu 22.04.5、内核 6.8.0-40-generic、x86_64。systemd-detect-virt（含 vm/container）均输出 none、退出 1；DMI 为 THUNDEROBOT MIX G2，证据支持原生 Ubuntu，未发现 VMware/WSL/容器。最初工具沙箱限制已解除，不是宿主虚拟化证据。
- i9-14900HX，24 核/32 线程；内存约 30 GiB、可用 28 GiB；根 ext4 磁盘 938 GiB、可用 875 GiB，与 HOME 共盘。用户 lab307，HOME/检查 cwd 为 /home/lab307。
- /opt/ros 仅见 humble，ros-humble-desktop 已安装（0.10.0-1jammy.20260804.223343）。当前非交互 shell ROS_DISTRO/ROS_VERSION 未设置，ros2 未在 PATH；其可执行文件存在。
- Python /usr/bin/python3 3.10.12，CMake 3.22.1，GCC/G++ 11.4.0，colcon-core 0.21.2+upstream-1，rosdep 0.27.0。未找到 candump/can-utils，未发现 rosdep 标准 sources.list.d 与用户缓存。
- 未激活 Conda/venv，sys.prefix/base_prefix 同为 /usr；PYTHONPATH、LD_LIBRARY_PATH 未设置，无已见 Conda/SLAM 路径污染。PATH 含 NVM，系统编译工具解析正常；未穷尽验证 Python 包兼容。
- ROS_DOMAIN_ID、RMW_IMPLEMENTATION 及 DDS 配置变量未设置；已安装 rmw_fastrtps_cpp，未安装 rmw_cyclonedds_cpp。use_sim_time 是节点参数：本地 rclpy/time_source.py:83 默认 False，未查询运行节点，实际覆盖值未知。
- ~/.bashrc:124 source /opt/ros/humble/setup.bash，非交互 shell 提前 return；另加载 NVM，无发现其他 workspace/Conda source。部分系统 debuginfod 启动文件普通用户不可读。
- 深度 5 限定搜索 /home/lab307、/opt、/srv、/mnt、/media，剪枝缓存、bags/maps/data/datasets/build/install/log；未发现 ranger_ros2、ugv_sdk、Livox-SDK2、livox_ros_driver2、lightning-lm。五仓库分支/完整 SHA/修改均不适用，未克隆。未扫描大型数据内容。
- USB-CAN 1d50:606f，gs_usb，接口 can0 DOWN/STOPPED、restart-ms 0；未显示 bitrate，48 MHz 是控制器时钟。RX/TX 与所有已显示错误/bus-off 计数为 0，TEC/REC 未显示。零计数不证明通信正常。所查 systemd/udev/modules/用户配置及筛选历史未找到旧 CAN 设置。
- 用户确认 Mid-360 已接板载网口，序列号尾号 8745。PCI 02:00.0 以太网 ID 1f0a:6801，无 driver 绑定、无有线接口，精确 modules.alias 查询无匹配；当前不能直接接收雷达数据。
- 上网接口 wlo1/iwlwifi，192.168.3.18/24；默认 via 192.168.3.1 metric 600，另有 169.254.0.0/16 路由。IPv6 地址/路由见证据。NetworkManager active，networkd inactive；netplan renderer NetworkManager，NM 仅 Wi-Fi 连接 10-307。
- 官方默认雷达 IP 规则为 192.168.1.1XX，XX 为 SN 最后两位，故候选 192.168.1.145（实际配置未读到）。此 /24 与上网 /24 不重叠，但当前未配置雷达网段。[官方快速指南](https://dl.djicdn.com/downloads/Livox/Mid-360/QSG/Livox_Mid-360_Quick_Start_Guide_multi.pdf)。
- 用户要求连接检查后：ip route get 192.168.1.145 退出 0，走 Wi-Fi 默认网关；ping 3 发 0 收、退出 1。不是板载直连测试，不能判定雷达故障。未启动雷达/底盘驱动。
- HTTP/HTTPS 大小写代理指向 127.0.0.1:7897，端口监听；未设 ALL_PROXY/NO_PROXY，未检出 Git/APT 显式代理。报告凭据脱敏。GNOME proxy 返回 none，但有 dconf 缓存失败提示，仅辅助证据。
- APT ROS 源为中科大 jammy。现有镜像 InRelease、github.com、raw.githubusercontent.com 的 README 经当前代理与直连各 HTTP 200/curl 退出 0；packages.ros.org HTTPS 两方式均证书主机名不匹配、退出 60。未绕过 TLS，未 apt update。HEAD README 只作连通性测试，未用作选定源码版本。
- ufw.conf ENABLED=no，ufw 服务 active；sudo -n ufw status verbose 退出 1、需要密码。不能据服务或配置断言运行时规则。

## 尚未确认
底盘实物版本、旧 candump 来源/bitrate、接线与帧接收；网卡所需驱动、雷达实际 IP/供电/UDP；防火墙运行规则；ROS 节点参数与实际 RMW、完整构建依赖及 ROS Python 动态加载。VMware 直通不适用。本机资源适合作为集成基础，但真机连接未通过。

## 建议修改（阶段 0 未实施）
保留 Ubuntu/Humble；查明板载网卡驱动，再为实际雷达地址配置独立直连网段，保留 Wi-Fi 默认路由；找回旧 CAN 配置后再处理底盘；后续独立 shell 显式 source ROS，按需初始化 rosdep。所有配置修改先保存原值并给出恢复办法。

## 命令、文件与证据
- 00_audit_evidence.txt：初始检查、包清单、软件/环境/配置；复合命令退出码仅是最后一条，需读内部错误。
- 00_audit_host_network.txt：脱离沙箱后 lsusb/ip/NM 各退出 0；curl 与 sudo 单独退出码和原输出。
- 00_audit_followup.txt：主机、PCI driver 缺失、shell、CAN 历史及软件细节；virt 退出 1/none，NetworkManager 0/active，networkd 3/inactive。配置 rg 无匹配为 1，限定 find 为 0。
- 00_lidar_connectivity.txt：雷达候选地址路由和 ping 证据。
- 新增以上四份证据及本报告，更新 STATUS.md；未修改系统/网络，无需配置回滚。无安装、CAN 控制、底盘驱动或导航动作。

## 最少人工操作
铭牌/旧 CAN 配置线索后续补充；sudo 受限项目需要本机正常授权，勿发送密码。阶段 0 只读审计结束。用户已另行要求先打通雷达，后续动作独立记录。
