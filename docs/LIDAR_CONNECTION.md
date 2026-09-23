# Mid-360 连接处理

2026-09-13。用户在阶段 0 后明确要求“先连接上雷达再说”。本任务范围为板载网卡恢复与雷达连接，不授权底盘动作。

## 当前状态：PASS（2026-09-14 板载直连 IP 通信；ROS/UDP 点云尚未验收）

雷达 SN 尾号 8745，用户确认接板载网口。官方默认候选 IP 为 192.168.1.145；3 次 ping 无回复，实际走 Wi-Fi 网关，不能认作板载链路测试。详见 00_lidar_connectivity.txt。

## 已完成

- PCI 1f0a:6801 / 02:00.0 未绑定驱动，modinfo yt6801 退出 1。
- 官网下载 YT6801 1.0.34，并核对源码 fuxi-gmac-pci.c:252 包含 PCI_DEVICE(0x1f0a, 0x6801)。
- 来源：https://www.motor-comm.com/download?category=&kw=6801&tp=1&wd=1
- 实际下载：https://www.motor-comm.com/Cn/Skippower/downloadFile.html?id=1817
- 本地归档：vendor/yt6801/yt6801-linux-driver-1.0.34.zip
- SHA256：877e6953fc6eb47232a74e47b0daf33de0e7f9e9a395d3bc10d500d3c4150af8
- 厂商归档不是 Git 仓库，分支与 commit SHA 不适用，以版本、URL、完整归档 SHA256 固定来源。源码未改动；构建产生的临时文件与源码同目录。
- 解压前拒绝绝对路径、上级路径和链接成员。审查 yt_nic_install.sh、Makefile、dkms.conf 后，未运行安装脚本：它会删除已有解压目录、执行 make install、insmod/rmmod，未处理本机 Secure Boot 信任。
- 仅执行无 root 的 make build -j4：退出 2，缺 gcc-12，未产出可加载模块；日志 yt6801_build.log。内核 headers 已安装，内核构建编译器为 GCC 12；当前系统 GCC 11。
- apt-get -s install gcc-12=12.3.0-1ubuntu1~22.04.3 退出 0，预计新增 cpp-12、gcc-12、libasan8、libgcc-12-dev、libtsan2 共 5 包，升级/移除均 0。仅预览，未安装；见 gcc12_install_preview.txt。
- SecureBoot enabled，lockdown 为 integrity；/var/lib/shim-signed/mok 为空，标准 DKMS MOK 文件不存在。没有尝试禁用 Secure Boot、导入密钥或重启。
- sudo -n true 退出 1，需要密码。工具完全权限不等于 root，也不绕过认证。

## 原值与回滚

网络与模块/编译器原状态：lidar_network_before.txt；当前没有系统/网卡变更，无需回滚。Wi-Fi 连接 UUID 793ad98f-388b-496a-9343-eea24935b112、192.168.3.18/24、默认网关 192.168.3.1 保持原样。

若按下一步仅安装预览的五个包，可在未有其他任务开始依赖它们的前提下，先用 apt-get -s remove gcc-12 cpp-12 libgcc-12-dev libasan8 libtsan2 审核，然后用 sudo apt-get remove 对同一列表回退；不使用 autoremove，不改默认 gcc 链接。实际安装前若计划与预览不同应重新审查。

网卡配置、模块安装、签名密钥注册都尚未实施，对应操作将在实际接口名、签名和模块构建通过后另行记录精确回滚；不提供虚构接口名的可执行改网命令。

## 下一步最少人工操作

在本机终端运行下列已预览命令，密码只在本机 sudo 提示输入，不在聊天发送：

```bash
sudo apt-get install gcc-12=12.3.0-1ubuntu1~22.04.3
```

然后继续无 root 编译及模块元数据验收。Secure Boot 加载第三方模块须采用受信任签名；推荐保留 Secure Boot、使用签名与 MOK 注册流程，届时需要现场重启进入 MokManager，不能由当前 SSH/工具会话代为确认。尚未请求或执行重启/密钥注册。

参考：[Ubuntu Secure Boot 文档](https://documentation.ubuntu.com/security/security-features/platform-protections/secure-boot/)。

已新增厂商归档/源码、下载页面证据、本报告、构建日志、依赖预览和原状态快照；更新 STATUS.md。雷达 IP/UDP 未验证，CAN 保持 DOWN，没有运动许可。


## 后续更新：GCC 12 已安装，驱动构建通过

用户反馈安装完成后，dpkg 确认五包均为 12.3.0-1ubuntu1~22.04.3。make build -j4 退出 0，日志 docs/yt6801_build_gcc12.log。编译器补丁版本与内核构建时不同，均为 GCC 12.3.0；出现版本提示但构建成功。缺 vmlinux 跳过 BTF，未导致构建失败。

模块 vendor/yt6801/source/yt6801-1.0.34/yt6801.ko：version 1.0.34，vermagic 6.8.0-40-generic，PCI alias 匹配 1f0a:6801，无依赖、尚未签名。SHA256：aed6951c8e9656d0443c2a52b07ce0be8061a56a46d83309b9d8bebba3c83918。srcversion A3B1ED3EA1E69BCEBC8A686 是模块标识，不是 Git commit。

已创建并审查 scripts/prepare_yt6801_secureboot.sh，bash -n 与 --dry-run 均退出 0，证据 docs/yt6801_secureboot_preflight.txt。尚未执行 --apply。

脚本应用时检查指定内核、模块版本/PCI/SHA256、无既有项目状态目录及无待注册 MOK，然后仅创建 /var/lib/ranger-nav-yt6801（root-only），保存状态，生成项目专用模块签名密钥（10 年有效，限制为 module signing），签署固定模块副本，最后在本机终端运行 mokutil --import。私钥不放进工程或报告。脚本不安装/加载模块、不改网、不重启；不修改其他工程的标准 MOK 密钥。

签名配置参照本机 /usr/lib/shim/mok/openssl.cnf 的 CA:FALSE 与 module-only EKU；读取审查了 update-secureboot-policy，其 --enroll-key 会检查 DKMS 模块数量，而本方案没有安装 DKMS，因此采用显式 mokutil 注册。没有关闭 Secure Boot。

### 当前最少人工操作（替代前面的 gcc-12 安装步骤）

```bash
sudo bash /home/lab307/ranger_nav/scripts/prepare_yt6801_secureboot.sh --apply
```

这次执行会请求将项目专用模块签名证书加入信任，须在本机终端自行输入 sudo 密码及设置临时 MOK 登记密码，不能在聊天发送。命令成功后保存所有工作，自行重启；MokManager 选择 Enroll MOK → Continue → Yes，核对证书 CN 为 ranger-nav YT6801 module signing，输入刚设置的临时登记密码，再 Reboot。重启回来告知助手继续验证。

注册前/后状态与签名校验仍待实际执行；dry-run 不是签名成功或驱动加载成功证据。重启后须重新核对 uname -r；若内核变化，先为新内核构建，不能强行加载旧模块。

### 本次待执行操作的回滚

- 尚未执行 --apply：无需回滚，原网络未变。
- 如果只生成密钥/签名副本、未成功导入：没有固件信任变更。保留 root-only 项目目录供检查；不要重复盲跑脚本，它会因目录存在而退出。
- 已请求导入但未重启确认：先 sudo mokutil --list-new 核对仅有本项目证书，随后 sudo mokutil --revoke-import 取消待登记请求；如有其他项目请求则不要使用该全局取消命令，先人工处理。
- 已登记：sudo mokutil --delete /var/lib/ranger-nav-yt6801/MOK.der，请求删除本项目证书，再由现场重启确认；这会撤销该证书签署模块的加载信任。当前脚本不会安装模块，所以此时不需要改 /lib/modules。仅在确认撤销完成且模块不再使用后才考虑删除专用私钥，避免影响恢复。
- 模块及网卡配置安装/加载仍未实施，其具体回滚将在后续执行前补齐；Wi-Fi 默认路由保持原值。

当前没有任何 CAN/底盘动作，雷达实际通信仍未通过。等待本机 sudo/MOK 人工环节，不能伪造已连接。


## 更新：用户已执行签名与登记请求

用户终端输出显示 --apply 完成：模块副本签名成功，证书 CN 为 ranger-nav YT6801 module signing，SHA256 指纹为 84:B2:A7:E5:EA:31:84:56:1D:A7:B9:B2:0E:A6:A7:A7:EB:00:99:C0:D1:C5:4F:99:BA:68:05:33:CB:87:8A:98，mokutil 提示请求已提交。没有记录任何认证或临时登记密码。

助手只读复核：项目 root-only 状态目录已存在；EFI MokNew 变量文件存在，修改时间对应此次操作；Secure Boot 仍开启。普通用户 mokutil --list-new 退出 0 但无输出，故没有独立核实待登记证书内容；--list-enrolled 未发现项目 CN。当前启动时间未改变，尚未完成重启确认。证据 docs/yt6801_mok_pending.txt。

当前状态仍 BLOCKED：等待现场保存工作、重启，并在 MokManager 选择 Enroll MOK → Continue → Yes，输入用户自行设置的临时 MOK 登记密码，随后 Reboot。这个临时密码与 sudo 密码是不同用途。助手不接收或使用聊天提供的认证信息，不自动重启。

登记后继续核对证书、内核版本、签名模块，再加载网卡并配置雷达直连网络。当前未安装/加载网卡模块，Wi-Fi 路由和 can0 状态未改，雷达尚未连通。


## 更新：重启与 MOK 登记已确认，等待本机加载步骤

本机 uptime -s 为 2026-09-13 21:55:14，内核仍 6.8.0-40-generic；SecureBoot enabled。mokutil --list-enrolled 已出现 ranger-nav YT6801 module signing，证书序列号 65:15:d3:ff:55:35:3e:cb:17:4e:b2:ec:62:43:3a:38:cd:52:8e:bb，SHA1 为 8a:32:bb:8c:aa:f2:dc:89:68:51:58:cd:b3:a1:a6:37:3c:e6:5f:e3。MOK 登记 PASS；root-only DER 的 SHA256 和模块签名将在加载脚本内再严格校验。

沙箱外网络复核成功，仍只有 lo/wlo1、无板载以太网接口；本次 can0 也未枚举（此前 DOWN 是重启前状态），没有操作底盘。sudo -n 的三项模块/证书读取仍因需密码退出 1。证据 docs/yt6801_post_reboot.txt。

已创建 scripts/connect_mid360.sh，默认 dry-run；bash -n 与 dry-run 退出 0，证据 docs/mid360_connect_preflight.txt。待用户本机执行：

```bash
sudo bash /home/lab307/ranger_nav/scripts/connect_mid360.sh --apply
```

应用内容：校验当前内核、证书 SHA256、MOK 信任、已签名模块 SHA256/版本/PCI；仅加载 root-only 签名模块；从 PCI 02:00.0 实际 net 子目录取得接口名；建立专用 NM 连接 ranger-mid360-8745，192.168.1.50/24，无默认路由、无自动 DNS、IPv6 disabled、autoconnect no，激活后指定接口 ping 192.168.1.145 五次。该 IP 是默认地址候选，尚非设备实际查询结果。脚本不运行任何 ROS/底盘驱动。

每次执行前生成 root-only /var/lib/ranger-nav-yt6801/connect-XXXXXXXX 状态目录，记录原地址、路由、NM 连接以及新模块加载后的接口状态；预先生成 rollback.sh，通过该次唯一 UUID 删除专用 NM 连接并卸载 yt6801，命令将在终端打印。不使用模糊接口名，不删除现有连接；发现已有项目连接、PCI 驱动或模块时拒绝重复执行。失败时保留状态供检查，不假报成功。

这是本次临时联网测试：不将模块安装到 /lib/modules，不设置开机自动联网。连通性通过后再决定持久配置；重启后此临时模块不会自动加载。ping PASS 也不等于 Livox UDP 与 SN 确认。

当前状态：MOK/构建 PASS，雷达连接 BLOCKED（本机 sudo 操作待执行）。当前可读状态已留档，网络未由助手修改。


## 2026-09-14：授权执行尝试

用户再次授权连接雷达。通过沙箱外正常认证流程尝试：
- sudo -n bash /home/lab307/ranger_nav/scripts/connect_mid360.sh --apply：退出 1，sudo 提示需要密码，脚本未启动。
- timeout 45s pkexec --disable-internal-agent /usr/bin/bash /home/lab307/ranger_nav/scripts/connect_mid360.sh --apply：退出 127，No authentication agent found，脚本未启动。

本会话无可用桌面认证代理；工具执行授权不能替代 sudo/PolicyKit 身份认证。没有使用、记录或向命令传递聊天中的认证信息。系统/网络未变更。状态 BLOCKED，下一步只能由用户在自己的终端执行已审查的 sudo 命令，以终端提示完成认证；不需要再向助手授予相同权限。


## 2026-09-14 20:58 复核

用户反馈“可以了”后，重新读取实际状态：MOK 项目证书仍在信任列表；内核 6.8.0-40-generic，启动时间仍为 2026-09-13 21:55:14。/sys/module/yt6801 不存在，PCI 02:00.0 未绑定驱动，只有 lo/wlo1/can0 接口。NM 查询 ranger-mid360-8745 返回 10、配置不存在；到 192.168.1.145 仍经 Wi-Fi 默认网关。因此不能标记雷达连通。

尝试在当前授权下执行 sudo -n bash /home/lab307/ranger_nav/scripts/connect_mid360.sh --apply，sudo 返回 1、需要密码；脚本未运行，无系统或网络变更。未使用聊天中出现过的认证信息。

结果：BLOCKED（本机 sudo 执行待完成）。最少人工操作仍为在本机终端执行上述命令（去掉 -n），提供完整输出；如用户已执行过，则需其终端输出定位中断点，不能反复猜测已成功。证据 docs/mid360_connection_verification_20260914.txt。本轮不额外 ping，因为当前路由仍不经过雷达物理链路。


## 2026-09-14 修复连接脚本 MOK 返回码处理

用户提供完整输出表明脚本在证书已登记提示后立即退出。已复现：本机 mokutil 0.6.0 的 --test-key 对已登记证书返回 1，导致原脚本 set -e 提前退出；这是脚本错误，不是用户 MOK 操作失败。

读取官方源码 https://raw.githubusercontent.com/lcp/mokutil/0.6.0/src/mokutil.c 的 test_key 函数，未登记返回 0、已登记返回 1（1537–1542 行）；本地副本 docs/mokutil_0.6.0_source.c，SHA256 046037be762aa07babb9ffd3c2c3eff1e4e63941d6a7c979468f8cab55215e35。该文件作为官方版本文档证据获取，未克隆该仓库，不将 0.6.0 tag 冒充完整 commit SHA。

已修改 scripts/connect_mid360.sh：捕获返回码，仅接受 0/1 且 stdout 精确为目标证书“is already enrolled”；任何读取错误或未登记文本都拒绝。添加行号/退出码错误提示。只读检查指定 --ignore-keyring，用于直接检查 MOK 登记，避免用户态访问 builtin keyring 失败；不更改内核信任、Secure Boot 或任何验证开关，加载时仍由内核校验签名。前置证书 SHA256、模块 SHA256/签名者/内核/PCI 检查保留。

实测：从当前 MOK 导出的项目公共证书通过修复后的检查（0）；不存在证书被拒绝（1，mokutil 内部 255）；bash -n 与默认 dry-run 为 0。证据 docs/mok_test_key_diagnostic.txt、docs/mok_check_fix_validation.txt。未读取任何私钥。

本轮没有加载驱动或修改网络。连接仍待本机 sudo 执行修复后的同一命令：sudo bash /home/lab307/ranger_nav/scripts/connect_mid360.sh --apply。无需重新登记或重启。先前 sudo 认证限制仍需用户在本机终端处理，不能使用聊天密码。脚本会在任何修改前记录原值并提供原有回滚命令。

本轮文件修改：连接脚本、连接报告、STATUS.md；新增官方源码证据与诊断/验证记录。结果：脚本修复 PASS；雷达连接待验，尚不标记 PASS。


## 2026-09-14 21:10 最终连接结果：IP 通信 PASS

已完成板载网卡驱动加载、独立雷达地址配置与实际 ping 验证。此前 BLOCKED 条目是历史记录，以本节为当前结果。

### 实测事实

- 驱动 yt6801 1.0.34 已绑定 PCI 0000:02:00.0；系统将临时 eth0 重命名为 enp2s0。
- 板载网卡 enp2s0，192.168.1.50/24，UP，carrier=1；当前协商速率 100 Mbit/s、full duplex。
- 用户所述直连 Mid-360 的候选默认地址 192.168.1.145 已响应：ping -n -I enp2s0 -c 5 -W 1 192.168.1.145，退出 0，5 发 5 收，0% 丢包，平均 RTT 1.829 ms。
- ip route get 192.168.1.145 确认直接经 enp2s0，源地址 192.168.1.50；邻居 MAC ec:72:f7:da:cf:c8、REACHABLE。
- NM 配置 ranger-mid360-8745，UUID 7d3735fd-229f-4461-bebd-421146eb06d2，已激活；ipv4.never-default=yes、无 gateway、IPv6 disabled、autoconnect=no。
- Wi-Fi wlo1 仍为 192.168.3.18/24，默认 via 192.168.3.1 metric 600。激活前后默认 IPv4 路由逐字比较相同，退出 0。
- can0 保持 DOWN，没有启动底盘节点或发送 CAN 控制。

### 本次实际修复与操作

1. 修复 udev 改名竞态：等待 udev settle，并核对最终 ID_NET_NAME、内核接口与 NM 可见性；允许版本/srcversion 匹配的已加载模块继续。离线延迟改名回归通过，见 interface_rename_fix_validation.txt。
2. 修复 NM 创建连接兼容性：不能在本机 nmcli connection add 中修改 connection.uuid，改由 NM 生成 UUID；连接详情读取字段改为 connection.uuid（列表摘要才使用 UUID）。
3. 中间尝试 connect-pbTzX75l 因指定 UUID 失败，未创建目标连接；connect-jQpZBTzH 已创建目标连接，但旧 UUID 详情字段使读取阶段退出 2，尚未激活。上述失败均未冒充成功。
4. 读取并确认现有连接的 UUID、接口、IP 和 never-default 后，沿用已创建配置恢复：修复该次回滚脚本字段，将原回滚文件留作 rollback-before-field-fix.sh，写入正确 profile-uuid.txt，保存激活前地址及默认路由，然后执行 nmcli --wait 20 connection up uuid 7d3735fd-229f-4461-bebd-421146eb06d2，退出 0。
5. 用户明确授权后，sudo 通过正常终端交互认证；认证信息未写入脚本、命令行参数或项目报告/日志。

### 证据与文件

- docs/mid360_ip_connected_20260914.txt：成功 ping、路由、邻居、IP、NM 和物理链路速率的逐命令退出结果。
- docs/yt6801_interface_rename.txt：正式名称/驱动/NM 的故障诊断证据。
- docs/interface_rename_fix_validation.txt：延迟改名模拟与 shell 语法/dry-run 结果；最终脚本 bash -n 再验为 0。
- /var/lib/ranger-nav-yt6801/connect-jQpZBTzH/activation.log：激活成功日志；activation-default-before.txt / activation-default-after.txt 与 activation-address-before.json 为修改前后证据。
- 本次修改 scripts/connect_mid360.sh、docs/LIDAR_CONNECTION.md、docs/STATUS.md，新增上述证据；新建并激活指定 NM 连接。官方驱动源码未改，未另建 ROS 包。

### 回滚

```bash
sudo bash /var/lib/ranger-nav-yt6801/connect-jQpZBTzH/rollback.sh
```

回滚按精确 UUID 删除本次雷达连接，并恢复该次操作前的有线连接（若记录存在）；保留进入该次操作前已加载的 yt6801。Wi-Fi 配置不变。若需恢复到驱动加载前状态，先完成连接回滚、确认 enp2s0 不再被使用，再 sudo rmmod yt6801；不删除 MOK 密钥或其他项目文件。

### 待验项与停止点

- PASS 仅限板载链路和 IP 通信；设备实际完整 SN 尚未通过 Livox 协议读取，UDP 点云/IMU、ROS 话题、时间同步和录包尚未测试。
- 目前是临时加载模块，未安装到 /lib/modules；NM 配置虽保存但 autoconnect=no。重启后不保证自动恢复，持久化需后续指定工作。
- 未启动 Livox/底盘 ROS 驱动，未进入建图/定位/导航。下一步无需额外人工连接操作，等待用户指定后续阶段。
