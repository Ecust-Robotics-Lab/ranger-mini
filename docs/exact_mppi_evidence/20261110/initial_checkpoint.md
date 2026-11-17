# EXACT-MPPI 实施中检查点（2026-11-10）

本轮用户授权完整计划的软件实施/构建/离线验证，后续追加允许安装适合 RTX 5060 的驱动与 CUDA。未授权底盘/传感器启动、运动、重启、提交或推送。**尚未完成导航替换，禁止把这些初始核心文件当已交付导航。**

## 已保存与完成

- 用户计划已保存 `docs/EXACT_MPPI_NAVIGATION_PLAN.md`。
- Git HEAD `e8245fa80310a4e91b5d2eb303f33883bb965216`，分支 `refactor/pluggable-navigation`；原有文档修改已保留并备份 `.runtime/backups/exact_mppi_20261110/`。该目录还保存已有底盘源码修改和原始 cpp/hpp；不得覆盖。
- 初始未见设备进程；boot `17cd3a99-3b30-4307-adb9-4d94740c9e55` 仅此会话快照。
- 上游只读检出 `vendor/exact_mppi_review` 的固定提交 `bdc5456938a54267408f2b15870b00ff4e91c58f`；`EXACT_MPPI_core/exact_mppi/mppi_jax` 已原样复制进导航包，附 GPLv3 许可；来源锁、打包和许可证清单仍待补齐。
- 新增 `geometry.py`、`models.py`、`planner.py`、`obstacles.py`、`exact_controller.py`。真实矩形、非零外参加载、SE(2)格点与SAT/扫掠、障碍记忆和模式内JAX预测已初步实现。尚需正式测试/审查，特别是搜索父节点保存、覆盖/清除边界和控制性能。
- 128样本/60步/512点 CPU无硬件算例退出0：三模式冷JIT约0.55/0.39/0.42s，5次预热调用约9–19ms。仅开发冒烟，没有真实性能验收或闭环验收。
- 隔离Python环境 `.runtime/exact_mppi_venv` 已用本机Python3.10创建；系统缺ensurepip，使用PyPA get-pip在该环境引导。CPU JAX0.6.2、numpy2.2.6等已安装，未修改系统Python。CUDA依赖正在后台pip下载，见后文。

## GPU安装当前检查点

硬件 PCI `10de:2d19`，RTX5060 Laptop，Ubuntu22.04.5，内核6.8.0-40，SecureBoot enabled。原无nvidia驱动/libcuda，nouveau模块存在。

用户在本机终端执行 `bash scripts/install_exact_mppi_gpu.sh --execute`，安装Ubuntu源 `nvidia-headless-580-open=580.178.04-0ubuntu0.22.04.1` 与 `nvidia-utils-580`，新增12包、升级0、卸载0。备份/安装日志/回退脚本 `.runtime/backups/exact_mppi_gpu_2dg9OIbf/`。`dkms status` 已显示当前内核installed，`modinfo`有本机签名。`dpkg --audit`无输出。当前 `nvidia-smi`无法通信，因为模块尚未成功加载；**未重启，不代表GPU已可用**。

MOK公钥 `/var/lib/shim-signed/mok/MOK.der`，SHA256 `B4:43:EB:0F:22:D9:43:26:54:26:EB:D7:19:B1:75:A9:C0:3A:9D:D5:1A:F6:04:43:7F:A3:FF:4C:5A:F5:E7:77`。私钥不读取/不复制。用户已执行sudo导入，提示already in enrollment request；EFI `MokNew`和`MokAuth`存在，权限0600，普通`mokutil --list-new`静默空白，需 `sudo mokutil --list-new`。不要据非root空白判无请求，不要撤销/重复造密钥。用户须本机重启进入Enroll MOK→Continue→Yes，使用其自己设置的临时密码；Agent不索取密码、不自动重启/禁用SecureBoot。

580明确支持5060且满足CUDA；595-open为Ubuntu当前推荐分支的开放变体，仓库595.99.02同样可候选。已向用户纠正“580最合适”的过早判断；没有本机实测证明595更稳/更快，不在APT中途混装。不要把数据中心分支LTS时长冒充GeForce整机稳定性保证。

CUDA/JAX：exec会话36176运行 `.runtime/exact_mppi_venv/bin/python3 -m pip install 'jax[cuda12]==0.6.2'`，日志 `docs/exact_mppi_evidence/20261110/cuda_dependencies.log`。可能仍在下载大型cuDNN包，新上下文先按PID/命令核对，不重复安装竞争。CPU环境已有，GPU/JIT实测须完成驱动加载后进行。

## 必须继续的工作

P0/P1文档/来源锁/依赖复现与基准补齐；P2/P3/P5/P6现有初始实现审查测试；P4显式V2模式接口及真实新鲜反馈/零速边界；P7任务/action、独立worker/超时/仲裁/安全/末端防撞；P8正式入口替换/旧运行算法删除/软件和闭环仿真验收。当前原启动入口和旧配置尚未改，原生产runtime仍Lightning。所有实车P9未授权、PARTIAL。

继续按用户原计划自主完成软件；用户的GPU问答是任务补充，不取消导航目标。不得在只有初始核心文件时声称替换已完成。

## 重启前最终追加

用户用sudo执行list-new，已明确显示本机签名证书的待登记请求（SHA1 88:49:ee:7b:59:c9:4b:e5:92:77:a1:29:38:e1:0d:99:17:c4:dd:4f）。Agent为方便用户手动重启，按完整命令核对本轮pip PID后发SIGINT暂停未完成CUDA下载，保留环境/缓存/日志；没有停止设备或用户进程，没有自行重启。重启后先核对boot/MOK/nvidia-smi，再恢复项目内CUDA依赖安装并继续导航软件。GPU驱动580已安装/签名但尚未加载验收；CUDA extras安装未完成，不得报告GPU JAX通过。

## 首次手动重启后补记

用户手动重启后未看到蓝色MOK界面。新boot_id `baa879da-c1e1-4e8c-b8cc-c712a7782954`，启动时间2026-11-10 15:59:55。BootCurrent0004路径为EFI/ubuntu/shimx64.efi，未绕过shim；SecureBoot仍enabled。原MokNew/MokAuth请求变量已消失，MOK.der未登记，nvidia-smi仍无法通信。不能据此唯一确定超时/界面/固件原因；下一步先检查MokManager文件并设置较长显示倒计时再安排新请求，不禁用SecureBoot、不改启动顺序。原设备和运动许可不继承，未启动硬件。
