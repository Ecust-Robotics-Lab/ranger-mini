# 前端定位相机启动排查

诊断与状态提示软件验收 PASS，外部同步与触发链路现场复核 PARTIAL。没有操作设备或重启用户会话。源码基点为 `bb418321ea139503242bcc5d975906896c808714`，保留既有未提交修改和原始记录。

最新失败会话 `.runtime/sessions/fast_livo_online_localization/run-0fxc91ct` 能通过 USB3 打开目标海康相机并配置 Line0、20ms 曝光。SDK 连续报 `MV_E_NODATA`，一共 13 次进入抓图状态，12 次因连续超时重连，没有任何首帧。健康记录中雷达 956 帧、IMU 19392 条、同步图像 0 帧，writer 就绪，三融合前端等待相机。另一会话 `run-pe_ndti8` 有同样现象。旧驱动的 `Camera stream recovered successfully` 表示重开抓图成功，是否已收到图像要看首帧日志和健康计数。

近期 15 个可查会话中，5 个零图像会话的雷达和 IMU 启动包均为 `time_type=0`，10 个有图像会话均为 `time_type=2`。其中成功的 `run-h6f_ii4n` 一次打开即出图，记录 4656 份同步图像。按 [Livox Mid-360 系列协议](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html)，这两种类型表示无同步源和 GPS 同步时间。这是排查 PPS 时间输入和 Line0 触发的关联证据。是否共用同一信号源、脉冲实际到达情况、线路和供电状态需要实测确认。writer 正常只证明软件收到了雷达扫描时间，不能确认物理触发。

新增 `scripts/fast_livo2/camera_startup.py`，使用本会话健康文件和相机、雷达日志，在相机启动仍未就绪且雷达和 writer 已就绪时区分等待首帧、SDK 无帧超时、已有图像但同步输出不新鲜。工作台在等待和终态超时中显示这些原因，按 boot/PID/start_ticks 校验归属。独立定位命令在状态输出中也报告原因。原有外触发配置、相机二进制和三融合就绪条件照常使用。

运行方式与验收：

```bash
.runtime/navigation_workbench_venv/bin/python -m unittest tests.test_navigation_workbench tests.test_navigation_workbench_reset tests.test_fast_livo_frontend_upgrade -q
bash scripts/localize_fast_livo2.sh --help
git diff --check
```

66 项测试通过，含相机无帧、超时终态及有原始帧后的原因切换，退出码 0。独立定位入口帮助与 diff 检查退出码 0。对上述两份真实失败日志和一份成功日志直接运行诊断，分别给出无帧原因和成功时不告警。会话、相机二进制及脚本身份见 [analysis.json](workbench_evidence/20261118/camera_startup/analysis.json)，测试见 [tests.log](workbench_evidence/20261118/camera_startup/tests.log)。改动为 Python 源码，下一次启动工作台进程加载。现有工作台仍运行，定位和底盘已退出。

现场待办是核对外部同步源和触发信号供电，确认雷达 PPS/时间信号及相机 Line0 是否收到脉冲，再用首帧和同步图像连续增长确认恢复。无需移动底盘来做这项检查。
