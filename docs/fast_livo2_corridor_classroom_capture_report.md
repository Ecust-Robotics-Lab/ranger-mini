# 走廊/教室新包录制与中断后只读核验

日期：2026-11-05。输入为用户自行录制的 `bags/fast-livo-corridor_classroom_map01-182fdzyj/`。Agent只读核对数据、日志和会话，并将审查结果另存证据；没有启停设备、重写原包或补造原目录的完成标志。

## 录制事实

录包器已进入RECORDING，随后由信号正常停录：`recorder_result.json` 中exit=0、forced=false、stopped_by_signal=true，实际390.383635411秒，约6分30秒，未达到请求的900秒。原 `recording.json` 为RECORDED，rosbag日志有缓存写回和Recording stopped，metadata已生成。

原始SQLite数据库32,824,840,192字节，SHA256为 `b6f7902c1b5fbffbaea69bf241efff540d48c0a85358d1267a4f98d468496137`。实际数据库计数与metadata相符：

| 话题 | 消息数 |
| --- | ---: |
| /livox/lidar | 3903 |
| /livox/imu | 78068 |
| /front/image_sync | 3904 |
| /front/image | 3904 |
| /front/camera_info | 3904 |
| /fast_livo/odometry | 3904 |

总计97,587条，存储时间跨度390.381313615秒。预检订阅器的observed_messages还包括录包订阅就绪前的等待窗口，不能直接拿它与数据库计数相减当作丢包量。各话题首尾订阅时刻不同，消息总数不要求完全相等。

本次68份来源归档文件SHA核对通过，归档前端配置确为img_time_offset=+0.1秒，require_livo=true、图像启用、初始重力对齐开启。没有修改Tag时间有效性或启用真实Tag候选。

## 异常位置与审查结果

用户粘贴的KeyboardInterrupt发生在父进程调用 `audit(folder/'raw')` 之后，属于停录后的数据审查被中断。原目录因此没有audit.json、result.json、CAPTURE_COMPLETE；这属于原收尾流程未完成，不能仅凭此推断原始数据丢失，也没有把原目录手工改成成功。

Agent在独立Python进程调用既有audit、quality_report和sync_summary，以SQLite mode=ro读取原包，审查及SHA核对退出0，证据另存 `docs/fast_livo2_evidence/corridor_classroom_capture_20261105/`：

- `audit.json`：六路header均无回退/重复，未发现超过0.5秒的采样间隔；LiDAR/同步图像/前端位姿周期约100ms，IMU约5ms。
- 78,049,056个原始点的时间均合法，无非有限值、早于header或扫描内部回退；IMU无非有限消息。
- `quality.json`：data_checks=PASS，failures/warnings为空。整个相机会话3926帧状态均OK，设备帧号无缺失/回退；该计数包含录制时间之外的启动窗口，不是数据库图像帧数。
- `integrity.json`：原始DB大小/mtime在检查前后保持，记录完整SHA；审查约9.67秒，连同SHA约33.93秒。

受管录包会话 `run-whjr7l4e` 为STOPPED、live=false、owned=[]、guardian_cleanup_complete=true。旧field状态文件仍写DEGRADED，但其历史进程身份已不存活；历史owned列表不能作为当前进程仍在运行的证明。没有据此自动重启任何会话。

PASS：原包正常停录落盘、来源/消息计数/时间与数据基本审查、所属录包进程清理及原文件保护。PARTIAL：原收尾审查被中断，原完成文件缺失；物理同步、地图质量、定位精度和真实Tag等未由录包审查验收。未实施：恢复或改写原收尾文件、修复审查中断行为、重新录制、硬件操作、Git提交推送。

该包可以继续作为离线三融合/建图输入；用户随后已授权完整PCD与轨迹展示，处理记录见后续建图报告。无需仅因这次审查中断立即重录同类数据。
