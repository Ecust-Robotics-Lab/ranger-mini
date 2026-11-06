# 新重定位访问包只读核验

日期：2026-11-06。用户自行录制 `bags/fast-livo-relocalization_visit01-trpozzyj`，提前停录后父进程审查被中断。本轮只读原包，将补充结果另存证据，没有补写原完成标志或启动设备。

录包器exit=0、forced=false、stopped_by_signal=true，实际282.0966秒（约4分42秒），请求300秒。日志明确记录缓存写回和Recording stopped。会话STOPPED、live=false、owned=[]，guardian已完成清理。父进程的 `capture_complete=false` 和审查中断错误保留，`CAPTURE_COMPLETE`仍不存在。

原数据库24,520,503,296字节，SHA256 `9720afcc7d2d696c23bb580c3a35fe6786af63f18c632abf5f309530a983c21d`。数据库共73,353条消息、记录跨度282.1110秒，与metadata计数一致：LiDAR、同步图像、原图、CameraInfo、前端位姿、去畸变云各2821条，IMU56427条。前端位姿/云2821对整数同戳，无重复。订阅预检计数包含录包就绪前窗口，不能将其与数据库计数相减作为丢包量。

既有audit及quality_report检查PASS，无failures/warnings。七话题均无采样戳回退/重复或超过0.5秒的采样间隔；56,410,752个原始点的时间均合法，IMU无非有限值。整个相机会话2843帧均OK，设备帧号无缺失/回退；该数量包含录包窗口之外的帧。75份归档来源SHA匹配，前端配置为+100ms、完整三融合、初始重力对齐开启。

补验程序以SQLite mode=ro读取，原DB大小/mtime保持，另核对122份原小文件内容SHA/大小/mtime不变。证据在 `docs/fast_livo2_evidence/relocalization_visit01_capture_20261106/completed/`。实际命令：

```bash
source /opt/ros/humble/setup.bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -u docs/fast_livo2_evidence/relocalization_visit01_capture_20261106/audit_capture.py docs/fast_livo2_evidence/relocalization_visit01_capture_20261106/completed
```

退出0，约16.10秒。首次辅助审查已完成数据/来源核验，但生成摘要时误从imu组读取gravity_align_en而KeyError退出1；修正为实际uav组后另存completed完整重验。原失败日志及首次证据保留，此脚本错误不属于传感器数据失败。

PASS：停录落盘、消息/同戳配对、基本数据、来源与原文件保护。PARTIAL：原入口审查收尾未完成；物理同步、地图覆盖、独立访问定位和绝对精度未验。未实施：改写原包/结果、真实设备操作、算法重放或Git提交推送。

无需仅因少录约18秒重录。该包作为后续独立采集查询；当前专用录包的旧前端版本与新地图I/O派生版本不同，后续重放原始三路输入并绑定实际匹配前端身份，不能直接将旧录制位姿冒充匹配版本。覆盖与位置正确性须分别核验；参与调参后不再作为盲测。
