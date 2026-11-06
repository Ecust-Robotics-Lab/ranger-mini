# 走廊/教室完整前端PCD与轨迹

日期：2026-11-05。承接[新包只读核验](fast_livo2_corridor_classroom_capture_report.md)。用户要求完整PCD和轨迹展示；只做离线处理，没有设备/运动操作、提交推送或覆盖既有改动。

主交付目录：`outputs/fast-livo-demo-20261105-corridor-classroom/pcd_and_trajectory/`。

- `map_full.pcd`：23,337,461点、373,399,539字节，XYZI，完整3899个有效区间云；`frontend_map_full.pcd`与其逐字节相同。每点仅做一次 `p_local=U_t*p_imu`，不重施LiDAR–IMU外参。
- `map_with_trajectory.pcd`：完整点云加RGB显示标记，红轨迹、绿起点圈、紫终点圈，仅用于展示；标记不作环境几何。
- `trajectory.pcd`、`trajectory_imu.tum/.csv`：3899个IMU位姿，保留传感器时刻；估计累计93.348m、首尾0.096m，非独立真值。
- `map_overview.png`显示完整空间范围；`map_detail.png`显示轨迹外扩5m和约[-0.077,1.673]m高度，方便看墙体。图片降采样/切片不改变完整PCD。

主交付为正式+100ms、Tag关闭、初始重力对齐开启的三融合前端结果，未应用B回环优化。录制时已运行FAST局部里程计，但未保存完整去畸变云/PCD，因此本次只读重放导出，不表述为C1/C2全局定位。

## 实际执行与证据

证据目录：`docs/fast_livo2_evidence/corridor_classroom_mapping_20261105/`。完整argv和SHA见provenance.json、backend_command.json、export_command.json。

| 实际执行 | 结果 | 证据 |
| --- | --- | --- |
| 既有ranger_livo_replay，offline.max_frames=0，domain202/localhost，输入本包归档+100ms配置 | 退出0，52.86秒；85875条所选输入全部消费，3899对严格同戳输出、25575次视觉求解、1次重力对齐 | frontend_exit.json、frontend_validation.json；输出frontend/replay.log和summary.json |
| 复用既有B验证脚本，PCL默认门槛，domain203/localhost | 3899对全部处理；节点退出0，完整验收脚本退出1 | backend_exit.json、backend_diagnostic_summary.json；输出backend_pcl/result.json |
| export_and_plot.py --execute --mode frontend，完整native_frames和frames.csv | 退出0，5.74秒；检查配对、有限性、点数及PCD负载长度，人工查看两张图 | export_exit.json、delivery_summary.json；输出summary.json、export_and_plot.log |

复用原零gyro初始化的 `independent_time_source_20261104`，55文件SHA匹配锁定清单；harness SHA `e84f21c2a98d339fc0f79d0aa56fecf328878ef05543ab51389e2c9e0348876e`。隔离源码Log运行前备份，运行后保存新日志并恢复原日志文件，见log_restore.json。没有改变源码、系统依赖或生产部署。

## B诊断限制

额外B导出快照 `backend_pcl/snapshot`，UUID `2c08461dbe605a9929d1d885b32d4ab7`、graph414、306关键帧/108已接受回环；922份归档文件SHA匹配。但i=3、j=299的一次已接受更新，同因子图目标7.0466911873→7.0477510475，增加约0.00105986（约0.015%），触发既有“Accepted graph objective increased”，因此完整B验收FAIL。没有放宽门槛，也不把诊断快照冒充已验收优化地图。

额外B运行约615.52秒造成等待；用户要求先看前端后，主交付直接提供前端完整PCD。没有为本次展示继续修改后端或重复长运行。

PASS：全包三融合、来源/配对/点数/PCD导出和轨迹展示。FAIL：本包B完整验收的图目标单项。PARTIAL：绝对精度、物理同步、C1/C2独立定位、真实Tag及D。未实施：真实设备操作、原收尾文件恢复、后端修复、Git提交推送。原始包SHA和中断现场保留，见录制核验报告。
