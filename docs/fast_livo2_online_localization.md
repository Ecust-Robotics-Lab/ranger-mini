# FAST-LIVO2 在线先验地图定位与重定位

日期：2026-11-06。这是私有在线定位入口，复用现有C1/C2与三融合前端。默认dry-run；软件与离线结果见[交付报告](fast_livo2_online_localization_report.md)，计划见[实施计划](FAST_LIVO2_ONLINE_LOCALIZATION_PLAN.md)。生产仍为Lightning/wheel；本入口不启动B建图、底盘、Nav2或公共TF。

## 构建与入口

```bash
cd /home/lab307/ranger_nav
bash scripts/build_fast_livo2_online_localization.sh --execute
bash scripts/localize_fast_livo2.sh --archive outputs/fast-livo-demo-online-mapping-20261105/full_replay_12/map
```

构建复用已有 `online_install` 后端、项目内GTSAM/small_gicp/KISS前缀，C1/C2安装到 `online_localization_install`，不替换原安装。前端继续使用 `online_frontend_install` 的I/O派生版本。默认auto、自有多尺度v2检索、small_gicp、domain215/localhost、900秒、磁盘保留20GB；正式+100ms、重力对齐开启、Tag关闭。`--registration pcl_icp`显式回到PCL；`--descriptor radial_pair_distribution_v1`选择旧检索基线。切换顺序、同条件比较和限制见[检索/GICP报告](fast_livo2_retrieval_comparison_report.md)。

无设备原速重放：

```bash
bash scripts/localize_fast_livo2.sh --execute \
  --archive outputs/fast-livo-demo-online-mapping-20261105/full_replay_12/map \
  --replay-view /ABS/AUDITED_HEADER_CLOCK_VIEW \
  --output /ABS/EXISTING_PARENT/NEW_RESULT \
  --visualization none --seconds 400
```

视图使用已有 `make_replay_view.py` 从原始三路融合数据生成，不播放原包旧前端输出，不改原消息/原包。实物模式省略`--replay-view`并显式`--execute`；Agent仍需该次设备授权。前端/定位/录包由同一受管会话启动，遇到其他会话资源占用会拒绝。

```bash
bash scripts/localize_fast_livo2.sh --status
bash scripts/localize_fast_livo2.sh --stop
```

Ctrl+C、定时与重放EOF结束本次所属组件并完成录包，不封存新地图。启动输出 `LOCALIZATION_DIR`，当前状态同时包含业务状态与进程存活；搜索、等待运动支持和LOST不等于进程死亡。进程故障/输入断流按既有受管规则退出，不自动重启设备。

## 自动与人工初值

auto默认执行C2多帧全局确认，交给C1独立几何验证并持续跟踪；C1丢失后重新搜索。manual只等待人工初值，LOST后仍需重新给初值。既有私有LoadMap/Initialize/Relocalize服务及消息格式保持。

`--mode manual`选择人工模式。前端输入与地图就绪后，在已加载ROS和本次隔离安装的终端可给IMU参考点的map初值（XYZ米，RPY弧度）：

```bash
source /opt/ros/humble/setup.bash
source ws_fast_livo/online_install/setup.bash
source ws_fast_livo/online_localization_install/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 scripts/fast_livo2/localizer/initialize_online.py \
  --domain 215 --xyz X Y Z --rpy R P Y
```

XYZ/RPY需替换为实际初值。helper从当前结果读取地图/会话身份，以stamp=0绑定下一对输入，不将now写入传感器时间。服务accepted只表示受理，仍须观察VALID。auto模式中人工初值会暂停搜索；恢复自动搜索沿用原Relocalize服务，manual模式未开启C2不能通过该服务临时启用。

## 调度、显示与记录

新参数 `online_scheduling`默认false，旧离线行为保持。新入口显式启用：只保留最新两对待处理完整扫描，保留正在处理任务及原控制代际；跳过扫描累计可观察。旧扫描的几何结果不直接对外发布有效位姿，等待新扫描验证；0.5秒采样新鲜度与原几何/多帧/歧义门槛不放宽。求解器仍为软预算，库内不能硬中断。

`quality_json`新增online_scheduling、skipped_scans、processed_scans、stale_results、sample_age_seconds。终端显示状态/reason、处理/跳帧及位姿年龄。`--visualization rviz`使用map固定系，显示只读静态地图与已确认轨迹；无DISPLAY时默认none，不影响定位。

结果目录含冻结配置、查询 `source_identity.json`、地图加载日志、实际程序/库及源码SHA来源、原始/前端/定位话题录包、逐心跳轻量 `localization_samples.jsonl`、会话日志、结果摘要。workflow_success只代表会话/录包正常结束，不代表初始化成功或位置正确。统计可执行：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 scripts/fast_livo2/localizer/analyze_online.py \
  /ABS/RESULT /ABS/NEW_SUMMARY.json
```

来源身份从当前查询前端实际配置/程序计算，并用共用读取器核对地图；不会复制地图身份冒充查询配置。实物私有时钟仍为雷达header/steady锚点，不等于wheel/主机同步。定位不回灌前端，公共桥接、真实wheel和导航接入继续单独验收。
