# FAST-LIVO2 C1：离线先验地图定位交付

日期：2026-10-28。用户在接手核对与范围讨论后明确授权实施 C1-0 至 C1-5，随后要求继续。本轮完成源码、构建、合成/故障、已有数据功能检查与隔离桥接；未启动传感器、底盘、CAN 或导航，未实施 C2、前端恢复、BA 或高程。Lightning/wheel/terrain disabled 生产默认保持。

## 结论

**PASS：C1 离线实现及本轮功能范围。** 已有独立 `ranger_fast_livo_localizer`，可在新进程加载 B 不可变归档，接受人工给定初值，经六自由度几何确认后持续定位；包含只读子图、资源边界、状态/失效、重初始化、代际隔离、独立消息/适配和受管清理。C++ 18/18、ROS 9/9 及相关 12 条验收命令退出 0。

**PASS：同份采集包的地图定位功能检查。** PCL/GICP 各处理完整 1362 对原采样输入；第 366 对之前等待人工初值，随后两对用于运动/几何确认。PCL 确认 994 个有效采样；GICP 确认 988 个，另有 6 个不收敛采样被拒绝为 DEGRADED，保留最后已接受采样时间，后续几何确认恢复。两者均无 LOST/ERROR，退出 0，未发布公共 TF/速度。

**PARTIAL：独立会话泛化、物理同步、实车定位与精度。** 地图和查询来自同一采集，初值从同一 B 图中取得。排除同戳地图关键帧只是直接自匹配防护，不形成独立测试集。无独立真值，不报告 APE/RPE、绝对精度或算法排名；逐帧确认调度不是实时吞吐验收。实测 T_base_imu、实时 wheel 历元、重复结构误接受、多环境和长期资源仍待补证。

## 实现与复用

- `ws_fast_livo/src/ranger_fast_livo_localizer/`：归档读取、三维空间索引/有界子图、粗细配准、初始化与持续跟踪、五态/生命周期、消息/服务、archive_tool、C++ 测试、MIT 与依赖声明。
- B 的 CMake 补齐安装/导出 target，复用 `ranger_backend_core` 的输入检查、整数配对、数学、配准、SHA/分片缓存；B 算法源码未改。没有重新实现 B 或让 Lightning 导入 FAST 专用消息才能运行。
- `config/fast_livo2/localizer.yaml`：独立 C1 试验配置；`scripts/build_fast_livo2_localizer.sh` 构建 B/C1 及可选 ws_nav 适配入口，不覆盖前端安装。
- `scripts/fast_livo2/localizer/`：显式来源兼容说明、监视器、受管会话、验收与同包检查；`scripts/run_fast_livo2_localizer.sh` 默认 dry-run，显式 `--offline-bridge` 才增加标准适配器/公共桥接。
- `fast_livo_localizer_adapter.py`：保留原戳与冻结地图身份，将 IMU 位姿按非零外参转为 base_link，检查代际/已退休会话、采样与心跳；公共契约及桥接的门槛、wheel 插值、禁止外推和创新锁定保持。

加载器直接消费 B schema 1，不改旧档。它校验 COMMITTED、manifest 和全部文件 SHA、必需文件、规范路径/符号链接、唯一 ID、整数时间、位姿/字段、有限点和解析/文件/内存上限。完整来源身份缺失时只接受显式绑定源 manifest 和证据 SHA 的 `offline_experiment` 说明，不能标成物理同步或精度通过。两份兼容说明保存在本轮证据目录。

每个目标分片只用自己的 `X_map_imu` 变换一次，不使用累计世界云再变换。初始化在局部 13 个六自由度种子间检查歧义，并要求多帧时间/运动基线；静止重复不凑确认。跟踪仅用前端相对运动作配准预测，不把预测发布成新地图确认。拒绝不更新已接受缓存；LOST 要求新的人工初值，未暗中增加 C2。

完整字段、公式、默认上限、服务语义和操作见 [C1 契约](fast_livo2_localizer_interfaces.md)。初值服务受理、地图读取完成和定位成功分别可观察；旧地图/会话/请求结果不能提交。

## 版本、构建和许可证

起始及结束主仓库分支为 `refactor/pluggable-navigation`，HEAD 为 `46c2efad12c6cd065739a747f6edfa6a06ee52c1`。本轮未 commit/push。开工时工作树不干净，完整差异和 3897 份既有文件 SHA 在 `.runtime/backups/stage_c1_20261028_161707/`。

未增加或升级第三方依赖。实际 GTSAM 为 `4f66a491ffc83cf092d0d818b11dc35135521612`，small_gicp 为 `fa0cfc983417c044b6036f71dfa448038bb9c820`，沿用项目内 `backend_deps`。前端仍为 humble `837b7bbc1431cb04cf936528e52c83c835efba8e` 加已有补丁。开工核对的全部 15 个仓库均符合 lock；前端补丁核验通过。

本轮代码独立 MIT；既有 GTSAM、small_gicp、SimplePGO 声明和许可文本继续由 B 包保留，C1 有单独 THIRD_PARTY_NOTICES。没有引入 Scan Context/STD/iBTC/KISS/HoloAgent/OpenFleX 源码或 CUDA。前端组合的分发许可问题并未由本轮解决。

构建及本机 ldd 核验确认 localizer/core、GTSAM/small_gicp 来自项目前缀，无缺库，PCL 使用现有系统 1.12。见 `build_audit_final.json` 和 ldd 日志。没有修改全局动态库路径、系统、驱动或网络。

## 实际命令与验收

```bash
cd /home/lab307/ranger_nav
bash scripts/build_fast_livo2_localizer.sh --execute
source /opt/ros/humble/setup.bash
source ws_nav/install/setup.bash
source ws_fast_livo/backend_install/setup.bash
export LD_LIBRARY_PATH="$PWD/ws_fast_livo/backend_deps/lib:${LD_LIBRARY_PATH:-}"
export PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
python3 scripts/fast_livo2/localizer/run_checks.py --execute
```

最终整组命令记录在 [acceptance-hotzb1e0/results.json](fast_livo2_evidence/stage_c1_20261028/acceptance-hotzb1e0/results.json)，12 条全部退出 0：

| 验证 | 结果 | 实际覆盖 |
| --- | --- | --- |
| C1 C++ | 18/18 | B 导出/读取、逐点变换、坏归档/哈希/版本/字段/越界/非有限/超限、显式兼容、索引、六自由度偏差网格、错误初值、历史插值、拒绝缓存、失效、退化、多解、取消/预算、400 次连续配准 |
| C1 ROS | 9/9 | 非零 base↔IMU 外参、私有服务/五态、wheel 同刻/无外推、大跳变锁定、错误初值/重复请求、切图失败/切图、旧请求与旧任务、时钟/重启/多发布者、NaN/1 ns 缺对、已退休 session、受管子进程故障清理 |
| B C++ | 21/21 | 复用/导出后的 B 核心回归 |
| 既有桥接/契约 | 全部通过 | 拒绝缓存 2、旧桥接 14、时间 7、B 适配 3、架构契约 11、配置整合 10 |
| dry-run | 3/3 | 构建、私有入口、显式离线桥接入口；不启动设备 |

实际新进程加载命令：

```bash
ws_fast_livo/backend_install/ranger_fast_livo_localizer/lib/ranger_fast_livo_localizer/archive_tool \
  --archive "$PWD/outputs/fast-livo-backend-validation-20261028-pcl/snapshot" \
  --compatibility "$PWD/docs/fast_livo2_evidence/stage_c1_20261028/pcl_compatibility.json" \
  --index-output "$PWD/outputs/fast-livo-c1-index-pcl-20261028"
```

PCL/B 归档已有 49 帧、图版本 62、151 项文件 SHA，完整原进程已退出；C1 在新进程读取并生成独立派生索引。GICP 归档同样检查。原目录、manifest、扫描和地图均未改写。索引写入使用新目录、参数/源身份、fsync 和 no-replace 提交，失败撤销成功标志并保留失败目录；没有断电或磁盘填满的实际验收。

运动数据命令（均已实际执行，省略 --execute 只显示计划；输出已存在，复现必须换新目录）：

```bash
python3 scripts/fast_livo2/localizer/validate_motion.py --execute \
  --archive outputs/fast-livo-backend-validation-20261028-pcl/snapshot \
  --compatibility docs/fast_livo2_evidence/stage_c1_20261028/pcl_compatibility.json \
  --native outputs/fast-livo-demo-20261028-fHJTbGCU/frontend/native_frames \
  --output outputs/fast-livo-c1-final-20261028-pcl --domain 197 --registration pcl_icp
python3 scripts/fast_livo2/localizer/validate_motion.py --execute \
  --archive outputs/fast-livo-backend-validation-20261028-gicp/snapshot \
  --compatibility docs/fast_livo2_evidence/stage_c1_20261028/gicp_compatibility.json \
  --native outputs/fast-livo-demo-20261028-fHJTbGCU/frontend/native_frames \
  --output outputs/fast-livo-c1-final-20261028-gicp --domain 198 --registration small_gicp
```

| 本包功能观察 | PCL ICP | small_gicp |
| --- | --- | --- |
| 原采样输入/确认处理 | 1362/1362 | 1362/1362 |
| 等待初值/确认帧 | 366/2 | 366/2 |
| 有效地图定位 | 994 | 988 |
| DEGRADED/LOST/ERROR | 0/0/0 | 6/0/0，不收敛拒绝 |
| 节点峰值 RSS | 84500 KiB | 86144 KiB |
| 全检查墙钟 | 189.0 s | 173.7 s |
| 包括首个初始化确认的接受配准耗时 p50/p95/max | 95.6/116.4/1673.1 ms | 84.9/86.7/1222.6 ms |
| 公共 TF/静态 TF/速度发布者 | 均 0 | 均 0 |
| 进程退出码 | 0 | 0 |

节点 RSS 来自 `/proc/PID/status` 的 VmHWM，不是整个重放进程树总内存。PCL 部分运行与回归并行；两种求解器使用各自 B 优化地图，不据此作性能排名。没有宣称确认节流运行可原速接车。

源 native DB SHA 为 `2eecd2c75e0c33eac80cda41f043f4c8ebe6bd5c089da06e6018b40deeaf62b4`。最后只读复核全部样本的单调原戳、VALID 对应确认戳、拒绝保留最后确认戳、无 LOST/ERROR、两份原图各 151 项 SHA；见 `motion_postcheck.json`。原始三传感器包 `bags/fast-livo-motion-w5phdosj/raw` 保留，没有重录或重打 now。

## 调试失败与修正

失败证据保留，不混入最终通过结论：

1. 初次构建缺尚未写入的测试文件，随后跨包链接 target 名称不正确；补齐文件和带 namespace 的导出 target 后构建通过。
2. 首轮 ROS 夹具有未来 wheel、共享时间戳对象和时钟回退时移动到地图外的问题；修正夹具后，1 ns 缺对和禁止外推才真正被触发。受管测试还修正了首次无会话状态的 None 处理。
3. 最初在运动前静止段提交初值，13 假设求解和缺运动支持触发 10 s 墙钟超时；保留输出。最终从第 366 对已经开始移动时给初值，未增加总超时或降低运动确认要求。
4. 初始确认健康计时从求解开始计，慢初始化完成后立即过期；改为几何确认完成计，位姿仍保留原采样戳，ROS 新鲜度单独检查。
5. PCL float 矩阵的微小旋转应变在连续预测中积累，先前检查在约 315 个有效采样后拒绝并失效。现在先检查原始求解结果刚体合法性，再投影微小数值误差到 SO(3)，对实际接受的变换重新做独立几何门控。新增 400 次连续配准回归，未放宽刚体/质量/公共门槛。此前部分输出的 passed 仅反映旧脚本的较弱功能条件，已被最终无 LOST/ERROR 检查取代，不能引用作完整跟踪通过证据。
6. 最终整组验收及完整运动检查之后，只补齐派生索引失败路径的成功标志撤销；另行构建、C++ 与新进程索引检查保留于 `build_index_final*`、`cpp_index_final*`、`archive_gicp.json`，定位/跟踪算法未再改动。

## 证据、交接与最少待办

小型结果入口为 [verification_summary.json](fast_livo2_evidence/stage_c1_20261028/verification_summary.json)。同目录有构建/失败日志、两份 compatibility、`pcl_result.json`、`gicp_result.json`、`motion_postcheck.json`、完整验收命令、链接审查、工作树/来源审查和最终会话状态。逐输入 quality 与大输出在 `outputs/fast-livo-c1-final-20261028-{pcl,gicp}/`；先前失败/部分运行均保留。

已更新 STATUS、HANDOFF、总计划、有效约束、根 README 和接口/文档索引。用户原雷达文档修改、六份 driver 证据、相机/标定/采集等既有成果不混入提交，不 reset/clean，不删除原包/地图/失败证据/备份。

后续最少人工事项：另行安排独立建图/定位及相似地点负样本采集，补物理同步、实际 T_base_imu 和实时 wheel 时钟证据；无硬件授权时继续保持离线。算法下一阶段为总计划第 5 节 C2，须新实施指令；本轮到 C1 停止。无提交/推送指令，全部本轮成果保留在工作树。
