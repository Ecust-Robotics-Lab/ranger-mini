# FAST-LIVO2 阶段 B：真实运动数据后端验证

日期：2026-10-28。用户明确授权完成阶段B后端验证并提交Git。本次不启动传感器/底盘/导航，不进入C1/C2；Lightning生产默认、wheel与公共TF链保持。

## 结论

**PASS：这份运动数据上的独立后端闭环管线。** 实际运行ROS2 C++后端，PCL ICP与small_gicp分别完整处理1362/1362对输入。两者均生成49个关键帧、35个位置候选，接受13个回环、完成13次iSAM2提交；3个候选因地点歧义拒绝，其余19个为非最优候选。没有降低任何默认质量、安全或排除阈值。

**PARTIAL：物理同步、真值精度、实车/长时能力。** 当前证据是已有三传感器运动包的离线实测，不是新实车实验或精度认证。没有外部真值，不报告APE/RPE、误环率或实车绝对定位精度。该包未录制真实wheel，不能验收实时wheel对齐与公共桥接接车。C1/C2未实施。

| 量 | PCL ICP（仍为默认） | small_gicp |
| --- | --- | --- |
| 源帧/确认处理帧 | 1362 / 1362 | 1362 / 1362 |
| 关键帧 / 因子 / 图版本 | 49 / 62 / 62 | 49 / 62 / 62 |
| 接受回环 / 歧义拒绝 | 13 / 3 | 13 / 3 |
| 非关键帧传播数值核查 | 1313项，最大误差1.78e-15 | 1313项，最大误差1.78e-15 |
| 原始首尾距离 | 0.457307 m | 0.457307 m |
| 优化首尾距离 | 0.445081 m | 0.449568 m |
| 最大关键帧平移修正 | 0.053396 m | 0.039419 m |
| 回环子图NN RMSE均值：原始→最终优化 | 0.084790 → 0.083369 m | 0.084790 → 0.083735 m |
| 地图点数 | 296594 | 296594 |
| 逐帧重建地图最大坐标误差 | 1.38e-6 m | 2.87e-6 m |
| 归档文件SHA核验 | 151/151 | 151/151 |
| 归档原扫描/原位姿/整数戳核验 | 49/49 | 49/49 |
| 无效输入、配对异常、残留队列 | 全部0 | 全部0 |
| 后端退出码 | 0 | 0 |

本包原始漂移较小，修正只有厘米量级；首尾距离变化约1.2cm/0.8cm，NN一致性变化也很小。首尾距离并不保证物理起终点完全相同，NN指标也不是独立真值，不能据此排名两种算法的绝对精度。

## 输入与时间审查

保留原包`bags/fast-livo-motion-w5phdosj/raw`，约136.6秒，未改写。原包含1367条PointCloud2、27343条IMU、1367条同步图像、1367条前端位姿；三类输入无header回退，雷达/图像间隔约99.84～100.40ms，IMU约3.96～6.06ms。27338880个逐点时间没有NaN/Inf、扫描内回退或早于扫描起点。

相机同步话题复制的header与1367帧雷达精确相等，仅证明写戳关系。原生图像/CameraInfo使用主机历元，雷达/IMU/同步图像/前端仍是2020历元；不通过重打当前时间处理。物理曝光与扫描相位仍未验收。

本次复用制作演示时已完成的三融合重放输出：`outputs/fast-livo-demo-20261028-fHJTbGCU/frontend/native_frames`，1362对VIO后同刻IMU位姿与单区间去畸变IMU云，10911次视觉求解更新。与原包1367帧的差别来自新前端初始化/区间边界，不是后端丢帧。源数据库SHA256保存于各运行metadata。原录制末尾有发布者变化错误，本次不把它改称完整180秒采集。

调用约定沿用阶段A审查：`T_A_B`将B点变到A。后端里程计因子为`inverse(U_i)*U_j`，历史子图在历史i系，配准返回总`T_i_j`；未重复应用初值或LiDAR–IMU外参。首帧先验保留FAST local原点、轴与yaw，未声称重力已对齐。

## 实际运行与核查方法

新增`validate_motion.py`只读加载同戳对，逐对发布原始消息内容并等待后端确认该采样戳，使用原整数时间发布模拟时钟。PCL/GICP分别隔离在domain192/193、localhost。本次采用确认后发下一帧的变速调度：约206/205秒墙钟耗时及约149ms确认延迟受50ms定时器和配对窗口影响，**不是实时性或配准吞吐排名**。

关键帧、检索与验收均使用`config/fast_livo2/backend.yaml`原默认：0.5m/10°、时间选帧需最低运动、最小间隔0.1s；位置半径3m、排除20个ID和30s、最多5候选；0.15m配准分辨率；对应距离0.5m、至少30对应、重叠0.45、RMSE0.2m、退化比1e-4；歧义间隔1m；大修正1m/30°及独立证据规则保留。两次运行只切换注册器，没有放宽门槛。

开头约30秒静止段保持首关键帧，末尾停留也没有持续增加关键帧。最终因子为1个首帧先验、48个相邻约束、13个回环。13次提交均逐项验证同一因子集合下目标函数不增。PCL接受候选RMSE约0.073～0.138m、重叠约0.857～1.0；GICP约0.073～0.138m、0.855～1.0。没有将fitness/Hessian逆直接当因子协方差。

实测候选求解耗时PCL p50/p95/max约38.3/69.4/77.0ms，GICP约25.0/30.8/39.3ms。这是本包、当前负载、各自优化后子图的有限样本；第二次运行与部分功能回归并行，不能作为严格性能基准或普遍速度倍数结论。

导出后独立读取每个完整关键帧云，按其**自己的**最终优化位姿重新计算全部296594点，与map.pcd逐点比较；同时校验COMMITTED和全部151项SHA、原始/优化轨迹、唯一因子ID、相邻约束方向、子图排除窗口及源扫描不自匹配。49份scan.cdr的所有字段、header、点数据字节、原始位姿与整数戳逐一对回源输入。

非关键帧按同图版本前后输出的原始相对运动检验，最后一帧另对最终快照检查`X_k*inverse(U_k)*U(t)`，未使用未来关键帧或混合图版本。所有1362个输出只属于同一有效会话，scan_id连续；四个公共TF/速度话题发布者均为0。后端没有反馈前端滤波器或视觉地图。

## 构建、测试与复现

工作树开始时HEAD为`a76d83a767d5691ca40351f3a57f65b398572a52`，分支`refactor/pluggable-navigation`。开始状态见证据目录`initial_status.txt`，完整tracked/index diff保存在`.runtime/backups/stage_b_motion_20261028/`。保留已有相机/标定、原雷达IP文档及六份driver证据，没有reset/clean。

```bash
bash scripts/build_fast_livo2_backend.sh --execute
source /opt/ros/humble/setup.bash
source ws_livox/install/setup.bash
source ws_nav/install/setup.bash
source ws_fast_livo/backend_install/setup.bash
export LD_LIBRARY_PATH="$PWD/ws_fast_livo/backend_deps/lib:${LD_LIBRARY_PATH:-}"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1

python3 scripts/fast_livo2/backend/validate_motion.py --execute \
  --native outputs/fast-livo-demo-20261028-fHJTbGCU/frontend/native_frames \
  --output outputs/fast-livo-backend-validation-20261028-pcl \
  --domain 192 --registration pcl_icp
python3 scripts/fast_livo2/backend/validate_motion.py --execute \
  --native outputs/fast-livo-demo-20261028-fHJTbGCU/frontend/native_frames \
  --output outputs/fast-livo-backend-validation-20261028-gicp \
  --domain 193 --registration small_gicp

python3 scripts/fast_livo2/backend/run_checks.py --execute
python3 tests/test_fast_livo_motion_validation.py
python3 scripts/fast_livo2/backend/analyze_motion.py \
  outputs/fast-livo-backend-validation-20261028-pcl \
  --output outputs/fast-livo-backend-validation-20261028-pcl/comparison --video
```

`validate_motion.py`省略`--execute`只显示计划；所有output目录须为新目录，以上已存在输出不可覆盖。GICP图表使用同一analyze命令替换路径、去掉`--video`。后续读者需保留本机原包/前端派生数据，这些二进制资产不入Git。

PASS：项目内GTSAM/small_gicp及后端构建退出0；真实运动PCL与GICP运行/导出/检查均退出0；16项既有验收命令全部退出0，包括C++21/21、ROS6/6、适配器3/3、桥接拒绝2/2、整合10/10、契约11/11、时间7/7、旧桥接14/14、门禁12/12、前端13/13及静态三传感器→前端→后端55/55。

新增验证器故障/数值测试3/3退出0，覆盖非零三轴旋转与平移/逆变换、NaN四元数、截断/非有限PCD、重复及差1ns缺对拒绝。第一次单独调用构建审查时未source工作空间导致ldd缺库，按标准环境source后完整回归审查通过；保留首次失败JSON，不计作通过证据，也未修改全局链接路径。

## 证据与交付

统一小型证据目录：[run-PYV5azso](fast_livo2_evidence/stage_b_motion_20261028/run-PYV5azso/verification_summary.json)。

- [PCL结果](fast_livo2_evidence/stage_b_motion_20261028/run-PYV5azso/pcl/result.json)、[GICP结果](fast_livo2_evidence/stage_b_motion_20261028/run-PYV5azso/gicp/result.json)：实际参数、确认数量、图版本、地图/传播核查和退出码。
- 两者同目录`manifest.json`、`candidates.jsonl`、`original.tum`、`optimized.tum`、`archive_input_checks.json`、`analysis.json`与`loop_geometry.csv`：冻结图、每个候选及几何指标。
- [构建日志](fast_livo2_evidence/stage_b_motion_20261028/run-PYV5azso/build.log)、[16项命令结果](fast_livo2_evidence/stage_b_motion_20261028/run-PYV5azso/regression/results.json)、[新增测试](fast_livo2_evidence/stage_b_motion_20261028/run-PYV5azso/validator_tests.log)。完整既有验收原目录为`stage_b_20261026/acceptance-j58vps9_`，本报告目录保存副本。
- [PCL对比图](fast_livo2_evidence/stage_b_motion_20261028/run-PYV5azso/pcl/comparison.png)、[24秒1080p对比视频](../outputs/fast-livo-backend-validation-20261028-pcl/comparison/comparison_1080p.mp4)。视频480/480帧完整解码，使用相同源点与同一最终图版本分别重建两侧，变化为真实厘米量级，没有夸大修正或称为实车精度提升。

完整PCD/CDR分片、快照、逐帧输出和视频留在`outputs/fast-livo-backend-validation-20261028-{pcl,gicp}/`，不把大文件加入Git。下一步可在新授权下实施C1归档加载与给定初值定位；物理同步/真值/误环泛化/实时wheel/长期实车仍PARTIAL，本次到B停止。

## Git提交范围

本轮先将此前尚未提交的B核心实现、必要前端契约/构建输入、依赖锁、私有消息/服务、适配器/桥接回归及已有小型证据入库，核心提交为`9e7c97e`。随后单独提交本次运动验证脚本、独立故障检查、报告、状态与约1.7MB可定位证据。原包、地图、视频、大型分片、SDK和vendor依赖不入库；既有相机/标定迁移、原雷达IP文档、六份driver证据及其他原有修改保留在工作树，不随本次B提交。仅作本地commit，未执行push。
