# P0 后端离线消融

独立于原实验；不改原 rosbag、图、汇总、正式运行参数或安装。没有设备、ROS发布者或底盘入口。

`bash experiments/fast_livo2_backend_ablation/run.sh` 显示计划，添加 `--execute` 完成构建与离线对照。需要系统 ROS Humble 和项目当前 `online_install`、`backend_deps`。保留每次新结果；失败结果不覆盖。

## 两层实验

1. `run_factor_study.py`：从原包读前端位姿，按原图更新顺序重放真实 GTSAM Graph。固定原回环测量，比较原配置、无回环、历史子图重叠超过50%且source平移不足1m/旋转不足20度的筛选、odom标准差减半、筛选+减半及Cauchy核。原版本逐时刻修正用于核对在线位姿。
2. `run_native_study.py`：每个原始关键帧的点云全部解码字段与原包逐一相同，位姿也与原包核对；CDR重新序列化填充可不同，归档文件自身SHA仍严格检查。调用真实 Backend::process，重新构建子图、检索、配准、拒绝门控与GTSAM优化。比较PCL原配置、small_gicp、small_gicp+odom标准差减半、PCL+Cauchy。保持原关键帧输入与采样戳；非关键帧按因果修正传播。

第二层完整重跑**关键帧后端计算**，没有重跑原始传感器到前端，也没有模拟DDS排队、节点健康和实时心跳。`causal_offline` 是理想顺序处理的采样时刻位姿，不能当作真实在线延迟或在线成功率。

## 判断限制

固定旧因子删减会破坏“原测量依赖当时已优化子图”的条件，不等价于新策略在线重新配准。因子层只能隔离灵敏度；对策应重跑完整配准后评估。

所有参数跨轮一致，不注入P0闭环因子，不强迫首尾重合，不拟合尺度/轨迹。第1/2轮用于开发，第4轮作候选保留检查（其原始结果此前已见，不能称盲测），第3/5轮起点静止门控未过，只作诊断。单P0人工复位、静止厘米级波动和缺独立全程真值限制仍在；不能从闭合数字推出全程精度排名。

大结果在忽略的 `results/`，本轮结论及可版本化摘要在 `docs/fast_livo2_p0_backend_optimization_20261111.md`。

## 待实施：按里程计链距离过滤

计划见 `docs/fast_livo2_loop_path_filter_plan_20261111.md`，固定协议在 `path_distance_protocol.json`。`audit_loop_path_distance.py` 只审计原归档候选的原U关键帧链距离；不执行新过滤。当前 `run.sh` 仍仅运行上面已有两层实验，不会自动运行尚未实现的距离过滤试验。
