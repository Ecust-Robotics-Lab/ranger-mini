# 办公室—走廊往返实车准备

现场任务是从办公室出发，沿走廊到端点掉头，返回办公室。分段结果和当前待验收项见 [接手指南](HANDOFF.md) 与 [当前状态](STATUS.md)。设备、控制、导航目标和非零速度需要当次授权。

## 地图与路线

正式地图路径及 UUID 见 [接手指南](HANDOFF.md)。可用 [路线准备报告](exact_mppi_office_corridor_preparation_20261116.md) 的静态候选复核点位。路线位置、地图版本及起始朝向应在现场重新核对。静态验证只用于准备，整体以实际执行记录验收。

可通过 [工作台](../workbench/README.md) 预览单点路径，也可将候选路线导出为 JSON 后离线复核。输出使用新目录，以便比较不同次结果。

```bash
python3 scripts/prepare_exact_field_test.py \
  --navigation-map outputs/fast-livo-demo-changed-scene-20261110/navigation_map_05 \
  --clearance-profile narrow \
  --route /绝对路径/office_corridor_route.json \
  --output outputs/exact-field-preparation-review-NEW
```

`preparation.json` 的 `all_static_legs_passed` 记录静态结果。当前速度、覆盖及净空从 [项目 YAML](../ws_nav/src/ranger_navigation/config/exact_navigation.yaml) 读取，参数含义见 [导航接口](exact_mppi_interfaces.md)。

## 分阶段现场顺序

| 阶段 | 操作与记录 | 进入下一阶段的条件 |
| --- | --- | --- |
| S1 静止定位与观察 | 当次授权后启动三融合、C2/C1、wheel映射和公共TF；只接收底盘可使用`ws_base/install_exact` 的 receive-only 模式；观察门口原始云、地图、真实车体位置 | 同图身份、新采样连续就绪、TF唯一；不得靠放宽定位条件继续 |
| S2 底盘与停车 | 另获底盘控制和运动授权；停掉自启receive-only会话后启用全模式驱动，避免重复CAN拥有者；空旷区逐项核对前后/左右/斜行/转向、真实模式确认、停稳切换、取消/急停/遥控接管 | 按当次验证配置低速，实测反馈及地面标记/视频记录制动；没有未解释的轮向或模式错误 |
| S3 门口单段 | 从实际定位设置出发位；先到门内等待位，再逐段穿门和返回；完整保留失败地图及轨迹 | 门口实际净宽、全车扫掠、当前观测覆盖和单段往返通过 |
| S4 走廊分段 | 先短距离走廊往返，再延长至端部；静态障碍、临时堵路、取消和定位撤销停车分项验证 | 各段实际执行无碰撞，定位撤销会停车，掉头空间与实际模式确认通过 |
| S5 全程往返 | 依次发送单点目标，上一点成功且反馈停稳后发下一点；取消、失败、接管时停止后续目标，不盲目继续返回 | 办公室出发—走廊端部掉头—办公室返回真实完成；≥30分钟重复任务另记成功率/超时/切换失败/定位撤销/接管 |

## 记录

现场操作可使用工作台轻量记录与原始话题录制。独立定位入口的 `--public-tf --record-navigation` 同时记录定位、wheel、TF、控制、执行反馈与动作状态。保留实际目标、执行/取消结果、原采样时刻和反馈。缺少外部真值时报告估计到点误差，真实制动距离需独立测量。
