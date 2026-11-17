# P0 闭环结果

数据状态：PASS_DATA（不代表精度验收通过）

路线：synthetic_only
已接受回环事件：0
共同参考路程：18.100 m，原始前端估算

| 轨迹 | 窗口 | 位置误差 cm | 航向误差 ° | 漂移率 % |
|---|---|---:|---:|---:|
| frontend | immediate | 10.000 | 1.146 | 0.5525 |
| frontend | settled | 10.000 | 1.146 | 0.5525 |
| optimized_history | immediate | 0.000 | 0.000 | 0.0000 |
| optimized_history | settled | 0.000 | 0.000 | 0.0000 |
| online_backend | immediate | 10.000 | 1.146 | 0.5525 |
| online_backend | settled | 0.000 | 0.000 | 0.0000 |

无实测路程时，表中漂移率均为按前端估算路程归一化；独立路程漂移率字段保持 null。
没有接受回环时，本轮只测得未发生回环修正的在线返回误差。
P0 物理复位由操作员确认；结果受复位重复性和工作外参误差影响。
optimized_history 为最终冻结图校正的全采样轨迹，非所有采样点独立参与优化。
online_backend 保留当时首次发布的有效新鲜采样；缺输出不补值。
窗口跨度、缺口、静止范围、跳变见 metrics.json；无外部全程真值，不报告 APE/RPE。
