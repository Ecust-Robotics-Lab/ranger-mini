# 复制到新对话

请接手 `/home/lab307/ranger_nav`，分支 `refactor/pluggable-navigation`。先读 `AGENTS.md`、`docs/HANDOFF.md` 和 `docs/STATUS.md`，涉及定位和建图设计时再读 `docs/FAST_LIVO2_BACKEND_PLAN.md`。

项目是 Ranger Mini V2 的 FAST-LIVO2 三融合定位与 EXACT-MPPI 导航。工作台入口见 `workbench/README.md`。先核对 Git、实际启动来源、项目 YAML 和受管会话。用导航 worker 状态检查覆盖参数是否加载，用任务事件和执行链日志区分规划、门禁、驱动与实际运动的问题。

目前已有现场单点成功记录。整段办公室出门、走廊端点掉头和回返待验收。遇到相机 `MV_E_NODATA` 或低速请求未执行，先按 HANDOFF 的定位方法核对，不凭症状调高速度或修改时间参数。

先给我简短接手摘要，再按我的下一条指令推进。启动设备、操作 CAN、实车目标、非零速度及重启须当次明确授权。保护原包、地图、标定、安装资产、失败证据和未提交修改。用隔离 ROS 做离线验证。
