# 工程整理、标定迁移与配置切换

日期：2026-10-26。范围为用户授权的三个整理任务；没有重新启动实物、底盘或导航，没有进入 FAST-LIVO2 B–D。

## 结果

**PASS：清理、迁移、配置分派、构建与本轮离线验证。PARTIAL：迁移后真实设备复测、标定/物理同步与运动验收。** Lightning 仍是 `config/runtime.yaml` 的建图及全局定位默认，轮速、公共 TF 与安全链未替换。

1. 删除两份逐字节相同 DDS 派生回放、原始 XWD、自有字节码及生成的 SDK 缓存，释放 567230464 bytes 分配空间（约 541 MiB）。保留 final DB、PNG、原始录包、地图、正式基线、失败证据和清理前 sidecar。
2. 原 `~/calibration` 在同一文件系统原子迁至 `ranger_nav/calibration`，迁后核对 1818 个文件/链接的身份与大小。旧路径保留兼容链接；后续核对 1158 项原始数据、结果、配置与旧工作空间，inode/大小/mtime 或链接目标保持。没有复制别机标定或改写历史结果。
3. 当前相机/共享戳包纳入 `ws_camera/src/`，复用原 SDK；本机 camera_info 以 package URI 装载。标定工具在新 integrated 目录编译，旧构建与工作空间保留。历史绝对路径读取时解析到新目录，原配置 SHA 不变。
4. 新增 `run_mapping.sh` 与 `run_calibration.sh`。runtime 支持继承与建图前端选择，在线计划及离线回放实际使用所选参数。校验三传感器均启用，专用入口同样检查。定位仍支持既有 lightning/standard，FAST-LIVO2 暂不能作为全局定位后端。

## 当前操作

工程根目录执行，以下均为默认 dry-run：

```bash
bash scripts/run_mapping.sh online
bash scripts/run_mapping.sh online --runtime-config config/profiles/fast_livo2_frontend.yaml
bash scripts/run_mapping.sh offline --runtime-config config/profiles/fast_livo2_frontend.yaml
bash scripts/run_calibration.sh sync
```

也可以修改 `config/runtime.yaml` 的 `mapping_backend`，或设置 `RANGER_RUNTIME_CONFIG` 指向 profile。`extends` 和配置中的相对路径以工程根目录为基准；支持嵌套字典覆盖、检测继承环。环境中的旧算法选择变量不覆盖配置选择。

`mapping.fast_livo2` 配置前端、相机、雷达、RViz 文件及 ROS domain/localhost；`mapping.lightning.config` 选择 Lightning 建图参数。生产定位图关联由 `map_bundle` 校验，不随前端切换。选择 standard 定位还须提供匹配的地图关联及符合 LocalizationEstimate 契约的真实后端，见 [定位契约](localization_backend_contract.md)。

实际执行增加 `--execute`。FAST-LIVO2 online 会启动真实雷达、相机和 RViz，须先获得该次传感器测试授权。Lightning online 延续既有 accepted-offline 等预检；本轮没有绕过。底盘/导航仍沿用逐次授权与运动门禁。标定流程、依赖、自由运行/外触发区别见 [calibration/README.md](../calibration/README.md)。

## 命令与验证证据

证据根目录：`docs/consolidation_evidence/20261026/`。本轮执行：

| 命令/检查 | 结果 | 证据 |
| --- | --- | --- |
| `bash scripts/build_camera_sync.sh --execute` | 退出 0，2 包 | build_camera_sync.log、build_camera_sync_final.log |
| `bash scripts/build_calibration.sh --execute` | 退出 0，FAST-Calib 编译安装 | build_calibration.log |
| ws_nav `colcon build --packages-up-to ranger_navigation --executor sequential`；最终重建 bridge | 退出 0，3 包及最终 bridge；仅禁止字节码生成的 warning | build_nav.log、build_bridge_final.log |
| `PYTHONDONTWRITEBYTECODE=1 python3 tests/test_project_consolidation.py` | 退出 0，10/10 | test_consolidation.log |
| ROS 环境下 `python3 -m unittest discover -s calibration/lidar_camera_extrinsics/tests -v` | 退出 0，11/11，含数值/图像接口及 supervisor | test_calibration.log |
| 既有时间对齐 / 进程管理 / 定位契约测试 | 分别退出 0，7/7、11/11、11/11 | test_time_alignment.log、test_sessions.log、test_localization_contract.log |
| 临时移除旧链接：新包解析、C++ 标定检查、真实包读取、writer、LIVO 检查、在线计划、同步计划、短回放 | 8/8 退出 0，链接已恢复 | relocation_verification.json、各项 log；可审查 verify_relocation.py |
| 新位置真实 writer + 合成 ROS 点云 | 5736 次一致快照；重复 writer 拒绝且未截断 | real_writer_synthetic_ros.log |
| FAST-LIVO2 定向 C++ 检查 | 13/13 | livo_cpp_checks.log |
| `run_mapping.sh offline --runtime-config config/profiles/fast_livo2_frontend.yaml --execute --max-frames 20` | 退出 0，20 对原始静态包帧、20 帧 VIO、121 次视觉迭代 | fast_short_replay.log；docs/fast_livo2_evidence/replay-Vp1pxGy1；bags/fast-livo2-baseline-tdSmpee7/result |
| 原始资产与来源核对 | 1158 受保护项保持；原相机用户修改保留 | protected_assets.json、config/calibration_sources.json |

隐藏兼容链接时只运行离线工具、合成 writer 和 dry-run，没有启动真实相机/雷达。短回放生成 20 帧小结果验证新入口，不重复生成全量 DDS 派生包；计数只证明迁后读包、视觉参与与入口有效，不证明运动精度。

本轮没有复跑 Nav2 全安全链，保留其原日期与范围；没有用历史通过替代迁后真机验证。FAST-LIVO2 给定初值定位、无初值定位、回环、PGO 和优化地图保存仍未实现；同步/标定继续 PARTIAL。

## 修改与版本

- 清理迁移：`.gitignore`、`.gitattributes`、calibration 目录、`ws_camera/src/hik_camera_trigger_driver` / `timeshare_writer`、SDK 相对链接；11 个已跟踪 pyc 的删除留在工作树。
- 配置运行：runtime.yaml、profiles、calibration_sources.json、公共 runtime_config.py、mapping_session.py / run_mapping.sh、两个 FAST-LIVO2 入口与 online_session.py、标定入口和构建脚本。
- 标定适配：common.py、工具路径、编译输出目录、内参采集入口、stage_b 默认 dry-run 与每会话共享戳。
- 验证文档：test_project_consolidation.py、证据、calibration/README、PROJECT_LAYOUT、STATUS、README、AGENTS。

主仓库仍在 `refactor/pluggable-navigation`，起点 HEAD `a76d83a767d5691ca40351f3a57f65b398572a52`，本轮未提交、未推送。原 `docs/03_mid360_validation.md` 与六份 driver 证据等用户修改保留。前端固定版本沿用 `config/fast_livo2/sources.lock.json`，没有切换算法提交。

## 回退与下一步

台账 cleanup.json、migration.json、camera_package_migration.json 均保留。可再生成的缓存未备份；删除 DDS 的等价 final 与 XWD 的 PNG 保留。整理前 tracked diff、源码/配置检查点、标定原 Git 元数据及完整 history bundle 在 `.runtime/backups/consolidation_20261026/`，不可整体清除。

回退迁移前须确认无相关进程并保存本轮新源码/修改，再恢复整理前入口和配置、移除兼容链接，将 `ranger_nav/calibration` 原子移回 `/home/lab307/calibration`；原 `.git` 可从备份恢复。旧工作空间仍在原子目录，新 integrated 构建不覆盖它。继续使用 Lightning 无需回退，默认配置已保留。

本轮完成不需要用户补充信息。下一次真机复测或三传感器运动录包需另行指定；迁后的设备连接、物理同步、地图精度及 B–D 不在本轮 PASS 范围。
