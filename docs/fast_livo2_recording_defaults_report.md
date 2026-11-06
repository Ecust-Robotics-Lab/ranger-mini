# 录包入口默认行为简化

日期：2026-11-05。用户明确要求 `--execute --start-sensors --runtime-config config/profiles/fast_livo2_frontend.yaml` 三项作为默认，不必每次输入。本轮修改专用录包入口、相关测试和当前文档，没有启动真实传感器、底盘或导航，没有录制新大运动包、修改正式时间配置、构建部署、提交或推送。基线HEAD为 `4f4977992438dcca6e84c5d2a904d7dea94d1f70`。

## 实际行为

- `bash scripts/record_fast_livo2.sh` 现在直接录制，默认自行启动雷达、同步writer、相机和FAST前端，并读取 `config/profiles/fast_livo2_frontend.yaml`。默认时长仍60秒、可视化仍auto、磁盘保留仍5GB；用户本次只要求修改上述三项。
- `--dry-run` 显式预演，只显示计划；输出增加实际runtime/frontend配置路径。`--status`仍只读状态，`--stop`仍仅清理所属录包会话。
- `--attach` 才接入已有FAST预览。默认遇到已有预览或其他设备进程占用仍拒绝，不自动接管、停止或重启。自建会话结束时停止所属传感器，接入模式保留外部预览。
- 旧 `--execute`、`--start-sensors` 仍兼容；显式 `--runtime-config PATH` 仍可选择其他配置。该入口默认profile会覆盖仅由 `RANGER_RUNTIME_CONFIG` 指定的值；需要自定义profile时通过入口参数传入。
- 其他运行/构建入口的默认行为不变。录包仍不启动底盘/CAN/Nav2或公共TF，不改变Tag默认关闭、原消息时间戳、磁盘/输入检查、来源归档和故障收尾规则。

本次15分钟大范围采集可直接使用：

```bash
cd /home/lab307/ranger_nav
bash scripts/record_fast_livo2.sh --seconds 900 --label corridor_classroom_map01 --visualization none --min-free-gb 100
```

只查看上述计划时加 `--dry-run`。正式前端仍是用户指定的+100ms；本轮预演核对了所选YAML为0.1，没有将历史+110ms实验追认为新配置结果。

## 修改与验证

源码为 `scripts/record_fast_livo2.sh`、`scripts/fast_livo2/record_session.py`；相关测试为 `tests/test_fast_livo_recording.py`。测试增加3个行为检查：默认进入独立录制并选中FAST profile、已有预览不会被默认接入、dry-run可显式覆盖profile且不创建输出。原有预演调用显式加--dry-run；自建合成会话不再传两个冗余开关。合成入口仅在测试内替换物理设备占用/IP检查，真实入口的保护保留。

证据目录：`docs/fast_livo2_evidence/recording_defaults_20261105/`。合成包和来源副本留在被忽略的 `.runtime/validation/recording_defaults_20261105/artifacts/`，不混入真实原包。

| 实际命令/检查 | 退出码与结果 | 证据 |
| --- | --- | --- |
| `ast.parse`检查两个变更Python文件；`bash -n scripts/record_fast_livo2.sh`；`git diff --check` | 0，语法与空白检查通过 | 本轮执行记录、`final_checks.json` |
| `bash scripts/record_fast_livo2.sh --help` | 0，默认及兼容参数说明正确 | 本轮执行记录 |
| `PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=189 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 tests/test_fast_livo_recording.py`，设置独立EVIDENCE_DIR | 0，21项通过，约39秒 | `command.json`、`recording_tests.log`、`test_result.json` |
| `bash scripts/record_fast_livo2.sh --dry-run --seconds 900 --label corridor_classroom_map01 --visualization none --min-free-gb 100` | 0，start-sensors/FAST profile、domain175/localhost、YAML时偏0.1；没有创建输出 | `dry_run.json`、`dry_run_command.json` |
| 当前文档链接、真实主程序SHA及会话只读收尾检查 | 0；没有替换二进制或接管既有会话 | `final_checks.json` |

21项包含8项隔离ROS合成录包检查和13项单元/入口检查，覆盖定时与Ctrl+C收尾、所属进程清理、外部预览保留、录包器崩溃、重复发布者、无数据/断流、磁盘保留和上述新默认。没有重跑无关算法构建或123/74等历史验收，不把这些数量相加。

PASS：默认简化、兼容入口、显式预演/接入、相关21项检查及当前文档同步。PARTIAL保持：完整+100ms建图/定位评估、物理同步、真实Tag地标/恢复/回环和总计划D。未实施：真实新录制、硬件/运动、部署、Git提交推送。

接手实查的field和旁边K集成会话仍须由操作员通过所属入口协调，录包默认执行并不绕过其占用。最少人工待办：释放同一雷达的既有占用后，执行所需录包命令；等待 `CAPTURE RECORDING` 再开始采集动作。
