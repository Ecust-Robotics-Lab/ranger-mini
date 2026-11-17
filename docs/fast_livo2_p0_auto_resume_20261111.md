# P0 实验默认自动读取进度

日期2026-11-11。用户要求再次运行原命令即可读取已有进度。

## 行为

修改 `experiments/fast_livo2_p0_closure/suite.py` 和实验README：

- 默认扫描已有实验目录，优先当前存活组，否则选择最近有实际trial记录的组。较新的空组不遮住此前采集结果；没有非空组时可复用最近空组，没有历史时新建。
- COMPLETE/PARTIAL轮次沿用原处理方式跳过，失败轮写新attempt。五轮已全部完成则展示结果位置并退出，不启动新的五次。
- `--new` 显式新建，`--resume ABS_RUN` 显式选择，两者互斥。
- `--dry-run` 展示自动选择的组和下一轮次；`--status/--stop` 使用相同组选择规则。
- 只有组选择、主入口调度、来源检查和离线分析可以通过版本登记继续原组；逐轮采集函数、配置、共享脚本及二进制仍必须匹配原身份。原source_hashes.json不改。

当前 `runs/latest.json` 指向误建空组 `20261111_174522-lfpfyfan`。默认选择检查实际返回原有效组 `20261111_171817-liu6_7ya`、next_trial=2、hardware_started=false；已有第1轮不重做，原空组也不删除。

## 验证

证据：[p0_auto_resume_20261111](fast_livo2_evidence/p0_auto_resume_20261111/)。

- 实验离线回归：`python3 -m unittest discover -s experiments/fast_livo2_p0_closure/tests -v`，退出0，30项通过，含自动选择、空组遮挡、新建、全部完成、手动选择和采集身份拒绝等检查。
- `bash experiments/fast_livo2_p0_closure/run.sh --dry-run`：退出0，选已有组、下一轮2。
- `bash experiments/fast_livo2_p0_closure/run.sh --new --dry-run`：退出0，选新组、下一轮1。
- 只读组选择和来源核对通过，未执行实际resume、未启动设备。
- 备份 `.runtime/backups/p0_auto_resume_20261111_174855/`，原包、地图、已完成/失败结果均保留；未提交或推送。

用户仍用原命令：

```bash
bash /home/lab307/ranger_nav/experiments/fast_livo2_p0_closure/run.sh
```

程序先显示读取到的组和进度，仍在本轮P0提示处等待用户回车后才启动设备。
