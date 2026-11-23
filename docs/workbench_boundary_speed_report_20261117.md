# 最近任务后角未知格与正式速度档修正

状态：软件/构建/记录分析/独立几何与GPU/隔离ROS/网页PASS；本轮没有启动设备、CAN或实车导航，没有新实车速度/制动验收，无提交推送。源码基点仍bb418321ea139503242bcc5d975906896c808714，保护此前及本轮未提交工作。

## 用户现场记录的事实

用户自行完成的最近工作台记录为`.runtime/navigation_workbench/records/live-3f7387d238d0407587ad3c0e8dce2b1b/events.sqlite3`，navigation run-azrdqqr3、base run-394skz0z、FAST定位run-h6f_ii4n均已STOPPED。4个任务中第1和第3个返回SUCCEEDED，第2和第4个失败；这不是Agent新增实车实验，也不等于绝对停车精度验收。明细保存在[证据](workbench_evidence/20261117/coverage_speed/record_analysis.json)。

1. `guidance_unobserved`仍出现，但新2.5cm策略已加载：记录是保护范围未知1～4格、外缘0格、额度8。旧额度只管最外1cm带；保护范围内哪怕1格也拒绝，所以并非8格参数未生效。停住后的未知格集中在后角：5cm完整网格与旋转车体边界相交，一部分格在车体边界内、另一部分在外；旧策略将任意相交都当作内部缺观测。
2. 正式launch仍默认`low_speed=true`，工作台/robot_session未显式覆盖，因此实际采用首次实车档0.10m/s，而非motion中的0.30m/s。最后直线任务非零请求中位0.0807/P95 0.0960/最大0.0981m/s，实测wheel非零中位0.075m/s；任务请求近零比例约23%、最终安全输出约25%。低速主要已在发令前形成，不能先归因底盘标定。

保存了该次4格后角输入的网页观测快照2073，以及显示格阵重建的`recorded_boundary_fixture.npz`。格阵拟合误差小于1e-10格，但显示来自完整3D投影，再投回平面导航格阵有近似：旧检查复现3个保护格+1个外缘格，而实时记录为4个保护格。因此它只作边界分类回归，不冒充原始scan或精确闭环回放。`boundary_fixture_provenance.json`保留假设；最后实际计算邮箱也独立复制，原记录/地图/包/标定未修改。

## 修改后的覆盖规则

新增YAML布尔参数`coverage.allow_boundary_unknowns`，默认true，全部传至路径前段/GPU/独立验证/末端门禁。总外扩仍2.5cm，不缩车体、不改未知格值、不改全局地图未知策略。

- true：未知单元若**完整落入车体+protected_margin矩形**，仍拒绝；只部分触及该矩形边界的格，与外缘格共用`unknown_cell_budget`总额度。9格不能用8格额度放行。该规则按完整单元与矩形几何判断，适用于旋转姿态。
- false：恢复旧严格方式，只要未知单元与保护范围有任何相交即拒绝；额度只适用于保护范围以外的边缘。
- 已知占据格、越界、完全未知的车体内部均不豁免。JAX与NumPy分别实现，并交叉核对；预测全车与制动轨迹、全量障碍点仍检查。

这明确放宽了车体边缘局部未知的接受范围，**不表示这些格已被观测**；不能再把新默认描述为“完整保护范围绝无未知”。旧报告和旧AGENTS中的“仅最外1cm/保护范围任意格必拒”被本节的新默认替代，可用false恢复。

用户提出整块忽略后方。本轮没有这样做：前进、倒车、侧移、转向均可能让车尾扫入新区域。当前只解除少量边界格的一票否决，不按固定前后半区切掉检测；点障碍防撞与全预测扫掠保留。新格阵回归中4格可使用额度，完整内部未知、超额和已知障碍仍拒绝。

## 速度与YAML

正式launch默认`low_speed=false`。默认工作台/robot_session直接读取项目YAML的motion速度；显式`low_speed:=true`仍保留首次实车档。显式CLI max_speed必须不超过该YAML motion上限。

```yaml
coverage:
  total_margin: 0.025
  protected_margin: 0.015
  unknown_cell_budget: 8
  allow_boundary_unknowns: true
motion:
  max_speed: 0.30       # m/s，正式导航平移上限
  max_yaw_rate: 0.50    # rad/s，正式导航旋转上限
  acceleration: 0.30
  yaw_acceleration: 0.50
```

项目文件`ws_nav/src/ranger_navigation/config/exact_navigation.yaml`在新导航会话启动时读取，改值无需重构建；GPU reset不重新读取YAML。`/navigation/state.control_limits`新增实际max_speed/max_yaw_rate，网页“实际导航上限”直接显示当前进程值，不用配置文件猜测运行档。上限不是恒定发令值，接近目标、轮角/模式切换、障碍及反馈约束仍可能降低指令。

## 验证命令与证据

以下从项目根运行，最终退出码均0，输出位于`docs/workbench_evidence/20261117/coverage_speed/`。同组及后续检查存在重叠，不相加为总数。

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m pytest tests/test_exact_field_preparation.py tests/test_exact_coverage_policy.py tests/test_exact_navigation_core.py tests/test_exact_temporal_visibility.py tests/test_exact_visibility_precision.py -q
# core_tests.log：87通过
source ws_base/install_exact/setup.bash
source ws_nav/install_exact/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .runtime/exact_mppi_venv/bin/python -m pytest tests/test_exact_coverage_critic.py tests/test_exact_optimizer.py tests/test_exact_warm_start.py tests/test_exact_control_recovery.py tests/test_exact_sampling_ros.py tests/test_exact_navigation_reset.py -q
# gpu_tests.log：49通过，含边界容忍true/false下随机旋转位姿CPU/GPU一致
/usr/bin/python3 -m pytest tests/test_exact_field_preparation.py -q
# yaml_strict_flag.log：后续25通过，含真实LaunchDescription默认档、非默认YAML .24/.35速度、严格false传递及非法类型拒绝
.runtime/navigation_workbench_venv/bin/python -m unittest tests/test_navigation_workbench.py tests/test_navigation_workbench_reset.py tests/test_fast_livo_frontend_upgrade.py -v
# workbench_tests.log：65通过
/usr/bin/python3 tests/check_exact_coverage_ros.py --evidence-directory docs/workbench_evidence/20261117/coverage_speed/ros_cruise
PLAYWRIGHT_BROWSERS_PATH="$PWD/.runtime/workbench_browsers" .runtime/navigation_workbench_venv/bin/python tests/check_workbench_coverage_browser.py --evidence-directory docs/workbench_evidence/20261117/coverage_speed/browser
```

隔离ROS_DOMAIN_ID=231/localhost，仅模拟反馈，无CAN/SDK设备：真实安装的协调器+GPU+独立门禁回报新策略/0.30上限，直线输出超过0.23m/s并继续运行1秒、峰值不越0.30，观测断流和取消均停车，共6项通过。峰值见`ros_cruise/result.json`，是模拟安全输出，不是实测车速/制动距离；不以少量计算样本宣称P95整链实时通过。第一轮短时检查保留在`ros/`。浏览器5项通过，包括实际0.30上限显示，无设备控制请求。

构建前确认无存活受管会话，随后在`ws_nav`执行：

```bash
colcon --log-base log_exact build --build-base build_exact --install-base install_exact --packages-select ranger_navigation --executor sequential
```

`build.log`：1包成功、退出0，正式nav已更新。最终源码/安装和会话身份见`final_identity.json`，测试自有进程已停止。静态2.5cm、实时点障碍裕量、FAST低延迟来源锁、三融合/+100ms/外参、定位400ms和其他输入期限、100ms租约保持。真实重新单点、较高速度下连续运动/停车及后角变化仍需现场验证，本轮未代启。
