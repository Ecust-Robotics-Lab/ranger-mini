# 阶段 4：人工遥控小闭环采集

已有静止包和一个手持诊断包（`bags/mid360-manual-loop-4uwmxbzb/`，2026-09-22 手持雷达走动录制，**只作数据链路与管线诊断，不作为建图验收**）；正式小地图验收等待本说明的正式运动包。机器人由人全程遥控，助手不启动底盘驱动、不发 CAN/速度/导航目标。

## 现场准备

- **先把 Mid-360S 牢固安装回 Ranger Mini V2 车体**：安装牢固、线缆固定、窗口无遮挡后再采集；手持走动数据不再用于建图验收。安装后记录 LiDAR 相对 base_link 的 xyz/rpy（量不到就写待测）。
- 空旷平地，有可见且固定的墙体/立柱供配准，避免只有空地、玻璃或移动人群。
- 保持已确认的 Mid-360S 原厂线、12 V、固定及无遮挡条件。先确认现场遥控停车/急停可用；阶段 2 停车反馈异常仍未验收。
- 记录安装位置与姿态：LiDAR 相对 base_link 的 xyz、roll/pitch/yaw；未知就写待测，不填零。
- 记录至少一段可量测的墙距或两固定物间距，供地图尺度检查；为后续二维图测量地面在传感器坐标下的位置、安装倾角、要保留的障碍物高度范围。
- 若网线/雷达电源此前断开，先恢复实物连接并只读检查 `ip -br addr show enp2s0`、`ip route get 192.168.1.101`。预期 host .50/24、当前新雷达 .101 走 enp2s0；不盲目重配、重跑网络脚本。

## 两个终端（由用户准备好现场后执行）

终端 A，只启动一份雷达驱动，最多 300 秒：

```bash
cd /home/lab307/ranger_nav
source scripts/slam_env.sh
python3 scripts/start_mid360.py --execute --seconds 300
```

确认原始话题在收流；终端 B 开始最多 180 秒录包：

```bash
cd /home/lab307/ranger_nav
bash scripts/record_slam_loop.sh --execute --seconds 180
```

1. 录制开始后静止约 10–15 秒，供 IMU 初始化。
2. 人工以可随时停车的低速走一个含转弯的小闭环，避免急加速、急转、碰撞和颠簸；返回起点附近。
3. 结束前再静止约 10–15 秒。可在录包终端 Ctrl-C 正常收尾，保留 metadata.yaml；不要 kill -9。
4. 驱动正常停止后保留其日志。把 `BAG_RUN=...` 完整目录路径及“路线、起止静止时长、现场异常、量测参考距离”交给助手。

录包脚本只录原始 CustomMsg/IMU 与当时已有的底盘状态/TF；没有的不会伪造，也不会为补话题启动底盘。新包使用 `mid360-manual-loop-*` 唯一目录；不得只交 db3。

阶段 3 检查器可能因帧内 offset 回退退出 2。保留该结果和原包，不能直接改成通过。Lightning 的排序可处理点序无序，但主机接收时间模式的动态误差还需审查。

## 收到运动包后的顺序

1. SQLite、完整消息、时间戳检查和 `bash scripts/check_lightning_input.sh /完整包目录/raw`。
2. 离线三维建图，例如：

```bash
bash scripts/run_slam_offline.sh --bag /完整包目录/raw \
  --motion-data-note '人工平地小闭环；起止静止各15秒；参考距离另附' --execute
```

3. 检查 `maps/lightning-offline-*/` 中全部定位地图、关键帧、日志；审查墙体重影、闭环错配、地面、实测尺度。审查结论写入该实验 `acceptance.json`，不能仅因程序退出 0 填 PASS。
4. 相同 bag、相同配置再生成一个新目录；`python3 scripts/check_slam_map.py 实验A --compare 实验B` 检查几何重复性。不同线程/容器顺序可能使 PCD 二进制哈希不同，需看双向最近邻统计、轨迹和闭环，而非只比文件字节。
5. 三维通过后才建立新的二维配置：填写已测地面/安装及障碍物高度，打开 with_g2p5，在新目录离线验证 map.pgm + map.yaml。在线入口同样要求已验收三维目录；本轮不运行在线模式。

二维栅格成功不代表坡道通行性评估完成。
