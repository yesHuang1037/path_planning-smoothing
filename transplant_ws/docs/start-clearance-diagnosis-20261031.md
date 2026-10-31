# 端点净空失败诊断（2026-10-31）

## 结论与证据边界

用户观察到 `planning snapshot; execution disabled` 后出现
`planning failed: invalid start or goal clearance; new goal required`。
本轮读取活动 `/snapshot_planner` 的持久状态，收到同一失败消息；实际节点名不是 `/snap_planner`。

**当前实时取样中，起点本身已不满足净空；不需要目标参与即可触发相同失败。**
原失败任务未启用 `snapshot_file`，本轮也未接收到新的 `/goal_pose`，因此不能恢复原任务的目标与精确地图快照，也不能断言原目标是否合法。
以下数值来自后续当前输入取样，不冒充原失败时刻的数据。

## 当前输入与计算结果

- 活动参数：`body_length=0.60 m`、`body_width=0.60 m`、`clearance_margin=0.08 m`、`occupied_threshold=65`、`static_only=false`。
- 判据：中心到阻塞区域的保守净空必须严格大于 `hypot(0.30, 0.30)+0.08 = 0.504264 m`。
- TF 起点：`map -> base_link = (0.0114005074, -0.2211529541) m`；TF 秒/纳秒为 `1793452717 / 51444636`。
- 静态图：`map`，296×388，分辨率约 0.05 m，原点 `(-8.75, -14.5) m`。
- 动态图：`map`，80×80，分辨率约 0.10 m，原点约 `(-3.9, -4.2) m`；消息秒/纳秒为 `1793452717 / 23941098`。

用已构建的实际 `libtdt_planning_core.a` 创建 `Environment` 并查询：

| 环境 | 起点所在格 | 起点所在格值 | 保守净空 | 严格门槛 | 是否安全 |
| --- | --- | --- | --- | --- | --- |
| 仅静态图 | (175,285) | 0 | 0.445486 m | >0.504264 m | 否 |
| 静态+动态图 | (175,285) | 0 | 0.353624 m | >0.504264 m | 否 |

起点所在格为 0 不代表车体包络安全。最近静态阻塞格原始值为 100，索引 (168,292)，中心约 `(-0.325, 0.125) m`；到其格面积距离约 0.447336 m。
最近动态阻塞格原始值同为 100，动态图索引 (35,37)，格面积约为 `[-0.4,-0.3] × [-0.5,-0.4] m`；到该面积距离约 0.359105 m。
保守净空还受距离场下界约束，故不等于上述最近格面积的精确距离。

活动 `/costmap/costmap` 参数只列出 `stvl_layer`，无 inflation 插件，`always_send_full_costmap=true`。
因此本次最近阻塞格不是未知格，当前配置也不支持“costmap 膨胀层重复膨胀”这一解释。
障碍是否对应实物、点云自回波或配准偏差，本轮未验证。

## 消息触发路径

- `src/tdt_path_planning/src/node.cpp:247`：起点取 TF 平移，目标取 `/goal_pose`。
- `src/tdt_path_planning/src/node.cpp:257`：异步任务已发起，报告只规划、不执行；不是失败原因。
- `src/tdt_path_planning/src/search.cpp:13`：`!environment.safe(start) || !environment.safe(goal)` 抛出本错误；尚未进入 A* 搜索循环，更未进入平滑优化。
- `src/tdt_path_planning/src/grid.cpp:189`：安全判据使用严格大于车体外接圆半径加余量。
- `src/tdt_path_planning/src/node.cpp:203`：包装失败消息并要求新目标；不自动重试旧请求。

## 复现命令与产物

只读 ROS 取样需要宿主 DDS 权限；本轮经批准执行：

```bash
source /opt/ros/humble/setup.bash
ROS_LOG_DIR=/tmp/snap-planner-diagnosis /usr/bin/python3 /tmp/snap_clearance_capture.py
/tmp/snap_clearance_probe /tmp/snap-clearance-current.snapshot
```

捕获脚本只订阅地图、TF、状态、目标并调用参数读取服务，不发送目标或控制。
由于未收到目标，保存快照使用 **合成 goal=start**，仅用于起点净空复现，不是原始失败请求。
探针链接现有核心，打印两种环境净空，再调用实际 `tdt::plan()`；输出
`actual plan result: invalid start or goal clearance`，退出码 1，精确命中本次错误分支。

临时产物：`/tmp/snap_clearance_capture.py`、`/tmp/snap_clearance_probe.cpp`、`/tmp/snap_clearance_probe`、`/tmp/snap-clearance-current.json`、`/tmp/snap-clearance-current.snapshot`。
这些是本机临时诊断材料，不是不可变版本或异机备份；清理 `/tmp` 后不能保证保留。

## 后续建议与未执行项

1. 对照 RViz 中上述静态/动态阻塞区域与实物，先确认 `map -> base_link` 定位及静态图配准，再检查近车体点云与 STVL 标障来源。
2. 当前仅换远处目标不能解决起点不安全；即使不融合动态图，当前静态净空仍未过门槛。
3. 不以缩小车体、降低余量、跳过起点检查或开启执行作为修复。安全处理输入/实际位置问题后，需新的显式目标触发规划。
4. 若要精确复核原目标，下一次应保存节点自身请求快照；本轮未改参数、未重启节点、未发送目标，未修改规划源码或外部工程。

本轮结果只证明当前采样输入的净空失败及代码触发路径，不代表 ROS 全链路、最终轨迹或实机安全验收通过。
