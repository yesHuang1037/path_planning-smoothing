# 原导航补充调查：地图、净空、重规划与失败参考

日期：2026-10-27。性质：用户补充实机启动入口后的静态复核；不是新开发规范，不批准修改导航、SLAM、MPC 或移植副本。未启动 ROS、未构建、未测量活动参数或话题。

路径缩写：`N=/home/hustlyrm/source/navigation_ws/src`；`H=/home/hustlyrm/slam_source/odom_hyx`；`P=N/path_searching`。行号对应本轮读取的工作树。

## 1. 用户确认与启动入口

- 用户明确：实机里程计与感知从 `/home/hustlyrm/slam_source/start_dlil_hyx.sh` 启动；相对原始 DLIL，用户描述主要优化参数配置、整体思路一致。本调查不将该描述扩大为源码逐字节相同的结论。
- 脚本第 2～3 行依次 source `livox_ws/install/setup.sh`、`odom_hyx/install/setup.sh`，第 4、6、7 行分别启动 Livox 驱动、`direct_lidar_inertial_odometry livox_fusion_node`、`dlil dlil.launch.py`。据此后续沿 odom_hyx 调查，不再以 odom_ws 代替该入口。
- `H/install/dlil/share/dlil/launch/dlil.launch.py` 的本地软链接解析到 `H/src/dlil/launch/dlil.launch.py`。它证明这份安装 launch 与源码的关联，不证明所有实机可执行文件与当前源码一致，也不替代活动节点身份采集。

## 2. 栅格分辨率：两张图不同，但不是静态图与导航预期不符

| 对象 | 本地定义 | 结论 |
| --- | --- | --- |
| 用户入口加载的静态图 | `H/src/dlil/launch/dlil.launch.py:34` 默认地图路径，`:175` map_server；`H/grid_map/map.yaml:2` | **0.05 m/格**；原点 `[-8.75, -14.5, 0]`；frame 为 map |
| 原导航包自带静态图 | `P/maps/map.yaml:2` | 同样 **0.05 m/格**，但原点 `[-17.1, -11.25, 0]`；不能混用两份地图的 origin 或图片 |
| 感知局部 costmap | `H/src/dlil/launch/dlil.launch.py:213`～`:227` | **0.10 m/格**、8 m × 8 m、rolling_window=true；STVL voxel_size 也是 0.10 m |
| 规划主地图几何 | `P/src/plan_manager.cpp:858`～`:887` | 首次 `/map` 回调读取消息的尺寸、原点和 resolution，A* 主栅格随静态图；没有独立的 YAML 搜索分辨率 |
| 既有内部硬编码 | `P/include/esdf_map.h:44`、`:50`；`P/src/smoother.cpp:130`、`:149` | ESDF 坐标换算使用乘/除 20，平滑部分距离换算乘 0.05、连线采样乘 20，隐含 0.05 m；不是完整多分辨率支持 |

`P/launch/debug_astar.launch.py:50` 虽定义了读取包内地图的 map_server，但 `:104` 的返回列表只启动 plan_manager；不能把该未启用的地图当成当前入口的 `/map` 来源。

**结论**：用户判断静态图 0.05 m 正确，也与原导航包的静态图分辨率及内部 0.05 硬编码一致。实际差异是局部 costmap 为 0.10 m，而非全局搜索使用 0.10 m。两种分辨率并非必须相等，但转换必须保留障碍覆盖范围。

### 已发现：粗图转细图仅映射格中心

`P/src/dynamic_obstacle_manager.cpp:323`～`:335` 计算 costmap 格中心世界坐标，再转成一个静态地图格索引；`:417`～`:430` 只插入该单一索引。没有把 0.10 m 占据格的整个面积保守栅格化到 0.05 m 图上。

因此在轴对齐、边界对齐的理想情况下，一个粗格覆盖 2×2 个细格，但当前只标记其中一个；边界不对齐时覆盖关系还会变化。这是从实现可确认的**障碍面积欠覆盖风险**，不是已复现实机碰撞。净空融合也使用该转换后的格集合，不能恢复丢掉的面积。

建议后续切片先修“粗格足迹→所有相交细格”的保守映射，连同增量清除的多源引用关系一起设计、测试；不应只把所有分辨率统一为 0.05，或仅增加净空阈值来掩盖问题。本轮没有实施。

## 3. Frame 与 origin：跟车移动的是窗口，不是坐标系

实际配置（`H/src/dlil/launch/dlil.launch.py:213`）：

- costmap `global_frame=map`，`robot_base_frame=base_link`，`rolling_window=true`。地图格坐标以 map 表达；窗口跟随 base_link 移动。
- `origin_x=-4.0`、`origin_y=-4.0` 是配置值；滚动时不能把它们当成永久固定原点，消费端应使用全量消息 `info.origin`。
- `sensor_frame=livox` 是观测源参数，不意味着 costmap 的 header.frame_id 是 livox。
- 启动列表启用 `map→odom` 零变换（`:278`、`:309`）、`livox→base_link` 平移 `(0, -0.196, 0)`、零旋转（`:286`、`:310`）。`localizer_node` 虽被定义，却在返回列表中注释（`:313`）。
- `H/src/dlil/cfg/params.yaml:16`～`:18` 设置 odom frame 为 odom、DLIL baselink frame 为 livox；`H/src/dlil/src/dlio/odom.cc:457` 发布 odom→livox，`:562` 将 deskewed 点云标为 odom。
- 原规划从 `base_link`、`baselink` 依次取位姿（`P/src/plan_manager.cpp:259`、`:1114`）；MPC 配置 `N/mpc_follower/cfg/mpc_params.yaml:8` 使用 map/base_link。按定义可接成 `map→odom→livox→base_link`，不应忽略 livox 与 base_link 的 0.196 m 偏移。

**静态结论**：默认配置下，规划与 costmap 都以 map 表达，地图转换直接加 origin 的前提基本匹配；不是需要把局部格先按 livox 坐标直接拼入 map。`convertCostmapCellToStaticMapIndex` 不读取 header、不做 TF，也不处理 origin 旋转，因此不能宣称支持任意 frame/旋转图。

**仍待实测**：消息 header.frame_id、滚动 origin、TF 唯一发布者、静态地图与开机 odom 的物理配准。存在 TF 连通或 map→odom 为零，不等于静态地图已与实物正确对齐；当前入口也不能被描述为已启用 localizer 定位。

## 4. 净空融合是什么

“净空”是代码估计的采样位置到最近障碍的距离，不是占据概率，也不是两张图的平均值。它是否代表车体边界余量，还取决于地图是否已膨胀、参考点和车体尺寸。

`P/src/path_safety_checker.cpp:45`～`:84`：

```text
required = max(0, robot_inscribed_radius + clearance_margin)
d_esdf = ESDF 格距离 × 静态图 resolution
d_costmap = 转换后 costmap 障碍格集合在查询半径内的最近距离
d_used = min(d_esdf, d_costmap)
safe = d_used > required
```

例如同一位置 ESDF 为 0.30 m、costmap 为 0.12 m，则按 0.12 m 判断；是取更保守的值，不是 0.21 m。

`P/src/dynamic_obstacle_manager.cpp:253`～`:305` 的 costmap 查询是有限半径格扫描，不是另一张完整距离场；关闭融合、集合为空或半径内未找到障碍时返回无穷大。当前 `P/cfg/plan_param.yaml:25` 为 false，所以实际直接使用 ESDF 距离。**costmap 动态占据仍写入 ESDF**，关闭这个开关不等于忽略动态障碍。融合也不会自动扩大障碍，不能替代机器人外形检查。

当前净空要求为 0.01+0=0.01 m（同配置 `:42`、`:46`）。这只是参数事实，不能作为完整车体安全尺寸；如上游未进行车体膨胀，尤其不能解释为已经补足车体半径。

## 5. global_monitor：检查结果和重规划许可是两层

`P/src/plan_manager.cpp:317` 强制关闭路径截断及 CostmapBlocked、PointcloudBlocked、PathCollision 触发；`P/src/replan_coordinator.cpp:151` 再次约束这些原因。因此即使 YAML 子项写 true，当前模式仍拒绝这些原因。

定时检查发现阻塞 → 记录报告 → 以 PathCollision 请求重规划 → 因原因禁用被拒绝。**被禁止的是触发原因，不是检查函数，也不是所有重规划**。`P/src/plan_manager.cpp:1336`～`:1400` 在请求未获接受时继续停滞/周期分支，所以之后可能以 Periodic 获批并搜索；新搜索仍使用包含动态障碍的环境。监控本身不发送停车命令。

### 最小重规划间隔到底与什么有关

当前 `monitor_period_sec=0.50`、`min_replan_interval_sec=1.5`（`P/cfg/plan_param.yaml:89`、`:91`）。判定见 `P/src/replan_coordinator.cpp:184`～`:217`、`:257`：

1. 先检查原因是否允许，再检查起终点距离是否小于 0.1 m；不满足则不放行。
2. 除 NewGoal、Manual 外，请求必须满足 `now-last_replan_time >= 1.5 s`。
3. `last_replan_time` 在**请求放行、实际 plan 调用之前**就更新，不是 A* 或平滑成功时才更新；搜索失败也会消耗这一间隔。
4. 新目标和手动请求绕过间隔检查，但放行时也更新同一时间戳；其他受节流原因共享这一时间戳，不是各自单独计时。
5. `triggerReplan` 从 `this->now()` 取时（`P/src/plan_manager.cpp:381`）；monitor 使用 wall timer。该机制不是独立的 1.5 s 周期调度器，也不是搜索超时/耗时预算。

理想的稳定 0.5 s 检查节拍下，若目标有效、TF 可取且周期触发允许，常见的是约每三次检查有一次周期请求获批；不是每次检查都完整搜索，也不是任何状态下都精确每 1.5 s 搜索。回调负载、规划耗时、时钟、目标事件都会影响实际时刻。

**技术权衡，尚非参数变更决定**：间隔控制重复搜索与参考更新的频率，需综合地图刷新/时延、机器人速度、可用停车距离、MPC 对频繁换参考的适应性和 CPU 负载评估。它不由栅格分辨率直接算出，也不等于 stall_window。检测、停车和允许启动重搜索应解耦，不能让安全停止等节流到期；全链路响应与制动尚无实测安全保证。

## 6. 上游动态障碍是否已膨胀

**目前不能确认“已按车体膨胀”，反而在所给入口中没有看到显式启用膨胀层。**

- `H/src/dlil/launch/dlil.launch.py:223` 活动插件列表只有 `stvl_layer`，没有 InflationLayer，也没有配置 inflation_radius。`:211` 的外部 STVL YAML 加载被注释。
- STVL voxel_size=0.10 是体素离散尺寸，mark_threshold=2、voxel_min_points=3 是标记/过滤条件，不能据此认定完成车体半径膨胀。上游 Livox fusion 读到的主链路是时间配对、外参转换与合并，不是车体膨胀。
- 规划参数 `obstacle_expand_radius=0.10` 只在点云源转换中扩张；当前点云源 false。costmap 占据转换不应用该半径（`P/src/dynamic_obstacle_manager.cpp:59`、`:323`、`:391`）。
- MPC `N/mpc_follower/cfg/mpc_params.yaml:47` 配置 `ObstacleInflation=0.2`，源码 `src/mpc_controller.cpp:622`、`:741` 将其用于控制器内部距离/软代价；它不反向修改上游 `/costmap/costmap`，不能证明 A* 收到了已膨胀障碍，也不是硬碰撞约束证明。

边界：本轮没有读取 STVL 完整实现或活动插件参数，不能排除外部覆写、其他发布者或地图本身的预处理。建议下一步只读采集实际 costmap plugins/footprint/地图及消息，而不是直接叠加某个膨胀半径。本轮不要求用户先解释架构。

## 7. 失败后发布机器人位置：可作候选，但不建议冒充正常路径

用户提出：若标志位不能使调用者拒绝失败路径，发布到机器人当前位置。本轮仅评价，没有实施。

原代码事实：

- `P/src/plan_manager.cpp:1055` 搜索失败直接返回，不撤销旧 `/sPath`；`:1059` 已在平滑前发布 reachable=true，不代表最终路径通过安全检查。
- MPC `N/mpc_follower/src/mpc_controller.cpp:452` 收到 Path 就先清空旧参考，再装入全部点，不读取规划 reachable 作为门控。`:986` 对空参考持续发布零控制。
- MPC `:880` 按参考末点与当前位置距离判 reached；阈值配置为 0.20 m（`cfg/mpc_params.yaml:38`）。单点当前位置路径可能因此触发 reached=true，把“规划失败等待”误报成“到达”。延迟或机器人移动后，也不能保证仍保持到达判定。
- `N/path_publisher/src/path_publisher.cpp:20` 对少于两个点的路径直接忽略，并保留旧缓存。因此单点、空 Path 都不会自动撤销旧 `/Path_downsampled`。两个重合点虽可越过此门槛，仍有伪到达、退化路径和参考点过时的问题。

**建议的分层方案（待确认）**：

1. 不把失败伪装为目标到达。明确区分正常参考、参考无效/等待、到达目标；当前 reachable 只能作为状态线索，不能独自保证控制停止。
2. 若局限于当前 MPC 源码，显式发布空 Path 比伪造单点更接近“清除旧参考”的语义：它会走零控制分支，空参考时 reached=false。前提是所有相关消费端统一空消息清除语义，尤其 path_publisher；消息接收、命令转发和实际停车仍需验证，不能直接宣布全链可用。
3. 若允许后续修改接口行为，增加控制器侧无效参考/停止保持及超时保护，等待安全的新参考后恢复；失效与新参考要有可关联的版本/顺序语义，避免独立状态与路径到达次序不一致。停车不受 1.5 s 搜索节流阻挡。
4. 新规划失败是否撤旧路径应按旧路径在最新环境下是否仍可认证区分。已知旧路径失效或安全性未知时，不静默继续跟随；只有仍经验证有效才讨论限时保留。最终输出（包括平滑曲线）也应通过相应安全检查后再声明有效。

## 8. 验证与保护记录

- 本轮只新增本报告及索引链接，保留 `algorithm-and-behavior-qa.md` 已有三行删除，不改原工程、移植副本或固定清单，不提交。
- `python3 scripts/verify_baseline.py`：当前规划包 62 文件、历史证据 25 文件、反向重建原导入 58 文件均通过；随后在 `docs/original-path-searching/algorithm-and-behavior-qa.md` 的 index/worktree 比对失败。差异是已有文档末尾三行删除；未静默刷新基线，也未恢复用户修改。
- 当前 Git 已有提交 `47b16dd`，与 2026-10-25 历史文档中“尚无 HEAD”的状态不同。本轮没有创建提交，也不把历史暂存描述作为当前事实。
- 未运行算法测试、ROS 联调或实机验证。尚需运行核对的重点：实际消息分辨率/frame/origin、TF 与地图配准、全量/增量 QoS 连通和窗口移动后的清除、实际障碍膨胀范围、最终参考无效时的整链停止行为。

### 增量接收的直观说明与代码现状

全量消息相当于“整张局部地图连同尺寸、分辨率和 origin 一起发送”；增量消息相当于“只发从第 x 列、第 y 行开始的一块矩形，替换上次地图中的这部分”。它不表示障碍速度或运动预测。

`P/src/dynamic_obstacle_manager.cpp:110` 保存最近全量的 metadata 并重建该源集合；`:126` 增量用最近 metadata，将局部 x/y 矩形对应的格逐个插入或删除（`:425`～`:435`）。尚无全量时增量直接跳过。规划端两类订阅见 `P/src/plan_manager.cpp:200`～`:207`，均要求 reliable+transient_local。

**待验证而非已证实故障**：滚动窗口换 origin 时上游是否及时发全量、发布 QoS 是否匹配、丢失/重启后的重同步；当前接收端没有用 header 或版本号验证增量属于哪一个 origin 的快照，也没有对短于矩形面积的数据直接拒收（consumeCostmapRegion 使用 min 长度）。不能因“代码有增量回调”就认为真实增量链路已验证，或认为动态障碍清除已正确。
