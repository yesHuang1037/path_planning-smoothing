# 原 path_searching 探索记录

归档说明（2026-10-26）：本文从 `docs/original-path-searching-exploration.md` 迁入本目录；保留首次探索的日期和验证范围。进一步的七问核对见 [算法与行为问答](algorithm-and-behavior-qa.md)。

日期：2026-10-25。用途：理解原工程的阅读笔记，**不作为后续新开发的需求、架构规范或兼容性承诺**。不表示决定继续在原工程上修改功能；本轮不改变既有开发计划或业务代码。

## 1. 范围与证据等级

- **已确认（静态代码）**：实际阅读 `/home/hustlyrm/source/navigation_ws/src/path_searching`，并沿本地启动脚本阅读目标、地图、控制和转发侧源码。下文不是对移植副本的描述。
- 原导航仓库 HEAD：`2e7c75d2253ced85789bd835fd7831459bd7f046`；它不是纯提交快照：规划 YAML、MPC 代码/参数等已有工作树修改，本轮按当前磁盘内容阅读并保留。
- `transplant_ws/src/path_searching` 已有后续化简实现，不能把其新增算法和五项测试倒算为原工程已有功能。
- **未验证（运行）**：没有启动 ROS 节点、构建、运行算法测试、采集活动话题或实机执行；源码接线不证明当前机器正在运行相同二进制。
- 本文的 `N/` 表示 `/home/hustlyrm/source/navigation_ws/src/`；`P/` 表示 `N/path_searching/`；`L/` 表示 `/home/hustlyrm/slam_source/odom_ws/src/dlil/`。定位中的行号对应本次阅读版本。

## 2. 先建立整体认识

本包的运行中心是 `PlanManager`：它既管理事件，又调用搜索、环境和路径处理算法；核心算法不是各自独立运行的 ROS 节点。

```text
上游：静态地图 + 动态障碍地图/点云 + 目标 + 机器人位姿
                         |
                         v
                   PlanManager
          +--------------+----------------+
          |                               |
     环境与事件管理                     一次规划
     动态障碍合并                  A* 栅格路径搜索
     路径通行检查                        |
     重规划触发/节流                抽点 + 迭代平滑
          |                               |
          +---- 再次触发规划          三次样条采样
                                          |
                                   /sPath 几何路径
                                          |
                     +--------------------+------------------+
                     v                    v                  v
                 MPC 跟踪器          region_detector    path_publisher
                     |                区域行为判断        路径降采样输出
                  /cmd_vel
                     |
                vel_forwarder
                     |
                /ly/navi/vel -> 外部驱动/底盘（本轮未验证末端）
```

A*寻找栅格路线。MPC在下游计算跟踪速度。**本包不是底盘控制器，也不发布包含到达时刻、速度和加速度的定时轨迹。**

## 3. 入口、构建和文件导航

| 入口/文件 | 代码确认的职责 |
| --- | --- |
| `P/src/plan_manager.cpp:1446` | `main()` 创建 `PlanManager` 并 `rclcpp::spin`；构造默认节点名 `navi_planner`，所查 launch 覆盖成 `plan_manager` |
| `P/CMakeLists.txt:1` | C++17、ament_cmake；静态库 `path_searching_lib`，可执行文件 `plan_manager`、`esdf_benchmark` |
| `P/package.xml:1` | ROS 消息/TF 依赖和包元数据；版本 0.0.1；描述、维护/许可证字段尚有占位内容 |
| `P/launch/astar.launch.py:10` | 启动规划节点；参数文件可由 `param_file` 覆盖，默认从包安装路径反推工作区后读取 `src/path_searching/cfg/plan_param.yaml` |
| `P/launch/debug_astar.launch.py:104` | 返回列表只启用规划节点；文件虽定义地图服务器、RViz（机器人可视化工具）、静态变换等，不能据此说它们已启动 |
| `P/cfg/plan_param.yaml:1` | 当前工作树参数，使用 `/**` 匹配节点；与 C++ 构造默认值不一致，不可混读 |
| `P/include/`、`P/src/` | 算法接口和实现；`include/cubic_spline/` 是头文件形式的样条实现 |
| `P/test/`、`P/src/benchmark_esdf.cpp:9` | 三个独立测试程序、一个环境距离查询基准程序，不是完整系统测试 |

主要依赖：`rclcpp` 节点接口，`nav_msgs/map_msgs/geometry_msgs/sensor_msgs/std_msgs` 消息，TF2（坐标变换库），Eigen（线性代数），PCL（Point Cloud Library，点云处理库），OpenCV（图像处理库）。实际构建要求 OpenCV，但 `package.xml` 未列出对应依赖；launch 还引用 Nav2 地图/生命周期节点和 RViz，清单未完整表达这些运行工具依赖。这里只记录可移植性疑点，不修配置。

## 4. 核心模块怎样协作

| 模块 | 接收/返回 | 代码事实与位置 |
| --- | --- | --- |
| `PlanManager` | ROS 消息、事件 → 调用算法并发布结果 | `P/src/plan_manager.cpp:35`；集中保存目标、当前路径、环境和各协调模块 |
| `GridGeometry` | 世界坐标 ↔ `[行, 列]` 索引 | `P/include/path_searching/grid_geometry.h:13`；采用消息分辨率与原点平移，索引转世界坐标不加半格 |
| `ESDF_enviroment::esdf` | 二值占据、动态栅格 → 碰撞/距离/最近障碍查询 | `P/src/esdf_map.cpp:64`；ESDF（Euclidean Signed Distance Field，欧氏有符号距离场）是常见术语，但此实现实际暴露的是到障碍距离和独立占据标志，不能仅凭类名认为占据内部有负距离 |
| `DynamicVoronoi`、`BucketPrioQueue` | 障碍占据变更 → 距离传播/最近障碍信息 | `P/src/dynamicvoronoi.cpp:97`、`:193`；Voronoi（沃罗诺伊，按最近障碍划分区域）相关实现藏在环境内部；A* 并不是沿 Voronoi 骨架搜索 |
| `DynamicObstacleManager` | 全量/增量 costmap、地图系点云、临时禁行区 → 合并动态栅格 → 环境更新 | `P/src/dynamic_obstacle_manager.cpp:110`、`:472`；静态障碍单独保存，清动态障碍时不清静态障碍 |
| `PathSafetyChecker` | 当前路径、位置、环境 → 前视/全路径检查、截断结果 | `P/src/path_safety_checker.cpp:50`、`:88`；净空为到障碍的距离，要求值由内接半径加余量构成 |
| `ReplanCoordinator` | 原因、时间、起终点 → 允许/禁用/节流等决策 | `P/src/replan_coordinator.cpp:183`；也维护停滞判断；它本身不运行 A* |
| `Astar` | 起终点栅格 → 搜索状态、栅格路径 | `P/src/Astar.cpp:137`；八邻域，步长加距离障碍代价，启发项权重 2；不是默认单位启发权重的最短路保证 |
| `smoother` + `GenTraj` | 世界坐标折线 → 采样曲线 | `P/src/smoother.cpp:6`、`:117`，`P/include/cubic_spline/cubic_spline_ros.h:39`；先抽取关键点，再十轮迭代调整，最后三次样条按约 0.10 m 弧长间隔采样；不是 MinimumSnap 算法 |

### 一次规划的实际顺序

1. 首次 `/map` → `map_callback()`：保存尺寸/原点/分辨率，把所有非零值（含未知值 -1）作为占据，初始化距离场并给搜索、平滑、检查模块接入同一个环境；后续静态地图消息直接忽略（`P/src/plan_manager.cpp:858`）。
2. `/goal_pose` → 转到 `map` 的二维位置 → 更新目标状态；首次或相对上次目标移动大于 0.05 m 才走新目标分支。先取机器人位姿，太近的目标有跳过逻辑（`:1196`）。目标朝向未进入规划约束。
3. `triggerReplan()` → `ReplanCoordinator::request()` 判定 → `plan()`。动态回调与定时器也可走此入口；并非收到任何障碍更新都必然重规划（`:372`、`:1308`）。
4. `plan()` 转索引并检查边界；目标不安全时可选择附近替代目标；A* 可在起点占据或净空不足时暂时放宽约束。仅首轮失败且起点被占据时，管理器另有扩大脱困窗口重试（`:973`）。
5. A* 成功 → **先发布** `reachable=true` → 取栅格路径 → 转世界坐标并补起终点 → `smooth(..., 0.3, 0.04)` → 空结果回退原折线 → 补最终端点 → 保存 `path_now` 并发布 `/sPath`（`:1054`、`:1067`、`:1088`、`:1109`）。
6. 后续定时器清理临时禁行区、取位姿、检查当前路径/停滞，再按允许原因重规划。可选截断先发布短路径，下一周期再申请重规划。**未见每次发布前统一认证最终样条及补接端点段的调用。**

## 5. 上下游接口和配置上下文

QoS（Quality of Service，消息传输质量策略）影响发布订阅能否连接；`reliable` 是可靠传输，`transient_local` 表示订阅者可请求发布者保留的历史样本。

| 方向/接口 | 类型与行为 | 静态来源/去向与限制 |
| --- | --- | --- |
| 输入 `/map` | `nav_msgs/msg/OccupancyGrid`；深度 1，reliable + transient_local | 所查 `L/launch/dlil.launch.py:175` 配置 Nav2 map_server；只接纳首张，未根据地图 header 做 TF 转换 |
| 输入 `/goal_pose` | `geometry_msgs/msg/PoseStamped`；深度 1 | 工程有 `navigoal_manager` 发布者（`N/navigoal_manager/src/manager.cpp:40`），但所查总脚本未启动它；实际目标源未确认。空 frame 当 map，非 map 按最新 TF 转换，不按目标时间戳对齐 |
| 输入 `/costmap/costmap` | `nav_msgs/msg/OccupancyGrid`；深度 1，reliable + transient_local | 话题可配置；全量地图缓存几何信息，栅格中心映射到静态图；读取原点平移，未用原点旋转和 frame 做转换 |
| 输入 `/costmap/costmap_updates` | `map_msgs/msg/OccupancyGridUpdate`；深度 10，reliable + transient_local | 话题可配置；必须先收到全量地图；运行时发布者的 durability 是否匹配未核实，不能宣称增量订阅已连接 |
| 输入 `/cloud_livox_obs` | `sensor_msgs/msg/PointCloud2`；深度 1 | 可按最新 TF 转 map，再投影二维并圆形扩张；当前 YAML 关闭此来源，实际发布者未追定 |
| 隐式输入 TF | `/tf`、`/tf_static`，由 TF2 监听器接入 | 优先 `map <- base_link` 或 `baselink`；代码还尝试经 `odom`/`lidar` 组合变换。没有 odometry 消息订阅，也不发布定位 TF |
| 输出 `/sPath` | `nav_msgs/msg/Path`；深度 1，默认非历史保留发布 | frame 固定 map；仅 x/y，有统一当前时间，四元数设单位姿态，不携带切线航向、逐点到达时刻或速度；迟启动消费者可能等不到先前路径 |
| 输出 `/ly/navi/reachable` | `std_msgs/msg/Bool`；深度 1 | A* 成败标记，不是已到达标记；提前返回分支不一定发布它；MPC 没有订阅该状态 |

所查启动脚本 `/home/hustlyrm/begin_serial_mpc.sh:2` 加载导航与雷达环境，启动规划 `debug_astar.launch.py`、MPC、region_detector、vel_forwarder、path_publisher。**脚本本身不启动地图/定位，也不启动 navigoal_manager 或 speed_manager。**

外部地图/感知候选链路：`L/launch/dlil.launch.py:34` 默认地图是 `/home/hustlyrm/slam_source/shared_maps/map.yaml`；`:205` 的 costmap 使用 STVL（Spatio-Temporal Voxel Layer，时空体素障碍层），从 `/dlio/odom_node/pointcloud/deskewed` 标记/清除障碍。配置为 map 系、8×8 m 滚动窗口、0.1 m/格、更新/发布 10 Hz，仅列 STVL，未列独立 inflation layer（障碍膨胀层）。这不是已经验证的真实障碍尺寸或安全膨胀证明。

运行身份仍有歧义：`/home/hustlyrm/slam_source/start_dlil_hyx.sh:3` 实际加载 `odom_hyx/install/setup.sh`，而本次调查定位源码是 `odom_ws`。不能拿其中一套源码自动代表当前运行版本。

下游不只一个消费者：

- `N/mpc_follower/src/mpc_controller.cpp:39` **直接订阅 `/sPath`**，不是经过 path_publisher。`:452` 替换全部参考点，读取 x/y 和 yaw，不按 Path 时间戳执行参考；速度受自身参数、`/setFollowSpeed` 等影响。输出 `/cmd_vel`（`geometry_msgs/msg/Twist`），到达状态另发 `/ly/navi/reached`。
- `N/region_detector/src/region_detector.cpp:17` 订阅路径和速度级别，发布 `/ly/navi/should_rotate` 等；MPC 的该话题回调实际切换障碍代价开关，不能仅按话题名理解成直接控制旋转。
- `N/path_publisher/src/path_publisher.cpp:6` 独立订阅路径，最多抽成 50 点，每 100 ms 发布 `/Path_downsampled`，不是 MPC 前置平滑器；少于两点的更新被忽略。
- `N/vel_forwarder/src/forwarder.cpp:35` 把 `/cmd_vel` 的二维线速度映射成自定义 `gimbal_driver/msg/Vel`，输出 `/ly/navi/vel`。具体串口消费和底盘执行端本轮未确认。
- 工程还存在 `path_following` 的其他跟踪接线，但所查总脚本未启动，不能画成和 MPC 同时执行的确定链路。

### 当前参数实际意味着什么

| 当前 YAML 内容 | 结合代码后的含义 |
| --- | --- |
| `avoidance_mode: global_monitor` | 禁用动态阻塞/碰撞原因重规划和截断；**不关闭动态环境更新，也不关闭周期/停滞重规划**。因此周期搜索仍可能受到动态障碍影响 |
| 点云源 false、costmap 源 true、clearance fusion false | 动态 costmap 仍写进 ESDF；关闭“额外 costmap 净空融合”不等于忽略动态障碍 |
| `obstacle_cost_weight: 10.0` | 活动 A* 代价含 `10/(距离格数+0.1)`；离障碍越近代价越高 |
| `dynamic_penalty_weight: 0.0` | 参数确实声明/保存，但当前活动 A* 代价表达式没有使用该成员；不要把它说成已有独立动态惩罚项 |
| 半径 0.01 m、净空余量 0、碰撞检查半径 0.10 m | 不同检查使用的阈值不完全相同；这些数字不证明真实车体只有这么大 |
| 监控周期 0.50 s、最小重规划间隔 1.5 s、periodic true | 定时器轮询不等于每周期都执行搜索；申请仍受节流等判定影响 |
| `enable_downstairs: false` | 有高度图与阶梯检查函数，但活动 A* 的 `checkUpStairs` 调用已注释；不能称当前搜索已施加地形高度约束 |

上述是磁盘参数与代码事实，不是活动节点参数 dump，也不是推荐设置。

## 6. 已知边界、测试与未接通的功能

- **坐标与地图边界（确认）**：管理器/GridGeometry 使用消息分辨率；环境 `Index2pos/Pos2index` 却固定乘除 20，平滑抽点也固定 0.05（`P/include/esdf_map.h:44`、`P/src/smoother.cpp:130`）。原点旋转未纳入这些换算；不能声称全链路支持任意分辨率或旋转地图。静态未知格按占据处理，动态未知格不按超过阈值的障碍插入，语义不同。
- **搜索与最终输出边界（确认）**：搜索进入终点的一格邻域就可成功（`P/src/Astar.cpp:195`），管理器再补接目标；起点脱困刻意放宽约束。局部检查和平滑候选点检查不构成整条最终曲线安全认证。
- **失败协议（确认）**：A* 无路时发 false 后返回，不清旧 `path_now`，不发布空 `/sPath`；平滑之前已发 true。由此可推断下游可能继续旧参考，但是否实际发生、如何停车需运行验证；不能把 reachable=false 等同全车停车。
- **生命周期风险（确认代码关系，未运行复现）**：ESDF 与 DynamicVoronoi 共享同一 `bin_map/gridMap`，各自析构均释放该图（`P/src/esdf_map.cpp:226`、`P/src/dynamicvoronoi.cpp:29`、`:97`）。存在重复释放风险；两个环境相关测试用空 deleter 避开析构，不能证明退出安全或无泄漏。
- **部分预留/未接通功能（确认）**：Manual、MapUpdate 触发原因存在于协调器，但未见管理节点外部手动服务/动作入口；静态地图更新被直接忽略。动态独立惩罚、阶梯约束如上，并非仅调整参数就会生效。是否未来需要它们未决，不直接定性为必须补齐的需求。
- **路径处理异常边界（待测）**：短路径、重复点、零距离避障向量、越界平滑点对算法的影响，现有测试不能覆盖；这是需要用例验证的问题，不记录为必然崩溃事实。
- **现有测试（确认注册，未运行）**：`P/CMakeLists.txt:105` 起在 `BUILD_TESTING` 下注册 `replan_coordinator_periodic_test`、`path_safety_checker_truncation_test`、`astar_start_escape_test`。分别检查周期触发、截断、起点净空不足脱困的特定场景；使用普通程序和 `assert`，测试时需避免断言被 `NDEBUG` 禁用。
- **测试方式（阅读所得，非已执行）**：后续可在批准的隔离构建中以 Debug 和 `BUILD_TESTING=ON` 编译，再运行 CTest（CMake 测试执行器）或 `colcon test --packages-select path_searching`；`esdf_benchmark` 只测距离更新/查询与对照查询，不能证明 A*+平滑+通信+跟踪端到端性能。
- **覆盖缺口（确认本包注册范围）**：未见最终样条安全、ROS 消息/QoS 连通、定位时间同步、失败撤销旧参考、全链路停车和实机跟踪的自动验收测试。移植副本历史测试不填补这些缺口。

## 7. 少数最值得澄清的问题

1. **到底要参考哪个运行版本？** 实机加载 odom_ws 还是 odom_hyx，导航包安装产物是否对应当前工作树？实际目标发布者是谁？先确认源码与活动图，避免研究错版本。
2. **新开发要理解/沿用哪种结果语义？** 几何路线、可安全跟踪路线还是定时轨迹，是不同职责；原项目的 `/sPath` 只是几何参考。这里只提出问题，不约束新项目必须兼容。
3. **失败与安全由谁负责？** 最终曲线检查、无路后的旧参考撤销、停车及真实车体尺寸/地图膨胀，原链路没有已验证的一致保证；这些比先选新搜索算法更值得明确。

## 8. 待验证假设与建议下一步

| 假设（不是结论） | 最小核验方式 |
| --- | --- |
| 本地总脚本代表实际启动方式，感知/地图来自独立 DLIL 链路 | 用户确认入口后，只读采集活动进程、包解析路径、节点列表与参数 |
| costmap 全量/增量都能被规划器接收且坐标一致 | 对照活动话题类型、发布订阅 QoS、frame、原点、更新计数；特别核查增量 durability |
| 旧路径在规划失败后可能继续被执行 | 在不驱动底盘的受控记录/回放中观察失败后的 Path、控制器参考和速度，不能直接在实车冒险验证 |

建议先选一个普通目标，按“输入地图/TF → 搜索折线 → 抽点 → 样条 → MPC 参考”逐层阅读一个实例；之后再决定是否采集运行快照或增加离线观测。**本轮停在调查汇报，不实施新功能，不建立完整项目规范。**

本轮只新增本文。候选基线固定清单核验 `python3 scripts/verify_baseline.py` 通过：既有规划副本 62 文件、历史证据 25 文件、反向重建原导入 58 文件及既有暂存/导出 105 文件；它不等于验证本文所述原工程算法，也不包含本文。没有重跑构建、CTest、性能或实机测试。
