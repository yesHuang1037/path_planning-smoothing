# T01 交付：现有导航调用链与 TDT 移植边界

日期：2026-10-24。依据：[当前需求](requirements.md)，不是早期草案中的独立接入方案。

**结论：T01 的静态源码调查与移植对照已完成；实机运行版本、运行时接口和安全策略仍有待确认项。未开始 T02，不代表第一阶段实现或实车验收完成。** 本轮只新增本文并更新需求状态索引，不修改已完成的导航、里程计、TDT 源码或配置，不启动控制链路。

## 1. 调查基线与保护范围

| 对象 | 实际位置 / 基线 | 检查结果 |
| --- | --- | --- |
| 本工作空间 | `/home/hustlyrm/path_planning_transplant` | 根目录不是有效 Git 仓库，不能给根目录生成 Git diff；文档以文件内容核对 |
| TDT | `tdt-nav-kit/`，`main`，`814d5ce0237acc7faa8c070e5c5e61e2ddc81996` | 原有 `scripts/setup.sh` 仅权限从 100755 变为 100644；`.marscode/`、`osqp/`、`osqp-eigen/`、`tmp/` 未跟踪，全部保留 |
| 导航 | `/home/hustlyrm/source/navigation_ws`，`Navigation`，`2e7c75d2253ced85789bd835fd7831459bd7f046` | 原有 7 个修改文件：README、MPC 参数/头文件/实现、规划参数、区域参数、速度参数；调查以工作树为准，不以 HEAD 替代 |
| 里程计 | `/home/hustlyrm/slam_source/odom_ws`，所属仓库为 `/home/hustlyrm/slam_source`，`Slam`，`4bf3302d8a826d25cd68410f0a00511cd6957477` | 原有地图增删、DLIL/DLIO 配置及源码修改、未跟踪脚本等；全部只读 |

证据路径缩写：`N/` = `/home/hustlyrm/source/navigation_ws/src/`；`O/` = `/home/hustlyrm/slam_source/odom_ws/src/`；`T/` = 本工作空间 `tdt-nav-kit/`。行号是本轮读取的工作树位置。

导航和指定 odom_ws 未找到 `.understand-anything/` 图；未借用其他备份项目的图来证明当前链路。下一节是本轮形成的持久调用关系图；代码和活动 launch 优先于图及注释。

## 2. 实际启动边界与调用图

### 2.1 入口追踪

`/home/hustlyrm/begin_serial_mpc.sh:2` 先 source `navigation_ws/install/setup.sh`，再 source `livox_ws/install/setup.sh`，启动：

| 入口 | 活动实现 / 配置 | 边界 |
| --- | --- | --- |
| `path_searching debug_astar.launch.py` | `N/path_searching/launch/debug_astar.launch.py:29` → `plan_manager`；从 navigation_ws 的 `src/path_searching/cfg/plan_param.yaml` 读取参数 | 返回列表只启用规划节点；文件中定义的 map_server、生命周期管理、RViz、静态 TF **未加入返回列表**，不能算已启动 |
| `mpc_follower mpc_follower.launch.py` | `N/mpc_follower/launch/mpc_follower.launch.py:11` → `mpc_node` → `MpcController`，读取 `cfg/mpc_params.yaml` | MPC 直接消费 `/sPath`，不是消费降采样后的路径 |
| `region_detector region_detector.launch.py` | `N/region_detector/src/region_detector.cpp:17` | 读 `/sPath`、速度等级和 TF；发布方向区域及 `/ly/navi/should_rotate` |
| `vel_forwarder forwarder_node` | `N/vel_forwarder/CMakeLists.txt:34` → `src/forwarder.cpp` | 是直接 `ros2 run`，没有执行其 launch 中的 remapping，也没有启动 debug forwarder |
| `path_publisher path_publisher` | `N/path_publisher/src/path_publisher.cpp:6` | `/sPath` 的旁路消费者，每 100 ms 发布最多 50 点到 `/Path_downsampled`；不是 MPC 中间适配器 |

规划和 DLIL 的安装 launch 是指向各自源码的软链接；导航安装可执行文件指向 build 产物。**软链接不证明现有二进制已经包含当前工作树的全部修改**，本轮也未在实机验证 source 叠加后的最终包解析结果。

### 2.2 源码关系图

```mermaid
flowchart TD
    L[点云 CustomMsg + IMU] --> D[DLIL OdomNode / 先验 PCD 配准]
    D --> TF[odom → livox 动态 TF]
    ST[DLIL launch 静态 TF] --> MT[map → odom → livox → base_link]
    TF --> MT
    D --> PC[deskewed PointCloud2 / odom]
    PC --> STVL[Nav2 costmap / STVL 插件]
    MS[DLIL launch map_server / PGM+YAML] --> MAP[/map]
    MAP --> PM[PlanManager]
    STVL --> CM[/costmap/costmap + updates]
    CM --> DM[DynamicObstacleManager]
    DM --> ES[原静态图 + 动态格 / ESDF]
    PM --> ES
    G[/goal_pose / 实际发布者待运行确认] --> PM
    MT --> PM
    PM --> RC[ReplanCoordinator → plan]
    RC --> A[Astar::search / getPath]
    ES --> A
    A --> S[smoother::smooth / 化简+迭代+三次样条]
    ES --> S
    S --> P[/sPath / 几何 Path]
    P --> MPC[MpcController]
    CM --> MPC
    MT --> MPC
    P --> RG[region_detector]
    RG --> ROT[/ly/navi/should_rotate]
    ROT --> MPC
    P --> PP[path_publisher → /Path_downsampled]
    MPC --> V[/cmd_vel / body vx vy]
    V --> F[forwarder_node]
    F --> OUT[/ly/navi/vel / gimbal_driver Vel]
    OUT -.未在上述工程中找到消费实现.-> HW[串口与底盘 / 本轮边界外]
```

`PlanManager`、A*、平滑器、ESDF、动态管理、安全检查和重规划协调是**同一规划进程中的 C++ 调用**，不是各自独立 ROS 节点；`N/path_searching/CMakeLists.txt` 将相关实现编入 `path_searching_lib`，由 `plan_manager` 链接。MPC 是另一个进程，使用 NLOpt；不应为了 TDT 的 OSQP 后端更换 MPC 求解器。

### 2.3 DLIL 主线与发现的版本差异

- 指定主线是 `O/dlil/launch/dlil.launch.py:107` → `dlil/dlio_odom_node`，不是同目录下的 `direct_lidar_inertial_odometry` 或 `dlio_hardened`。默认点云 `/merged/pointcloud` 为 `livox_ros_driver2/msg/CustomMsg`，IMU `/livox/imu_192_168_1_5` 为 `sensor_msgs/msg/Imu`；源订阅见 `O/dlil/src/dlio/odom.cc:61`。
- DLIL 构造时加载先验 PCD，失败会抛异常；默认 `odom/prior_map_path` 为 `/home/hustlyrm/slam_source/shared_maps/dlio_map.pcd`，GICP 使用该地图。该参数与 launch 的 `map_pcd_path` **不是同一传参入口**：后者用于本文件中未启用的 localizer。证据：`O/dlil/src/dlio/odom.cc:30`、`:290`、`:921`；`O/dlil/launch/dlil.launch.py:152`、`:317`。
- `O/dlil/cfg/params.yaml` 中 `frames/odom=odom`、`frames/baselink=livox`；`publishPose()` 发布 `odom → livox` TF 及 `/dlio/odom_node/odom`。`publishCloud()` 把点云变换后标为 `odom`，发布 `/dlio/odom_node/pointcloud/deskewed`。证据：`O/dlil/src/dlio/odom.cc:412`、`:548`。不要因变量名 baselink 就把该位姿当成 `base_link` 位姿。
- launch 实际启用静态 `map → odom`（零变换）及 `livox → base_link`（平移 `[0,-0.196,0]`、零旋转），形成用户描述的 TF 链。`dlio_map_node`、localizer、gridmapper、保存 PCD 定时器未启用；不能把 `/initialpose` 配准算作本主线活动功能。证据：`O/dlil/launch/dlil.launch.py:275`、`:286`、`:294`。
- **待确认运行版本，不据此改方向：** 本机额外找到 `/home/hustlyrm/slam_source/start_dlil_hyx.sh:3`，它 source 的是 `odom_hyx/install/setup.sh`，不是用户指定 odom_ws。比较两边 DLIL launch，差异为默认地图 YAML/PCD 路径；不能由此推断两边源码、参数和二进制全部一致。本文仍以用户指定 odom_ws 为主线，不自动切换到 odom_hyx。

## 3. 输入、地图与规划触发的实际语义

| 项目 | 源码已确定 | 对移植的约束 |
| --- | --- | --- |
| 静态地图 | `/map`：`nav_msgs/msg/OccupancyGrid`，规划订阅 reliable + transient_local、深度 1；只接收首次地图，非零（包括 -1 未知）全部转为占据 | 不直接把 0/100 占据值当作 TDT 灰度；不要声称当前已支持静态图滚动/重载 |
| 坐标 | `map` 米制；`GridGeometry` 使用 `[row,col]=[y,x]`、`floor((p-origin)/resolution)`，逆变换取格角，不加半格；仅使用原点平移，未使用原点姿态 | TDT 使用 `(x,y)`，必须明确行列转换；不能照搬 PNG 示例的 Y 翻转；带旋转原点不是现有已支持能力 |
| ESDF | 静态占据 + DynamicVoronoi 距离场；动态替换清理旧格但保留静态占据 | 保留现有职责，不另外重建感知/动态融合节点 |
| 动态来源 | 当前 `use_pointcloud_dynamic_obstacles=false`、`use_costmap_dynamic_obstacles=true`；全量 `/costmap/costmap` 和增量 `/costmap/costmap_updates` | `/cloud_livox_obs` 虽建了订阅但未作为当前启用源 |
| 动态转换/清除 | 全量帧清空旧 costmap key 集合重建；增量在区域内插入/删除；使用动态图格中心转到静态图索引；未知归一化为 -1，不插入；默认占据阈值 65 | 要让搜索、化简、走廊和平滑检查读取同一次合成状态，不只给 TDT 静态 PNG；完整动态格几何覆盖仍需用输入验证 |
| 目标 | `/goal_pose`：`geometry_msgs/msg/PoseStamped`，深度 1；空 frame 当 map，其他 frame 用最新 TF 转换；本链只取 x/y | 不把目标 yaw 或消息时间戳写成已执行的终端约束；未收到地图、TF 失败、距离起点 <0.2 m 会跳过；目标位置变化 >0.05 m 或首次触发 |
| 状态 | 规划从最新 `map ← base_link`（候选 baselink）TF 取位置；有 odom/lidar 锚点回退组合；不订阅 DLIL Odometry 速度 | MinimumSnap 若要运动中速度衔接，不能假设已有速度直接传入规划器 |
| 重规划 | `goal_callback → triggerReplan → ReplanCoordinator::request → plan`；另有手动、周期、停滞、动态检查等原因，受模式和节流约束 | 保留协调器；当前参数 `global_monitor` 会关闭动态阻塞/碰撞触发及截断，但仍有周期、停滞等触发，不能按注释推断为全局主动动态避障模式 |

证据：`N/path_searching/src/plan_manager.cpp:190`、`:300`、`:380`、`:743`、`:828`、`:858`、`:1114`、`:1196`、`:1310`；`N/path_searching/include/path_searching/grid_geometry.h:43`；`N/path_searching/src/dynamic_obstacle_manager.cpp:110`、`:308`、`:323`、`:338`、`:504`；`N/path_searching/src/esdf_map.cpp:64`、`:130`；`N/path_searching/cfg/plan_param.yaml`。

一致性边界：规划 main 使用 `rclcpp::spin(node)`，`plan()` 同步执行，所查规划回调没有独立并行规划线程；本次计算不会同时由这些回调改图，但新目标/新障碍会在计算期间排队，未见贯穿算法的取消和发布前最新障碍复核。costmap 转换直接按消息原点算 map 坐标，未使用原点旋转或 frame TF，依赖发布端确为 map 系。增量订阅在规划侧要求 reliable + transient_local、深度 10，MPC 侧仅 reliable、深度 10；发布端实际 QoS 尚未核验，不能因话题同名就宣称两边均成功收到更新。

**分辨率与膨胀不能混为一谈：** 指定 DLIL launch 的 STVL 局部图是 map 系、8×8 m、0.1 m/格、滚动窗口，更新/发布配置均为 10 Hz；仅列 `stvl_layer`，没有显式 Nav2 inflation layer；有体素过滤、标记和清除（`voxel_decay=1.0`），不等价于按车体轮廓安全膨胀。静态 YAML `/home/hustlyrm/slam_source/shared_maps/map.yaml` 为 0.05 m/格、原点 `[-12.05,-20.15,0]`；图片是否预膨胀无法仅凭 YAML 证明。MPC 另有 `ObstacleInflation=0.2 m`；规划 clearance 是 `robot_inscribed_radius=0.01 m + clearance_margin=0`，点云扩张参数在点云源关闭时不证明活动 costmap 已扩张。以上都是**配置值，不是已验证的实车外轮廓**。

另有直接相关的原有一致性限制：`GridGeometry` 跟随消息分辨率，但 `N/path_searching/include/esdf_map.h:48` 的坐标换算固定乘/除 20，原平滑器还有 0.05 m 常量。指定静态 YAML 恰为 0.05 m，不能推广成任意分辨率已兼容。本轮仅记录，不修改原模块。

## 4. 搜索到执行的协议及失败处理

### 4.1 进程内搜索和平滑

`N/path_searching/src/plan_manager.cpp:973`：世界起终点 → 栅格索引 → 起点处理/终点替代搜索 → `planner_1.search`（可重试起点脱困）→ `getPath` → 转世界坐标并补实测起点/实际使用终点 → `smoother_1.smooth(path,0.3,0.04)` → 恢复端点 → `Path_pub`。A* 是八邻域，返回 `REACH_END/NO_PATH`，并保留障碍距离/动态代价与脱困语义，不是当前缺少普通 A*。

现有平滑器并非空缺：`N/path_searching/src/smoother.cpp:6` 先 `pathResampling`（直连碰撞及距离判据）→ 10 轮平滑/障碍梯度调整 → `GenTraj(...,0.10)` 三次样条。`N/path_searching/include/cubic_spline/cubic_spline_ros.h:35` 的采样变量是累计路径长度，不是秒；输出第三分量是曲率，调用方仅保留 x/y。该流程没有独立 SFC 矩形走廊模块，也没有向 MPC 发送定时轨迹。

### 4.2 ROS 参考与控制

| 边界 | 实际契约 |
| --- | --- |
| 规划 → MPC | `/sPath`：`nav_msgs/msg/Path`；发布深度 1、MPC 订阅深度 10，默认 reliable/volatile。frame 为 map；单位 m；每点 header 与整条路径相同发布时间，姿态置单位四元数（yaw=0）。没有逐点相对执行时间、速度或加速度 |
| MPC 消费 | `onReferencePath` 清旧列表、读取 x/y/theta 并重置索引和到达标志；未依据 header 做坐标转换或定时执行。优化按最近路径点加索引（带取模）选后续参考；点密度变化会改变预测窗口覆盖距离，不能把 TDT 每段固定 11 点直接视作等价替换 |
| MPC 状态与限值 | map/base_link TF 差分得到 body vx/vy；`Ts=0.05 s`、`N=15`、目标速度 2.0 m/s，配置线速度轴向范围 ±2.5 m/s，线加速度上限 5.0 m/s²。当前优化变量只有 body vx/vy，预测航向保持不变；角速度/转弯半径参数的存在不等于正在优化底盘转向。实际轮系、合速度边界和物理限值仍待确认 |
| MPC → forwarder | `/cmd_vel`：`geometry_msgs/msg/Twist`，linear.x/y 为 body m/s，angular 未赋非零值；MPC 到达话题为 `/ly/navi/reached`（Bool），不是规划的 reachable |
| forwarder → 下游 | `/ly/navi/vel`：**`gimbal_driver/msg/Vel`**，不是包内未启用生成的同名 Vel。轴向裁剪 ±2.4 m/s、绝对值 <0.08 置零，再乘 50 转整数等级；不是仍以 m/s 传出。实际串口消费实现未在指定导航/DLIL 工程找到，本轮不推断硬件端停车/轴向约定 |

证据：`N/path_searching/src/plan_manager.cpp:743`；`N/mpc_follower/src/mpc_controller.cpp:34`、`:185`、`:296`、`:376`、`:452`、`:836`、`:974`；`N/mpc_follower/include/robot_model.h:27`；`N/mpc_follower/cfg/mpc_params.yaml`；`N/vel_forwarder/src/forwarder.cpp:5`、`:35`；`N/vel_forwarder/include/forwarder.h:5`。

限值解释需注意：当前 YAML 同时设置 `MinAccelLinear=5.0`、`MaxAccelLinear=5.0`，而 `accelerationConstraints`（`:262`）对加速度模长同时施加下界/上界，按当前公式等价于要求模长为 5.0，而非通常的 0～5.0。这是已有配置与实现的组合，尚未运行验证其影响；不据此擅改 MPC 或将其当作已确认的物理安全限值。

`/ly/navi/should_rotate` 的 MPC 回调实际切换障碍代价启用标志，不是设置角速度；见 `N/mpc_follower/src/mpc_controller.cpp:849`。`/setFollowSpeed` 可设置目标速度，但本入口没有启动 speed_manager，实际发布者待运行确认。

### 4.3 失败与旧参考：已定位，未擅改策略

| 情况 | 当前行为 | 对接风险 / 后续要求 |
| --- | --- | --- |
| A* NO_PATH / 成功 | NO_PATH 时 reset 搜索、发布 `/ly/navi/reachable=false` 后返回；不清 `path_now`，不发布空 `/sPath`；成功时在平滑之前发布 reachable=true | MPC 未订阅 reachable，可能继续旧参考；false 不等同整链停车，true 也不等同平滑和最终参考安全验证通过 |
| 平滑返回空 | 回退原 A* 路径；非空则补端点并发布 | 不能仅凭非空宣布最终曲线/端点连接安全；本次 plan 发布路径前没有无条件最终整段安全复核 |
| 新目标、无地图、TF 失败、过近目标 | 成功规划时以新 `/sPath` 替换；跳过/失败分支没有统一撤销旧参考 | 新算法失败需要明确保留/撤销旧参考条件，而非照搬“只不发新路径” |
| MPC 空参考、TF 查询失败、重复 TF 时间戳、已到达 | controlLoop 发布零速度；收到空 Path 会清参考 | 可以调查利用既有空 Path 语义，但尚未约定为整链撤销协议；path_publisher 会忽略少于 2 点消息并保留旧旁路路径 |
| 时间有效性 | 规划用最新 TF；MPC 有重复时间戳检查，但未见路径超龄撤销和完整 TF/动态图年龄门槛 | 不宣称已有完整超时停车/动态输入失效协议；forwarder 回调也无独立命令超时机制 |

证据：`N/path_searching/src/plan_manager.cpp:1042`、`:1093`、`:1196`；`N/mpc_follower/src/mpc_controller.cpp:452`、`:974`；`N/path_publisher/src/path_publisher.cpp:20`；`N/vel_forwarder/src/forwarder.cpp:35`。这些是本轮未改代码中的既有行为，**不是本次引入的问题，也不是已实测发生的事故**。

## 5. TDT 逐模块对照与建议

| TDT 内容 / 证据 | 现有对应位置 | 建议、理由与接口影响（均未执行） |
| --- | --- | --- |
| 地图/代价、Mask：`T/src/YAstar/yastar.cpp:404`、`:418`、`:749`、`:760` | `map_callback`、GridGeometry、ESDF、DynamicObstacleManager | **保留原职责，适配算法输入**。TDT 灰度重载 0 为障碍、非零自由；另有 OccupancyGrid 风格重载 0 自由、非零障碍，不能笼统说 TDT 完全没有 ROS 编码入口。若复制 YAstar，需把同次静态+动态合成占据同步给前后端；不启用其 Mask TTL 去替换既有清除规则 |
| YAstar 普通八邻域搜索：`T/src/YAstar/yastar.cpp:802` | `N/path_searching/src/Astar.cpp:137` 与 PlanManager::plan | **优先提取思路，是否整体类替换待决策**。原项目已有普通 A*、代价、起点脱困、终点替代及失败枚举；直接替换可能丢语义。TDT 无解/部分越界返回 `{start}`，边界检查还使用 `>`，不能以非空当成功；需映射到原 REACH_END/NO_PATH 并严格校验端点/越界 |
| DP + 弦化简：`T/src/YAstar/yastar.cpp:195`、`:239`、`:246`；`T/main.cpp:133` | `N/path_searching/src/smoother.cpp` 的 pathResampling | **建议最先借鉴的局部算法边界**。使用能查询当前合成占据/净空的连线检查，不调用不查障碍的 `YAstar::Tools` 几何版；保留实测起终点及现有输入输出类型，不新增 ROS 接口 |
| SfcSquare：`T/src/MinimumSnapOsqp/sfcSquare.cpp:71`、`:355` | 当前无独立矩形走廊；平滑器直接查 ESDF | **暂不直接搬运，T03 再适配**。活动实现 `(p+origin)/resolution` 与前端减原点不一致，开放边缘扩张有边界风险；指定静态地图原点非零，不能绕过。矩形只约束连接点，不保证整段曲线安全；不能以生成走廊替代曲线复核 |
| MinimumSnap：`T/src/MinimumSnapOsqp/minimumSnap.hpp:179`；`minimumSnap.cpp:237`、`:907`、`:996` | 原 smoother + 三次样条；输出仍由 Path_pub 构造 | **后端可替换候选，T03 决策后实施**。SolveOutput 是位置点列、success、求解耗时；每段固定 11 点，setDt 未驱动活动采样，time 不是轨迹持续时间。MPC 当前不消费时间/速度，因此先保持几何协议并匹配空间采样，不默认新增消息；若目标改为定时/速度跟踪须先暂停说明接口设计影响 |

TDT 平滑的额外落地条件：默认 `strictCollision=false` 有障碍容差，地图不可用时会跳过相应检查；速度/加速度参数用于自动时间分配，不是全程硬约束；迭代失败仍可能带点列，单点成功也不等于碰撞通过。OSQP 每轴有独立时间上限，外层 TL 不是贯穿预算。必须在 T03 处理状态判定、严格碰撞、曲线采样误差和预算，不能通过参数名宣称已满足实车安全及 15～20 ms。详见已有 [探索记录](../Exploration.md)，本轮对照了当前实现，不重复把示例测试当验收。

## 6. 下一项最小修改范围（T01 时点提案）

后续状态：本提案已获用户确认，T02-A 的隔离实现、测试与剩余问题见 [T02-A 交付](T02-delivery.md)。以下保留 T01 调查时点记录，不作为当前未实施状态。源码勘误：当前 ESDF 的 `cv::imwrite` 已注释，调试函数计算图片但不写 PNG，下文关于潜在写图副作用的提醒不代表当前会写图。

**T02-A：在隔离的可写集成副本中实现“保留原 A* 与 ROS 协议的碰撞感知路径化简”作为 T02 第一片。** 不在本轮或未经批准修改原 navigation_ws；这只是最小范围建议，不把它等同已经完成 YAstar 全量移植。

| 范围 | 建议涉及的原文件 / 约束 |
| --- | --- |
| 算法 | `N/path_searching/include/smoother.h`、`src/smoother.cpp`：在 pathResampling 对应位置提取 TDT DP/弦思路，沿用现有合成环境查询；不复制一套订阅、地图管理或控制器 |
| 接线 | 如需模式参数，仅限 `src/plan_manager.cpp`、`cfg/plan_param.yaml`；默认行为及接口保持兼容，原目录已有修改必须独立保留。不调整 avoidance_mode、车体限值或动态层职责以掩盖差异 |
| 验证设施 | `test/` 新增聚焦化简的回归用例，`CMakeLists.txt` 注册；保留既有 A* 脱困、重规划和截断用例。若直接提取类需要更广接口变更，先停止并说明，不在提案外扩展 |
| 明确不动 | DLIL、map_server/STVL、DynamicObstacleManager/ESDF 的公共契约、ReplanCoordinator、安全检查模块、MPC、区域检测、forwarder、消息定义和串口；整体 YAstar 与 MinimumSnap 的替换另行决策 |

可单独验收的结果：固定地图和同一原始折线下，保留首尾、不产生非有限点或越界，化简新连线满足当前占据与净空约束；覆盖非零原点、0.05 m 主线、窄通道、贴边、对角障碍、重复/空/单点输入、动态出现/清除。比较原 pathResampling 的点数和耗时；可视化不是唯一通过标准。测试应隔离 ESDF 原有绝对路径调试图片写出副作用，避免运行测试修改已完成导航目录。以上测试**尚未新增或执行**。

实施门槛：确认这一个切片及集成副本位置；明确起点脱困路径与严格连线判据冲突时的处理，不能静默删除既有脱困能力。若选择整体 YAstar 替换，则重新列出失败/代价/脱困兼容清单再实施，不将该选择视作默认已批准。

## 7. 状态与最少待决信息

| 状态 | 事项 | 后续动作 |
| --- | --- | --- |
| **已完成** | T01 入口追踪、地图/状态/目标/搜索/平滑/MPC/转发关系、五项 TDT 对照、最小后续范围 | 本文作为统一交付和知识图，不为每个模块再生成文档 |
| **已确定（源码）** | 已找到导航源码；有平滑器；MPC 接口是几何 Path；动态层由 STVL→costmap→规划/MPC 消费；当前 global_monitor | 替代早期“工程未提供/接口全部未知”的状态，但不将源码配置等同运行测量 |
| **待确定（运行）** | 用户指定 odom_ws 与本机 start_dlil_hyx.sh 的运行版本关系、最终包解析/二进制与工作树一致性 | 后续运行采集确认；本轮保持 odom_ws 主线，不要求补齐所有无关启动脚本 |
| **待确定（资料）** | 真实车体外轮廓、有效约束、静态图片预膨胀情况；目标与 setFollowSpeed 实际发布者、gimbal_driver/串口消费端 | 先从实机配置和当前活动图核对；同工程已有 `N/navigoal_manager/src/manager.cpp:40` 发布 goal_pose，但本入口没启动它，不能指定其必为实际目标源 |
| **待决策** | 原 A* 保留程度、提取算法还是整体复用 YAstar、T02-A 是否作为第一片、集成副本位置 | 建议先局部化简验证，不触碰已完成导航模块；由用户确认后进入实现 |
| **待决策（安全/接口）** | 失败时何时保留/撤销旧参考，是否约定空 Path；将来是否需要定时/速度/yaw 协议 | 建议优先复用几何 Path，不改 MPC；空 Path 不能未经确认当成整链安全协议。如需改消息或设计方向，先暂停说明 |
| **待测量/待决策** | NUC 基线、15～20 ms 口径、成功率和统计验收线 | 现有 plan 只分别打印 A* 与 Smooth 耗时，尚无覆盖地图合成、检查、构造输出的统一基线；不承诺 50 Hz 或最坏时限 |

缺失信息已先查指定完整工程与本地 TDT；未找到的串口消费实现、运行身份和物理配置不以其他导航副本代填。它们不妨碍本次静态交付，但约束后续集成/实机验收。

## 8. 实际核验记录

| 核验 | 本轮结果 |
| --- | --- |
| 工作区/Git/已有修改 | 已执行根目录及各实际仓库检查；根目录 Git 命令失败原因为该目录不是有效仓库，不是构建失败。结束复核的 TDT/导航状态与先前输出一致，三仓库 tracked diff 的 SHA-256 与编辑前输出完全一致；未覆盖已有 tracked 修改。中断后临时快照文件不可用，因此不宣称做过未跟踪文件逐字节校验 |
| 源码对照 | 已检查活动 launch 返回列表、CMake 目标、安装软链接、调用函数、消息消费和参数；比较 odom_ws/odom_hyx 的 DLIL launch 默认地图路径差异 |
| 历史构建日志 | 导航 `log/latest_build → build_2026-08-01_16-15-32` 仅构建 MPC，JobEnded rc=0；odom_ws `log/latest_build → build_2026-08-01_22-55-38` 各已记录包含 dlil 的 JobEnded rc=0。它们是历史记录，**不是本轮构建/测试通过**，也不证明当前工作树已编译 |
| 构建失败归因 | 本轮没有运行构建；没有取得所谓“上一步失败”的对应命令和错误日志，不能虚构根因或做依赖修复。后续若失败，应保留命令/日志，与未修改基线比较，修根因后重跑相关测试；不得删测试、屏蔽错误或随意换依赖 |
| 构建、单元/集成测试、实车、性能 | **全部未运行**。本轮是只读源码调查和文档交付，未修改源码；未启动 ROS 控制链路，未运行可能向原导航绝对路径写调试图的用例 |

已定位后续可复用的现有用例：`N/path_searching/test/astar_start_escape_test.cpp`、`path_safety_checker_truncation_test.cpp`、`replan_coordinator_periodic_test.cpp`，以及 `N/mpc_follower/test/test_mpc_controller.cpp`。本文不将其存在、历史日志或旧探索构建当作本轮测试结果。
