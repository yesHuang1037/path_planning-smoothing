# path_searching 搜不出路排查指南

本文按当前代码实现列出 `path_searching` 无法产出可用路径的全部内部原因，并标注关联参数、典型日志及处理方向。先按 `avoidance_mode` 区分动态障碍和重规划行为，再说明所有模式共通的 A\* 失败原因。

这里的“搜不出路”分为两类：

1. **没有进入 A\* 搜索**：规划请求在地图、TF、坐标、距离或重规划门控处被跳过。
2. **A\* 返回 `NO_PATH`**：搜索已执行，但没有找到满足当前占据和净空约束的连通路径。

配置文件为 `cfg/plan_param.yaml`。参数实际生效值以节点启动时输出的 `[Params]` 日志为准。

## 1. 按避障职责模式区分 NO_PATH 原因

三种模式不会取消静态地图、起终点、净空、终点逃生和起点脱困的 A\* 约束。它们的核心区别是：动态障碍是否加入 ESDF，以及动态阻塞后是否允许自动再搜一次。

| 模式 | 动态障碍是否加入 ESDF | 自动重规划能力 | 本次 A\* 可能无路的障碍来源 |
| --- | --- | --- | --- |
| `global_replan` | 由 `use_pointcloud_dynamic_obstacles`、`use_costmap_dynamic_obstacles` 控制 | 完整支持，但仍受 `replan_triggers.*` 和节流控制 | 静态地图、costmap、点云、阻塞记忆和净空约束。 |
| `global_monitor` | **仍会加入**，开关仍由两个动态障碍参数控制 | 动态阻塞、路径碰撞被模式强制关闭；新目标、停滞和周期重搜可用 | 静态地图，以及在新目标/停滞/周期实际发起 A\* 时已写入 ESDF 的 costmap、点云。 |
| `global_only` | 模式强制关闭点云、costmap、costmap 净空融合和阻塞记忆 | 新目标和周期重搜可用；不因动态阻塞、路径碰撞或停滞自动重搜 | 仅静态地图及由静态地图计算的净空约束。 |

### 1.1 `global_replan`：动态避障重规划模式

全局规划节点承担动态避障。动态障碍更新后，前视检查或全路径检查发现阻塞，会尝试以 `costmap_blocked`、`pointcloud_blocked` 或 `path_collision` 原因重新调用 A\*；若启用路径截断，还会先截断并把阻塞区加入临时记忆。

此模式下“搜不出路”有两种含义：

1. **本次 A\* 返回 `NO_PATH`**：静态地图本身不连通，或 costmap、点云、阻塞记忆把通道占据/压缩到净空阈值以下。完整原因见第 4 至第 7 节。
2. **发现阻塞但没有发起下一次 A\***：对应的 `replan_triggers.costmap_blocked`、`replan_triggers.pointcloud_blocked` 或 `replan_triggers.path_collision` 被关闭，无法获得起点 TF，距离目标过近，或者受到 `min_replan_interval_sec` 节流。

重点参数：`use_pointcloud_dynamic_obstacles`、`use_costmap_dynamic_obstacles`、`use_costmap_clearance_fusion`、`enable_forward_path_check`、`enable_path_collision_check`、`enable_blocked_path_truncation`、`enable_blocked_zone_memory`、`replan_triggers.*`、`min_replan_interval_sec`。

### 1.2 `global_monitor`：监控动态障碍但不主动动态重规划

此模式**不会关闭动态障碍源**。若 `use_costmap_dynamic_obstacles` 或 `use_pointcloud_dynamic_obstacles` 为 `true`，costmap/点云仍会写入 ESDF，因而在后续的新目标、停滞或周期触发 A\* 时，仍可能直接把路封死。

模式会强制关闭 `costmap_blocked`、`pointcloud_blocked` 和 `path_collision` 重规划触发，并关闭路径截断。因此，出现下列现象并不表示 A\* 一定无路：动态障碍已让当前路径不可通行，但节点不会因这次动态更新或前视碰撞而立即再搜；会在下一次周期、新目标或停滞触发时重搜（前提是对应开关开启）。

本模式下没有可用路径的原因：

1. 所有第 3 至第 7 节的共通原因，且 A\* 执行时动态障碍层可能参与封路。
2. 当前路径被动态障碍阻断，但自动动态重规划被模式关闭，导致没有新的路径输出。
3. `enable_stall_trigger` 或 `replan_triggers.stall` 被关闭，或尚未满足 `stall_window_sec`/`stall_move_threshold`，使停滞也不能作为兜底重试。

重点参数：`use_pointcloud_dynamic_obstacles`、`use_costmap_dynamic_obstacles`、`costmap_occupied_threshold`、`obstacle_expand_radius`、`enable_stall_trigger`、`stall_window_sec`、`stall_move_threshold`、`replan_triggers.stall`。动态阻塞相关的 `replan_triggers.*` 即使在 YAML 中设为 `true`，也会被该模式覆盖为 `false`。

### 1.3 `global_only`：纯静态全局规划模式

此模式强制关闭点云动态障碍、costmap 动态障碍、costmap 净空融合、阻塞记忆、前视检查、全路径检查、路径截断和停滞重规划。因而不会由运行时动态障碍造成 A\* 失败，也不会因为动态阻塞主动生成替代全局路径；但会按周期基于静态地图重新搜索。

本模式下 A\* 仍然不能保证有路。它会因为下列原因失败：

1. 静态地图占据、未知区域（当前实现将 `-1` 也视为占据）或静态地图不连通。
2. 起点/目标超出地图、起点被静态障碍占据且脱困失败、终点逃生找不到替代点。
3. 起点或通道的静态 ESDF 净空不满足 `robot_inscribed_radius + clearance_margin`。起点“净空不足但未占据”不会触发起点脱困。
4. 地图过大或绕行空间过复杂，A\* 达到固定的 500000 节点池上限。
5. `replan_triggers.new_goal` 被关闭，或起点 TF 不可用、目标过近，因而新目标也根本不执行 A\*。

重点参数：`robot_inscribed_radius`、`clearance_margin`、`enable_start_escape_mode`、`start_escape_steps`、`enable_goal_escape_mode`、`goal_escape_search_radius`、`goal_escape_clearance_tolerance`、`replan_triggers.new_goal`、`replan_triggers.periodic`、`monitor_period_sec`。本模式下设置动态障碍和动态重规划参数不会生效。

## 2. 先看哪类日志

| 日志或现象 | 含义 | 优先检查 |
| --- | --- | --- |
| `尚未收到静态地图，跳过目标点规划` | 没有开始规划 | `/map`、地图 QoS、静态地图初始化 |
| `目标点坐标转换失败` | 目标不能转换到全局坐标系 | `global_frame` 与 TF |
| `无法获取当前位姿，跳过规划` | 起点不可获得 | `robot_frame`、`robot_frame_candidates`、`anchor_frame` 与 TF |
| `目标点离起点太近，忽略` | 新目标直接被忽略 | 目标距离是否小于 0.2 m |
| `跳过重规划：...` | 触发被配置、距离阈值或节流拦截 | `avoidance_mode`、`replan_triggers`、`min_replan_interval_sec` |
| `路径规划失败：...` | A\* 已执行并失败 | 以下第 4 至第 7 节 |
| `A*搜索失败：节点池耗尽` | 可搜索节点超过容量 | 地图规模、障碍/净空约束、代码内固定容量 |
| `A*搜索失败：开放列表耗尽` | 没有满足约束的连通通道 | 占据、净空、动态障碍、阻塞记忆 |

## 3. 所有模式共通：A\* 没有执行的原因

### 3.1 静态地图尚未初始化

`/map` 未到达前，目标回调和 `plan()` 都会直接返回。静态地图只在首次回调时初始化；后续 `/map` 消息被忽略。

关联因素：

| 项目 | 影响 |
| --- | --- |
| `/map` | 必须有有效 `OccupancyGrid`，节点使用可靠、transient-local QoS 订阅。 |
| `enable_downstairs` | 影响 ESDF 的高度/楼梯约束建图，但不影响是否收到地图。 |
| 地图数据 `map.data` | 当前实现把任何非零值都当成障碍，**包括 `-1`（unknown）**。未知区域会使可搜索空间缩小。 |

### 3.2 起点或目标 TF/坐标转换失败

目标必须能从其消息 `frame_id` 转到 `global_frame`；起点必须能由候选机器人坐标系，或经 `anchor_frame` 间接转到 `global_frame`。任一转换失败，规划请求被跳过。

关联参数：`global_frame`、`robot_frame`、`robot_frame_candidates`、`anchor_frame`。

典型日志：`目标点坐标转换失败：无法转换到 ... 坐标系`、`Unable to resolve robot start pose ...`。

### 3.3 起终点距离过近

新目标回调中，起终点距离小于 **0.2 m** 时不触发首次规划；重规划协调器中，小于 **0.1 m** 时拒绝重规划；定时器同样在小于 **0.1 m** 时返回。这些阈值目前是代码常量，**没有 YAML 参数**。

典型日志：`目标点离起点太近，忽略。`、`机器人距离目标过近，跳过重规划`。

### 3.4 重规划请求被门控

已有目标但没有新路径时，阻塞检测不一定会立即再次调用 A\*。下列条件会跳过重规划：

| 条件 | 关联参数 | 说明 |
| --- | --- | --- |
| 该原因的触发开关关闭 | `replan_triggers.new_goal/manual/map_update/costmap_blocked/pointcloud_blocked/path_collision/stall/periodic` | 对应原因不会进入 `plan()`。 |
| 处于模式限制下 | `avoidance_mode` | 三种模式的强制开关见第 1 节；被模式强制关闭的触发无法通过 YAML 重新打开。 |
| 重规划过于频繁 | `min_replan_interval_sec` | 除新目标、手动重规划外，其余原因在间隔内会被节流。 |
| 定时监控未及时执行 | `monitor_period_sec` | 影响发现阻塞和下一次重试的延迟；运行时下限为 0.02 s。 |
| 停滞条件尚未满足或被关闭 | `enable_stall_trigger`、`stall_window_sec`、`stall_move_threshold`、`stall_goal_distance_threshold` | 停滞不触发时不会以此原因重规划。 |

这类问题不是“路径不存在”，而是当前配置没有让系统再次搜索。

说明：`manual` 和 `map_update` 虽然是 `ReplanReason` 及对应 YAML 参数，但当前 `plan_manager.cpp` 没有以这两种原因调用 `triggerReplan()`；它们不是当前节点可用的重规划入口。

## 4. 所有模式共通：起点和终点无效

### 4.1 起点或终点落在地图外

起点或目标经 `worldToIndex()` 转换后越过静态地图边界，`plan()` 会在调用 A\* 前返回。

关联因素：静态地图 `origin`、`resolution`、尺寸，`global_frame`/TF，以及目标坐标。

典型日志：`路径规划失败：起点或目标超出地图范围`。

### 4.2 起点被占据

起点栅格为静态障碍或动态障碍时，A\* 会启动“起点脱困”。从起点出发的节点路径深度不超过 `start_escape_steps` 时，可放宽碰撞和净空限制；仍找不到能离开的路径时，A\* 返回 `NO_PATH`。首次失败后，若起点仍为占据状态，管理器会把窗口扩大到至少 40 步再重试一次。

关联参数：

| 参数 | 作用 |
| --- | --- |
| `enable_start_escape_mode` | 是否允许起点占据时脱困。 |
| `start_escape_steps` | 首次搜索允许放宽约束的扩展步数。 |
| `use_pointcloud_dynamic_obstacles`、`use_costmap_dynamic_obstacles` | 动态障碍也可使起点成为占据。 |
| `obstacle_expand_radius` / 兼容参数 `voronoi_radius` | 点云障碍扩张后可能覆盖起点。 |
| `costmap_occupied_threshold` | costmap 值达到该阈值即写入动态占据层。 |

**实现边界：**起点脱困的启动条件严格是“起点栅格被占据”。若起点栅格空闲、但安全距离低于要求，脱困不会启动，也不会触发 40 步重试。详见 5.2。

### 4.3 终点被占据或终点净空不足

在 A\* 前会执行终点逃生：原终点占据、净空无效或小于等于放宽后的阈值时，系统在 `goal_escape_search_radius` 内寻找替代终点。候选顺序是：满足完整净空、满足放宽净空、最后任意非占据候选。找不到候选时保留原目标，因此 A\* 仍可能失败。

关联参数：

| 参数 | 作用 |
| --- | --- |
| `enable_goal_escape_mode` | 关闭后不会替换不可达终点。 |
| `goal_escape_search_radius` | 替代终点的最大搜索半径。半径过小可能无候选。 |
| `goal_escape_clearance_tolerance` | 放宽量。放宽阈值为 `max(0, required_clearance - tolerance)`。 |
| `robot_inscribed_radius`、`clearance_margin` | 决定完整净空阈值。 |

典型日志：`Goal escape failed... keep original goal.` 或 `路径规划失败：目标栅格被占用/目标安全距离不足`。

## 5. 所有模式共通：净空（安全距离）约束导致没有可行通道

### 5.1 净空阈值如何计算

对 A\* 和前视检查，必须严格满足：

```text
实际净空 > required_clearance
required_clearance = max(0, robot_inscribed_radius + clearance_margin)
```

“等于阈值”也会被判为不安全。A\* 将该米值除以地图分辨率，得到 `min_clearance_cells`；邻居栅格若 `dist < min_clearance_cells` 就被拒绝。

关联参数：`robot_inscribed_radius`、`clearance_margin`，以及地图 `resolution`。

`check_collision_radius` **不参与 A\* 的 `min_clearance_cells`**；它只在“整条路径碰撞检查”中使阈值变为：

```text
max(check_collision_radius, required_clearance)
```

### 5.2 起点安全距离不足

这是“起点栅格未占据，但太贴近障碍”的情况。A\* 初始节点会进入开放列表，但因为脱困仅由 `start_in_collision` 触发，第一层邻居仍受净空约束；若周围没有净空达标的邻居，开放列表耗尽并失败。

关联参数：`robot_inscribed_radius`、`clearance_margin`、`enable_start_escape_mode`、`start_escape_steps`。

注意：后两个参数在该场景**不会生效**，因为起点不占据。这正是日志显示“起点安全距离不足”但没有看到脱困日志的原因。

典型日志：`路径规划失败：起点安全距离不足，当前=... m，要求>... m`。

### 5.3 通道整体净空不足

即使起终点安全，走廊宽度不足、障碍膨胀后封路、或者所有绕行区域均低于净空阈值时，A\* 会拒绝邻居直至开放列表耗尽。

关联参数：

| 参数 | 增大后的效果 |
| --- | --- |
| `robot_inscribed_radius` | 提高所需净空，更容易封闭窄通道。 |
| `clearance_margin` | 同上，作为额外安全余量。 |
| `obstacle_expand_radius` / `voronoi_radius` | 仅点云源；扩大点云障碍的占据范围。 |
| `costmap_occupied_threshold` | 降低后更多 costmap 格被占据，更容易封路。 |
| `use_costmap_clearance_fusion` | 开启后通行净空取 ESDF 与 costmap 净空的较小值，更保守。 |
| `enable_blocked_zone_memory`、`blocked_zone_memory_radius`、`blocked_zone_memory_ttl_sec` | 临时阻塞记忆会作为动态占据加入搜索；半径/保留时间越大，短时间内越可能无路。 |

`obstacle_cost_weight` 仅改变靠近障碍的代价偏好，正常情况下不会把净空合格的栅格判为不可走；`dynamic_penalty_weight` 当前未接入 A\* 邻居代价计算，不影响可达性。

## 6. 占据障碍使起终点不连通

在通过净空判据前，A\* 邻居本身若是占据栅格就会被拒绝。以下来源会写入 ESDF 占据层，任何一个都可能切断连通性：

| 障碍来源 | 关联参数或输入 | 说明 |
| --- | --- | --- |
| 静态地图 | `/map` | `map.data != 0` 都视为占据，unknown (`-1`) 也会被视为障碍。 |
| costmap | `use_costmap_dynamic_obstacles`、`costmap_topic`、`costmap_update_topic`、`costmap_occupied_threshold` | costmap 中数值大于等于阈值的格会映射到静态地图坐标系并占据。增量更新必须先收到全量 costmap。 |
| 点云 | `use_pointcloud_dynamic_obstacles`、`/cloud_livox_obs`、`obstacle_expand_radius` | 点云必须能转换至 `global_frame`；有效点会按半径离散扩张。 |
| 阻塞记忆 | `enable_blocked_zone_memory`、`blocked_zone_memory_radius`、`blocked_zone_memory_ttl_sec` | 仅在路径截断成功后写入；到期由定时器清除。 |

典型 A\* 诊断：`开放列表耗尽，起点到目标之间不存在满足当前碰撞/安全距离约束的连通路径`；如果 `reject_collision` 较高，主要是占据障碍造成。

## 7. A\* 资源上限

`Astar::setParam()` 将节点池固定为 **500000** 个节点，未暴露 YAML 参数。搜索需扩张的节点达到该上限时立即返回 `NO_PATH`。

常见诱因：地图过大、高分辨率、目标过远、迷宫型障碍、净空阈值造成大量绕行。先看日志中的 `节点池耗尽`；需要调整时只能修改代码，使容量成为可配置参数或降低搜索规模。

## 8. 已经有路径，但后续被判为不可通行

这不是首次 A\* 搜索失败，但表现为“机器人没有可跟踪路径”或持续重规划。前视/全路径检查会因为下列原因判定当前路径阻塞：路径点越界、净空无效、净空不足或碰撞。

| 检查 | 关联参数 | 阈值 |
| --- | --- | --- |
| 前视检查 | `enable_forward_path_check`、`forward_check_distance`、`forward_sample_step` | `required_clearance`。采样包括当前机器人位置。 |
| 全路径检查 | `enable_path_collision_check`、`check_collision_radius` | `max(check_collision_radius, required_clearance)`。 |
| 路径截断 | `enable_blocked_path_truncation`、`blocked_path_truncation_margin` | 截断终点要求 `required_clearance + blocked_path_truncation_margin`。 |

在 `global_monitor` 和 `global_only` 模式下，某些动态重规划触发会被强制关闭。此时即使检测到阻塞，也可能只记录日志或由局部控制器处理，而不会立即重新搜索。

## 9. 推荐排查顺序

1. 确认节点启动参数，尤其是 `avoidance_mode`、两个动态障碍开关、`robot_inscribed_radius`、`clearance_margin`、两个 escape 参数。
2. 查看失败日志中的起点/终点世界坐标、栅格坐标、当前净空和要求净空。
3. A\* 内部保留了 `start_collision`、`start_dist_cells`、`escape_mode`、`reject_collision`、`reject_clearance`、`bypass_escape_constraints` 和 `exit` 等诊断字段；当前版本默认不会把完整字段打印到节点日志。需要这些计数时，应通过调试器查看 `last_search_debug_`，或在 `Astar::formatLastSearchDebugInfoChinese()` 接入受控调试日志。
4. 若起点未占据却净空不足，当前实现不会脱困；应先确认定位误差、地图障碍膨胀和净空参数是否合理。
5. 若 `reject_collision` 高，检查静态地图的 unknown 区、costmap 阈值、点云转换和障碍扩张半径。
6. 若 `reject_clearance` 高，检查机器人半径、余量、costmap 净空融合及走廊实际宽度。
7. 若是动态障碍后没有再搜索，检查 `avoidance_mode`、对应 `replan_triggers.*`、节流时间和 TF 可用性。

## 10. 当前默认配置的关键点

仓库内 `cfg/plan_param.yaml` 当前配置为 `avoidance_mode: global_monitor`。该模式会强制关闭 `costmap_blocked`、`pointcloud_blocked` 和 `path_collision` 重规划触发，并关闭路径截断；周期重搜不再受模式关闭，仍由 `replan_triggers.periodic` 控制。当前起点脱困开启（20 步），终点逃生开启（半径 1.5 m）。

## 代码位置

| 模块 | 位置 |
| --- | --- |
| 规划入口、参数、地图/TF/终点逃生 | `src/plan_manager.cpp` |
| A\* 占据、净空、起点脱困、节点池 | `src/Astar.cpp` |
| ESDF 的占据与距离查询 | `src/esdf_map.cpp` |
| 净空与前视/全路径检查 | `src/path_safety_checker.cpp` |
| 动态障碍、costmap 与阻塞记忆 | `src/dynamic_obstacle_manager.cpp` |
| 重规划门控与避障模式 | `src/replan_coordinator.cpp` |
