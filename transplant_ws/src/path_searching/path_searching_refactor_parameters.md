# path_searching 重构类参数与使用说明

本文记录本次从 `PlanManager` 中抽出的 4 个类的职责、参数来源、使用方式，以及 `avoidance_mode` 对全局规划/避障行为的影响。

相关代码位置：

- `src/path_searching/include/path_searching/grid_geometry.h`
- `src/path_searching/include/path_searching/dynamic_obstacle_manager.h`
- `src/path_searching/include/path_searching/path_safety_checker.h`
- `src/path_searching/include/path_searching/replan_coordinator.h`
- `src/path_searching/src/plan_manager.cpp`
- `src/path_searching/cfg/plan_param.yaml`

## 总体模式

顶层参数：

```yaml
avoidance_mode: global_replan
```

可选值：

| 值 | 含义 | 适用场景 |
| --- | --- | --- |
| `global_replan` | 全局规划节点承担动态障碍检查、路径截断和触发重规划 | 没有局部避障控制器时 |
| `global_monitor` | 接收/监控动态障碍，但不因动态障碍主动重规划 | 过渡调试，观察动态障碍影响 |
| `global_only` | 纯全局规划，只处理新目标等全局规划事件 | 后续接局部控制器避障 |

`global_only` 会强制关闭：

- 点云动态障碍源
- costmap 动态障碍源
- costmap clearance 融合
- blocked memory
- 前视路径检查
- 整条路径碰撞检查
- 路径截断
- 停滞触发
- 动态避障相关重规划触发

## 1. GridGeometry

职责：统一管理地图几何信息和坐标转换。

它不直接读取 ROS 参数，而是在收到 `/map` 后由 `PlanManager` 用地图信息初始化：

```cpp
grid_geometry_ = path_searching::GridGeometry(map_size, map_offset, resolution);
```

数据来源：

| 字段 | 来源 |
| --- | --- |
| `size` | `map->info.height`, `map->info.width` |
| `offset` | `map->info.origin.position.x/y` |
| `resolution` | `map->info.resolution` |

主要接口：

| 接口 | 作用 |
| --- | --- |
| `valid()` | 判断地图几何是否已初始化 |
| `worldToIndex(pos)` | 世界坐标转栅格索引，返回 `(row, col)` |
| `indexToWorld(idx)` | 栅格索引转世界坐标 |
| `inBounds(idx)` | 判断栅格是否在地图范围内 |
| `metersToCellsCeil(m)` | 米转栅格，向上取整 |
| `metersToCellsRound(m)` | 米转栅格，四舍五入 |
| `packKey(idx)` / `unpackKey(key)` | 栅格索引用于 hash 存储 |

使用方：

- `PlanManager`：替代原来的 `Pos2index()` / `Index2pos()` 内部实现。
- `DynamicObstacleManager`：costmap/点云转静态地图栅格。
- `PathSafetyChecker`：路径采样点转栅格并查询 ESDF。

## 2. DynamicObstacleManager

职责：管理动态障碍物来源，并把合并后的动态障碍写入 ESDF。

它只处理 map frame 下的数据。TF 转换仍由 `PlanManager` 做，避免把 ROS TF 逻辑塞进这个类。

配置结构：

```cpp
path_searching::DynamicObstacleManager::Config
```

参数映射：

| Config 字段 | ROS 参数 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `use_pointcloud_dynamic_obstacles` | `use_pointcloud_dynamic_obstacles` | `true` | 是否使用 `/cloud_livox_obs` 点云动态障碍 |
| `use_costmap_dynamic_obstacles` | `use_costmap_dynamic_obstacles` | `false` | 是否使用 costmap 动态障碍 |
| `use_costmap_clearance_fusion` | `use_costmap_clearance_fusion` | `true` | 路径 clearance 查询时是否融合 costmap |
| `enable_blocked_zone_memory` | `enable_blocked_zone_memory` | `true` | 是否把路径阻塞点附近临时加入动态障碍 |
| `blocked_zone_memory_radius` | `blocked_zone_memory_radius` | `0.45` | 阻塞记忆半径，单位 m |
| `blocked_zone_memory_ttl_sec` | `blocked_zone_memory_ttl_sec` | `2.0` | 阻塞记忆保留时间，单位 s |
| `costmap_occupied_threshold` | `costmap_occupied_threshold` | `50` | costmap 占据阈值，`>=` 判为障碍 |
| `obstacle_expand_radius` | `obstacle_expand_radius` / `voronoi_radius` | `0.40` | 点云障碍膨胀半径，`voronoi_radius > 0` 时兼容覆盖 |

主要接口：

| 接口 | 作用 |
| --- | --- |
| `configure(config)` | 设置参数 |
| `setMap(geometry, esdf)` | 绑定地图几何和 ESDF |
| `updateFromPointCloudMapFrame(points)` | 用 map frame 下的二维点更新点云动态障碍 |
| `updateFromCostmapFull(msg)` | 用 costmap 全量地图更新动态障碍 |
| `updateFromCostmapUpdate(msg)` | 用 costmap 增量更新动态障碍 |
| `addBlockedZone(blocked_idx, now_sec, reason)` | 把阻塞点附近临时加入动态障碍 |
| `pruneExpiredBlockedZones(now_sec)` | 清理过期 blocked memory |
| `costmapClearanceMeters(idx, radius_m)` | 查询某栅格附近 costmap 动态障碍距离 |
| `counts()` | 返回点云/costmap/blocked/merged 动态障碍数量 |

返回结构：

```cpp
ObstacleUpdateReport
```

用于日志和调试，包含：

- 是否接受更新
- 是否发生变化
- 来源
- 插入/删除/越界/unknown/occupied 数量
- 当前动态障碍数量统计

## 3. PathSafetyChecker

职责：判断路径是否安全，并生成路径截断结果。

它不负责：

- 是否触发重规划
- 是否发布路径
- 是否写 blocked memory

这些动作仍由 `PlanManager` 协调。

配置结构：

```cpp
path_searching::PathSafetyChecker::Config
```

参数映射：

| Config 字段 | ROS 参数 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `robot_inscribed_radius` | `robot_inscribed_radius` | `0.25` | 机器人内接半径，单位 m |
| `clearance_margin` | `clearance_margin` | `0.05` | 额外安全余量，单位 m |
| `collision_radius` | `check_collision_radius` | `0.50` | 整条路径碰撞检查半径，单位 m |
| `forward_check_distance` | `forward_check_distance` | `2.0` | 前视检查距离，单位 m |
| `forward_sample_step` | `forward_sample_step` | `0.10` | 路径采样步长，单位 m |
| `blocked_path_truncation_margin` | `blocked_path_truncation_margin` | `0.20` | 截断终点额外安全距离，单位 m |
| `enable_blocked_path_truncation` | `enable_blocked_path_truncation` | `true` | 是否允许截断当前路径 |

调用开关：

| ROS 参数 | 作用 |
| --- | --- |
| `enable_forward_path_check` | 控制 `PlanManager` 是否调用前视路径检查 |
| `enable_path_collision_check` | 控制 `PlanManager` 是否调用整条路径碰撞检查 |

注意：`global_only` 模式会强制关闭以上两个调用开关。

主要接口：

| 接口 | 作用 |
| --- | --- |
| `configure(config)` | 设置路径安全参数 |
| `setMap(geometry, esdf, dynamic_obstacles)` | 绑定地图、ESDF 和动态障碍管理器 |
| `requiredClearanceMeters()` | 返回 `robot_inscribed_radius + clearance_margin` |
| `clearanceAt(world_pt, radius)` | 查询某点安全距离 |
| `checkForward(path, current_pose)` | 检查前方路径是否可通行 |
| `checkWholePath(path)` | 检查整条路径是否碰撞/距离不足 |
| `truncateBeforeBlockedPoint(path, current_pose)` | 在阻塞点前截断路径 |

报告结构：

```cpp
PathCheckReport
```

包含：

- `passable`
- 阻塞原因 `PathBlockReason`
- 最小安全距离
- 要求安全距离
- 阻塞点世界坐标
- 阻塞点栅格坐标

阻塞原因：

| 枚举 | 中文含义 |
| --- | --- |
| `None` | 路径可通行 |
| `OutOfMap` | 路径点超出地图范围 |
| `InvalidClearance` | 路径点安全距离无效 |
| `ClearanceTooLow` | 路径安全距离不足 |
| `Collision` | 路径发生碰撞 |

## 4. ReplanCoordinator

职责：决定某个原因是否允许触发重规划。

它不负责：

- 检查路径
- 管理障碍
- 调 A*
- 发布路径

配置结构：

```cpp
path_searching::ReplanCoordinator::Config
```

参数映射：

| Config 字段 | ROS 参数 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `avoidance_mode` | `avoidance_mode` | `global_replan` | 顶层避障职责模式 |
| `min_replan_interval_sec` | `min_replan_interval_sec` | `0.5` | 两次重规划最小间隔 |
| `enable_stall_trigger` | `enable_stall_trigger` | `true` | 是否启用停滞触发 |
| `stall_window_sec` | `stall_window_sec` | `8.0` | 停滞检测时间窗 |
| `stall_move_threshold` | `stall_move_threshold` | `0.05` | 时间窗内移动小于该值视为停滞 |
| `stall_goal_distance_threshold` | `stall_goal_distance_threshold` | `0.30` | 距目标小于该值不触发停滞 |

重规划原因开关：

```yaml
replan_triggers:
  new_goal: true
  manual: true
  map_update: false
  costmap_blocked: true
  pointcloud_blocked: true
  path_collision: true
  stall: true
  periodic: true
```

对应枚举：

| ROS 参数 | ReplanReason | 说明 |
| --- | --- | --- |
| `new_goal` | `NewGoal` | 收到新目标 |
| `manual` | `Manual` | 手动触发，当前预留 |
| `map_update` | `MapUpdate` | 静态地图更新，当前预留 |
| `costmap_blocked` | `CostmapBlocked` | costmap 更新后发现当前路径被挡 |
| `pointcloud_blocked` | `PointcloudBlocked` | 点云动态障碍更新后发现当前路径被挡 |
| `path_collision` | `PathCollision` | 定时检查发现路径碰撞或安全距离不足 |
| `stall` | `Stall` | 机器人停滞 |
| `periodic` | `Periodic` | 周期性重规划 |

模式默认行为：

| 模式 | 默认启用的重规划原因 |
| --- | --- |
| `global_replan` | `new_goal`, `manual`, `costmap_blocked`, `pointcloud_blocked`, `path_collision`, `stall`, `periodic` |
| `global_monitor` | `new_goal`, `manual`, `stall`, `periodic` |
| `global_only` | `new_goal`, `manual`, `periodic` |

注意：代码中 `global_monitor` 和 `global_only` 会强制关闭部分动态避障触发，即使 YAML 中把它们设为 `true`；`periodic` 不属于模式强制关闭项，仍由 YAML 参数控制。

主要接口：

| 接口 | 作用 |
| --- | --- |
| `defaultTriggersForMode(mode)` | 获取某个模式的默认触发配置 |
| `configure(config)` | 设置模式、触发开关和停滞参数 |
| `triggerEnabled(reason)` | 查询某个原因是否允许触发 |
| `request(reason, now_sec, start, goal)` | 判断本次是否允许重规划 |
| `detectStall(now_sec, start, goal)` | 判断机器人是否停滞 |
| `resetStallAnchor(now_sec, start)` | 重置停滞检测锚点 |

`request()` 返回：

```cpp
ReplanDecision
```

包含：

- `should_replan`
- `trigger_disabled`
- `throttled`
- `goal_too_close`
- `reason`
- `message`

## 常见配置

### 纯全局规划，避障交给局部控制器

```yaml
avoidance_mode: global_only
```

效果：

- 仍然接收目标并做 A* 全局规划。
- 不再因为 costmap/点云动态障碍触发重规划。
- 不再做前视路径检查、路径截断、blocked memory。
- 局部避障应由下游局部控制器负责。

### 保留动态障碍建图，但禁用 costmap 阻塞重规划

```yaml
avoidance_mode: global_replan

replan_triggers:
  new_goal: true
  manual: true
  map_update: false
  costmap_blocked: false
  pointcloud_blocked: true
  path_collision: true
  stall: true
  periodic: false
```

效果：

- costmap 动态障碍仍可进入 ESDF。
- A* 新规划时仍会避开 costmap 动态障碍。
- 但 costmap 更新后不会因为“当前路径被 costmap 挡住”立即重规划。

### 只观察动态障碍影响，不主动避障重规划

```yaml
avoidance_mode: global_monitor
```

效果：

- 可以保留动态障碍更新和日志。
- 不截断路径。
- 不因动态障碍主动触发重规划。
- 可用于接局部控制器前后的对比调试。

## PlanManager 中的调用关系

初始化：

```cpp
configureDynamicObstacleManager();
configurePathSafetyChecker();
configureReplanCoordinator();
```

收到地图：

```cpp
grid_geometry_ = GridGeometry(map_size, map_offset, resolution);
dynamic_obstacle_manager_.setMap(grid_geometry_, esdf_1);
path_safety_checker_.setMap(grid_geometry_, esdf_1, &dynamic_obstacle_manager_);
```

收到动态障碍：

```cpp
dynamic_obstacle_manager_.updateFromCostmapFull(...);
dynamic_obstacle_manager_.updateFromCostmapUpdate(...);
dynamic_obstacle_manager_.updateFromPointCloudMapFrame(...);
```

动态障碍更新后：

```cpp
checkPathAfterDynamicObstacleUpdate(reason, source_label);
```

触发重规划：

```cpp
triggerReplan(ReplanReason::CostmapBlocked, "costmap 增量更新");
```

最终由 `ReplanCoordinator::request()` 决定是否真正调用 `plan(start, goal)`。
