# path_searching 障碍物截断/重规划参数说明

这份文档只解释一件事：路径上出现障碍物时，`path_searching` 怎么判断“前面被挡住了”，然后怎么发布截断后的短路径，再触发重规划。

## 先看结论

如果你想让“遇到障碍物先截断路径”稳定触发，优先用下面这组参数：

```yaml
avoidance_mode: global_replan

use_costmap_dynamic_obstacles: true
use_pointcloud_dynamic_obstacles: false
use_costmap_clearance_fusion: false

robot_inscribed_radius: 0.05
clearance_margin: 0.00
forward_check_distance: 1.50
forward_sample_step: 0.10
enable_forward_path_check: true
enable_path_collision_check: false

enable_blocked_path_truncation: true
blocked_path_truncation_margin: 0.20

monitor_period_sec: 0.50
min_replan_interval_sec: 1.50
replan_triggers:
  costmap_blocked: true
  path_collision: true
  periodic: true
```

最关键的是：

```yaml
robot_inscribed_radius: 0.05
```

不要设成 `0.0`。当前代码的前视检查用：

```text
required_clearance = robot_inscribed_radius + clearance_margin
```

如果这两个加起来是 `0`，前视检查会直接认为“无需检查安全距离”，路径截断就很难触发。

## 触发链路

路径截断不是一个单独的开关，它依赖前面的检查先判定路径被挡住。

实际流程是：

```text
costmap / 点云收到障碍物
  -> DynamicObstacleManager 写入动态障碍
  -> ESDF 更新障碍距离
  -> PathSafetyChecker 检查当前 path_now
  -> 发现前方不可通行
  -> truncatePathOnBlockingObstacle()
  -> 发布截断后的 /sPath
  -> 下一次 timer 再触发重规划
```

所以只开：

```yaml
enable_blocked_path_truncation: true
```

还不够。还要保证前面的路径检查能判定“不通行”。

## 参数分组解释

### 1. avoidance_mode：总模式

```yaml
avoidance_mode: global_replan
```

可选值：

- `global_replan`：全局规划负责动态障碍检查、路径截断、触发重规划。要调截断时用这个。
- `global_monitor`：接收障碍物，但不主动因为动态障碍重规划；这个模式下会强制关闭截断。
- `global_only`：纯全局路径规划，不管动态避障；这个模式下会强制关闭前视检查、碰撞检查、截断。

要测试障碍物截断，必须用：

```yaml
avoidance_mode: global_replan
```

### 2. 动态障碍来源

#### costmap 来源

```yaml
use_costmap_dynamic_obstacles: true
costmap_topic: /costmap/costmap
costmap_update_topic: /costmap/costmap_updates
costmap_occupied_threshold: 65
```

含义：

- `use_costmap_dynamic_obstacles: true`：启用 costmap 动态障碍。
- `costmap_occupied_threshold: 65`：costmap 栅格值大于等于 65 才认为是障碍。

如果障碍物明明在 costmap 里，但日志里 `occupied=0` 或 `accepted=0`，优先检查：

- costmap 话题名是否对；
- costmap frame/地图原点是否和静态地图对齐；
- `costmap_occupied_threshold` 是否太高。

#### 点云来源

```yaml
use_pointcloud_dynamic_obstacles: false
obstacle_expand_radius: 0.10
```

如果你用 `/cloud_livox_obs` 这类点云障碍，才打开：

```yaml
use_pointcloud_dynamic_obstacles: true
```

`obstacle_expand_radius` 是把点云点膨胀成小圆。点云稀疏时可以调大，比如：

```yaml
obstacle_expand_radius: 0.20
```

### 3. 前视检查：决定会不会发现“前面挡住了”

```yaml
enable_forward_path_check: true
robot_inscribed_radius: 0.05
clearance_margin: 0.00
forward_check_distance: 1.50
forward_sample_step: 0.10
```

这是截断最关键的一组。

#### robot_inscribed_radius

机器人内接半径，单位米。

当前代码里，前视检查的安全阈值是：

```text
required_clearance = robot_inscribed_radius + clearance_margin
```

建议不要为 `0.0`。如果不知道该填多少，先填：

```yaml
robot_inscribed_radius: 0.05
```

如果机器人实际半宽更大，再逐步调到：

```yaml
robot_inscribed_radius: 0.10
```

或：

```yaml
robot_inscribed_radius: 0.15
```

#### clearance_margin

额外安全余量。

建议先保持：

```yaml
clearance_margin: 0.00
```

如果发现太贴障、截断太晚，再加到：

```yaml
clearance_margin: 0.03
```

或：

```yaml
clearance_margin: 0.05
```

#### forward_check_distance

只检查机器人前方多远的路径。

```yaml
forward_check_distance: 1.50
```

含义：障碍物在当前机器人前方 1.5m 内，才会被前视检查发现并触发截断。

如果你觉得触发太晚，调大：

```yaml
forward_check_distance: 2.00
```

如果你觉得太敏感、远处障碍太早打断路径，调小：

```yaml
forward_check_distance: 1.00
```

#### forward_sample_step

沿路径每隔多远采样一次。

```yaml
forward_sample_step: 0.10
```

建议：

- `0.10`：比较稳，能看到小障碍；
- `0.20`：更省 CPU，但可能漏掉窄障碍；
- 不建议大于 `0.20`，除非地图分辨率很粗。

### 4. 整条路径碰撞检查

```yaml
enable_path_collision_check: false
check_collision_radius: 0.10
```

这组和前视检查不一样。

- `enable_forward_path_check`：只看机器人前方一段。
- `enable_path_collision_check`：检查整条 `path_now`。

如果打开：

```yaml
enable_path_collision_check: true
```

远处路径上有障碍也会触发，可能更激进。

建议现场先保持：

```yaml
enable_path_collision_check: false
```

靠前视检查处理近处障碍，避免远处 costmap 噪声过早打断当前路径。

注意：当前代码里，`check_collision_radius` 主要影响整条路径碰撞检查；如果 `enable_path_collision_check: false`，不要指望只调 `check_collision_radius` 就能让前视截断触发。

### 5. 截断行为

```yaml
enable_blocked_path_truncation: true
blocked_path_truncation_margin: 0.20
```

#### enable_blocked_path_truncation

是否在发现阻塞后先发布一条短路径。

必须是：

```yaml
enable_blocked_path_truncation: true
```

#### blocked_path_truncation_margin

截断终点距离阻塞区域再多留多少安全距离。

```yaml
blocked_path_truncation_margin: 0.20
```

如果车停得太靠近障碍，调大：

```yaml
blocked_path_truncation_margin: 0.30
```

如果截断太保守、路径缩得太短，调小：

```yaml
blocked_path_truncation_margin: 0.10
```

### 6. 阻塞记忆

```yaml
enable_blocked_zone_memory: false
blocked_zone_memory_radius: 0.15
blocked_zone_memory_ttl_sec: 2.0
```

阻塞记忆的作用是：截断时，把阻塞点附近临时标成禁行，避免重规划又搜回同一个贴障位置。

如果你发现反复截断、反复规划回同一处，可以打开：

```yaml
enable_blocked_zone_memory: true
blocked_zone_memory_radius: 0.20
blocked_zone_memory_ttl_sec: 2.0
```

如果打开后绕路太大，把半径降到：

```yaml
blocked_zone_memory_radius: 0.10
```

### 7. 重规划频率

```yaml
monitor_period_sec: 0.50
min_replan_interval_sec: 1.50
```

#### monitor_period_sec

定时检查周期。越小越灵敏。

```yaml
monitor_period_sec: 0.50
```

表示每 0.5 秒检查一次当前路径。

想更快触发：

```yaml
monitor_period_sec: 0.20
```

#### min_replan_interval_sec

两次重规划之间的最小间隔。

如果太大，会出现“已经截断了，但重规划要等一会儿”的感觉。

建议调试时先用：

```yaml
min_replan_interval_sec: 0.50
```

稳定后再调回：

```yaml
min_replan_interval_sec: 1.50
```

## 推荐调参顺序

不要一次改很多。按这个顺序来：

### 第一步：让截断先能触发

```yaml
avoidance_mode: global_replan
enable_forward_path_check: true
enable_blocked_path_truncation: true
robot_inscribed_radius: 0.05
clearance_margin: 0.00
forward_check_distance: 1.50
forward_sample_step: 0.10
```

观察日志里是否出现：

```text
定时检查发现前方路径不可通行
路径已在阻塞点前截断
截断路径已发布，重规划延后到下一次监控周期
```

### 第二步：如果还是不触发

先确认障碍物有没有进系统。看日志：

```text
动态障碍更新：来源=...
occupied=...
accepted=...
动态障碍[点云=... costmap=... 合并=...]
```

如果 `occupied=0` 或 `合并=0`，问题在障碍物输入，不在截断参数。

如果动态障碍有数量，但前视还是不触发，把判据调敏感：

```yaml
robot_inscribed_radius: 0.10
clearance_margin: 0.03
forward_check_distance: 2.00
forward_sample_step: 0.10
```

### 第三步：如果触发太频繁

把判据降敏：

```yaml
robot_inscribed_radius: 0.05
clearance_margin: 0.00
forward_check_distance: 1.00
min_replan_interval_sec: 1.50
```

如果是远处障碍导致频繁触发，保持：

```yaml
enable_path_collision_check: false
```

### 第四步：如果反复规划回同一障碍

打开阻塞记忆：

```yaml
enable_blocked_zone_memory: true
blocked_zone_memory_radius: 0.20
blocked_zone_memory_ttl_sec: 2.0
```

## 三套可直接试的配置

### A. 调试用：容易触发

```yaml
robot_inscribed_radius: 0.10
clearance_margin: 0.03
forward_check_distance: 2.00
forward_sample_step: 0.10
enable_forward_path_check: true
enable_path_collision_check: false
enable_blocked_path_truncation: true
blocked_path_truncation_margin: 0.25
monitor_period_sec: 0.30
min_replan_interval_sec: 0.50
```

用途：先确认截断链路能跑通。

### B. 正常用：比较稳

```yaml
robot_inscribed_radius: 0.05
clearance_margin: 0.00
forward_check_distance: 1.50
forward_sample_step: 0.10
enable_forward_path_check: true
enable_path_collision_check: false
enable_blocked_path_truncation: true
blocked_path_truncation_margin: 0.20
monitor_period_sec: 0.50
min_replan_interval_sec: 1.50
```

用途：现场默认建议。

### C. 低敏用：少打断

```yaml
robot_inscribed_radius: 0.05
clearance_margin: 0.00
forward_check_distance: 1.00
forward_sample_step: 0.20
enable_forward_path_check: true
enable_path_collision_check: false
enable_blocked_path_truncation: true
blocked_path_truncation_margin: 0.15
monitor_period_sec: 0.50
min_replan_interval_sec: 2.00
```

用途：costmap 噪声多、路径老被打断时使用。

## 常见误区

### 误区 1：只打开 enable_blocked_path_truncation

不够。截断函数只在路径检查发现阻塞后才会调用。

### 误区 2：robot_inscribed_radius 和 clearance_margin 都是 0

这种情况下前视检查很可能直接跳过安全距离判断。要触发前视截断，至少给一个非零值：

```yaml
robot_inscribed_radius: 0.05
```

### 误区 3：只调 check_collision_radius

如果：

```yaml
enable_path_collision_check: false
```

那么 `check_collision_radius` 不会主导前视截断。前视检查主要看：

```yaml
robot_inscribed_radius
clearance_margin
forward_check_distance
forward_sample_step
```

### 误区 4：障碍物不在 forward_check_distance 范围内

例如：

```yaml
forward_check_distance: 1.20
```

障碍物离机器人 2 米，那前视检查不会触发。要么等机器人靠近，要么调大：

```yaml
forward_check_distance: 2.00
```

### 误区 5：动态障碍根本没进 ESDF

如果日志里动态障碍数量一直是 0，先查输入：

```bash
ros2 topic echo /costmap/costmap --once
ros2 topic echo /costmap/costmap_updates --once
ros2 topic hz /costmap/costmap
ros2 topic hz /costmap/costmap_updates
```

再看 `costmap_occupied_threshold` 是否太高。

## 建议你现在先改哪几个

你当前最值得先改的是：

```yaml
robot_inscribed_radius: 0.05
forward_check_distance: 1.50
forward_sample_step: 0.10
monitor_period_sec: 0.50
min_replan_interval_sec: 0.50
```

先不要打开整条路径碰撞检查：

```yaml
enable_path_collision_check: false
```

等确认“前方障碍 -> 截断 -> 延后重规划”这条链路能跑通，再考虑是否打开整条路径检查。

