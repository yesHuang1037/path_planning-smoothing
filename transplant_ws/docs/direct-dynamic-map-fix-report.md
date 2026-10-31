# 地图更新导致无法规划或路径消失：问题与修复汇总

## 1. 状态与适用范围

- 日期：2026-10-31；适用于本工作区本次未提交的修改。
- 需求入口：`docs/direct-dynamic-map-requirements.md`。
- 用户确认：完整动态地图直消费；保留时效检查；输入不足立即拒绝目标，恢复后重发；新目标撤销旧结果。
- 已完成：节点最小修复、计算回归、隔离 ROS 消息链路回归、文档更新。
- 未完成：实机地图流的频率/QoS/时间戳验收、实际 RViz 窗口观察及实机执行安全验收。消息链路通过不等于已经观察到实车 RViz 或允许执行。

## 2. 问题及原因

用户观察到 `poses=[]`，增量提示反复覆盖状态，发目标时出现笼统的 `map/costmap missing or stale`。

源码中存在三个独立原因：

1. 增量回调不拼接数据，而是每次清除动态缓存，使原本收到的完整图也不可用。
2. 静态图、完整动态图和增量回调都清空三条 Path 和走廊。即使规划成功，下次地图消息也会擦掉展示，且地图更新并不触发重新规划。
3. 动态接收时间只由完整图更新；仅注释增量回调的 reset，不会阻止空消息发布，也不会使旧完整图重新满足时效。固定增量提示还会掩盖真正的目标拒绝原因。

另一个相关问题是优化失败时 `Result` 可能保留非空 raw/simplified，原节点仍会发布中间结果，不能保证新目标失败后全部输出保持为空。

本次从修改前源码重新构建后，用输出计数回归观察到：

```text
AssertionError: map callbacks published output after successful planning
```

这是本次源码级缺陷的复现证据；不能据此断定用户现场地图本身有错。

## 3. 修复后的行为

| 事件 | 地图/任务状态 | 路径及走廊 |
| --- | --- | --- |
| 收到合法完整静态/动态图 | 转换校验成功后替换对应缓存；动态图更新接收时间及消息时间戳 | 不清除、不重发旧结果 |
| 收到增量 | 节点不再订阅，不作为新鲜度信号 | 不影响输出 |
| 完整地图转换失败 | 清除对应缓存，保留具体拒绝原因；新规划不得使用 | 不擦除已有历史展示 |
| 动态图过期 | 输入检查失败；日志按状态变化报告 | 不擦除已有历史展示 |
| 新目标到来 | 递增代次，替换待处理目标 | 立即发布三条空 Path 和走廊 DELETEALL |
| 新目标输入不足 | 消费并拒绝本次目标，提供具体原因 | 保持为空；恢复输入不自动重试 |
| 本轮规划失败 | 不发布部分 raw/simplified 等产物 | 保持为空，不恢复旧目标 |
| 旧异步任务完成 | 目标代次不符则忽略结果及异常状态 | 不得覆盖新目标 |
| 本轮成功且最终复核通过 | 接受结果 | 发布三类 Path 和走廊，保持到下个目标 |

继续保留：静态底图与动态障碍融合、非法数据校验、动态时效、TF时效、起点漂移、最新环境最终路径碰撞检查、计算预算和执行禁用。

输入健康由节点内部 `inputState()` 统一判断，启动规划和发布前共用。`reportInputState()` 仅在输入状态变化时输出日志，不用地图消息反复覆盖目标的 `~/status`。年龄错误附带实际秒数、限制和所用时钟。

正常地图回调不再调用 `clearOutput()`；该函数仅在新目标回调使用。未新增自定义消息、参数、服务、状态管理框架或新算法模块。

## 4. 文件清单及复用

| 文件 | 修改目的 |
| --- | --- |
| `src/tdt_path_planning/src/node.cpp` | 删除增量订阅；转换后替换地图；拆分输入错误；解除地图与输出清理的绑定；抑制失败中间结果及旧任务状态回写 |
| `src/tdt_path_planning/cfg/planner.yaml` | 只补时效单位与完整图语义注释，保留 0.5 s 等所有值 |
| `src/tdt_path_planning/test/core_test.cpp` | 新增快照隔离/完整图替换/最新环境拒绝旧路径回归；强化优化失败保留中间产物的测试前提 |
| `src/tdt_path_planning/test/ros_smoke.py` | 复用原隔离测试；监听三种 Path、走廊和状态，增加保留/撤销/输入异常/异步替换等测试 |
| `docs/direct-dynamic-map-requirements.md` | 记录已确认选择，将修改前代码事实明确标为历史快照 |
| `docs/direct-dynamic-map-fix-report.md` | 本汇总，供整套任务检查修改范围、证据和后续验收 |

复用了 `convert()`、`Grid::validate()`、`clearOutput()`、发布函数、目标代次、`pending_`、`work_`、`Snapshot`、`Environment`、`plan()`、`pathSafe()` 和快照重放。没有修改核心算法实现、CMake、包依赖或启动接口；`map_msgs` 仍由 ROS 回归用于主动注入增量。

## 5. 实际运行命令与结果

以下命令在 `transplant_ws` 下执行；涉及 DDS 的命令获得沙箱外运行许可。没有提交、建分支、暂存或修改外部工程。

### 5.1 初始检查和输入取样

```bash
git status --short
git diff --stat
git diff -- src/tdt_path_planning/src/node.cpp
git rev-parse --show-toplevel
git status --porcelain=v1 --untracked-files=all
source /opt/ros/humble/setup.bash
ROS_LOG_DIR=/tmp/tdt-map-input-probe /usr/bin/python3 /tmp/tdt_map_input_probe.py
```

初始 Git 无已跟踪文件修改；已有未跟踪的工作区需求文档和工作区外 `../docs/phase1-baseline/map-update-review.md`。前者按本任务更新，后者未触碰。

取样脚本是本轮 `/tmp` 中的只读工具，仅订阅两种完整地图、统计消息并查询发布者 QoS。沙箱内出现 DDS socket 权限错误，获准后重跑无权限错误，但当前默认 ROS 域 6 秒内两种话题接收数均为 0，也没有发现发布者。因此不能获得实际频率/QoS证据，更不能把“未发现”解释为地图错误。

### 5.2 构建、核心回归与静态检查

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select tdt_path_planning --event-handlers console_direct+
build/tdt_path_planning/core_test --gtest_filter=Fusion.FullSnapshotCopyAndLatestMapValidation:Optimizer.LimitsFailClosed
colcon test --packages-select tdt_path_planning --event-handlers console_direct+
colcon test-result --verbose
/usr/bin/python3 -m py_compile src/tdt_path_planning/test/ros_smoke.py
git diff --check
```

- 当前源码构建成功，安装目录二进制和 YAML 已更新。
- 两个针对性核心测试通过。
- 完整 GTest：16 个用例、8 个测试套件全部通过；CTest 注册项为 1 个 `core_test`。`colcon test-result` 汇总为 17 tests、0 errors、0 failures、0 skipped，不能将其误写为 17 个独立 GTest 用例。
- Python 语法检查及 diff 空白检查通过。
- ROS 脚本未注册为 CTest，下面是单独执行结果，不能仅凭 colcon test 声称其已运行。

构建和核心回归的本机临时输出保存于 `/tmp/tdt-map-build-final.log` 和 `/tmp/tdt-map-core-regression.log`；临时日志不是异机备份。

### 5.3 隔离 ROS 回归

```bash
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/tdt-map-regression \
  /usr/bin/python3 src/tdt_path_planning/test/ros_smoke.py \
  --bin-dir "$PWD/install/tdt_path_planning/lib/tdt_path_planning"
ROS_DOMAIN_ID=177 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/tdt-map-regression \
  /usr/bin/python3 src/tdt_path_planning/test/ros_smoke.py \
  --bin-dir "$PWD/install/tdt_path_planning/lib/tdt_path_planning"
```

最终脚本在两个隔离域分别完整通过，退出码均为 0。两次实际执行还使用 `set -o pipefail` 配合 `tee` 保存了 `/tmp/tdt-map-ros-regression.log` 和 `/tmp/tdt-map-ros-regression-repeat.log`。

已实际通过的检查：

- 启动拒绝非法/执行参数，安装 YAML 的参数确实加载。
- 有效输入产生三条非空路径和非空走廊；在线输出与保存快照的离线计算逐点一致。
- 完整静态/动态更新及持续增量不清输出，不覆盖目标状态；节点无增量订阅。
- 过期和畸形地图不擦历史展示；新目标清空旧结果，再被明确拒绝。
- 未收到静态/完整动态图、转换失败、接收超时、零/旧/未来时间戳均产生对应拒绝原因。
- 增量不能刷新完整图时效；输入恢复不自动重试目标。
- 非法端点、QP失败不发布部分结果。
- A/B/C 快速目标替换后仅最新目标发布非空结果；测试显式确认替换已被接收时旧任务尚未报告成功，避免将顺序规划误当成并发覆盖验证。
- 规划期间输入新障碍后，最终环境复核拒绝旧候选路径；起点漂移和陈旧动态 TF 仍被拒绝。
- 显式 static_only 模式保持可用；没有 `/sPath` 或 `/cmd_vel` 发布者。

### 5.4 过程中出现的失败及处理

- 首次沙箱 DDS 权限错误：请求许可后在隔离域运行，不绕过沙箱限制。
- 旧安装产物运行结果不能直接代表当前源码；重新构建修改前源码后，输出保留测试按预期变红。该基线失败属于目标缺陷，不是最终修复残留失败。
- 原测试收到状态就立即检查另一话题，存在跨话题到达顺序假设；改为等待各输出实际到达，再断言空结果。
- 隔离域 174 的一次启动拒绝子进程超过 10 s，原因未能确定；在域 175 单独启动同一拒绝用例正常返回 2。最终完整回归在域 176/177 通过；没有通过增加超时或修改节点掩盖这次失败。
- 初版陈旧 TF 测试错误地向 `/tf_static` 发布旧时间戳，该场景仍允许规划；调查后确认静态 TF 不适合作为年龄拒绝用例。改用独立机器人坐标系的 `/tf` 动态变换后通过，未修改节点 TF 语义。

## 6. 后续实机检查入口与限制

部署时需在实机所用 ROS 域加载本工作区安装环境并重启节点；不只是编辑源码。以下是建议手动检查命令，本轮没有启动实机 RViz 或实机导航：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tdt_path_planning debug.launch.py
ros2 topic echo /snapshot_planner/status
ros2 topic echo /snapshot_planner/optimized_path --field poses
```

检查顺序：有效输入下发目标并观察非空路径/走廊；保持地图更新但不发新目标，结果不应消失；发无效目标，旧结果应清除且给出失败原因；修正输入后须再次发送目标。

仍存在的明确限制：

1. `max_input_age_s=0.5` 未放宽。若现场完整图周期、延迟或时钟不满足，它仍会拒绝规划；必须结合新诊断实测，不应再用持续增量作为健康证明。
2. 地图订阅 QoS 保持 reliable/transient-local，尚未与实机发布者验证匹配；若不匹配，应先报告并确认适配，不随意改接口或上游。
3. 保留的是历史几何结果，可能已不安全。只有新一轮成功候选在发布前被复核；没有持续安全认证、地图触发重规划或实车执行许可。
4. 输出 QoS 未改为 transient-local；后启动/重连的 RViz 不保证收到旧路径。当前消息测试只证明已订阅端可获得并保留输出，不证明 GUI 实际渲染通过。
5. 异步旧任务不立即取消，最新目标可能等待旧计算结束；不新增抢占/自动等待重试。
6. 未运行实机执行、GUI渲染、压力测试或本次专用 sanitizer；不以本轮通过替代这些验收。

## 7. 后续现场排查：完整图过期（2026-10-31）

用户再次提供日志：收到完整图时消息年龄约 362.451 s，随后接收年龄超过 0.5 s，发目标时接收年龄已为 183.370 s / 237.570 s。这说明曾收到可转换的完整图，但不是持续新鲜完整图；后者年龄按 steady_clock 计算，不能单用 ROS 时间差解释。

本轮获准执行只读现场查询，实际结果：

- `ros2 param get /costmap/costmap always_send_full_costmap` → `False`。
- `ros2 param get /costmap/costmap publish_frequency` → `10.0`。
- `ros2 param get /costmap/costmap use_sim_time` 与 `/snapshot_planner use_sim_time` → 均 `False`。
- `ros2 lifecycle get /costmap/costmap` → `active [3]`。
- `timeout 8 ros2 topic echo /costmap/costmap --field header --qos-durability transient_local --once` → 未收到消息，到时退出。
- `timeout 8 ros2 topic hz /costmap/costmap_updates` → 约 5.8..6.2 Hz，说明更新主要在增量通道继续到来。
- 初次短时 `topic info` 的完整图/增量发布者数均为 0，但参数服务及后续增量接收成功，故不能以该短时发现结果认定发布者不存在。

结论：现有“持续增量”的上游发布方式与本节点“只消费新鲜完整图”的输入契约不匹配。地图内容不一定错误，亦非本轮路径展示修复已证明实机契约满足。此前隔离回归持续提供完整图，并未覆盖当前现场发布模式；此项现在有实际证据。

与已确认方案一致的修复方向是让实际 costmap 启动配置提供 `always_send_full_costmap: true`，然后重启/按正确生命周期重新配置并实测完整图和时间戳；仅有 `publish_frequency=10` 不保证完整图持续发布。该项涉及外部 costmap 启动配置，超出本工作区授权，本轮未修改、未运行 param set 或重启任何节点，也不擅自取消时效检查。若用户要求不改外部发布配置，需要重新确认输入适配设计，不能悄悄恢复增量拼接。
