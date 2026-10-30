# 第一阶段实现交付记录（部分完成）

实施时间：2026-10-29～30。调查起点HEAD为`bdf74b3`，开始时工作树和暂存区干净。未提交、暂存、建分支、刷新历史清单或启动原机器人控制脚本。实际备份名为`transplant_ws_backup/`，未覆盖。以下是本轮新版本证据，不借用T02历史测试。

## 1. 已落盘范围及未完成门槛

| 切片 | 本轮实现 | 验收边界 |
| --- | --- | --- |
| S0 | 新包、核心库、依赖源码本地构建、来源与差异表、分区YAML、默认不驱动 | 本地构建通过；整体再分发许可仍待确认 |
| S1 | 全量/已绑定基底增量容器、粗细栅格面积融合、未知和窗口外语义、面积距离下界、普通8邻域A* | 核心专项通过；ROS增量与滚动基底关联仍未解决，不算实机输入接入完成 |
| S2 | 碰撞感知DP/弦化简、矩形路标走廊、段时长、五次minimum-jerk OSQP、C2/端点/残差检查、曲线误差界、有限插点/收紧、弧长重采样及最终折线复检 | 测试样例通过；不是任意地图都可成功的完备性或时间保证；部分故障注入仍待补齐 |
| S3 | 离线单文件快照、ROS地图/TF/显式目标、单工作任务及最新目标、调试路径/走廊/状态、最新图和起点漂移复检 | 隔离消息测试通过；不是原实机话题/QoS验证。增量基底、部分抢占/故障场景待验收 |
| S4/S5 | **未实施/未运行** | 没有MPC安全副本、执行许可、末端断流保护或受控实机移动证据；整体第一阶段未完成 |

`enable_execution=true`直接拒绝启动。节点不发布`/sPath`、`/cmd_vel`或其他执行接口；私有调试Path的清空不能冒充MPC停车。静态调试`static_only=true`忽略感知输入，但仍需地图和机器人TF；不允许靠该模式给实车绕过感知许可。

## 2. 文件与关键计算

新包统一位于`transplant_ws/src/tdt_path_planning/`：

- `CMakeLists.txt`、`package.xml`：一个核心库、离线/ROS入口和gtest，可选ASan/UBSan；依赖本机Humble/Eigen/OpenCV以及本地固定OSQP组合。
- `include/tdt_path_planning/core.hpp`和`src/grid.cpp`：参数与栅格校验、动态源容器、保守融合/净空。静态非零/-1阻塞；动态-1或≥65阻塞；窗口外保留静态层；后续动态自由格不能擦除静态占据。
- `src/search.cpp`：普通A*、禁止穿角、精确端点连接与化简，使用同一碰撞判据及资源预算。
- `src/optimizer.cpp`：归一化段时间下的五次minimum-jerk QP，微小对角正则；只接受Solved并复核残差、端点和C2。检测失败后沿安全参考插点或收紧走廊，耗尽即失败，不回退旧样条或未经认证折线。
- `src/snapshot_io.cpp`、`src/offline.cpp`：版本化输入快照及CSV输出；非法文件/参数退出2，规划失败退出1，成功退出0。失败时写CSV仅有表头，不保留旧成功路径冒充新结果。
- `src/node.cpp`：仅调试的ROS适配；地图必须在`global_frame`且轴对齐，其他frame/旋转明确拒绝。求解使用不可变快照；新目标作废旧目标，地图更新不一概取消，而是在发布前复检最新完整图。当前最终复检在回调内，**不得作为S4独立安全监控的实现**。
- `vendor/`：DynamicVoronoi三个文件原样导入；两个队列文件改为最小堆，消除固定距离上限导致的进程exit。用独占的`initializeEmpty`避免共享地图被二次释放。原来源编译警告保留，来源哈希及许可缺口见包内`SOURCES.md`。
- `cfg/planner.yaml`、`launch/debug.launch.py`：分区参数及非驱动启动；`test/core_test.cpp`、`test/ros_smoke.py`、`fixtures/`：核心/消息测试及离线示例。

工作区另修改`.gitignore`以排除新构建/安装/日志；更新状态、决策及阶段文档入口，AGENTS仅纠正实际备份目录拼写，未写入未确认方案或阶段参数。

### 数值与预算语义

- 车体0.60×0.60 m，余量0.08 m；所需面积净空由尺寸派生为约0.504264069 m，严格大于才通行。
- 距离查询是**保守下界**，饱和到`requiredClearance()+0.25 m`，不是精确全局距离。原格中心场减去查询偏移及格半对角线，同时由按行索引的局部占据面积检查约束；未知格和地图边界也限制净空。局部面积扫描不是全图暴力oracle。
- 线段按不大于`min(resolution/2,max_chord)`取样，检查时加半采样间距；曲线细分用二阶导数上界`M*du²/8`覆盖弦偏差。重采样后的新连线再检查。
- 速度/加速度参数仅分配内部段时长，单位m/s和m/s²；**不保证实车遵守该速度或加速度**。结果系数以`u∈[0,1]`表示，物理导数需除以段时长对应次幂；并非向MPC发送定时轨迹。
- 栅格单轴≤4096、总格数≤400万；控制点2～256、细化轮数1～64、检查规模2～100万。核心`Config`的双精度参数须有限，正值允许范围`[1e-6,60000]`，允许取零的非负参数另容许0；ROS整数先按64位读取并检查范围，避免截断成合法小值。此为数值/资源保护，不是实车参数范围。
- `qp_budget_ms`为每轴求解器时限；`total_budget_ms`包含核心搜索/化简/优化/检测，**不包含初始融合/距离场构建、文件I/O、DDS调度**。ROS末次验证另检查总预算，但不能据此承诺端到端硬实时。失败前未完成的阶段耗时字段可能仍为0；`total_ms`仅对应核心`plan`调用。
- 配置只在启动读取，在线改参数明确拒绝，需重启；不会“YAML写了但核心未使用”。输入过期/起点漂移阈值只是调试筛选，不是执行许可、停车距离或底层看门狗。

## 3. 实际运行结果及失败记录

证据保存在本机忽略目录`artifacts/phase1-20261029/`（跨日仍沿用同一批次名），不是提交内不可变证据或异机备份。

| 命令/检查 | 实际结果 |
| --- | --- |
| 本地OSQP/QDLDL及OsqpEigen源码构建安装到`/tmp/phase1-deps` | 成功；未联网升级。OsqpEigen CMake有原来源PARENT_SCOPE开发警告 |
| 新包colcon Debug构建 | 成功；初次完整编译有原样vendor未使用变量及未使用函数中的整数常量转换警告 |
| 最终colcon test / test-result | 15项gtest通过，CTest 1/1通过；test-result统计16包含外层测试条目，不能称16项独立核心用例 |
| `offline_planner fixtures/open.snapshot /tmp/phase1-open-path.csv` | 成功：11原始点→2化简点→58输出点，2次单轴QP，1轮；本次核心约8.77 ms，仅此小图观测，不是性能承诺 |
| `test/ros_smoke.py`，隔离域190/localhost | 完整复测通过：安装YAML实际加载、执行开关/非法速度拒绝；真实发布全量图/静态TF/目标，与导出的离线输入结果逐点一致；无执行发布者；非法目标、过期输入、未绑定增量安全拒绝。最终版本另加入超宽整数拒绝检查，结果见`ros-smoke-final.log` |
| ASan/UBSan/LSan，统一消毒器编译核心及OsqpEigen，沙箱外离线运行 | 15/15通过，无本次运行诊断；OSQP、OpenCV及系统库未全面消毒器化，不作其全覆盖承诺 |
| 三个原样导入vendor文件`cmp` | 与备份来源一致；两个队列文件为已说明差异 |

必须保留的失败经过：

1. 首轮普通测试12/13通过，末端重采样点有微小浮点偏移。修正输出末点赋值并重新做最终折线检测；没有降低测试精度掩盖问题。首轮另发现`setTimeLimit`无效，已修调用点并以极短QP时限拒绝测试覆盖。
2. 首轮消毒器混用普通OsqpEigen，Eigen分配/释放策略不一致导致越界；统一编译依赖后该问题消失。第二轮有1项搜索触及1秒预算及LSan沙箱ptrace错误；仅绕障/S弯几何测试显式使用5秒搜索/15秒总预算容纳插桩开销，**生产默认1秒/3秒不变**，失败预算测试保留。最终沙箱外15项通过。
3. 沙箱ROS运行出现接口枚举/UDP权限警告，但完整断言最终通过；不能仅凭前段错误认定全部失败。沙箱外域187启动检查10秒超时，域188的30秒复核也触发timeout；域189系统调用跟踪正常退出2，域190完整消息复测通过。超时根因未确认，不能将其抹去或称宿主启动已在所有域稳定。

核心覆盖：参数非法、负原点/边界/编码/数据长度、粗细图/未知/窗口外/静态持久/滚动替换、补丁容器失步、2000个固定种子查询对暴力面积oracle、长距离队列、车体边界、禁止穿角、搜索预算、重复点/窄通道、直线/斜线真实QP、绕障的独立C2/零端点速度及1001点/段曲线抽查、S弯安全成功或明确失败、QP迭代/时间限制、快照往返。曲线抽查不是连续碰撞证明的替代。

尚未完整覆盖：真实发布QoS/编码、ROS基底代际、乱序/重启、目标抢占时序和起点漂移故障注入、QP不可行/走廊不相交/细化耗尽的逐项强制分支、S4各进程断联与S5实机。S弯测试允许安全失败，不证明该例每次均成功；本次测试不是全部原计划门槛已满足。

## 4. 可复现命令

以下从项目根目录执行，使用本机已有ROS Humble/Eigen/OpenCV/gtests。`/tmp`依赖若被清理，先重新构建；未安装系统依赖的机器不能据此保证一次成功。

### 依赖（本轮实际执行的源码构建方式）

```bash
cmake -S tdt-nav-kit/osqp -B /tmp/phase1-osqp-build \
  -DCMAKE_INSTALL_PREFIX=/tmp/phase1-deps \
  -DFETCHCONTENT_SOURCE_DIR_QDLDL="$PWD/tdt-nav-kit/osqp/build/_deps/qdldl-src" \
  -DOSQP_BUILD_UNITTESTS=OFF -DOSQP_BUILD_DEMO_EXE=OFF \
  -DOSQP_ENABLE_INTERRUPT=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/phase1-osqp-build -j2
cmake --install /tmp/phase1-osqp-build
cmake -S tdt-nav-kit/osqp-eigen -B /tmp/phase1-osqp-eigen-build \
  -DCMAKE_PREFIX_PATH=/tmp/phase1-deps -DCMAKE_INSTALL_PREFIX=/tmp/phase1-deps \
  -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/phase1-osqp-eigen-build -j2
cmake --install /tmp/phase1-osqp-eigen-build
```

### 普通构建、核心与离线

```bash
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH=/tmp/phase1-deps/lib:$LD_LIBRARY_PATH
MAKEFLAGS=-j2 colcon --log-base /tmp/phase1-colcon-log build \
  --base-paths transplant_ws/src --build-base transplant_ws/build \
  --install-base transplant_ws/install --packages-select tdt_path_planning \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_PREFIX_PATH=/tmp/phase1-deps
colcon --log-base /tmp/phase1-colcon-test-log test \
  --build-base transplant_ws/build --install-base transplant_ws/install \
  --packages-select tdt_path_planning --event-handlers console_direct+
colcon --log-base /tmp/phase1-colcon-test-log test-result \
  --test-result-base transplant_ws/build --verbose
transplant_ws/install/tdt_path_planning/lib/tdt_path_planning/offline_planner \
  transplant_ws/src/tdt_path_planning/fixtures/open.snapshot /tmp/phase1-open-path.csv
```

### ROS非驱动（不得改成实机所在domain来跑合成输入）

```bash
export ROS_DOMAIN_ID=190 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/phase1-ros-logs
/usr/bin/python3 transplant_ws/src/tdt_path_planning/test/ros_smoke.py \
  --bin-dir "$PWD/transplant_ws/install/tdt_path_planning/lib/tdt_path_planning"
```

脚本自行拉起/退出调试节点，加载安装后的YAML；构造/map、/costmap、/tf_static、/goal_pose，不拉起MPC或串口。ROS smoke目前是单独运行项，**不包含在colcon的15项核心测试内**。同domain若存在其他节点，应换空闲的隔离域再运行。

### 消毒器（不要混用普通OsqpEigen）

```bash
cmake -S tdt-nav-kit/osqp-eigen -B /tmp/phase1-osqp-eigen-asan \
  -DCMAKE_PREFIX_PATH=/tmp/phase1-deps -DCMAKE_INSTALL_PREFIX=/tmp/phase1-deps-asan \
  -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Debug \
  '-DCMAKE_CXX_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer'
cmake --build /tmp/phase1-osqp-eigen-asan -j2
cmake --install /tmp/phase1-osqp-eigen-asan
cmake -S transplant_ws/src/tdt_path_planning -B /tmp/phase1-sanitized \
  '-DCMAKE_PREFIX_PATH=/tmp/phase1-deps-asan;/tmp/phase1-deps' \
  -DTDT_SANITIZE=ON -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON
cmake --build /tmp/phase1-sanitized --target core_test offline_planner -j2
LD_LIBRARY_PATH=/tmp/phase1-deps-asan/lib:/tmp/phase1-deps/lib:$LD_LIBRARY_PATH \
  ASAN_OPTIONS=detect_leaks=1 /tmp/phase1-sanitized/core_test
```

LSan在本次沙箱ptrace环境不能完成，最后一条经授权在沙箱外执行。不要用关闭泄漏检查代替本次已完成的LSan验收。相应日志包含早期失败与最终结果。

## 5. 接入暂停点与下一步

所查`/home/hustlyrm/slam_source/odom_hyx/src/dlil/launch/dlil.launch.py`配置10 Hz发布但未设置`always_send_full_costmap`；本机Nav2默认false。标准增量消息没有可绑定滚动基底的epoch，本轮未凭时间戳猜测其对应关系。**ROS适配收到增量后等待全量，不算正常增量接入完成。**活动实例是否另有覆盖尚待核对。

需要确认：是否允许后续在实机启动配置中开启`always_send_full_costmap=true`，先保持现有OccupancyGrid话题/编码贯通基线（带宽增加），而不是本轮扩建代际同步接口？本次没有自行改动上游。确认并完成S3真实数据验收后，再做S4的参考/会话关联、独立硬限速、停车确认、许可/命令断流保护；随后由现场确认速度、超时和制动距离，才可安排S5。0.20 m/s仍只是候选，8 cm不是制动保证。
