# T02-A 交付：碰撞感知路径化简

日期：2026-10-24。依据：`requirements.md`、`T01-delivery.md`、已确认的 `T02-plan.md`。本文在副本构建和测试结束后创建。

## 1. 结果与保护范围

**已完成 T02-A 的隔离副本实现及离线测试，不代表整个 T02、第一阶段或实机验收完成。**

调用链：`PlanManager` 参数 → `smoother::configureSimplification`；地图回调 → ESDF / 动态管理器 / PathSafetyChecker → 当前栅格可通行查询；原 A* 点列 → `smooth` → 新模式 `pathResampling` → DP / 弦化简 → 原梯度平滑 / 三次样条 → 原 `/sPath`。

- 将原工程当前工作树的 `path_searching` 完整复制到 `transplant_ws/src/path_searching`，复制时逐项 diff 无差异；不是从 HEAD 覆盖原有参数修改。
- 当前工作区根目录没有 Git 元数据；原 navigation_ws 的七项已有修改保持原样。原规划包 **58 个文件**逐文件 SHA256 与复制前清单一致；navigation、TDT、SLAM 的 tracked diff 哈希均与修改前一致。证据：`artifacts/T02/preservation.log`、`package.before.sha256`、`*-diff.before.sha256`。
- 未修改原 navigation_ws、TDT、DLIL 或 MPC；未修改副本的 A*、动态融合、安全检查、重规划协调器、样条实现及 ROS 消息；未启动车辆控制链路，未提交 Git。

## 2. 文件与目的

下表规划包路径均相对于 `transplant_ws/src/path_searching/`；未列出的复制文件不作功能修改。

| 新增 / 修改 | 文件 | 目的与复用 |
| --- | --- | --- |
| 新增 | `include/path_searching/path_simplifier.h`、`src/path_simplifier.cpp` | 提取 TDT DP/弦思路，不整体搬运 YAstar；复用 GridGeometry，注入逐格查询；迭代 DP 避免递归深栈，supercover 覆盖端点、角点、格线两侧 |
| 修改 | `include/smoother.h`、`src/smoother.cpp` | 保留 `smooth` / `pathResampling` 调用接口，新增配置和环境注入；legacy 有效输入沿原计算路径，新模式处理重复/单点并明确拒绝非法输入 |
| 修改 | `src/plan_manager.cpp` | 声明参数；复用当前 ESDF 占据检查与 PathSafetyChecker 净空（包含现有动态 costmap 查询）；处理异常时发布 reachable=false，不把非法输入当普通空结果回退 |
| 修改 | `cfg/plan_param.yaml` | 新增 `path_simplification_mode: legacy`、`path_simplification_tolerance_m: 0.1`，默认不开启新算法；avoidance_mode 等原参数不变 |
| 修改 | `CMakeLists.txt` | 编入独立算法，注册两个新测试；保留原三项测试，不新增依赖或变更版本 |
| 新增 | `test/path_simplifier_test.cpp` | 算法边界、确定性随机覆盖 oracle、长路径和 CSV 导出 |
| 新增 | `test/smoother_simplification_test.cpp` | legacy 兼容、新模式接线、实际 ESDF/净空复用、非法配置和局部耗时比较 |
| 新增 | 工作区 `scripts/verify_t02.sh`、`scripts/render_t02_observations.py` | 仅构建/离线测试，导出 CSV 和标准库生成的 SVG，无 ROS 节点启动、无需绘图库 |
| 文档 | `docs/T02-plan.md`、本文件、`docs/requirements.md`、`docs/T01-delivery.md` | 记录批准范围、实际交付和状态，修正 T01 调试图片写出风险的源码事实 |

## 3. 行为与非法参数

- 模式仅接受 `legacy` / `tdt`；阈值必须有限且非负，零有效。未知模式、负数、NaN/Inf 显式抛异常，ROS 参数类型由既有声明约束。本轮验证的是配置函数，不声称运行了 ROS 参数进程测试。
- 新模式拒绝空路径、非有限/不可表示/图外坐标、无效地图与缺少查询；先校验再转整数索引。相邻重复点去重，全重复返回单点，双点保留原边；缺少平滑 ESDF、非法平滑权重显式报错。
- 新捷径必须逐格满足占据和现有净空判据；不能证明安全时保留原点，不增加脱困豁免。查询读取当前环境，动态出现/清除不缓存旧可通行结论。
- 化简输出保持原始首尾、顺序子序列和不增加点数。**仅新连线获验证，保留的原边不重新认证**；单/双点也不表示整条路径安全。弦阶段不保证 DP 阈值是最终全局偏离上限。
- 处理异常不会发布新的 `/sPath`，也不发送空 Path 撤销旧参考；旧 MPC 参考策略不变。原 reachable=true 在 A* 成功后先发布，因此后续处理失败可能再发布 false；本轮不将其解释成最终轨迹安全状态。

## 4. 实际运行与结果

命令从工作区根目录执行；ROS 构建/测试前均 `source /opt/ros/humble/setup.bash`。日志位于 `artifacts/T02/`。

| 检查 | 实际结果 / 日志 |
| --- | --- |
| 原副本基线 | 直接 CMake 构建成功；原三项 CTest **3/3 通过**，`baseline-direct-build.log`、`baseline-tests.log` |
| 修改后 Debug 构建 | 配置、编译和链接成功，`configure.log`、`build.log`；最终 colcon 构建及本工作区安装成功，`colcon-final.log`，退出码 0 |
| 最终复测脚本 | `bash scripts/verify_t02.sh` 实际运行；CTest **5/5 通过、0 失败**，总耗时 1.65 s，`verify-tests.log`；脚本另运行两个测试导出观测 |
| 新算法用例 | 直线、障碍、角点/格线/端点、贴边/短线、负坐标/非零原点、非法值、重复/短路径、动态出现/清除、原脱困边保留、窄通道、5000 点长路径通过；额外 0.025/0.1 m 只验独立化简模块 |
| 栅格覆盖 oracle | 固定种子 20261024、160 对线段，用独立闭矩形相交算法检查未漏格，正反方向覆盖集合相同；不是所有地图/数值的形式化证明 |
| 接线用例 | 默认/显式 legacy 预处理及 smooth 结果相同；新模式真实 ESDF 占据、当前净空配置、异常和有限样条输出样例通过，`integration-observations.log` |
| 内存/未定义行为 | 独立算法 ASan + UBSan + LeakSanitizer 在沙箱外保留全部检查后成功退出，无诊断，`sanitizers-outside-sandbox.log`；不覆盖整个 ROS / ESDF 生命周期 |
| 脚本与保护 | `bash -n`、Python AST、SVG XML 解析成功；原包文件/原仓库差异保护检查通过；未通过图形查看器人工审图 |
| 测试驱动中的失败 | 直线未化简、障碍捷径未拒绝的初始实现各有一次预期 RED，之后实现对应根因并复测通过：`red-straight.log`、`red-obstacle.log`、`green-*.log`；不是最终遗留失败 |

关键命令（读源码还使用了 `rg`、`sed`、`cat`、`diff -qr/-u`，工作树检查使用了 `git status --short`、`git diff --binary | sha256sum`）：

```bash
cp -a /home/hustlyrm/source/navigation_ws/src/path_searching transplant_ws/src/
diff -qr /home/hustlyrm/source/navigation_ws/src/path_searching transplant_ws/src/path_searching
timeout 180 cmake --build transplant_ws/build-baseline/path_searching --parallel 2
ctest --test-dir transplant_ws/build-baseline/path_searching --output-on-failure
bash scripts/verify_t02.sh
```

基线的配置由以下 colcon 命令生成，但该次调用没有成功结束；不可作为 colcon 基线通过记录：

```bash
colcon --log-base artifacts/T02/log-baseline build --base-paths transplant_ws/src \
  --build-base transplant_ws/build-baseline --install-base transplant_ws/install-baseline \
  --packages-select path_searching --executor sequential \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Debug
```

最终实际执行并成功的 colcon 命令（经批准在沙箱外执行，stdout/stderr 写入 `colcon-final.log`）：

```bash
timeout -k 5 60 colcon --log-base artifacts/T02/log-colcon-final build \
  --base-paths transplant_ws/src --build-base transplant_ws/build-t02 \
  --install-base transplant_ws/install-t02 --packages-select path_searching \
  --executor sequential --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Debug
```

消毒器的实际编译和最终复测命令：

```bash
g++ -std=c++17 -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer \
  -Wall -Wextra -Wpedantic -I/usr/include/eigen3 \
  -Itransplant_ws/src/path_searching/include \
  transplant_ws/src/path_searching/test/path_simplifier_test.cpp \
  transplant_ws/src/path_searching/src/path_simplifier.cpp \
  -o artifacts/T02/path_simplifier_sanitized
ASAN_OPTIONS=detect_leaks=1 artifacts/T02/path_simplifier_sanitized
```

### 异常定位：环境与本次修改分开

1. **colcon 沙箱等待问题**：未改既有副本源码时，基线在 CMake 配置生成结束后持续等待，最终人工中断（130）。改后同命令在沙箱内显示全部目标 built 后仍超时（124，`colcon-sandbox.log`）；沙箱外相同源码/缓存成功完成构建、安装和退出，且直接 CMake 两种版本均成功。定位到沙箱/子进程收尾环境边界，**没有源码编译失败证据**；具体底层等待机制未进一步定论，不声称已修复沙箱。`strace -f -e trace=process` 在沙箱内因 ptrace 权限失败，经批准在沙箱外记录成功构建进程，见 `colcon-process.trace`、`colcon-check.log`。最小 `colcon_core.subprocess.run(cmake --version)` 探针成功，故不是所有 asyncio 子进程都失败。
2. **LeakSanitizer 沙箱限制**：算法断言执行完毕后，沙箱内 LSan 报 `does not work under ptrace` 并失败（`sanitizers.log`）；沙箱外原样保留 `detect_leaks=1` 后成功。没有关闭检查、删除测试或调整依赖版本。
3. **原有编译警告**：DynamicVoronoi 的整数溢出/未使用变量、原 smoother 的符号比较等在基线已存在。没有为消除警告而修改无关算法。

## 5. 复现与观察

运行 `bash scripts/verify_t02.sh` 会配置/构建副本，运行五项测试，打印局部对照，生成 `artifacts/T02/observations.csv`、`observations.svg`；无地图服务器、控制节点或机器人话题发布。脚本遇错退出，不把编译失败后旧二进制的运行计为通过。

本轮实际数据：`free` **4→2**；`blocked` **3→3**（保留绕行）；`cleared` **3→2**（清除后允许捷径）。SVG 灰虚线为输入、蓝线为化简结果、红方格为 0.05 m 占据格；可在编辑器/浏览器打开观察。SVG 是 CSV 渲染，不是最终样条或实车观测。

同一 5000 点自由直线、Debug、30 次均值：原 pathResampling **5000→3，26.5541 ms**；新模式 **5000→2，10.1079 ms**。两模式判据不同；这是单个合成场景的预处理观测，不包含搜索/梯度/样条/ROS/MPC，不是性能收敛或 15～20 ms 端到端保证。

## 6. 状态及剩余问题

| 状态 | 内容 |
| --- | --- |
| 已确定 / 已完成 | T02-A 最小范围、隔离副本、默认 legacy、复用现有环境接口、化简及接线离线用例；不是 YAstar 全量替换 |
| 待确定 / 未运行 | 实际 DLIL 工作空间/二进制身份，真实地图与负载、完整 ROS 发布订阅联调、参数进程启动、最终样条逐段安全、跟踪效果、整车/NUC 性能；不能记为通过 |
| 待决策 | T02 余下 YAstar/代价移植范围；T03 MinimumSnap/参考生成方案；失败时撤销旧参考与最终成功状态的协议；实际部署切换 |
| 已知限制 | 原 ESDF/梯度部分仍硬编码 0.05 m，未增加旋转地图支持；不能把独立模块多分辨率测试扩展为全链路支持；DP 最坏仍可能平方复杂度，无硬实时上限 |
| 原有生命周期问题 | ESDF 析构与 DynamicVoronoi 都释放共享 gridMap，存在重复释放所有权风险；既有测试以 no-op deleter 隔离地图生命周期，新接线测试沿用该夹具。未修复原模块，也不把此夹具通过当作进程退出/泄漏验证；消毒器通过仅指独立算法 |

T01 勘误：当前原 `src/esdf_map.cpp` 的调试函数仍被调用，但 `cv::imwrite` 已注释，不会按该行写出调试 PNG。本轮直接核对源码，没有为此修改 ESDF 或原导航目录。
