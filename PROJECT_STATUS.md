# 项目状态与接续入口

更新日期：2026-10-25。

## 总体目标

在现有 ROS 2 Humble 导航框架内，分阶段移植 TDT 普通 A*、路径化简和轨迹平滑思路，保持与现有感知、执行和 MPC 链路衔接，最终通过实机验证。规划与平滑 15～20 ms 是初步目标，尚无统一测量口径或端到端达标证据。需求以 [docs/requirements.md](docs/requirements.md) 为准；`Phase1Spec.md` 是早期草案，不自动转为实现要求。

## 已完成与验证边界

| 阶段 | 已完成 | 验证及限制 |
| --- | --- | --- |
| T01 调查 | 现有规划链路、TDT 对照及移植边界的静态调查 | 见 [T01 交付](docs/T01-delivery.md)；不是实机联调 |
| T02-A，2026-10-24 | 隔离副本内可选碰撞感知 DP / 弦化简；默认仍为 legacy | 历史日志确认 Debug / 最终 colcon 构建成功，CTest **5/5**；独立化简算法 ASan / UBSan / LSan 通过。见 [T02 交付](docs/T02-delivery.md) 及 `docs/baseline/evidence/verify-tests.log`、`docs/baseline/evidence/sanitizers-outside-sandbox.log`。消毒器不覆盖 ESDF / ROS 全生命周期，整个 T02 未完成 |
| 本地候选基线，2026-10-25 | 源码、来源、导入差异和精选历史证据已暂存 | 历史核验：当前包 62 文件、原导入 58 文件可反向重建、证据 25 文件、暂存及导出 105 文件一致。见 [基线交付](docs/baseline-delivery.md)。没有 commit、标签、远端或推送；未重测算法 |
| 本轮记录整理 | 核对 DynamicVoronoi、A*、平滑、安全检查调用关系，建立长期约束、决策和接续记录 | 只改文档，不改算法、参数、启动文件或原导航工程。A* 也直接使用距离场，纠正此前解释中的遗漏；没有新增 ROS / 实机测试 |

本轮内容核验：`python3 scripts/verify_baseline.py` 通过，范围为既有规划包、历史证据和 105 个暂存文件，不包含本轮新建的三个未暂存文档。三个新文档另作本地链接和空白检查。本轮未重跑构建、CTest 或消毒器；本表算法通过项均为历史结果，不是本轮重测。

注意：`docs/T02-delivery.md` 中“根目录没有 Git 元数据”是 2026-10-24 实施时的历史状态；2026-10-25 已初始化 Git，当前仍无 HEAD 提交。整份暂存内容的 `git diff --cached --check` 历史未通过，原因是导入旧源码及存档补丁的空白问题，未在本轮修复。

## 当前未解决的问题

- **已知安全边界**：新化简只认证新连线；原脱困边及最终梯度 / 样条曲线没有整体安全认证。失败时不撤销旧 `/sPath`，A* 成功状态也不等于最终轨迹成功；撤销与状态协议待决策。
- **已知所有权风险**：ESDF 与 DynamicVoronoi 析构均释放共享地图。既有接线测试使用 no-op deleter 绕开生命周期，不能作为进程退出或无泄漏证明；本轮未运行复现 / 修复。
- **已知适用限制**：部分 ESDF / 平滑坐标换算硬编码 0.05 m，不支持据此宣称全链路多分辨率或旋转地图支持；DP 无硬实时上限。DynamicVoronoi 等既有编译警告未处理。
- **待验证**：实际运行的 DLIL / 导航工作空间与二进制、实际轮系及物理约束、真实地图与感知膨胀 / 清除语义、完整 ROS 参数和发布订阅联调、MPC 跟踪效果、NUC 真实负载耗时。
- **环境与版本限制**：历史 colcon / LSan 沙箱内失败、沙箱外成功，底层沙箱等待机制未定论；不能记为已修复。当前候选基线仍可变且无异机备份。

## 下一阶段目标

先确定 T02 后续最小切片和安全验收边界，再实施；不把“讨论 DynamicVoronoi”升级成更换该模块的决定。优先澄清最终输出安全、失败后旧参考处理及当前运行版本，再讨论扩大算法替换范围。此顺序是接续建议，尚非新增批准任务；默认保持 legacy，不自动切换实机。

## 下次开始时检查什么

1. 先读本文件、`docs/decisions.md`、`docs/requirements.md`，再按需读 `docs/T02-plan.md`、`docs/T02-delivery.md`；不要按早期草案扩大范围。
2. 执行 `git status --short`、`git diff`、`git diff --cached --stat`，区分历史暂存和新增工作树改动。读 `docs/baseline/README.md` 后运行 `python3 scripts/verify_baseline.py`；该脚本只核对既有暂存项，不能替代对新文件的检查，也不要刷新固定清单掩盖差异。
3. 按已选切片核对 `transplant_ws/src/path_searching/src/plan_manager.cpp`、`src/Astar.cpp`、`src/smoother.cpp`、`src/esdf_map.cpp`、`src/dynamicvoronoi.cpp`、`src/path_safety_checker.cpp`，以及 `include/path_searching/path_simplifier.h`、`src/path_simplifier.cpp`、`cfg/plan_param.yaml` 和相关测试。此处 `src/` 等后续路径均相对该规划包。
4. 新代码需离线复测时，从工作区根目录执行 `bash scripts/verify_t02.sh`；脚本包含配置、构建、五项测试和观测导出，不启动车辆控制。会覆盖 `artifacts/T02/` 同名日志，先保留需要对比的证据；历史归档在 `docs/baseline/evidence/`，不应覆写。
5. ROS / 实机验证前先核对实际工作空间、launch、参数、frame 和参考接收 / 停用方式，明确批准范围；不要未经确认启动控制链路。

**下一步最合理的操作**：以现有交付中的安全边界为起点，先静态追踪 `plan_manager.cpp` 的最终曲线发布 / 失败分支与现有 MPC 参考接收 / 停用逻辑，核对实际运行版本，形成一个可确认的最小安全闭环任务。源码能回答的先查，不直接整体替换 A*、DynamicVoronoi 或 MPC。
