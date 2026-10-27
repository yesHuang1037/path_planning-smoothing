# 项目状态与接续入口

更新日期：2026-10-28。本文反映当前磁盘状态；历史交付与测试只在其原版本范围内有效。

## 当前总体目标

复用现有 ROS 2 Humble 的 odom_hyx 定位/感知和 MPC 执行链，在新隔离工作区建立 TDT 风格的普通 A*→碰撞感知化简→minimum-jerk QP→检测迭代基线，最终完成受控实机移动，不重建整套导航。

当前唯一阶段入口为 [docs/phase1-baseline/](docs/phase1-baseline/README.md)，内含策略、需求、实现计划及参数约定。已确认沿用栅格话题、不接点云，以当前完整快照显式规划；两种调试入口共用核心。完整周期/停滞/碰撞自动重规划后置，危险先停、不自动恢复旧参考。具体技术选择与权衡见 [决策](docs/decisions.md)，不在本文件重复参数表。

## 已完成及验证结果

| 阶段 | 已完成 | 验证边界 |
| --- | --- | --- |
| T01 原链调查，2026-10-24及后续补查 | 导航/TDT调用关系、用户启动脚本、地图/TF/MPC接口静态核对 | [T01交付](docs/T01-delivery.md)、[后续调查](docs/original-path-searching/runtime-map-and-replan-followup.md)；不是活动节点或实机证明 |
| T02-A旧隔离实现，2026-10-24 | 可选碰撞感知DP/弦化简，保留旧搜索/平滑，legacy默认 | 历史构建及CTest 5/5，独立化简ASan/UBSan/LSan通过；见 [交付](docs/T02-delivery.md)。不覆盖ESDF全生命周期、QP或新基线 |
| 历史候选基线，2026-10-25 | 来源、固定清单、反向补丁和精选历史证据 | 原记录为62文件、58文件反向重建、25证据及105暂存/导出一致；仅当时通过，不代表当前核验通过，见 [基线交付](docs/baseline-delivery.md) |
| 新策略需求与计划，2026-10-28 | 统一阶段文档，明确快照规划、调试/执行、安全边界和S0～S5切片；旧讨论稿已标记替代 | 已做9份Markdown的本地链接及行尾空白检查；本轮再次检查记录文档。没有新代码、参数YAML或启动改动，没有新构建、算法测试、ROS或实机测试 |

### 当前磁盘与本轮一致性检查

- `transplant_ws_bakup/src/path_searching` 存在；`transplant_ws/` 尚不存在。新包 `transplant_ws/src/tdt_path_planning` 和MPC安全副本仍是计划，不是已实现。
- Git当前HEAD为 `47b16dd`；`git diff --cached --stat` 为空，无相对HEAD的新暂存改动。工作树包含既有文档修改、旧路径删除和备份未跟踪，符合用户重命名状态；没有擅自恢复、暂存或提交。
- 本轮重新运行 `python3 scripts/verify_baseline.py`，首项报 `Current planner differs from staged-baseline snapshot`。原因是脚本仍查已移走的 `transplant_ws/src/path_searching`；没有刷新清单使其通过。该失败发生在后续证据/反向重建核验之前，不能记为其本轮通过。
- 本轮另作只读清单核对：备份规划包62文件的内容/权限与 `planner-current.json` 一致；不等于旧脚本完整通过或算法测试通过。
- 代码复核仍可见旧ESDF的0.05 m坐标硬编码和共享地图释放风险；TDT示例为6系数、3阶目标；原MPC按TargetSpeed缩放合速度，但没有实现本次新许可协议/独立测试限速的证据。保守面积距离、新停止机制均是待实现要求，不是代码现状。

## 当前未解决的问题

1. **新算法未实施**：S0尚未开始；依赖版本/API、来源范围、新包加载及测试入口需核对。
2. **地图与安全查询**：粗格面积映射、滚动origin/增量重同步、未知/覆盖语义、距离场到占据面积距离下界、ESDF所有权需在副本实现并测试。旧实现不能直接宣称满足新净空判据。
3. **执行接缝待实现**：参考撤销、许可关联/会话、超时保持、停止确认、求解不阻塞保护，以及MPC/转发进程断联后的末端停止均未验证。
4. **MPC定向风险待核验**：末端索引取模、加速度正下限及梯度/优化失败行为是否阻碍本次执行，先复现再局部修正，不扩成MPC重构。
5. **实机待验证**：静态图与物理场景配准、实际话题编码/QoS、四舵轮方向切换能力、外轮廓是否都包含在约定包络中、实际速度映射和制动能力。配置/TF连通不代表这些已验证。
6. **实机数值未批准**：首轮0.20 m/s只是候选；执行上限、前视范围、超时、停稳阈值需结合测量确认。15～20 ms仅是待评估目标，无端到端保证。

## 下一阶段目标

按 [实现计划](docs/phase1-baseline/implementation-plan.md) 依次开展：先S0来源/依赖/最小骨架，再S1栅格快照、保守距离与普通A*。后续S2化简/QP、S3双调试入口、S4执行保护和S5受控实机移动必须分层验收，不跳过非驱动验证直接开执行。优先最小必要改动，不提前做自动重规划或通用框架。

## 下一次开始时检查

1. 阅读 `AGENTS.md`、本文、`docs/decisions.md`，以及 `docs/phase1-baseline/` 下的策略、需求、计划、参数约定。旧 `Phase1Spec.md`、旧T02-A和已替代讨论稿不作当前默认要求。
2. 执行 `git status --short`、`git diff`、`git diff --cached --stat`，核对备份和新工作区目录；区分既有修改，不能因旧路径标D就恢复或删除备份。固定清单仍保持原样；核验失败先解释，不静默刷新。
3. 检查 `tdt-nav-kit/src/YAstar/`、`tdt-nav-kit/src/MinimumSnapOsqp/` 和备份包的 `include/esdf_map.h`、`src/esdf_map.cpp`、`src/dynamicvoronoi.cpp`、`include/path_searching/grid_geometry.h`、`src/path_simplifier.cpp` 及对应测试；只读来源，后续编辑新副本。
4. 核对 `/home/hustlyrm/slam_source/start_dlil_hyx.sh`、`odom_hyx/src/dlil/launch/dlil.launch.py`，以及原导航 `mpc_follower` 的源码/参数/launch、`vel_forwarder/src/forwarder.cpp` 和 `/home/hustlyrm/begin_serial_mpc.sh`；先查包解析与当前工作树，不启动旧控制脚本作无保护测试。
5. 按S0核对ROS/OSQP/OsqpEigen构建条件与来源，再决定最少导入文件。参数按用途分类并标注易混语义；默认执行关闭。所有测试记录实际命令、版本、覆盖范围，历史5/5不得挪作新QP通过证据。
