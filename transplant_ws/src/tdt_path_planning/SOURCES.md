# 来源与局部差异

2026-10-29，调查起点为本工作区 `bdf74b3`，开始时工作树/暂存区均无改动。实际备份目录为 `transplant_ws_backup/`（旧文档 `bakup` 拼写不符）；未修改备份。

## 导入文件

| 新包文件 | 来源，相对项目根目录 | 源SHA-256 | 差异 |
| --- | --- | --- | --- |
| vendor/dynamicvoronoi.h | transplant_ws_backup/src/path_searching/include/dynamicvoronoi.h | c7057cf0b7dd9e863fba258014c410b037b705ce0f3a4aa840182058d2d5de3b | 原样 |
| vendor/dynamicvoronoi.cpp | transplant_ws_backup/src/path_searching/src/dynamicvoronoi.cpp | a5080a95311c95070f4db086005a5dd6ce78f2ed50dd9f7d330ec7b20a78f587 | 原样 |
| vendor/point.h | transplant_ws_backup/src/path_searching/include/point.h | 5ed76078cc1b21b24aec9d2ea24b16aaec12cb6ff6497b1239d369213647551e | 原样 |
| vendor/bucketedqueue.h | transplant_ws_backup/src/path_searching/include/bucketedqueue.h | 397c0f5c2b330891a1536a78afe72c724056af5791fb37beab828ad02e0118f0 | 保留push/pop/empty接口，改为标准最小堆 |
| vendor/bucketedqueue.cpp | transplant_ws_backup/src/path_searching/src/bucketedqueue.cpp | 8ba4af9d14d0083aea3d566a8033af3fdf6d5f16560af89d04d3fd463f8e958d | 消除旧固定距离上限、进程exit和静态初始化问题 |

`Environment`仅使用`initializeEmpty`/`occupyCell`/`update`/`getDistance`；距离场独占内部地图，不调用接管外部指针的`initializeMap`，不调用旧ESDF的硬编码转换和所有权包装。未使用的可视化/最近Voronoi函数仍有原来源编译警告，不以本次任务改写。

## 数学思路与依赖

- A*与DP按TDT普通搜索/化简思路实现，未引入kinodynamicAstar或演示节点。查阅`tdt-nav-kit/src/YAstar/yastar.cpp`，SHA-256 `382e95043448f42493a7b5b79052e7cb103ac7406ec4428cdfb6d03e989b90a3`。
- QP按TDT矩形路标走廊、分段多项式minimum-jerk思路实现，使用归一化段时间、端点和C2约束；未原样复制演示主程序。查阅`tdt-nav-kit/src/MinimumSnapOsqp/minimumSnap.cpp`，SHA-256 `368e220c8cfc4928decd8668c3d09c7bebaf8492b59be134a8337b5f5a56d1c0`；`sfcSquare.cpp`为`05f9d5c028af2d0a353abf2a85ff44ef7569877758cf3c374cc735e8050986dd`。
- 本轮只读本地OSQP 1.0.0、QDLDL缓存源码和OsqpEigen 0.11.0，在`/tmp`新建构建/安装目录，没有使用原导出中不存在的`/home/yes/...`路径，没有联网升级。
- 该OsqpEigen副本的`Settings::setTimeLimit`仍判断旧`PROFILING`宏，与OSQP1的`OSQP_ENABLE_PROFILING`不匹配。新调用点直接设置公开`getSettings()->time_limit`，编译期要求OSQP开启profiling，不修改依赖。

## 许可边界

OSQP、OsqpEigen源码带各自LICENSE；DynamicVoronoi保留原来源说明，但所查备份规划包及TDT算法目录未见完整可据以确定本包整体再分发条件的许可文本。未替上游重新授予许可，`package.xml`明确保留TODO。当前为本地隔离开发验证；对外分发前需确认各来源许可及必要NOTICE，不能把占位元数据当作已获授权的发布许可。
