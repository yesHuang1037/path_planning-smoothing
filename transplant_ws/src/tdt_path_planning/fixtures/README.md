# 快照格式

`TDT_SNAPSHOT 1` 的单文件文本格式由 `saveSnapshot` 写出，保存全部核心参数、精确起终点、静态图以及可选动态图；与机器人接口无关。ROS入口设置 `snapshot_file` 后，每个有效显式目标保存一次输入，可交给 `offline_planner` 重放。不要手工把0～254原始代价编码当作OccupancyGrid写入。

核心测试自行构造地图，不依赖真实机器人或历史测试产物。
