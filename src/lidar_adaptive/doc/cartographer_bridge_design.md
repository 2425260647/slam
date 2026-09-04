# Cartographer 桥接设计记录

## 目的

将信息驱动选择用于地图更新/后端节点生成，同时保持每帧 scan matching。该阶段不是当前默认运行模式，也不要求修改 Ceres 残差或 Pose Graph 优化方程。

## 建议接口

选择器为每个 `LaserScan` 产生一个与时间戳对应的 `selected` 标志。桥接层在 `LocalTrajectoryBuilder2D` 的数据入口保存该标志，并在 `InsertIntoSubmap` 前应用：

```text
每帧 scan -> accumulation + scan matching
selected=true -> RangeDataInserter + 已有节点/子图更新
selected=false -> 不插入地图，但保留当前局部位姿
```

实际实现必须解决时间戳乱序、队列超时和 `selected` 缺失时的回退行为。缺失或超时标志应默认按 `selected=true` 处理，避免静默丢失建图数据。

## 允许的最小改动范围

- `cartographer_ros` 消息桥接或一个研究专用 side-channel；
- `LocalTrajectoryBuilder2D` 的地图插入条件；
- 统计 scan matching 次数、地图插入次数和跳过原因。

禁止直接修改 Ceres 残差、求解器、Pose Graph 目标函数或回环评分公式作为该阶段必要条件。

## 必须提供的证据

- 关闭桥接时与 C0 的地图、轨迹和处理计数回归一致；
- 开启桥接后逐帧报告 scan matching、地图插入、关键帧比例和队列延迟；
- Gazebo 连续真值上的 ATE/RPE/地图质量/CPU；
- 真实 bag 无真值时只报告闭合、结构和运行时代理指标；
- C0、C1、外部 C2 与内部桥接 C2 的独立消融，以及失败运行记录。

在上述证据完成前，外部 `forward_selected_scans=true` 只作为 scan-thinning 对比，不得包装为内部桥接结果。
