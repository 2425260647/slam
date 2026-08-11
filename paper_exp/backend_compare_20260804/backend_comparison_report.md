# ROS1 后端同输入对照

日期：2026-08-04  
数据：`bags/Corridor02/Corridor02.bag`，完整 293 s  ︎  
输入：MID-360 `CustomMsg` -> PointCloud2 -> `/scan`，以及 `/odom`；未回放 IMU、Avia 或视觉。

## 运行口径

- Cartographer：`cartographer_m3dgr_mid360_clean_baseline.lua`，关闭当前两个创新模块，保留原始 submap、回环和 pose graph。
- GMapping：ROS1 `slam_gmapping`，使用相同 `/scan` 和 `/odom`，通过仅用于预检的 `odom_combined -> base_footprint` TF 适配满足其 TF 接口。
- 两组使用相同官方 MID-360 外参、0.05 m 栅格和 30 m 激光范围；GMapping 使用固定 `[-100,100] m` 地图边界以防止全局重分配。
- 最终地图分别由 `/map` 的最后一帧保存，不使用连续 Ground Truth 计算 ATE/RPE。

## 结果

| 指标 | Cartographer | GMapping |
|---|---:|---:|
| 回放时长/s | 293.318 | 292.671 |
| `/scan` 记录数 | 2930 | 2882 |
| `/odom` 记录数 | 5867 | 5867 |
| 最终地图尺寸 | 2103 x 1446 | 4000 x 4000 |
| 最终地图占用单元 | 18080 | 15297 |
| 最终地图自由单元 | 266420 | 307338 |
| 占用连通分量 | 668 | 1592 |
| 最大占用分量占比 | 0.077 | 0.124 |
| 地图处理告警 | 初始 odom 等待、短队列 | 无崩溃，录制启动时等待 clock |

地图文件：

- Cartographer：`cartographer_full/final_map.pgm` 和 `.yaml`
- GMapping：`gmapping_full/final_map.pgm` 和 `.yaml`
- 预览：各目录中的 `final_map_preview.png`

## 观察与边界

预览中 Cartographer 的走廊和连接区域连续性更好；GMapping 出现明显放射状射线和更多碎片结构。占用连通分量只能作为固定评价脚本下的结构代理，不能单独等同于地图精度；两套地图画布范围不同，不能直接比较画布面积。

这次结果支持的结论是：在当前 Corridor02、无 IMU、使用 `/odom` 的输入条件下，Cartographer 是更稳妥的成熟后端，GMapping 可以作为轻量 ROS1 基线但不适合作为最终全局一致性后端。结果不支持连续 ATE/RPE 主张，也不证明新的退化前端已经有效。

## 参考算法的使用边界

- `plain_slam_ros2`：可借鉴“轻量前端、关键帧、异步闭环和图优化”的模块划分；ROS2 和其传感器接口不能直接移植到当前 ROS1 无 IMU 主线。
- `2DLIW-SLAM`：点线特征、轮速融合和闭环策略有参考价值，但原方法含 IMU，不能作为当前主对比或直接声称复现。
- `LeGO-LOAM`：面向 3D 地面车辆，不能作为 2D 栅格系统后端；只参考地面分割和局部地图思想。
- `GMapping`：已完成 ROS1 编译和完整回放，可保留为轻量基线；其长走廊结构退化和缺少显式 pose graph 是主要限制。
- `SLAM Toolbox`：当前压缩包为 ROS2 版本，未纳入本次 ROS1 对照；只有找到并验证 ROS1 Noetic 版本后才考虑。

## 后端决策

研究系统继续采用 Cartographer 的成熟后端：保留 submap、回环、pose graph、`/map` 和 TF。新的研究工作只进入局部前端和子地图准入策略，不再把 GMapping 的粒子滤波后端作为最终主系统。
