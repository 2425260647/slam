# 创新点二四组对比实验记录

## 实验设置

- Reference：原始 `corridor_01.bag`，使用 Proposed 配置，证明正常输入下地图本身应正常。
- 轻度 slip bag：`corridor_01_slip_injected.bag`，只对 `/odom` 注入轻度横向漂移和航向偏置，`/velodyne_points` 保持原始数据。
- Slip A：静态低后端 odometry 权重，关闭 slip-adaptive。
- Slip B：静态高后端 odometry 权重，关闭 slip-adaptive。
- Slip Proposed：高基础 odometry 权重，启用无 IMU 抗打滑 per-edge 动态降权。

## 关键结果

| 组别 | map面积(m2) | 占据连通分量 | PCA横向宽度P90(m) | 横向扩散中位数(m) | 最小权重scale | slip触发比例 | LiDAR可靠性均值 |
|---|---:|---:|---:|---:|---:|---:|---:|
| Reference original Proposed | 979.66 | 108 | 2.142 | 1.994 | 1.000 | 0.000 | 0.369 |
| Slip A static low odom | 1099.75 | 116 | 2.729 | 1.954 | 1.000 | 0.000 | 0.000 |
| Slip B static high odom | 1134.75 | 143 | 2.722 | 1.952 | 1.000 | 0.000 | 0.000 |
| Slip Proposed adaptive | 1120.56 | 128 | 2.741 | 1.953 | 0.145 | 0.000 | 0.364 |

## 触发证明

- Reference 原始 bag 地图面积：`979.66 m2`，用于确认正常输入下地图不应大幅歪斜。
- Slip Proposed `/slip_metric` 最大值：`0.0363`。
- Slip Proposed `/odom_weight_scale` 最小值：`0.1450`。
- Slip Proposed `/slip_state=true` 采样比例：`0.0000`。
- Slip Proposed `/slip_lidar_reliability` 均值：`0.3645`，P10：`0.0649`，P90：`0.6936`。
- 后端日志 `[Innovation2] Wheel slip detected` 次数：`1`。
- 日志中 gated slip score 最大值：`0.0331`，触发段 LiDAR reliability 均值：`0.6512`。

## 结论

Reference 组用于证明原始输入下 Proposed 配置能正常建图，不把正常地图跑歪。三组 slip 对比用于证明轻度 odom 异常下动态降权是否比静态高权重更稳。

本次第二版最重要的正向结果是 Reference 原始 bag 不再误触发：`/slip_state=true` 比例为 `0.0000`，`/odom_weight_scale` 最小值保持 `1.0000`。这说明退化感知门控有效抑制了旧版在正常 bag 中出现的低阈值误判问题。

在轻度 slip bag 上，Slip Proposed 相对 Slip B 静态高 odom 权重：

- map 面积从 `1134.75 m2` 降到 `1120.56 m2`，下降约 `1.25%`。
- 占据连通分量从 `143` 降到 `128`，下降约 `10.49%`，说明碎裂/重影趋势有所缓解。
- PCA 横向宽度 P90 从 `2.722 m` 增到 `2.741 m`，变差约 `0.69%`，该指标没有改善。
- 横向扩散中位数基本持平，变化约 `+0.05%`。

第二版额外观察 `/slip_lidar_reliability` 和日志中的 gated score：Proposed 组全程 LiDAR reliability 均值为 `0.3645`，P90 为 `0.6936`；唯一一次后端日志触发发生在 reliability `0.6512` 时，说明触发点确实通过了 `slip_lidar_reliability_min=0.6` 的门控。`/slip_state` 采样比例为 0，但 `/odom_weight_scale` 最小值降到 `0.1450` 且日志记录到一次触发，说明降权发生得很短，ROS 定时发布没有捕获到 true 状态帧。

相对 Slip A 静态低 odom 权重，Slip Proposed 并不全面更好：面积高约 `1.89%`，连通分量高约 `10.34%`。因此不能把第二版写成“所有地图指标最优”。更准确的论文结论是：第二版通过可靠性门控显著减少正常段误触发，并在轻度 odom 异常下相对静态高权重减轻地图膨胀和碎裂，但当前参数较保守，触发次数少，地图质量提升幅度有限。

本次 slip 是对 `/odom` 做轻度人工注入，不是真实地面打滑；它适合证明算法链路和鲁棒性趋势。当前只使用地图图像指标，没有外部真值轨迹，不能严格给出 ATE/RPE 级别结论。

对比图：`/home/slam/slam_ws/paper_exp/innovation2_slip_adaptive_backend/innovation2_map_comparison.png`
