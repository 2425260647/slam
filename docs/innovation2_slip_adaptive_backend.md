# 创新点二：退化感知门控的无 IMU 一致性异常动态调权模块

本文档描述创新点二的毕业论文落地版本。当前目标是保证 Cartographer 2D 在仅使用 2D LiDAR 和轮式里程计、没有 IMU 的条件下，能够识别 LiDAR local SLAM 与 wheel odometry 之间的明显运动不一致，并对后端 odometry 约束做稳定、可解释的标量动态调权。

本文所说的“一致性异常（Consistency Anomaly）”是 LiDAR local SLAM 与 wheel odometry 相对运动之间的异常差异。它可以作为轮式里程计打滑、侧滑或异常漂移的间接表征，但不等同于带物理真值的轮胎打滑检测。

代码中保留 `SlipDetector`、`SlipMetric` 和 `/slip_*` 等已有名称，以兼容现有实验脚本和 ROS 话题。论文、图表和新增说明统一使用“一致性异常”这一术语。

## 1. 创新点二解决什么问题

Cartographer 2D 后端会在相邻节点之间加入 wheel odometry constraint。如果 odometry 权重始终固定且较高，轮子打滑、侧滑或 odometry 突变会把轨迹和地图拉偏；如果权重始终固定且较低，长走廊中原本有价值的纵向 odometry 约束又得不到充分利用。

创新点二采用以下工程策略：

1. 比较相邻关键帧之间的 LiDAR local SLAM 相对位姿与 wheel odometry 相对位姿。
2. 重点检查横向位移和航向角的一致性。
3. 使用创新点一提供的方向性退化置信度作为门控，只在结构化走廊中 LiDAR 非退化方向较可信时允许触发异常判定。
4. 检测到一致性异常后，降低该段 odometry constraint 的标量权重。
5. 恢复正常后，通过一阶低通滤波平滑恢复 odometry 权重。

一句话定义：

> 创新点二是一种由前端方向性退化指标门控的、面向 Cartographer 2D 后端 odometry 约束的一致性异常检测与标量动态调权方法。

它不会发布 `/cmd_vel`，不会控制小车运动，也不会执行路径规划。它改变的是后端 Ceres 优化中 odometry residual 的影响强度。

## 2. 适用条件与边界

输入传感器只有：

- 2D LiDAR：`/scan`，或由 `/velodyne_points` 投影得到的 `/scan`。
- Wheel odometry：`/odom`。
- 不使用 IMU。

适用环境：

- 长走廊。
- 室内墙体环境。
- 货架通道和厂区通道。
- LiDAR 至少在横向或航向上仍有有效结构约束的场景。

能力边界：

- 本方法检测的是 LiDAR-odom 一致性异常，不是直接测量轮胎与地面的物理滑移。
- 长走廊纵向 LiDAR 不可观测时，纯纵向轮胎空转可能无法可靠识别。
- LiDAR 与 odometry 同时不可靠时，系统只能保持 unknown，不能判断谁对谁错。
- 当前版本追求硕士论文所需的稳定性、可复现性和实验完整性，不追求复杂车辆动力学建模。

## 3. 输入相对位姿

对于相邻关键帧 `i` 和 `j`：

```text
T_scan_ij = inverse(local_pose_i) * local_pose_j
T_odom_ij = CalculateOdometryBetweenNodes(i, j)
```

其中：

- `T_scan_ij` 是 Cartographer local SLAM 输出的相邻节点相对位姿。
- `T_odom_ij` 是根据 `/odom` 在两个节点时间处插值得到的相对位姿。

需要注意，local SLAM 的初始预测会使用 odometry，因此 `T_scan_ij` 不是完全独立的 LiDAR 真值。论文应将本方法描述为“多源运动一致性异常检测”，而不是“LiDAR 真值监督的打滑检测”。

## 4. 一致性异常指标

构造相对误差：

```text
E_ij = inverse(T_scan_ij) * T_odom_ij
```

提取：

```text
e_y     = abs(E_ij.translation.y)
e_theta = abs(NormalizeAngle(E_ij.rotation.yaw))
```

原始一致性异常分数：

```text
S_raw = w_y * e_y + w_theta * e_theta
```

当前版本故意不把纵向误差作为主要判据。原因是长走廊中 LiDAR 沿走廊纵向可能退化，直接使用纵向差异容易把 LiDAR 退化误判成 odometry 异常。

`w_y` 和 `w_theta` 是经验缩放系数。当前实现保持简单线性组合，不引入 Hessian、Fisher information、Mahalanobis distance 或复杂概率模型。

## 5. 创新点一可靠性门控

创新点一输出：

- 方向性退化置信度 `D_conf`。
- 点云协方差条件数。
- 退化主方向。
- 前端方向性权重缩放结果。

创新点二当前采用：

```text
G_lidar = D_conf(t_j)
S_gated = G_lidar * S_raw
```

其中 `D_conf(t_j)` 是与后端节点 `j` 时间对齐的前端指标。

工程含义：

- 在长走廊等方向性结构明显的场景中，`D_conf` 较高，LiDAR 横向和航向通常仍具有可用约束，可以使用一致性差异检查 odometry。
- 在开阔区、弱结构区或指标缺失时，不主动处罚 odometry。

这是一种面向长走廊实验场景的经验门控，不应解释为通用 LiDAR 位姿协方差。更完整的可观测性估计放入未来工作。

## 6. 时间对齐和历史边关联

Cartographer 前端与后端异步运行：

- 前端 local SLAM 高频处理当前 scan。
- 后端 pose graph 低频优化历史节点和历史约束。

因此不能把“最新 scan 指标”直接套到所有历史 odometry 边。

毕业版本采用两步关联：

1. 前端为每个退化指标记录传感器时间。
2. 后端节点创建时，按节点时间查询最近指标并绑定到该 `NodeId`。

后续每次 `Solve()` 直接读取节点已经绑定的指标，不再依赖当前全局 latest metric，也不因前端有新 scan 而改变历史边的门控输入。

实现要求：

- 使用 `trajectory_id + NodeId` 区分不同轨迹。
- 节点被 trim 时同步删除对应指标。
- 缓存读写保持线程安全。
- 锁内只执行查询、插入、删除和数据拷贝，不调用 ROS、Ceres 或复杂日志。

## 7. 双阈值迟滞状态机

异常状态使用高低双阈值：

```text
S_gated > high_threshold  -> anomaly
S_gated < low_threshold   -> normal
otherwise                 -> keep previous state
```

必须满足：

```text
low_threshold < high_threshold
```

这样可以避免指标在单一阈值附近波动时，状态不断切换。

当 LiDAR 门控指标不可用或低于可靠性阈值时：

```text
keep previous state
keep previous weight
do not update hysteresis
```

当前保留该 unknown hold 策略，不引入复杂三状态概率模型。

## 8. 标量动态权重

基础后端权重来自 Lua：

```lua
odometry_translation_weight
odometry_rotation_weight
```

每条 odometry 边使用：

```text
dynamic_translation_weight = base_translation_weight * weight_scale
dynamic_rotation_weight    = base_rotation_weight * weight_scale
```

检测到一致性异常时快速降权：

```text
weight_scale = slip_min_weight_scale
```

恢复正常时慢速恢复：

```text
weight_scale = recovery_alpha * slip_max_weight_scale
             + (1 - recovery_alpha) * previous_weight_scale
```

当前阶段明确保持标量缩放，不修改 Ceres CostFunction，不引入 3x3 各向异性后端信息矩阵。

## 9. 诊断输出

保留兼容话题：

```text
/slip_metric
/slip_state
/odom_weight_scale
/slip_lidar_reliability
```

论文中的解释：

- `/slip_metric`：LiDAR-odom 原始一致性异常代理指标。
- `/slip_state`：迟滞状态机是否判定为一致性异常。
- `/odom_weight_scale`：后端 odometry 标量权重缩放。
- `/slip_lidar_reliability`：创新点一门控置信度。

为避免短时异常被后端后续历史边覆盖，当前版本还记录：

- 本次 `Solve()` 历史边重放中的异常状态进入次数。
- 本次 `Solve()` 中的最小权重。
- 最近异常对应的节点时间和 NodeId。
- 最近异常的原始分数和门控分数。

这里故意不做“进程生命周期累计次数”。原因是 Cartographer 后端会在每次
`Solve()` 中重新遍历历史边，如果直接累加会把同一历史异常重复统计。新增 ROS
话题为：

```text
/consistency_anomaly_trigger_count
/odom_min_weight_scale
```

## 10. 配置参数

最终实验参数：

```lua
-- [Innovation 2: Consistency-Anomaly Adaptive Backend Parameters]
POSE_GRAPH.optimization_problem.slip_adaptive_odometry_weight_enabled = true
POSE_GRAPH.optimization_problem.slip_lateral_error_weight = 1.0
POSE_GRAPH.optimization_problem.slip_yaw_error_weight = 1.0
POSE_GRAPH.optimization_problem.slip_high_threshold = 0.03
POSE_GRAPH.optimization_problem.slip_low_threshold = 0.012
POSE_GRAPH.optimization_problem.slip_min_weight_scale = 0.1
POSE_GRAPH.optimization_problem.slip_max_weight_scale = 1.0
POSE_GRAPH.optimization_problem.slip_recovery_alpha = 0.05
POSE_GRAPH.optimization_problem.slip_min_motion_distance = 0.02
POSE_GRAPH.optimization_problem.slip_min_motion_angle = 0.01
POSE_GRAPH.optimization_problem.slip_lidar_reliability_gate_enabled = true
POSE_GRAPH.optimization_problem.slip_lidar_reliability_min = 0.5
POSE_GRAPH.optimization_problem.slip_degeneracy_metric_max_time_delta_sec = 0.25
POSE_GRAPH.optimization_problem.slip_unknown_keep_previous_weight = true
```

这些参数来自“异常 bag 能触发、正常 Reference bag 不误触发”的双条件筛选。它们是当前长走廊实验配置，不宣称对所有机器人和环境通用。

## 11. 实验设计

毕业论文采用四组：

1. Reference：完整正常长走廊 bag + Proposed。
2. Baseline A：异常 odometry bag + 静态低 odometry 权重。
3. Baseline B：异常 odometry bag + 静态高 odometry 权重。
4. Proposed：异常 odometry bag + 退化门控动态标量调权。

至少输出：

- 四组地图截图。
- 一致性异常指标曲线。
- LiDAR 门控置信度曲线。
- odometry weight scale 曲线。
- 异常触发时间段。
- 地图面积、墙体连续性、连通分量和横向宽度等简单指标。

实验结论应限定为：

> 相对静态高 odometry 权重，Proposed 在正常输入下减少误触发，并在人工 odometry 一致性异常段降低错误约束对地图优化的影响。

不能声称：

- 已直接测量真实轮胎滑移。
- Proposed 在所有地图指标中绝对最优。
- 当前经验阈值适用于所有场景。

## 12. 当前必须完成的工程修补

毕业冲刺阶段只完成以下内容。当前状态如下：

1. 已完成：将时间对齐退化指标稳定绑定到 `NodeId`，避免历史边依赖有限长度的全局时间缓存。
2. 已完成：节点 trim 时同步删除绑定指标。
3. 已完成：增加单次 `Solve()` 异常进入次数、最小权重、最近异常时间、NodeId 和分数诊断。
4. 已完成：为迟滞、角度周期、unknown hold、快降慢升增加小型单元测试，5 项测试全部通过。
5. 待完成：使用重新录制的完整长走廊 bag 生成最终四组对比图和实验报告。

当前实现的 `NodeId` 绑定位于 `OptimizationProblem2D` 内部。前端全局时间缓存只负责节点创建时的一次查询；节点建立绑定后，后续 `Solve()` 不再查询该缓存。反序列化载入的历史节点没有当前运行进程中的可靠前端指标，因此按 unknown 处理，不伪造数据。

## 13. 明确不在当前阶段实施

以下内容统一放入论文“局限性与未来工作”，当前不修改：

- 3x3 各向异性 odometry 信息矩阵。
- Hessian、Fisher information 和严格位姿可观测性分析。
- Mahalanobis distance、NIS 和统计假设检验。
- 自定义 Ceres InformationMatrix CostFunction 或 LossFunction。
- odometry covariance 深度融合。
- 轮胎动力学、摩擦系数和复杂增量重新积分。
- 真实纵向空转、横向侧滑和转向打滑的精细分类。
- 高精度外部真值系统和大规模多场景泛化实验。

未来工作可表述为：

> 后续可结合 scan matcher Hessian、wheel odometry covariance 和车辆运动学模型，进一步构造具有统计意义的方向性异常指标，并扩展为多方向独立调权。但这些内容不属于本文当前工程实现范围。

## 14. 论文推荐表述

> 针对无 IMU 条件下轮式里程计异常约束可能造成 Cartographer 2D 后端轨迹和地图畸变的问题，本文提出一种退化感知门控的一致性异常动态调权方法。该方法利用相邻关键帧间 LiDAR local SLAM 与 wheel odometry 的横向及航向运动差异构造一致性异常指标，并使用前端方向性退化置信度对异常判定进行门控。在异常状态下，方法对相应 odometry 约束进行快速标量降权；在恢复正常后，通过一阶低通滤波平滑恢复权重。实验表明，该方法能够减少正常场景误触发，并降低人工 odometry 异常对后端地图优化的影响。
