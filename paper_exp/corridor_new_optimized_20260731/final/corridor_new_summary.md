# corridor_new.bag 最终优化版回放实验记录

## 基本信息

- 实验时间：2026-07-31
- 输入 bag：`/home/slam/slam_ws/bags/corridor_new.bag`
- 输入 bag 时长：约 487 s
- 输入话题：`/velodyne_points`、`/odom`、`/tf`
- 未播放：bag 内 `/tf_static`
- 原因：bag 内部分静态 TF frame 带前导 `/`，正式运行使用手动发布的合法静态 TF。
- 算法配置：`real_scout_online_slam.launch` + `cartographer_scout_online_navigation.lua`
- 创新点状态：创新点一、创新点二均开启
- 记录文件：`paper_exp/corridor_new_optimized_20260731/final/diagnostics.bag`
- 最终地图：`paper_exp/corridor_new_optimized_20260731/final/map.pgm`、`map.yaml`

## 外参与雷达高度说明

本次正式运行使用：

- `base_link -> laser_link`：`z = +0.17 m`，yaw 约 `-1.5708 rad`
- `base_link -> base_footprint`：`z = -0.23479 m`

因此雷达相对 `base_footprint` 的高度为：

```text
base_footprint -> laser_link = 0.23479 + 0.17 = 0.40479 m
```

结论：如果“雷达高度 0.4 m”指的是相对地面或 `base_footprint` 的安装高度，则本次正式运行已经基本匹配，不需要因为高度问题作废。

边界：如果后续确认真实外参含义是 `base_link -> laser_link` 的 z 应为 `0.4 m`，则本次自有 bag 结果需要重跑；论文中不能混用两套外参生成的结果。

## 正式回放链路

- `/velodyne_points -> pointcloud_to_laserscan -> /scan`
- `/scan -> scan_quality_gate -> Cartographer 2D`
- `/odom -> Cartographer 2D`
- Cartographer 输出 `/map`、`/tracked_pose`、TF 与创新点诊断话题
- RViz 固定坐标系：`map`

## 主要统计结果

| 项目 | 数值 |
|---|---:|
| 诊断 bag 时长 | 487.04 s |
| `/scan` 消息数 | 4,643 |
| `/scan` 平均频率 | 9.53 Hz |
| `/odom` 消息数 | 24,352 |
| `/odom` 平均频率 | 50.00 Hz |
| `/tracked_pose` 消息数 | 24,189 |
| `/tracked_pose` 平均频率 | 49.67 Hz |
| `/map` 消息数 | 1,246 |
| `/map` 平均频率 | 2.56 Hz |
| 估计轨迹长度 | 168.26 m |
| 起终点估计位移 | 36.90 m |

## 创新点一诊断

| 指标 | 数值 |
|---|---:|
| `/degeneracy_metric` 消息数 | 19,089 |
| 最小值 | 0.0315 |
| 最大值 | 0.9999 |
| 平均值 | 0.1400 |
| 末值 | 0.0886 |

解释：创新点一在该 bag 中正常发布退化指标，且最高值接近 1，说明序列中存在明显方向退化片段；平均值较低，说明不是全程处于强退化。

## 创新点二诊断

| 指标 | 数值 |
|---|---:|
| `/slip_metric` 最大值 | 0.0570 |
| `/slip_metric` 平均值 | 0.0144 |
| `/slip_state=true` 比例 | 0.0000 |
| `/odom_weight_scale` 最小值 | 0.7934 |
| `/odom_weight_scale` 平均值 | 0.9936 |
| `/odom_weight_scale` 末值 | 1.0000 |
| `/odom_min_weight_scale` 末值 | 0.5059 |
| 一致性异常触发次数 | 6 |
| LiDAR reliability 平均值 | 0.7955 |

解释：创新点二在该 bag 中有短时一致性异常响应，累计记录到 6 次异常触发；实时 `slip_state` 最终未保持为 true，`odom_weight_scale` 末值恢复到 1.0。该现象更适合写作“自有 bag 中存在短时 LiDAR-odom 不一致，算法能够短时降权并恢复”，不能写成“证明真实打滑发生”。

## 最终地图统计

| 项目 | 数值 |
|---|---:|
| 地图分辨率 | 0.05 m |
| 地图尺寸 | 1545 × 783 cells |
| 地图物理范围 | 77.25 m × 39.15 m |
| known cells | 50,576 |
| free cells | 35,534 |
| occupied cells | 15,042 |
| known area | 126.44 m² |
| free area | 88.84 m² |
| occupied area | 37.61 m² |

## 证据边界

- 该 bag 是自有数据，没有连续外部真值；不能报告 ATE/RPE。
- 本次只运行最终优化版，没有同步运行 baseline；因此它是工程验证与迁移验证证据，不是单独的优化收益证据。
- 可用于论文的严谨表述是：最终优化版在 `corridor_new.bag` 上完成 487 s 完整回放，稳定输出 `/scan`、`/map`、`/tracked_pose` 与创新诊断话题，并在自有真实走廊数据中记录到方向退化和短时 LiDAR-odom 一致性异常响应。
- 不能写成：该单次结果证明算法相对 baseline 在 `corridor_new.bag` 上提升了 ATE/RPE。

## 三条走廊/重影复查

用户观察到：该 bag 物理场景为直长走廊，但最终地图出现多条走廊和重影。

已追加轨迹形态检查：

- 输出图：`trajectory_shape_check.png`
- 输出数据：`trajectory_shape_check.json`

关键事实：

| 项目 | 输入 `/odom` | Cartographer `/tracked_pose` |
|---|---:|---:|
| 路径长度 | 162.44 m | 168.26 m |
| 起终点位移 | 50.52 m | 36.90 m |
| 起终点位移 / 路径长度 | 0.311 | 0.219 |
| yaw 跨度 | 6.03 rad | 5.24 rad |

解释：

- 输入 `/odom` 本身不是一条简单直线轨迹，存在大转角或累计漂移；如果真实运动是在同一条直走廊内往返，则 `/odom` 已经不能作为“直线真值”理解。
- 最终地图中的多走廊形态与 Cartographer 输出轨迹的折线/分叉相对应，不是单纯地图显示错误。
- 雷达高度不是当前首要原因：本次外参合成后 `base_footprint -> laser_link ≈ 0.40479 m`，已经符合“雷达离地约 0.4 m”的解释；z 高度偏差也通常不会把 2D 直走廊变成多条平行走廊。
- 更可能的机制是：直长走廊强退化，scan matching 对纵向和重复结构约束不足；同时该 run 中创新点二记录到 6 次一致性异常和短时 odom 降权，在没有外部真值的长走廊里可能削弱了本来重要的里程计纵向约束；再叠加重复走廊结构下的后端约束，最终产生重影或多条走廊。

结论：该结果不能作为“最终优化版在自有 bag 上取得正向收益”的证据；当前应将其归类为失败/风险诊断样例。下一步必须用同一 bag 跑 clean baseline、innovation1-only、full proposed 三组对比，才能判断主要责任是数据/里程计、基础 Cartographer 退化，还是创新点二的动态降权策略。

## 路径拓扑补充说明

用户进一步澄清：本 bag 是“先直接走到走廊尽头，回程时才从中间的电梯等待区经过一次再回到起点”；而另外 3 个 bag 是“去回两次都进入电梯走廊等待区”。

这意味着：

- `corridor_new.bag` 的路径拓扑与另外 3 个 bag 不同；
- 它不是“同一轨迹的重复采样”，而是“回程只经过一次中间支路”的异构路径；
- 因此它更容易在地图里形成一条主走廊 + 一条回程重影 + 一个中间支路，看起来像三条走廊；
- 这类 bag 不能直接和另外 3 个 bag 作为同一分组做定量横比，必须在论文里单独标成“异构路径 / 非对称往返”。

严格结论：`corridor_new.bag` 更适合作为“路径拓扑不同的挑战样例”或“失败诊断样例”，不应和那 3 个双向都经过等待区的 bag 混成同一种实验条件。
