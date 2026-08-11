# corridor_repeat_03 参数审计与地图质量结论

日期：2026-08-10

## 结论

当前地图质量问题首先来自实验配置偏差，不是 Cartographer 前端或后端缺少新算法模块。使用纯净 Cartographer 源码，在同一份 `/scan + /odom` 公共输入上，仅恢复以下历史基线参数后，完整地图与历史 Cartographer 结果逐字节一致：

- `adaptive_voxel_filter.max_length = 0.5`
- `adaptive_voxel_filter.min_num_points = 100`
- `adaptive_voxel_filter.max_range = 30.`
- `loop_closure_adaptive_voxel_filter.max_length = 0.9`
- `loop_closure_adaptive_voxel_filter.min_num_points = 100`
- `loop_closure_adaptive_voxel_filter.max_range = 30.`
- `hit_probability = 0.60`

原始点云投影链的 `transform_tolerance` 同步统一为 `0.05 s`。

## 同口径指标

| 运行 | 地图尺寸 | 占用单元 | 最大占用连通段 | 占用/已知 |
|---|---:|---:|---:|---:|
| 当前默认配置 | 1621 x 375 | 6096 | 859 | 0.0805 |
| 参数优化后 | 1616 x 354 | 9341 | 1571 | 0.1216 |
| 历史 Cartographer 基线 | 1616 x 354 | 9341 | 1571 | 0.1216 |

参数优化后地图 SHA256：

`fdb328ba634d2a95072cad6b57b4c910be6afd4839bd60c8d83713f219532ac8`

该哈希与历史 Cartographer 基线完全相同。参数优化后的完整回放输出位于 `param_audit_full_20260810_rerun2/`；两次启动失败的审计目录保留但不作为结果：`param_audit_full_20260810/` 和 `param_audit_full_20260810_rerun/`。

## 当前边界

日志中出现的远距离候选约束是 Cartographer 的候选评估记录，并不等于全部被接受进最终图优化。参数优化后的日志、地图和历史 Cartographer 完全一致，因此当前没有证据支持直接修改前端匹配或后端约束代码。后续若要继续提高精度，应先在带连续真值的数据集上做前端/后端单变量实验，不能把本 bag 的无真值闭合代理包装成 ATE/RPE。

## 原始 bag 最终验证

最终部署回放目录：`final_tuned_full_20260810_rerun/`。

- 直接回放 `bags/corridor_repeat_03.bag`，时长 `578 s`，1x 实时处理。
- `/scan=5785`、`/map=588`、`/local_occupancy_grid=5785`、`/local_occupancy_grid_map=5784`、`/tf=90005`，`finish_trajectory` 成功。
- 日志无 Cartographer fatal、时间乱序或节点中途退出。
- 与原始默认结果相比，地图由 `1621 x 375` 变为 `1620 x 360`，占用单元由 `6096` 增至 `9107`，最大占用连通段由 `859` 增至 `1238`，占用/已知比例由 `0.0805` 增至 `0.1189`。
- 该真实投影结果与公共 `/scan` 对照的几何边界略有差异，因此不宣称逐像素相同；但墙体连续性和地图覆盖已得到直接改善。
