# M0_projection_optimized_wall_only_v2 实验说明

## 版本目的

该版本只针对用户当前要求：保留 `M0_projection_optimized` 的视觉形态优点，仅优化三维点云到二维栅格地图过程中的墙体连续性。

本版本仍属于源头投影优化，不是 `.pgm` 后处理，也不是手工补墙。处理位置在：

`/velodyne_points -> pointcloud_projection_optimizer.py -> /velodyne_points_optimized -> pointcloud_to_laserscan -> /scan -> hector_mapping -> /map`

## 生成位置

- 地图文件夹：`paper_exp/maps/mapping_01/M0_projection_optimized_wall_only_v2/`
- 地图文件：`my_map.pgm`
- 地图配置：`my_map.yaml`
- 指标文件：`paper_exp/analysis/mapping_01_M0_projection_optimized_wall_only_v2_metrics.json`

## 算法位置

核心代码：

`src/pointcloud_to_grid/scripts/pointcloud_projection_optimizer.py`

专用启动文件：

`src/pointcloud_to_grid/launch/innovation1_m0_wall_only.launch`

## 参数策略

该版本没有启用 v4 的低矮真实障碍补回逻辑：

- `z_low_obstacle_min=0.08`
- `preserve_supported_sparse_obstacles=false`

该版本只保留墙体连续性相关增强：

- 每个扫描角度保留最近有效障碍点：`publish_angular_best_points=true`
- 只连接很短的角度缺口：`max_bridge_gap_bins=2`
- 收紧补桥距离容差：`bridge_range_delta=0.28`

专业含义：地图主体仍接近 `M0_projection_optimized`，新增变化主要用于让送入 `hector_mapping` 的二维扫描墙线更连续。

## 指标对比

| 版本 | wall_continuity | free_connectivity | noise_density | occupied_cells | 说明 |
| --- | ---: | ---: | ---: | ---: | --- |
| M0_projection_optimized | 0.8149542765 | 0.9980631053 | 0.0000457764 | 1859 | 原视觉参考版本，形态较好但墙体连续性不足 |
| M0_projection_optimized_wall_only | 0.8270571827 | 0.9789990789 | 0.0000877380 | 2151 | 第一次 wall_only，扰动较大，不推荐 |
| M0_projection_optimized_wall_only_v2 | 0.9245283019 | 0.9963831379 | 0.0000305176 | 2067 | 推荐版本，墙体连续性明显增强 |
| M0_projection_optimized_v2 | 0.9429124335 | 0.9965067681 | 0.0000267029 | 2067 | 墙体连续性更高，但不是本轮“只改墙体”的专用版本 |
| M0_projection_optimized_v4 | 0.9236966825 | 0.9967493905 | 0.0000152588 | 2110 | 同时考虑真实小障碍保留的版本 |

## 结论

`M0_projection_optimized_wall_only_v2` 达到了本轮目标：

1. 墙体连续度从 `M0_projection_optimized` 的 `0.8149542765` 提升到 `0.9245283019`。
2. 视觉形态仍接近 `M0_projection_optimized`，没有改成明显不同的地图风格。
3. 相比第一次 `wall_only`，自由空间连通性恢复到 `0.9963831379`，噪点密度下降到 `0.0000305176`。
4. 该版本没有启用低矮小障碍补回逻辑，因此更符合“仅优化墙体连续性”的实验要求。

当前建议：将 `M0_projection_optimized_wall_only_v2` 作为“源头投影中墙体连续性增强”的推荐实验图；如果后续还要同时解决真实小障碍保留，则应单独使用 v4 或继续做另一组实验，不要混在这个 wall_only 实验里。
