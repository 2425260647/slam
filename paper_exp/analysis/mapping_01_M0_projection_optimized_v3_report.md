# M0_projection_optimized_v3 实验说明

## 版本目的

该版本是三维点云到二维栅格地图的源头投影优化结果，不是二维 `.pgm` 后处理。

本次优化针对用户确认的真实小障碍物被误消除问题，目标是在保留真实障碍物的前提下继续抑制孤立噪点，并尽量提升墙体连续性。

## 生成位置

- 地图文件夹：`paper_exp/maps/mapping_01/M0_projection_optimized_v3/`
- 地图文件：`my_map.pgm`
- 地图配置：`my_map.yaml`
- 指标文件：`paper_exp/analysis/mapping_01_M0_projection_optimized_v3_metrics.json`

## 算法位置

核心代码：

`src/pointcloud_to_grid/scripts/pointcloud_projection_optimizer.py`

实验启动文件：

`src/pointcloud_to_grid/launch/innovation1_m0.launch`

源头链路：

`/velodyne_points -> pointcloud_projection_optimizer.py -> /velodyne_points_optimized -> pointcloud_to_laserscan -> /scan -> hector_mapping -> /map`

## 本次源头优化内容

1. 增加低矮真实障碍候选点保护：`z_low_obstacle_min=0.035`，避免低矮障碍被直接当作地面删除。
2. 增加空间邻域支撑判断：稀疏点必须在相邻体素中有支撑，孤立点仍视为噪点。
3. 增加角度邻域支撑判断：相邻扫描角度距离一致的点被认为更可能是真实障碍。
4. 保留原有体素降采样和短角度缺口补偿，继续抑制噪声并缓解断墙。
5. 不进行二维地图补墙或手工修图。

## 指标对比

| 版本 | wall_continuity | free_connectivity | noise_density | occupied_cells | 说明 |
| --- | ---: | ---: | ---: | ---: | --- |
| M0_baseline | 0.9267831838 | 0.9957390814 | 0.0000152588 | 2117 | 原始基线，小障碍保留较完整 |
| M0_projection_optimized | 0.8149542765 | 0.9980631053 | 0.0000457764 | 1859 | 障碍形状较好，但真实小障碍被误删较多 |
| M0_projection_optimized_v2 | 0.9429124335 | 0.9965067681 | 0.0000267029 | 2067 | 墙体连续度最好，但部分障碍形态变化 |
| M0_projection_optimized_v3 | 0.9229671897 | 0.9967514213 | 0.0000152588 | 2103 | 更保守地保留真实障碍，同时噪点密度低 |

## 结论

`M0_projection_optimized_v3` 相比 v1 明显改善了真实障碍物误消问题：

- 占用栅格从 v1 的 `1859` 恢复到 `2103`，接近 baseline 的 `2117`。
- 噪点密度为 `0.0000152588`，与 baseline 相同，低于 v1 和 v2。
- 墙体连续度 `0.9229671897`，低于 v2，但高于 v1，并接近 baseline。

当前建议：

1. 将 v3 作为“真实障碍保留优先”的源头投影优化版本。
2. 如果论文强调导航安全和真实障碍不漏检，v3 比 v1 更合适。
3. 如果论文强调墙体连续度，可以将 v2 作为墙体连续性最优对照，v3 作为综合保守优化版本。
