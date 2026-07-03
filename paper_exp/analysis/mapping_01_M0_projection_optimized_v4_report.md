# M0_projection_optimized_v4 实验说明

## 版本目的

该版本继续做创新点1：三维点云到二维扫描/栅格地图的源头投影优化。

本版本不是对 `.pgm` 地图做后处理，也不是手工补墙；它在 `PointCloud2` 输入 `pointcloud_to_laserscan` 之前，对 16 线激光雷达点云进行筛选、保留和短缺口补偿，再交给 `hector_mapping` 建图。

## 生成位置

- 地图文件夹：`paper_exp/maps/mapping_01/M0_projection_optimized_v4/`
- 地图文件：`my_map.pgm`
- 地图配置：`my_map.yaml`
- 指标文件：`paper_exp/analysis/mapping_01_M0_projection_optimized_v4_metrics.json`

## 算法位置

核心代码：

`src/pointcloud_to_grid/scripts/pointcloud_projection_optimizer.py`

实验启动文件：

`src/pointcloud_to_grid/launch/innovation1_m0.launch`

常规链路启动文件：

`src/pointcloud_to_grid/launch/bag_to_2d_map.launch`

源头链路：

`/velodyne_points -> pointcloud_projection_optimizer.py -> /velodyne_points_optimized -> pointcloud_to_laserscan -> /scan -> hector_mapping -> /map`

## v4 优化内容

1. 主障碍通道保持 v1 的严格高度过滤思路，优先维持 `M0_projection_optimized` 的清晰障碍形状。
2. 低矮真实障碍补充通道只补回同时具备空间邻域支撑和角度邻域支撑的点，避免把孤立噪点误当成障碍。
3. 对短角度缺口进行轻量补偿，但补墙只使用主障碍高度范围内的点，避免低矮噪点参与墙体连接。
4. 将角度距离容差从 v3 的 `0.45` 收紧到 `0.32`，将补桥距离容差从 `0.35` 收紧到 `0.28`，减少结构变厚。
5. 输出保存到独立 v4 文件夹，没有覆盖 baseline、v1、v2、v3。

## 指标对比

| 版本 | wall_continuity | free_connectivity | noise_density | occupied_cells | 说明 |
| --- | ---: | ---: | ---: | ---: | --- |
| M0_baseline | 0.9267831838 | 0.9957390814 | 0.0000152588 | 2117 | 原始基线，小障碍保留较完整 |
| M0_projection_optimized | 0.8149542765 | 0.9980631053 | 0.0000457764 | 1859 | 视觉形状较好，但真实小障碍被误删 |
| M0_projection_optimized_v2 | 0.9429124335 | 0.9965067681 | 0.0000267029 | 2067 | 墙体连续度最好，但形态不如 v1 |
| M0_projection_optimized_v3 | 0.9229671897 | 0.9967514213 | 0.0000152588 | 2103 | 真实障碍保留增强，但局部较厚 |
| M0_projection_optimized_v4 | 0.9236966825 | 0.9967493905 | 0.0000152588 | 2110 | 更接近 baseline 的障碍保留，同时保持低噪声 |

## 结论

`M0_projection_optimized_v4` 在真实障碍保留方面比 v3 略有提升：

- `occupied_cells` 从 v3 的 `2103` 增加到 `2110`，更接近 baseline 的 `2117`。
- `noise_density` 保持为 `0.0000152588`，没有因为补回小障碍而增加孤立噪点。
- `wall_continuity` 从 v3 的 `0.9229671897` 小幅提升到 `0.9236966825`，但仍低于 v2 的 `0.9429124335`。

当前专业判断：

1. 如果论文强调“真实障碍不漏检”和“源头投影优化”，v4 比 v1 更安全，也比 v3 略更完整。
2. 如果论文强调“墙体连续度最高”，v2 仍是更强的墙体连续性对照。
3. 如果用户视觉上仍最喜欢 v1，需要人工查看 v4 图像，确认 v4 是否在不明显变厚的情况下补回真实小障碍。
