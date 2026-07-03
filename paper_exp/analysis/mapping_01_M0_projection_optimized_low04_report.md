# M0_projection_optimized_low04 实验说明

## 版本目的

该版本基于用户认为视觉效果最好的 `M0_projection_optimized` v1 算法，只调整高度过滤参数，尝试保留被地面高度估计误删的低矮真实障碍。

本版本仍是源头投影实验，不是 `.pgm` 后处理。

## 生成位置

- 地图文件夹：`paper_exp/maps/mapping_01/M0_projection_optimized_low04/`
- 地图文件：`my_map.pgm`
- 地图配置：`my_map.yaml`
- 指标文件：`paper_exp/analysis/mapping_01_M0_projection_optimized_low04_metrics.json`

## 参数变化

相对 v1：

- `z_ground_band`: `0.06 -> 0.04`
- `z_obstacle_min`: `0.08 -> 0.04`

专业含义：原 v1 只保留相对地面高度约 8cm 以上的点；low04 放宽到约 4cm，希望低矮真实障碍不再被删除。

## 指标对比

| 版本 | wall_continuity | free_connectivity | noise_density | occupied_cells | 说明 |
| --- | ---: | ---: | ---: | ---: | --- |
| M0_projection_optimized | 0.8149542765 | 0.9980631053 | 0.0000457764 | 1859 | v1 原始视觉参考 |
| M0_projection_optimized_low04 | 0.8429364063 | 0.9777262464 | 0.0000305176 | 2343 | 保留低矮点更多，但自由空间连通性明显下降 |

## 结论

low04 确实保留了更多低矮点，`occupied_cells` 从 `1859` 增加到 `2343`。但它也明显引入了额外低矮干扰，视觉上在地图下方出现多条横向占用线，`free_connectivity` 从 `0.9980631053` 降到 `0.9777262464`。

当前判断：`0.04m` 阈值过低，不建议作为最终版本。下一步建议试 `0.06m`，在 v1 的 `0.08m` 和 low04 的 `0.04m` 之间寻找更稳的折中。
