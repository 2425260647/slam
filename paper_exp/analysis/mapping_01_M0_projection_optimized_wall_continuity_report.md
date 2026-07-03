# M0_projection_optimized_wall_continuity 实验说明

## 版本目的

该版本以 `M0_projection_optimized` 为基础，因为该版本在视觉上对障碍物形状的保留最接近实际环境。

本次处理的目标不是重新融合三张地图，而是在尽量不改变 `M0_projection_optimized` 障碍物形状的前提下，提高墙体连续度，使其接近 `M0_projection_optimized_v2`。

## 生成位置

- 地图文件夹：`paper_exp/maps/mapping_01/M0_projection_optimized_wall_continuity/`
- 地图文件：`my_map.pgm`
- 地图配置：`my_map.yaml`
- 指标文件：`paper_exp/analysis/mapping_01_M0_projection_optimized_wall_continuity_metrics.json`

## 算法方法

使用脚本：

`src/pointcloud_to_grid/scripts/repair_projection_wall.py`

本版本使用新增的 `bridge` 模式：

1. 以 `M0_projection_optimized` 作为主地图，保留其原始障碍物形状。
2. 以 `M0_projection_optimized_v2` 作为墙体参考，只用于判断断墙附近是否存在可靠墙线。
3. 对主外墙连通域进行短距离桥接，连接原本属于同一墙体但被断开的墙段。
4. 对横向和纵向墙线中的小缺口做短线段补洞。
5. 不删除 `M0_projection_optimized` 中已有的障碍物栅格。

## 关键指标对比

| 版本 | wall_continuity | free_connectivity | occupied_cells | 说明 |
| --- | ---: | ---: | ---: | --- |
| M0_projection_optimized | 0.8149542765 | 0.9980631053 | 1859 | 障碍物形状最好，但墙体断裂明显 |
| M0_projection_optimized_v2 | 0.9429124335 | 0.9965067681 | 2067 | 墙体连续度高，但部分障碍物形状变化较明显 |
| M0_projection_optimized_wall_continuity | 0.9454253612 | 0.9980625000 | 1869 | 保留 v1 视觉形状，仅补少量断墙点 |

## 本次结论

`M0_projection_optimized_wall_continuity` 是当前最符合用户要求的版本：

- 障碍物形状基本保持 `M0_projection_optimized` 的视觉效果。
- 墙体连续度达到 `M0_projection_optimized_v2` 水平。
- 相比 `M0_projection_optimized` 只新增 10 个占用栅格，没有删除原有障碍物。
- 比之前的 `M0_projection_optimized_wall_repaired` 更保守，避免了整片补墙导致的墙体加粗问题。

建议优先人工查看：

`paper_exp/maps/mapping_01/M0_projection_optimized_wall_continuity/my_map.pgm`

重点检查右下角墙体闭合、右侧竖墙连续性、室内孤立障碍物是否仍然保留。
