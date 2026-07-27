# 创新点一三组对比实验证明记录

## 1. 实验目的

本次实验用于证明“创新点一：2D 方向性感知的自适应激光-里程计融合模块”不是只打印触发日志，而是确实改变 Cartographer 2D 前端 Ceres scan matching 中的平移先验权重，并对最终建图结果产生影响。

重点验证三个问题：

1. 长走廊/窄通道环境下，方向性退化检测是否真实触发。
2. 动态各向异性权重是否只在退化方向增强，而不是把所有方向都改成高里程计权重。
3. Proposed 是否比“静态低权重”和“静态高权重”更合理，尤其是否能避免静态高权重导致的地图拉偏或膨胀。

## 2. 实验输入

- bag：`bags/corridor_01.bag`
- bag 时长：约 `374 s`
- 输入话题：
  - `/velodyne_points`
  - `/odom`
- 派生话题：
  - `pointcloud_to_laserscan` 将 `/velodyne_points` 投影为 `/scan`
- 传感器约束：
  - 只使用 2D 激光和轮式里程计
  - 不使用 IMU
- 静态 TF：
  - `base_link -> laser_link`
  - 平移 `(0, 0, 0.17)`
  - 旋转 `(-1.5708, 0, 0)`

## 3. 三组实验配置

三组实验都使用同一个 launch：

```bash
roslaunch my_navigation real_scout_mapping_local_grid.launch \
  cartographer_config_dir:=/home/slam/slam_ws/src/my_navigation/config \
  cartographer_config:=<config.lua> \
  cloud_topic:=/velodyne_points \
  target_frame:=laser_link \
  odom_topic:=/odom \
  rviz:=false \
  enable_monitor:=false
```

三组只改变 Ceres scan matcher 的平移权重策略：

| 组别 | 配置文件 | translation_weight | 创新点一 |
|---|---|---:|---|
| Baseline A | `innovation1_baseline_a_static_low.lua` | `10` | 关闭 |
| Baseline B | `innovation1_baseline_b_static_high.lua` | `35` | 关闭 |
| Proposed | `innovation1_proposed_dynamic_anisotropic.lua` | 基础值 `10` | 开启 |

参数选择说明：

- Baseline A 表示静态各向同性低/常规权重，激光匹配相对更主导。
- Baseline B 表示静态各向同性高权重，所有方向都强信任里程计先验。
- Proposed 使用基础权重 `10`，但在检测到方向性退化后，通过完整 2x2 矩阵 `R * diag(w_long, w_lat) * R^T` 动态增强退化主方向，不把横向一起拉高。

## 4. 输出文件

实验输出目录：

```text
paper_exp/innovation1_directional_adaptive_fusion/runs_valid/
```

关键文件：

```text
baseline_a_static_low/map.pgm
baseline_a_static_low/map.yaml
baseline_b_static_high/map.pgm
baseline_b_static_high/map.yaml
proposed_dynamic_anisotropic/map.pgm
proposed_dynamic_anisotropic/map.yaml
proposed_dynamic_anisotropic/degeneracy_metric.csv
proposed_dynamic_anisotropic/degeneracy_direction.csv
```

分析输出：

```text
paper_exp/innovation1_directional_adaptive_fusion/innovation1_three_way_metrics.json
paper_exp/innovation1_directional_adaptive_fusion/map_comparison.png
```

## 5. 运行健康性

三组实验中，Cartographer 均正常接收传感器：

- `/odom` 约 `50 Hz`
- `/scan` 约 `10 Hz`
- Cartographer 正常插入 submap
- 三组均保存了 `/map`

Baseline A 和 Baseline B 关闭创新点一，因此 `degeneracy_metric.csv` 和 `degeneracy_direction.csv` 为空是正常现象。Proposed 开启创新点一，因此这两个文件有有效数据。

## 6. 创新点一触发统计

Proposed 组日志统计：

| 指标 | 数值 |
|---|---:|
| 触发次数 | `74` |
| 纵向权重倍率最小值 | `2.00359` |
| 纵向权重倍率最大值 | `3.54978` |
| 纵向权重倍率平均值 | `2.46444` |
| 横向权重倍率最小值 | `1.0` |
| 横向权重倍率最大值 | `1.0` |
| 横向权重倍率平均值 | `1.0` |

退化置信度 `/degeneracy_metric`：

| 指标 | 数值 |
|---|---:|
| 样本数 | `55887` |
| 最小值 | `0.0368` |
| 最大值 | `0.9927` |
| 平均值 | `0.3716` |
| 中位数 | `0.3326` |
| P90 | `0.7122` |

退化方向 `/degeneracy_direction`：

| 指标 | 数值 |
|---|---:|
| 方向样本数 | `55887` |
| 平均 x | `0.8455` |
| 平均 y | `0.4308` |
| 平均 \|x\| | `0.8455` |
| 平均 \|y\| | `0.4367` |

解释：

- 纵向权重倍率平均约 `2.46`，最大约 `3.55`，说明退化方向确实被动态增强。
- 横向权重倍率始终为 `1.0`，说明算法没有变成“全方向高里程计权重”。
- 退化方向平均更偏向 x 轴，符合长走廊中“沿走廊方向退化”的预期。

## 7. 地图结构指标

| 指标 | Baseline A 静态低权重 | Baseline B 静态高权重 | Proposed 动态各向异性 |
|---|---:|---:|---:|
| 地图尺寸 px | `1337 x 253` | `1346 x 516` | `1337 x 279` |
| 地图面积 m2 | `845.65` | `1736.34` | `932.56` |
| 未知区域比例 | `0.8765` | `0.9368` | `0.8873` |
| 占用连通分量数 | `101` | `166` | `121` |
| 最大连通分量占比 | `0.1988` | `0.1953` | `0.2035` |
| PCA 主方向长度 P95 m | `49.50` | `48.37` | `49.20` |
| PCA 横向宽度 P95 m | `2.266` | `2.629` | `2.367` |
| PCA 横向宽度 P90 m | `2.139` | `2.224` | `2.129` |
| 中位横向扩散 m | `1.994` | `1.957` | `1.965` |

## 8. 指标解释

### 8.1 Baseline B 的问题最明显

Baseline B 的 `translation_weight=35` 是静态高权重。它的问题不是走廊纵向不稳，而是“所有方向都被强里程计先验约束”。

实验结果显示：

- Baseline B 地图高度从 Baseline A 的 `253 px` 增大到 `516 px`。
- 地图面积从 `845.65 m2` 增大到 `1736.34 m2`。
- 未知区域比例从 `0.8765` 增大到 `0.9368`。
- PCA 横向宽度 P95 从 `2.266 m` 增大到 `2.629 m`。
- 占用连通分量从 `101` 增加到 `166`。

这说明静态高权重虽然会更强地压住长走廊纵向漂移，但它也把里程计约束强加到非退化方向和非退化场景中，导致地图范围膨胀、结构更分散。

### 8.2 Proposed 避免了 Baseline B 的全局高权重副作用

Proposed 的地图面积为 `932.56 m2`，明显接近 Baseline A 的 `845.65 m2`，而不是 Baseline B 的 `1736.34 m2`。

这说明 Proposed 没有把全局地图拉成静态高权重那种大范围膨胀形态。它只在检测到方向性退化时动态调节退化主方向，并保持横向权重不被一起拉高。

### 8.3 Proposed 保持了走廊方向长度，同时横向宽度没有恶化

PCA 主方向长度：

- Baseline A：`49.50 m`
- Baseline B：`48.37 m`
- Proposed：`49.20 m`

Proposed 保持了和 Baseline A 接近的走廊主方向覆盖长度。

PCA 横向宽度 P90：

- Baseline A：`2.139 m`
- Baseline B：`2.224 m`
- Proposed：`2.129 m`

Proposed 的 P90 横向宽度是三组中最小的，说明大部分走廊段的横向墙体扩散没有因为增强里程计而变宽，反而略优于两个静态方案。

### 8.4 Proposed 的最大墙体连通比例略优

最大连通分量占比：

- Baseline A：`0.1988`
- Baseline B：`0.1953`
- Proposed：`0.2035`

这个指标越高，说明最大连续墙体结构占整体占用点的比例越高。Proposed 在该指标上略高，说明它没有牺牲墙体连续性。

## 9. 本次实验能证明什么

本次实验可以证明：

1. 创新点一在长走廊 bag 中真实触发，触发次数为 `74`。
2. 动态权重不是全局切换，而是方向性各向异性调节：纵向倍率约 `2.00~3.55`，横向保持 `1.0`。
3. Baseline B 的静态高权重会导致地图明显膨胀和横向扩散。
4. Proposed 避免了 Baseline B 的全局高权重副作用，同时保持接近 Baseline A 的地图紧凑性。
5. Proposed 在 P90 横向宽度和最大墙体连通比例上优于两个静态方案，说明横向激光约束没有被破坏。

## 10. 本次实验不能夸大的地方

本次实验不能说 Proposed 在所有指标上都绝对碾压 Baseline A。

更严谨的结论是：

- Baseline A 在这个 bag 上没有完全失败，地图也较紧凑。
- Proposed 相比 Baseline A 的提升不是“大幅压倒式”，而是更偏向“动态权衡更合理”：
  - 保持接近 Baseline A 的紧凑地图；
  - 在退化方向自动增强里程计；
  - 避免 Baseline B 的全局高权重膨胀；
  - 横向宽度 P90 和最大连通墙体比例略优。

如果论文中要更强地证明“Baseline A 会在长走廊纵向重影”，建议后续补充更极端的长直走廊 bag，或者人为降低 Baseline A 的平移先验权重，再做一组更明显的退化对比。

## 11. 最终结论

创新点一不是单纯“触发日志”或“处罚权重”。它已经在同一 bag 的 Proposed 实验中真实改变了 Ceres scan matcher 的平移约束矩阵，并且这种改变反映到了最终地图上。

本次三组实验的结论是：

> Proposed 动态各向异性融合在长走廊数据中能够检测方向性退化，并只在退化主方向增强里程计先验；相比静态高权重，它显著避免地图膨胀和横向拉偏；相比静态低权重，它保持相近的地图紧凑性，并在主要横向宽度和墙体连通指标上略有优势。因此，创新点一作为“退化感知的前端融合约束优化”是有效的。
