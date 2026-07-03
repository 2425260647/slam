# A-LOAM Odom Path 改善对比

## 结论

当前保留方案：点云坐标预处理，将 `laser_link` 点云转换到 `base_link` 后再送入 A-LOAM。

不保留方案：直接对 A-LOAM 前端强行施加地面车平面约束。该方案虽然能把 `/laser_odom_to_init` 的 Z、roll、pitch 压到 0，但会让后端 `/aft_mapped_to_init` 和地图严重发散，不适合使用。

## 指标对比

| 指标 | 原始 A-LOAM | 点云转 `base_link` | 强行地面约束 |
|---|---:|---:|---:|
| `/laser_odom_to_init` Z 范围 | 7.425 m | 6.338 m | 0.000 m |
| `/laser_odom_to_init` 最终 Z 漂移 | 4.939 m | 3.261 m | 0.000 m |
| `/laser_odom_to_init` roll 最大值 | 179.419 deg | 160.606 deg | 0.000 deg |
| `/laser_odom_to_init` pitch 最大值 | 76.568 deg | 88.952 deg | 0.000 deg |
| `/aft_mapped_to_init` Z 范围 | 0.689 m | 0.483 m | 211.163 m |
| `/aft_mapped_to_init` 最终 Z 漂移 | -0.008 m | 0.021 m | -165.159 m |
| `/aft_mapped_to_init` XY 大跳点 | 0 | 0 | 1526 |
| `/aft_mapped_to_init` Z 大跳点 | 0 | 0 | 1594 |
| 重复位置 Z 偏移 P95 | 0.071 m | 0.111 m | 19.436 m |
| 最终地图点数 | 2634 | 2322 | 276492 |
| 评估结论 | pass_with_caution | pass_with_caution | fail_or_needs_calibration |

## 专业判断

`Odom Path` 上飘主要来自 A-LOAM 前端 scan-to-scan 里程计在 16 线雷达、室内结构、无轮速/IMU约束条件下的三维姿态漂移。它不是录包中 `/odom` 的问题，bag 内 `/odom` 的 Z 始终为 0。

点云坐标预处理只能部分改善前端漂移，但能让后端 `/aft_mapped_to_init` 的 Z 范围从 0.689 m 降到 0.483 m，属于有效但不彻底的改善。

后续导航定位应继续使用 `/aft_mapped_to_init` 或 `/aft_mapped_to_init_high_frec`，不要使用 `/laser_odom_to_init`。若必须彻底改善前端 Odom Path，需要进一步做轮速/IMU融合或换用带地面约束和回环优化的 LiDAR SLAM。
