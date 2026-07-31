# 自有 corridor_repeat 三包冻结版汇总

- 算法版本：`innovation_combined_v2_20260730`
- 运行配置：`innovation2_proposed_slip_adaptive.lua`（创新点一、二联合开启）
- 输入：`corridor_repeat_01/02/03.bag`，回放倍率 `2.0`

> 三条 bag 没有连续外部真值。本报告只报告地图结构代理指标，不报告 ATE/RPE。
> baseline 为同一批 bag 的现有 `innovation1` baseline_a 运行，不能替代独立真值。

## 汇总指标

| 指标 | baseline 均值 | proposed 均值 | 相对变化 |
|---|---:|---:|---:|
| 占据栅格数 | 9074.00 | 9164.00 | +0.99% |
| 最佳墙带覆盖率 | 0.2456 | 0.2651 | +7.95% |
| 最长连续墙段 / m | 9.900 | 10.817 | +9.26% |

## 运行验收

| 序列 | SUCCESS | FATAL 字节 | 异常触发最大值 | odom 权重最小值 | tracked_pose 行数 |
|---|---:|---:|---:|---:|---:|
| repeat_01 | yes | 0 | 0.0 | 1.000 | 26862 |
| repeat_02 | yes | 0 | 0.0 | 1.000 | 26437 |
| repeat_03 | yes | 0 | 0.0 | 1.000 | 27545 |

- 三条运行均无 FATAL，异常触发为 0，odometry 权重保持 1.0。
- 运行 topic 清单未发现 IMU 或 Avia 话题。
- 该冻结版本随后才用于自有 bag 迁移验证；本次不针对单条 bag 调参。
