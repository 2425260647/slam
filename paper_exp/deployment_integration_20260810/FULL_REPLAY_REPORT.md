# Cartographer 清理后双 Bag 落地回放报告

日期：2026-08-10

## 运行范围

- 使用已编译的纯净 Cartographer 运行空间：`cartographer_runtime_ws/install_isolated/`。
- 完整回放只启动 Cartographer 2D、全局占据栅格发布、点云转二维扫描和 `local_grid_mapper`。
- 按当前项目主线，完整回放不启动 `move_base`、DWA、TEB、Frontier 或自动控制节点。
- 在线输入只使用 bag 中的 LiDAR/点云、`/odom` 和必要 TF；未使用真值、IMU 或人工轨迹约束。

## 有效结果

### Corridor02

有效目录：`paper_exp/corridor02_results/full_clean_20260810_run3/`

- 原始 bag 时长：293.36 s。
- 记录 bag 时长：293.36 s，完整覆盖原始时段。
- `/scan`：2934 帧。
- `/odom`：5867 帧。
- `/map`：304 帧。
- `/local_occupancy_grid`：2934 帧。
- `/local_occupancy_grid_map`：2932 帧。
- `/tf`：28092 帧。
- `/scan_matched_points2`：2933 帧。
- 保存地图：2108 x 1362，分辨率 0.05 m/pixel。
- `trajectory.pbstream` 写入成功。
- MID-360 去畸变投影日志显示 2934 帧输出、运行期 `dropped=0`。
- 未发现 Cartographer fatal；只有首帧地图/TF 建立前的初始化警告。

标准 Cartographer 不发布 `/tracked_pose`。目录中的 `trajectory.csv` 来自完整回放记录中最后一帧 `/trajectory_node_list`，共 2620 个去重轨迹点；原始 TF 和可视化轨迹仍保存在 `topics.bag` 中。

### corridor_repeat_03

有效目录：`paper_exp/corridor_repeat03_results/full_clean_20260810/`

- 原始 bag 时长：578.82 s。
- 记录 bag 时长：578.81 s，完整覆盖原始时段。
- `/scan`：5782 帧，对应原始 5788 帧点云，覆盖率约 99.90%。
- `/odom`：28940 帧。
- `/map`：602 帧。
- `/local_occupancy_grid`：5781 帧。
- `/local_occupancy_grid_map`：5780 帧。
- `/tf`：89626 帧。
- `/tf_static`：4 帧。
- `/scan_matched_points2`：5773 帧。
- 保存地图：1621 x 375，分辨率 0.05 m/pixel。
- `/finish_trajectory` 返回成功，`trajectory.pbstream` 写入成功。
- 未发现 Cartographer fatal；只有首帧地图/TF 建立前的初始化警告。

目录中的 `trajectory.csv` 来自最后一帧 `/trajectory_node_list`，共 2784 个去重轨迹点。

## 输出说明

每个有效结果目录均包含：

- `map.pgm`、`map.yaml`：导航可读二维栅格地图。
- `map_preview.png`：便于快速查看的地图预览。
- `trajectory.csv`：精简后的全局轨迹点。
- `trajectory.pbstream`：Cartographer 状态文件。
- `topics.bag`、`topics_bag_info.txt`：关键输出话题及消息计数。
- `roslaunch.log`、`rosbag_play.log`、`record.log`：运行日志。
- `rosparams.yaml`、`nodes.txt`、`topics.txt`：运行参数和接口审计。
- `checksums.sha256`：输入和核心输出校验和。

## 无效运行记录

- `paper_exp/corridor02_results/full_clean_20260810/`：长任务会话在约 131/293 s 被执行环境中断，留下 `.bag.active`，不得用于验收。
- `paper_exp/corridor02_results/full_clean_20260810_run2/`：2 倍速回放造成 odometry 回调调度乱序，触发 Cartographer 时间单调性 fatal，不得用于验收。
- 离线审计确认两个原始 bag 的 `/odom` 时间戳均单调。最终有效运行统一使用 1 倍速。

## 只读规划目录审计

- `catkin_ws_planning_readonly/` 共 384 个 regular files。
- 当前哈希清单与解压后的基线清单逐项一致，`cmp` 返回 0。
- 可写 regular files 数量为 0；典型源码权限为 `444`。
- 本轮没有修改、删除或写入只读规划源码。

## 结论边界

本轮证明的是清理后的 Cartographer 与两种点云投影链能够稳定完成实时二维建图、TF 发布和车体中心局部地图输出。两组数据没有在本轮引入连续真值评价，因此不能据此宣称 ATE/RPE 或精度提升；地图预览检查也不能替代定量精度验证。
