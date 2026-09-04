# lidar_adaptive

`lidar_adaptive` 是当前 Cartographer 2D 研究代码的唯一归档目录。研究主线为：

1. 面向 16 线 LiDAR 的高度、线号和邻域一致性置信投影；
2. 基于投影质量、有效束比例、角度覆盖、新颖度和运动量的信息驱动扫描选择；
3. 在不重写 Ceres/Pose Graph 的前提下评估地图更新和实时性变化。

## 运行入口

先完成工作区构建并加载环境：

```bash
source /opt/ros/noetic/setup.bash
source install_isolated/setup.bash
```

只验证研究节点：

```bash
roslaunch lidar_adaptive lidar_adaptive_pipeline.launch
```

启动完整研究建图链路（假设外部已经提供 `/velodyne_points`、odom 和 TF）：

```bash
roslaunch lidar_adaptive lidar_adaptive_cartographer.launch
```

启动 Gazebo 仿真并接入研究链路：

```bash
roslaunch lidar_adaptive lidar_adaptive_cartographer.launch start_sim:=true gui:=false rviz:=true
```

使用带电梯间的退化走廊场景：

```bash
roslaunch lidar_adaptive lidar_adaptive_cartographer.launch \\
  start_sim:=true \\
  world_name:=$(find lidar_adaptive)/worlds/corridor_elevator.world \\
  gui:=false rviz:=true
```

记录研究话题和 Gazebo 真值：

```bash
rosrun lidar_adaptive record_lidar_experiment.sh \\
  src/lidar_adaptive/experiments/$(date +%Y%m%d_%H%M%S)_gazebo_corridor/run.bag 120
```

默认 `forward_selected_scans=false`，选择器只发布诊断并转发每一帧扫描。只有在明确的 C2 扫描抽帧实验中才设置为 `true`。这时结果应标注为外部 scan-thinning 实验，不能写成“每帧 scan matching、仅关键帧地图插入”。

选择器会短暂缓存 LaserScan，等待带相同时间戳的 `ScanQuality`；正常运行时可从
`/scan_selection.quality_synchronized` 统计同步比例。超过 `pending_timeout_sec` 后才会
回退到该帧有效束比例，回退帧必须在实验日志中单独统计。

## 研究记录

每次实验必须在 `experiments/<date>_<dataset>_<case>/` 下保存输入清单、参数快照、源码版本、ROS 话题导出、日志、地图、轨迹和指标。具体口径见：

- [`IMPLEMENTATION_PLAN.md`](IMPLEMENTATION_PLAN.md)
- [`doc/experiment_protocol.md`](doc/experiment_protocol.md)

没有连续外部真值的真实 bag 不得报告 ATE/RPE；人工 odometry 扰动只能称为可控一致性异常注入。
