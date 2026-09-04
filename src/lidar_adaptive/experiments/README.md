# 实验目录模板

复制本目录为 `experiments/<date>_<dataset>_<case>/`，再写入本次运行材料：

```text
README.md                 # 本次假设、配置、结果和失败原因
config_snapshot/          # YAML、Cartographer Lua、launch 快照
input_manifest/           # bag 文件名、时长、话题、SHA256
logs/                     # roslaunch、节点和系统日志
topics/                   # 质量、信息量、选择标志、TF 导出
maps/                     # PGM/YAML 或 pbstream
metrics/                  # JSON/CSV，含均值、标准差、失败数
figures/                  # 论文图及生成命令
```

结果必须保留成功和失败运行。没有连续外部真值的真实数据不得在 `metrics/` 中填写 ATE/RPE。

对录制的派生 bag 可运行：

```bash
rosrun lidar_adaptive analyze_experiment_bag.py \
  <run>/topics/derived.bag --output <run>/metrics/summary.json
```

脚本只汇总消息计数、质量、信息量、同步率、选择率和节点处理时延；没有连续外部真值时会明确写入
`ground_truth_available=false` 和 `accuracy_metrics_allowed=false`。

Gazebo 真值 bag 可进一步计算轨迹指标：

```bash
rosrun lidar_adaptive compute_gazebo_trajectory_metrics.py \
  <run>/topics/gazebo.bag --output <run>/metrics/trajectory.json
```

该脚本只接受包含 `/gazebo/model_states` 和 `map->odom->base_link` TF 的记录。

推荐使用统一 case runner 固定 C0/C1/C2 参数和日志：

```bash
rosrun lidar_adaptive run_bag_case.sh \
  bags/corridor_repeat_01.bag C1 \
  src/lidar_adaptive/experiments/20260904_corridor_repeat_01_c1 120 1.0
```

`C0` 显式启用基线 `pointcloud_to_laserscan`，`C1` 使用置信投影全量转发，`C2` 使用
置信投影和外部 scan-thinning。脚本不会自动声称精度提升；所有输入、日志、派生 bag
和指标都保存在给定实验目录。

跨运行汇总（真实 bag 仍不计算 ATE/RPE）：

```bash
rosrun lidar_adaptive aggregate_experiment_metrics.py \
  src/lidar_adaptive/experiments/<run1> \
  src/lidar_adaptive/experiments/<run2> \
  --output src/lidar_adaptive/experiments/<date>_aggregate/metrics/summary.json
```
