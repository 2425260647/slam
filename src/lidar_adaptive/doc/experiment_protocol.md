# lidar_adaptive 实验记录规范

## 目录规则

每次实验建立独立目录：

```text
src/lidar_adaptive/experiments/<date>_<dataset>_<case>/
├── README.md
├── config_snapshot/
├── input_manifest/
├── logs/
├── topics/
├── maps/
├── metrics/
└── figures/
```

## 必须保存的内容

- 输入 bag 文件名、时长、话题和 SHA256；
- 点云投影参数和 Cartographer Lua 快照；
- Git commit、源码指纹和构建环境；
- `/scan_confidence`、`/lidar_scan_quality`、`/lidar_information_score`；
- `/keyframe_selected`、`/map`、TF 和运行日志；
- 运行期输入/输出计数、丢帧、最大队列长度和处理时间；
- 地图 PGM/YAML、轨迹 CSV 和指标 JSON；
- 失败运行以及失败原因，不得只保留成功样本。

## 结果口径

Gazebo 的模型位姿或其他连续真值可以计算 ATE/RPE。真实 Scout 或公开 bag 如果没有连续外部真值，只能使用闭合误差、墙体结构、Chamfer 到干净参考等代理指标。代理指标必须在表格标题和图注中明确标注。

人工注入的 odometry 偏置用于验证鲁棒性链路，只能描述为“可控一致性异常注入”，不能写成真实地面打滑。

## 最低重复性要求

- 每个主要配置至少 3 次独立回放；
- 参数选择、验证和测试序列分离；
- 同一组对比不得改变输入投影、TF、odom 和回放倍速；
- 报告均值、标准差和失败运行数量；
- 不因结果不理想而删除运行记录。
