# 真实走廊 bag 清单

以下信息由 `rosbag info --yaml` 和首帧 PointCloud2 检查得到。三个序列均为长直走廊中部带电梯间的重复路线。

| 文件 | 时长 (s) | 点云消息 | 主要话题 | SHA256 |
|---|---:|---:|---|---|
| `bags/corridor_repeat_01.bag` | 565.527 | 5655 | `/velodyne_points`, `/odom`, `/tf` | `f17265814b44d2620a51eb47bf575989f74834486f426f414339b63d3ba12855` |
| `bags/corridor_repeat_02.bag` | 555.383 | 5553 | `/velodyne_points`, `/odom`, `/tf` | `c45b8ea922dbab4b05c198695dcc261b04d3cee87b876c3c2399430651c41fef` |
| `bags/corridor_repeat_03.bag` | 578.814 | 5788 | `/velodyne_points`, `/odom`, `/tf` | `d14c3f6f1ae1c878f479ea45e4cb18cc41c62aa9dd6cd8e6fec28655d8319c55` |

首帧点云字段均为：

```text
x FLOAT32, y FLOAT32, z FLOAT32
intensity FLOAT32, ring UINT16, time FLOAT64
frame_id=laser_link, height=1
```

bag 没有 `/scan`、相机图像、`/gazebo/model_states` 或外部连续真值。回放时必须从 `/velodyne_points` 重新生成扫描；真实数据只能报告闭合、结构和运行时代理指标，不能报告 ATE/RPE。
