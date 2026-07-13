# Cartographer 2D 源码中文导读与创新点实现说明

本文档面向本项目当前源码状态，用来帮助你阅读 Cartographer 2D、写论文方法章节、准备答辩问答。它不是逐行翻译源码，而是按“数据从哪里来、在哪个文件被处理、创新点插在什么位置、为什么这样设计”的顺序解释。

> 说明：文档中的行号来自当前工作区代码快照，后续继续改代码后行号可能轻微变化。阅读时优先按“文件路径 + 函数名”定位，行号作为快速跳转参考。

## 1. 当前项目一句话

本项目基于 ROS Noetic、Scout Mini、2D 激光雷达 `/scan` 和轮式里程计 `/odom`，使用 Cartographer 2D 完成实时建图定位，并在 Cartographer 原有激光匹配和后端位姿图优化基础上加入两个创新点：

1. **创新点一：具备 2D 方向性感知的自适应激光-里程计融合模块。**
   它在前端 Ceres scan matcher 中分析当前激光点云的 2D 几何协方差，识别长走廊的退化方向，并把原本标量平移先验改成 2x2 各向异性权重矩阵。

2. **创新点二：退化感知门控的无 IMU LiDAR-odometry 一致性异常后端动态调权模块。**
   它在后端相邻节点 odometry 约束处比较 LiDAR local SLAM 相对位姿和 wheel odometry 相对位姿，如果横向和航向不一致，则动态降低 odometry 约束权重。

项目明确不依赖 IMU。配置中：

```text
src/my_navigation/config/cartographer_scout_2d.lua:14
use_odometry = true

src/my_navigation/config/cartographer_scout_2d.lua:34
TRAJECTORY_BUILDER_2D.use_imu_data = false
```

所以本项目的主线传感器是：

```text
/scan + /odom + TF
```

不是：

```text
/scan + /odom + /imu
```

## 2. 总体数据流

先看整条链路：

```text
真实车或 Gazebo
  |
  | 2D LiDAR 或 3D LiDAR 投影
  v
/scan
  |
  v
cartographer_ros::Node::HandleLaserScanMessage()
  |
  v
SensorBridge::HandleLaserScanMessage()
  |
  v
TrajectoryBuilderInterface::AddSensorData(RANGE)
  |
  v
GlobalTrajectoryBuilder::AddSensorData(TimedPointCloudData)
  |
  v
LocalTrajectoryBuilder2D::AddRangeData()
  |
  v
LocalTrajectoryBuilder2D::ScanMatch()
  |
  |  RealTimeCorrelativeScanMatcher2D 粗匹配
  |  CeresScanMatcher2D 精匹配
  v
InsertIntoSubmap()
  |
  v
PoseGraph2D::AddNode()
  |
  v
OptimizationProblem2D::Solve()
  |
  v
优化后的 submap/node 位姿
  |
  v
cartographer_occupancy_grid_node -> /map
```

`/odom` 的链路是：

```text
/odom
  |
  v
Node::HandleOdometryMessage()
  |
  v
SensorBridge::HandleOdometryMessage()
  |
  v
GlobalTrajectoryBuilder::AddSensorData(OdometryData)
  |
  +-> LocalTrajectoryBuilder2D::AddOdometryData()
  |      用于前端 PoseExtrapolator 位姿预测
  |
  +-> PoseGraph2D::AddOdometryData()
         用于后端相邻节点 odometry 约束
```

注意：前端和后端都能用 odometry，但作用不同。

前端用 odometry 是为了预测下一帧 scan 大概在哪，给 scan matching 一个初值。后端用 odometry 是在 pose graph 中给相邻节点增加约束，让优化后的轨迹不要任意漂移。

## 3. 配置入口

主配置文件：

```text
src/my_navigation/config/cartographer_scout_2d.lua
```

关键位置：

```text
1-2    include "map_builder.lua" 和 "trajectory_builder.lua"
4-30   ROS/Cartographer 总配置
32     启用 2D trajectory builder
34     关闭 IMU
49-57  scan matching 基础参数
58-72  创新点一参数
84-94  pose graph 基础参数
95-113 创新点二参数
```

本项目最重要的配置是：

```lua
use_odometry = true
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.directional_adaptive_fusion_enabled = true
POSE_GRAPH.optimization_problem.slip_adaptive_odometry_weight_enabled = true
```

对应源码解析位置：

```text
src/cartographer_ros/cartographer_ros/cartographer_ros/trajectory_options.cc:45-99
src/cartographer/cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.cc:161-239
src/cartographer/cartographer/mapping/internal/optimization/optimization_problem_options.cc:25-120
```

其中 `trajectory_options.cc:85-96` 有一个很关键的保护：

```text
如果 use_odometry=false，则强制关闭创新点一方向性自适应融合。
```

原因是创新点一的设计目标是“激光-里程计融合权重调节”。如果没有 odometry，还让模块参与权重调节，会破坏 Cartographer 原有纯激光运行模式。

## 4. ROS 接口层源码

### 4.1 `node_main.cc`

文件：

```text
src/cartographer_ros/cartographer_ros/cartographer_ros/node_main.cc
```

作用：这是 `cartographer_node` 的 ROS 进程入口。

它主要做：

1. 初始化 ROS。
2. 读取 Lua 配置。
3. 创建 `MapBuilder`。
4. 创建 `Node`。
5. 启动默认 trajectory。

你可以把它理解为“Cartographer ROS 外壳的 main 函数”。

### 4.2 `node.cc`

文件：

```text
src/cartographer_ros/cartographer_ros/cartographer_ros/node.cc
```

这是 ROS 层最重要的文件之一。它负责：

1. 根据 Lua 配置决定订阅哪些话题。
2. 接收 `/scan`、`/odom` 等 ROS 消息。
3. 把消息交给 `SensorBridge`。
4. 发布 TF、轨迹、submap 列表。
5. 发布创新点一和创新点二的诊断话题。

重点函数和位置：

```text
Node::ComputeExpectedSensorIds()      node.cc:500-529
Node::AddTrajectory()                 node.cc:532-552
Node::LaunchSubscribers()             node.cc:555-606
Node::HandleOdometryMessage()         node.cc:942-959
Node::HandleLaserScanMessage()        node.cc:1002-1013
Node::PublishLocalTrajectoryData()    node.cc:284-335
```

`ComputeExpectedSensorIds()` 决定订阅哪些传感器：

```text
num_laser_scans = 1      -> 订阅 scan
num_point_clouds = 0     -> 不直接订阅 points2
use_imu_data = false     -> 不订阅 imu
use_odometry = true      -> 订阅 odom
```

`LaunchSubscribers()` 真正创建 ROS subscriber：

```text
node.cc:559-565 订阅 LaserScan
node.cc:599-605 订阅 Odometry
```

`HandleLaserScanMessage()` 是 `/scan` 进入 Cartographer 的入口：

```text
/scan
  -> Node::HandleLaserScanMessage()
  -> SensorBridge::HandleLaserScanMessage()
```

`HandleOdometryMessage()` 是 `/odom` 进入 Cartographer 的入口：

```text
/odom
  -> Node::HandleOdometryMessage()
  -> SensorBridge::ToOdometryData()
  -> SensorBridge::HandleOdometryMessage()
```

注意 `Node::HandleOdometryMessage()` 里做了两件事：

```text
node.cc:956
extrapolators_.at(trajectory_id).AddOdometryData(*odometry_data_ptr)

node.cc:958
sensor_bridge_ptr->HandleOdometryMessage(sensor_id, msg)
```

第一条是给 ROS 层外推器使用，第二条是把 odom 真正送进 Cartographer trajectory builder。

### 4.3 创新点 ROS 话题发布位置

文件：

```text
src/cartographer_ros/cartographer_ros/cartographer_ros/node.cc
```

话题 advertise 位置：

```text
node.cc:141-146
/degeneracy_metric
/degeneracy_direction

node.cc:147-165
/slip_metric
/slip_state
/odom_weight_scale
/slip_lidar_reliability
/consistency_anomaly_trigger_count
/odom_min_weight_scale
```

话题实际 publish 位置：

```text
node.cc:290-302
发布创新点一退化置信度和退化方向

node.cc:304-335
发布创新点二一致性异常、权重缩放和统计指标
```

这里为什么要通过 `GetLatestDirectionalDegeneracyMetric()` 和 `GetLatestSlipMetric()` 读取？

因为 Cartographer 前端、后端和 ROS timer 不一定在同一个线程里运行。直接共享普通变量会有 data race。当前实现使用 mutex 保护拷贝式读取，避免 ROS 发布线程和 SLAM 线程同时读写同一块数据。

### 4.4 `sensor_bridge.cc`

文件：

```text
src/cartographer_ros/cartographer_ros/cartographer_ros/sensor_bridge.cc
```

作用：把 ROS 消息转换为 Cartographer 内部传感器数据。

重点函数：

```text
SensorBridge::ToOdometryData()             sensor_bridge.cc:54-68
SensorBridge::HandleOdometryMessage()      sensor_bridge.cc:84-103
SensorBridge::HandleLaserScanMessage()     sensor_bridge.cc:204-213
SensorBridge::HandleLaserScan()            sensor_bridge.cc:239-286
SensorBridge::HandleRangefinder()          sensor_bridge.cc:288-315
```

`HandleLaserScanMessage()` 做的事情：

```text
LaserScan ranges/intensities
  -> ToPointCloudWithIntensities()
  -> PointCloudWithIntensities
  -> HandleLaserScan()
```

`HandleLaserScan()` 做 subdivision：

```text
一帧 LaserScan 不是同一瞬间扫完的。
Cartographer 用点的相对时间，把一帧 scan 切成若干时间片。
每个时间片再进入 HandleRangefinder()。
```

本项目配置：

```text
num_subdivisions_per_laser_scan = 1
```

所以一般不拆成多段，但源码逻辑仍然保留。

`HandleRangefinder()` 最关键：

```text
sensor_bridge.cc:294-295
tf_bridge_.LookupToTracking(time, frame_id)
```

这一步查询：

```text
scan.header.frame_id -> tracking_frame
```

本项目 `tracking_frame = base_link`，所以必须存在：

```text
laser_link -> base_link
```

或者等价 TF 链。

查到 TF 后，`sensor_bridge.cc:307-313` 调用：

```cpp
trajectory_builder_->AddSensorData(sensor_id, TimedPointCloudData{...});
```

这就是 `/scan` 真正进入 Cartographer 内核的位置。

## 5. 内核入口：GlobalTrajectoryBuilder

文件：

```text
src/cartographer/cartographer/mapping/internal/global_trajectory_builder.cc
```

重点位置：

```text
GlobalTrajectoryBuilder::AddSensorData(TimedPointCloudData)  global_trajectory_builder.cc:56-88
GlobalTrajectoryBuilder::AddSensorData(OdometryData)         global_trajectory_builder.cc:98-113
```

它是前端和后端之间的中转层。

对于 `/scan`：

```text
global_trajectory_builder.cc:61-63
local_trajectory_builder_->AddRangeData()

global_trajectory_builder.cc:72-74
pose_graph_->AddNode()

global_trajectory_builder.cc:82-86
local_slam_result_callback_()
```

解释：

1. `AddRangeData()` 完成前端 scan matching。
2. 如果当前 scan 被插入 submap，就创建一个 trajectory node。
3. `pose_graph_->AddNode()` 把节点送入后端。
4. `local_slam_result_callback_()` 把局部 SLAM 结果回传给 ROS 层，用于 TF 和可视化。

对于 `/odom`：

```text
global_trajectory_builder.cc:101-103
local_trajectory_builder_->AddOdometryData()

global_trajectory_builder.cc:112
pose_graph_->AddOdometryData()
```

解释：

1. 前端用 odom 更新 `PoseExtrapolator`。
2. 后端保存 odom 数据，后面在 `OptimizationProblem2D::Solve()` 中插入 odometry residual。

## 6. 2D 前端：LocalTrajectoryBuilder2D

文件：

```text
src/cartographer/cartographer/mapping/internal/2d/local_trajectory_builder_2d.cc
```

这个文件是 2D 前端核心。你读 Cartographer 2D，优先读它。

### 6.1 构造函数

位置：

```text
local_trajectory_builder_2d.cc:38-47
```

它创建：

```text
ActiveSubmaps2D
MotionFilter
RealTimeCorrelativeScanMatcher2D
CeresScanMatcher2D
RangeDataCollator
```

直观理解：

```text
ActiveSubmaps2D                  管局部子图
MotionFilter                     判断当前 scan 是否值得插入
RealTimeCorrelativeScanMatcher2D 粗匹配
CeresScanMatcher2D               精匹配
RangeDataCollator                多雷达/分包同步
```

### 6.2 `AddRangeData()`

位置：

```text
local_trajectory_builder_2d.cc:128-244
```

作用：接收一帧或多帧累积后的 range data。

关键步骤：

```text
132-139  RangeDataCollator 同步数据
145-147  无 IMU 时用第一帧激光初始化 PoseExtrapolator
167-185  对每个激光点做时间位姿外推，用于运动补偿
198-218  按 min_range/max_range 区分 returns 和 misses
221-241  累积够 num_accumulated_range_data 后进入 AddAccumulatedRangeData()
```

本项目：

```text
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
```

所以每帧 `/scan` 都可以进入匹配流程。

`returns` 和 `misses` 的区别：

```text
returns：真实命中障碍物的点，用于把栅格打黑。
misses：超过最大距离或无障碍方向，用于把射线经过区域打白。
```

### 6.3 `AddAccumulatedRangeData()`

位置：

```text
local_trajectory_builder_2d.cc:246-318
```

作用：真正做 2D 前端定位。

关键步骤：

```text
260-263  用 PoseExtrapolator 得到 pose_prediction
265-267  AdaptiveVoxelFilter 降采样
275-276  ScanMatch()
281-283  得到 pose_estimate 并加入 extrapolator
287-292  把 scan 转到 local frame，然后插入 submap
315-317  返回 MatchingResult
```

`pose_prediction` 和 `pose_estimate` 的区别：

```text
pose_prediction：根据 odom/恒速模型推出来的预测位姿。
pose_estimate：scan matching 后修正出的位姿。
```

论文里可以这样说：

> 前端首先利用轮式里程计和恒速模型提供扫描匹配初值，再通过 scan-to-submap 匹配修正位姿，得到局部一致的激光里程计结果。

### 6.4 `ScanMatch()`

位置：

```text
local_trajectory_builder_2d.cc:69-126
```

它做两级匹配：

```text
91-97   RealTimeCorrelativeScanMatcher2D 粗匹配
101-104 CeresScanMatcher2D 精匹配
105-113 创新点一：记录退化指标到时间缓存
```

流程：

```text
pose_prediction
  |
  | 如果开启 online correlative scan matching
  v
initial_ceres_pose
  |
  v
CeresScanMatcher2D::Match()
  |
  v
pose_observation
```

这里 `real_time_correlative_score` 被传给 Ceres scan matcher，用于创新点一诊断记录。它不是创新点一的核心退化判据，核心判据来自点云协方差的条件数。

### 6.5 `InsertIntoSubmap()`

位置：

```text
local_trajectory_builder_2d.cc:320-345
```

作用：决定当前 scan 是否插入 submap，并生成后端节点。

关键点：

```text
326-330  MotionFilter 判断运动太小则不插入
333-334  ActiveSubmaps2D::InsertRangeData()
335-344  创建 TrajectoryNode::Data
```

所以不是每一帧 `/scan` 都会变成后端节点。只有通过运动过滤的 scan 才会插入 submap，并进入 pose graph。

### 6.6 `AddOdometryData()`

位置：

```text
local_trajectory_builder_2d.cc:353-360
```

作用：把 `/odom` 加入前端 `PoseExtrapolator`。

它不直接修改地图，而是影响下一帧 scan matching 的初始预测。

## 7. Ceres Scan Matcher

文件：

```text
src/cartographer/cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.cc
```

### 7.1 原版 Ceres scan matching 在做什么

核心函数：

```text
CeresScanMatcher2D::Match()  ceres_scan_matcher_2d.cc:411-470
```

优化变量：

```text
ceres_pose_estimate[0] = x
ceres_pose_estimate[1] = y
ceres_pose_estimate[2] = yaw
```

也就是说，2D Ceres scan matcher 只优化：

```text
x, y, theta
```

不优化：

```text
z, roll, pitch
```

它添加三个 residual：

```text
429-434 占据空间残差 OccupiedSpaceCostFunction2D
451-456 平移先验 TranslationDeltaCostFunctor2D
460-463 旋转先验 RotationDeltaCostFunctor2D
```

原版直观目标函数可以理解成：

```text
总代价 =
  激光点落在已有地图上不合理的代价
  + 位姿平移偏离预测值的代价
  + 位姿角度偏离预测值的代价
```

### 7.2 为什么创新点一放在这里

长走廊问题本质发生在前端 scan matching：

```text
走廊纵向：激光几何约束弱，沿走廊前后移动一点，墙线仍然相似。
走廊横向：两侧墙约束强，左右偏一点就会明显错位。
```

所以最直接的位置是：

```text
CeresScanMatcher2D::Match()
```

因为这里正好决定当前 scan 在局部 submap 里的位姿，同时也有当前帧点云 `point_cloud`。

## 8. 创新点一：方向性感知自适应激光-里程计融合

### 8.1 一句话定义

创新点一不是简单“检测走廊”，而是：

> 根据当前 2D 激光点云的协方差特征值和特征向量，判断扫描匹配在哪个方向退化，并把 Ceres 中的平移先验从标量权重改成 2x2 各向异性权重矩阵。

### 8.2 核心源码文件

```text
src/cartographer/cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.cc
src/cartographer/cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h
src/cartographer/cartographer/mapping/internal/2d/scan_matching/translation_delta_cost_functor_2d.h
src/cartographer/cartographer/mapping/directional_degeneracy_metric.h
src/cartographer_ros/cartographer_ros/cartographer_ros/node.cc
src/cartographer_ros/cartographer_ros/cartographer_ros/trajectory_options.cc
src/cartographer/cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_2d.proto
src/my_navigation/config/cartographer_scout_2d.lua
```

### 8.3 指标结构体

文件：

```text
src/cartographer/cartographer/mapping/directional_degeneracy_metric.h:27-55
```

结构体：

```cpp
struct DirectionalDegeneracyMetric {
  bool enabled;
  common::Time time;
  double confidence;
  double condition_number;
  double real_time_correlative_score;
  Eigen::Vector2d direction;
  double longitudinal_scale;
  double lateral_scale;
};
```

字段含义：

```text
confidence                  退化置信度 D_conf，范围约 0 到 1
condition_number            协方差条件数 kappa = lambda_max / lambda_min
direction                   最大特征值对应方向，即退化主方向
longitudinal_scale          纵向权重相对基础权重的缩放
lateral_scale               横向权重相对基础权重的缩放
real_time_correlative_score 粗匹配分数，仅作诊断
```

### 8.4 线程安全缓存

文件：

```text
ceres_scan_matcher_2d.cc:45-75
ceres_scan_matcher_2d.cc:88-157
```

这里实现了两套缓存：

```text
LatestMetric
  保存最新一帧退化指标，给 ROS 话题发布使用。

MetricHistory
  保存带时间戳的历史退化指标，给创新点二后端按节点时间查询。
```

为什么要加 mutex？

```text
前端 Local SLAM 线程会写退化指标。
后端 Optimization 线程会查询历史退化指标。
ROS timer 线程会读取最新指标并发布话题。
```

如果不用锁，`std::deque` 写入和后端 `lower_bound` 读取同时发生，可能出现 data race，严重时会崩溃。

### 8.5 退化检测算法位置

核心函数：

```text
CeresScanMatcher2D::ComputeAnisotropicTranslationSqrtInformation()
ceres_scan_matcher_2d.cc:251-409
```

输入：

```text
当前帧 point_cloud
real_time_correlative_score
```

输出：

```text
Eigen::Matrix<double, 2, 2> sqrt_information
```

也就是给 Ceres 平移 residual 用的 2x2 权重矩阵。

### 8.6 数学过程

第一步：计算点云均值。

源码：

```text
ceres_scan_matcher_2d.cc:278-282
```

数学：

```text
mu = (1/N) * sum(p_i)
```

第二步：计算 2D 协方差。

源码：

```text
ceres_scan_matcher_2d.cc:284-290
```

数学：

```text
C = (1/N) * sum((p_i - mu)(p_i - mu)^T)
```

这里只使用点云的 x/y：

```text
point.position.head<2>()
```

不涉及 IMU，不使用 z、roll、pitch。

第三步：数值稳定保护。

源码：

```text
ceres_scan_matcher_2d.cc:292-294
```

做法：

```text
C = C + epsilon * I
```

原因：如果点云太少、过于集中，最小特征值可能接近 0，条件数会爆炸。

第四步：特征值分解。

源码：

```text
ceres_scan_matcher_2d.cc:296-308
```

得到：

```text
lambda_min
lambda_max
v_lat
v_long
```

在长走廊中：

```text
v_long = 最大特征值方向，通常沿走廊方向
v_lat  = 最小特征值方向，通常垂直走廊墙体方向
```

第五步：计算条件数。

源码：

```text
ceres_scan_matcher_2d.cc:302-306
```

数学：

```text
kappa = lambda_max / lambda_min
```

解释：

```text
kappa 越大，说明点云越像一条长条，方向性越强，走廊退化越明显。
kappa 接近 1，说明点云分布比较均匀，方向性退化不明显。
```

第六步：方向符号平滑。

源码：

```text
ceres_scan_matcher_2d.cc:314-324
```

原因：特征向量本身有符号二义性。

```text
v 和 -v 都是同一个特征向量方向。
```

如果不处理，`/degeneracy_direction` 可能一帧指向前、一帧指向后，在 RViz 或曲线中看起来会跳。

第七步：Sigmoid 退化置信度。

源码：

```text
ceres_scan_matcher_2d.cc:333-344
```

数学：

```text
D_raw = sigmoid(slope * (kappa - threshold))
D_conf = alpha * D_raw + (1 - alpha) * D_prev
```

为什么不用硬阈值？

```text
硬阈值会让权重突然切换，Ceres 前端匹配会抖。
Sigmoid 可以让权重连续变化。
一阶低通平滑可以避免指标在阈值附近跳变。
```

第八步：构造完整 2x2 权重矩阵。

源码：

```text
ceres_scan_matcher_2d.cc:355-383
```

数学：

```text
R = [v_long, v_lat]
S = R * diag(w_long, w_lat) * R^T
```

这里的重点是：**不再计算 D_x、D_y 这种标量投影。**

方向解耦由矩阵乘法自然完成：

```text
R 把坐标转到退化主方向坐标系。
diag(w_long, w_lat) 分别调整纵向和横向权重。
R^T 再转回原来的 scan/map 坐标系。
```

这比“投影到 x/y 再单独调权”更干净，因为真实走廊不一定刚好与机器人 x 轴完全重合。

### 8.7 权重含义

源码：

```text
ceres_scan_matcher_2d.cc:363-378
```

当前实现中：

```text
scan_long_scale = max(min_scan_scale, 1 - beta_long * D_conf)
scan_lat_scale  = max(min_scan_scale, 1 - beta_lat  * D_conf)
odom_long_scale = 1 + alpha_long * D_conf
odom_lat_scale  = 1 + alpha_lat  * D_conf
```

最终：

```text
w_long = base_translation_weight * odom_long_scale / scan_long_scale
w_lat  = base_translation_weight * odom_lat_scale  / scan_lat_scale
```

直观解释：

```text
长走廊纵向退化时：
  激光沿走廊方向不可靠，应该降低 scan 对纵向的支配。
  odometry 在纵向短时间内更可信，应该提高纵向先验。

横向墙体约束强时：
  LiDAR 横向仍然可靠，横向权重不应被大幅破坏。
```

论文表述建议：

> 本文将原有 Ceres scan matcher 中各向同性的平移先验扩展为由扫描几何结构驱动的 2D 各向异性信息矩阵，使优化在退化主方向上更依赖运动先验，在非退化方向上保持激光匹配约束。

### 8.8 Ceres AutoDiff 兼容写法

文件：

```text
src/cartographer/cartographer/mapping/internal/2d/scan_matching/translation_delta_cost_functor_2d.h
```

关键位置：

```text
translation_delta_cost_functor_2d.h:39-57
translation_delta_cost_functor_2d.h:59-68
translation_delta_cost_functor_2d.h:81-95
```

原版接口：

```cpp
CreateAutoDiffCostFunction(double scaling_factor, target_translation)
```

新增接口：

```cpp
CreateAutoDiffCostFunction(Eigen::Matrix<double, 2, 2> sqrt_information,
                           target_translation)
```

核心 residual：

```cpp
const Eigen::Matrix<T, 2, 1> error(pose[0] - T(x_), pose[1] - T(y_));
const Eigen::Matrix<T, 2, 1> weighted_error = sqrt_information_ * error;
```

为什么 `sqrt_information_` 保持 `double`？

因为 Ceres AutoDiff 的模板类型 `T` 可能是 `ceres::Jet`。如果在 functor 内部做 Eigen 特征值分解、矩阵开方，或者把大量矩阵强制转换成 `T`，容易引入编译错误和数值问题。

当前写法是：

```text
在 functor 外部提前算好 double 类型的 2x2 矩阵。
在 functor 内部只做 double 矩阵乘以 T 向量。
```

这是最稳的 AutoDiff 写法。

### 8.9 注入 Ceres 的位置

文件：

```text
src/cartographer/cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.cc
```

位置：

```text
ceres_scan_matcher_2d.cc:447-456
```

当前代码逻辑：

```text
先调用 ComputeAnisotropicTranslationSqrtInformation()
再把 2x2 sqrt_information 传给 TranslationDeltaCostFunctor2D
最后 AddResidualBlock()
```

这就是创新点一真正影响优化结果的位置。

如果只发布 `/degeneracy_metric`，但没有改这里，算法只是“检测到退化”，不会真正影响小车建图。当前实现已经改到了 Ceres residual，所以权重会进入优化。

### 8.10 创新点一 ROS 输出

文件：

```text
src/cartographer_ros/cartographer_ros/cartographer_ros/node.cc
```

输出话题：

```text
/degeneracy_metric      std_msgs/Float32
/degeneracy_direction   geometry_msgs/Vector3
```

发布位置：

```text
node.cc:290-302
```

意义：

```text
/degeneracy_metric
  越接近 1，说明越像长走廊方向性退化。

/degeneracy_direction
  退化主方向向量。长走廊中应大致指向走廊纵向。
```

### 8.11 创新点一配置参数

文件：

```text
src/my_navigation/config/cartographer_scout_2d.lua:58-72
```

参数解释：

```text
directional_adaptive_fusion_enabled
  是否启用创新点一。

directional_degeneracy_condition_number_threshold
  条件数阈值，kappa 高于附近时 D_conf 开始明显升高。

directional_degeneracy_sigmoid_slope
  Sigmoid 曲线斜率，越大越接近硬阈值。

directional_degeneracy_smoothing_alpha
  时间平滑系数。越小越平滑，但响应更慢。

directional_degeneracy_min_num_points
  点数太少时不做协方差判断。

directional_degeneracy_eigenvalue_epsilon
  特征值数值保护。

directional_adaptive_odom_longitudinal_alpha
  退化纵向 odom 权重增强系数。

directional_adaptive_odom_lateral_alpha
  横向 odom 权重增强系数。当前设为 0，表示横向不额外增强 odom。

directional_adaptive_scan_longitudinal_beta
  退化纵向 scan 权重削弱系数。

directional_adaptive_scan_lateral_beta
  横向 scan 权重削弱系数。当前设为 0，表示横向保持 LiDAR 约束。

directional_adaptive_min_scan_weight_scale
  scan 权重下限，防止某方向权重被压到接近 0。

directional_adaptive_log_scale_change_threshold
  日志触发阈值，避免每帧刷屏。
```

## 9. Submap 和概率栅格

### 9.1 ActiveSubmaps2D

文件：

```text
src/cartographer/cartographer/mapping/2d/submap_2d.cc
```

重点函数：

```text
ActiveSubmaps2D::InsertRangeData()  submap_2d.cc:165-185
```

逻辑：

```text
170-172  如果没有 submap，或者最新 submap 收满 num_range_data，则新建 submap
176-178  当前 scan 插入所有 active submap
179-183  老 submap 收到 2*num_range_data 后 Finish
```

Cartographer 为什么同时维护两个 active submap？

因为如果一个 submap 突然停止、下一个 submap 突然开始，scan matching 的参考地图会不连续。两个 submap 有重叠期，地图接力更平滑。

### 9.2 ProbabilityGridRangeDataInserter2D

文件：

```text
src/cartographer/cartographer/mapping/2d/probability_grid_range_data_inserter_2d.cc
```

重点函数：

```text
ProbabilityGridRangeDataInserter2D::Insert()  probability_grid_range_data_inserter_2d.cc:146-157
CastRays()                                    probability_grid_range_data_inserter_2d.cc:90-118
```

作用：

```text
hit 点：把障碍端点附近栅格占用概率提高，地图上变黑。
miss/free ray：把射线经过区域占用概率降低，地图上变白。
```

所以 RViz 中 `/map` 从灰色未知慢慢变白，是因为激光射线确认了自由空间，不是地图坏了。

## 10. 后端位姿图：PoseGraph2D

文件：

```text
src/cartographer/cartographer/mapping/internal/2d/pose_graph_2d.cc
```

### 10.1 添加节点

重点函数：

```text
PoseGraph2D::AddNode()  pose_graph_2d.cc:160-180
```

逻辑：

```text
164-165  把前端 local_pose 投到当前 global/map 坐标
169-170  AppendNode()
175-179  后端异步排队 ComputeConstraintsForNode()
```

意思是：

```text
前端给出局部位姿。
后端先用当前 local_to_global 变换给节点一个全局初值。
然后后台慢慢搜索 submap-node 约束和回环约束。
```

### 10.2 添加 odometry 数据

位置：

```text
PoseGraph2D::AddOdometryData()  pose_graph_2d.cc:231-240
```

这里不是直接优化，而是把 odom 数据排到后端工作队列中，最后写入 `OptimizationProblem2D`。

### 10.3 后端异步线程

位置：

```text
PoseGraph2D::AddWorkItem()  pose_graph_2d.cc:183-201
```

Cartographer 后端不是每帧同步优化，而是把工作放入队列，由后台线程处理。

这对创新点二很重要：

```text
前端 Local SLAM 高频运行。
后端 PoseGraph 低频、异步优化。
如果共享指标缓存不加锁，会有线程安全问题。
如果后端只用 latest metric，会和历史边时间不对齐。
```

当前创新点二第二版已经用 NodeId 绑定解决这个问题。

## 11. 后端优化：OptimizationProblem2D

文件：

```text
src/cartographer/cartographer/mapping/internal/optimization/optimization_problem_2d.cc
src/cartographer/cartographer/mapping/internal/optimization/optimization_problem_2d.h
```

### 11.1 OptimizationProblem2D 做什么

它把 pose graph 问题转换成 Ceres 优化问题。

优化变量：

```text
每个 submap 的全局位姿 x, y, yaw
每个 trajectory node 的全局位姿 x, y, yaw
可选 landmark 位姿
```

主要 residual：

```text
submap-node 约束
landmark 约束
odometry 相邻节点约束
local SLAM 相邻节点约束
fixed frame pose 约束
```

核心函数：

```text
OptimizationProblem2D::Solve()  optimization_problem_2d.cc:318-516
```

### 11.2 原版 odometry 约束在哪里

位置：

```text
optimization_problem_2d.cc:428-501
```

原版逻辑是：

```text
CalculateOdometryBetweenNodes()
  得到相邻两个节点之间的轮式里程计相对位姿

CreateAutoDiffSpaCostFunction()
  把这个相对位姿变成 Ceres residual

options_.odometry_translation_weight()
options_.odometry_rotation_weight()
  使用 Lua 中固定权重
```

创新点二就在这里把固定权重改成动态权重。

## 12. 创新点二：无 IMU 一致性异常检测与后端动态调权

### 12.1 一句话定义

创新点二不是严格意义上的“真实物理打滑检测器”，而是：

> 在无 IMU 条件下，利用 LiDAR local SLAM 和 wheel odometry 在相邻关键帧之间的相对运动一致性，检测 odometry 是否出现横向/航向异常，并在后端 pose graph 中按边动态降低 odometry 约束权重。

论文中建议使用术语：

```text
一致性异常检测（Consistency Anomaly Detection）
```

而不是过度宣称：

```text
真实打滑检测（Physical Slip Ground Truth Detection）
```

### 12.2 核心源码文件

```text
src/cartographer/cartographer/mapping/internal/optimization/slip_detector.h
src/cartographer/cartographer/mapping/internal/optimization/optimization_problem_2d.cc
src/cartographer/cartographer/mapping/internal/optimization/optimization_problem_2d.h
src/cartographer/cartographer/mapping/slip_metric.h
src/cartographer/cartographer/mapping/proto/pose_graph/optimization_problem_options.proto
src/cartographer/cartographer/mapping/internal/optimization/optimization_problem_options.cc
src/cartographer_ros/cartographer_ros/cartographer_ros/node.cc
src/my_navigation/config/cartographer_scout_2d.lua
```

### 12.3 SlipDetector 类

文件：

```text
src/cartographer/cartographer/mapping/internal/optimization/slip_detector.h
```

关键位置：

```text
SlipDetectorResult        slip_detector.h:15-26
SlipDetector              slip_detector.h:28-171
SlipDetector::Update()    slip_detector.h:74-151
```

输入：

```text
scan_relative：LiDAR local SLAM 相邻节点相对位姿 T_scan_ij
odom_relative：wheel odometry 相邻节点相对位姿 T_odom_ij
lidar_reliability_available
lidar_reliability
```

输出：

```text
lateral_error
yaw_error
slip_score
gated_slip_score
slipping
weight_scale
```

### 12.4 横向和航向残差怎么计算

源码：

```text
slip_detector.h:100-114
```

数学：

```text
E_ij = inv(T_scan_ij) * T_odom_ij
e_y = abs(E_ij.translation.y)
e_theta = abs(NormalizeAngle(E_ij.rotation.angle))
score = w_y * e_y + w_theta * e_theta
```

为什么不直接比较世界坐标下的 y？

因为世界坐标 y 不一定等于机器人横向。把 odom 相对运动放到 scan_relative 坐标系下比较，`E_ij.translation.y()` 才更接近“相对 LiDAR 观测的横向不一致”。

为什么不用纵向 x？

因为长走廊中 LiDAR 沿走廊方向本来就可能退化。创新点二故意重点看：

```text
横向 lateral
航向 yaw
```

这两个方向在走廊两侧墙体存在时通常比纵向更可靠。

### 12.5 退化门控

源码：

```text
slip_detector.h:112-125
optimization_problem_2d.cc:433-454
```

创新点二第二版不是盲目相信 LiDAR。

它会读取创新点一绑定到 NodeId 的退化指标：

```text
optimization_problem_2d.cc:436-446
```

得到：

```text
lidar_reliability_available
lidar_reliability
```

然后传给：

```text
SlipDetector::Update()
```

如果 LiDAR 可靠性不足或未知，并且：

```text
slip_unknown_keep_previous_weight = true
```

则本条边保持上一权重，不更新 slipping 状态。

这样做的意义：

```text
如果 LiDAR 自己都不可靠，就不能拿 LiDAR-odom 差异武断证明 odom 错。
```

### 12.6 迟滞状态机

源码：

```text
slip_detector.h:128-137
```

逻辑：

```text
if gated_score > high_threshold:
    slipping = true
else if gated_score < low_threshold:
    slipping = false
else:
    保持上一状态
```

为什么要两个阈值？

因为如果只有一个阈值，指标在临界值附近上下波动时，权重会频繁开关，后端优化不稳定。

专业名词：

```text
hysteresis，迟滞判定
```

### 12.7 快速降权和慢速恢复

源码：

```text
slip_detector.h:139-147
```

逻辑：

```text
如果 slipping = true:
    current_weight_scale = min_weight_scale

如果 slipping = false:
    current_weight_scale =
        recovery_alpha * max_weight_scale
        + (1 - recovery_alpha) * current_weight_scale
```

含义：

```text
异常时快速降权，避免错误 odom 继续强拉地图。
恢复时慢慢升权，避免 Ceres 约束突然变化。
```

专业名词：

```text
fast attack
slow recovery
first-order low-pass filter
```

### 12.8 NodeId 绑定历史退化指标

文件：

```text
src/cartographer/cartographer/mapping/internal/optimization/optimization_problem_2d.cc
src/cartographer/cartographer/mapping/internal/optimization/optimization_problem_2d.h
```

位置：

```text
optimization_problem_2d.cc:252-271
optimization_problem_2d.cc:278-288
optimization_problem_2d.h:128-133
```

为什么要这样做？

Cartographer 前端和后端是异步的：

```text
前端 Local SLAM：高频处理 scan。
后端 Global SLAM：低频批量优化历史节点。
```

如果后端优化历史边时直接读取“最新退化指标”，就会出现时间错位：

```text
后端正在优化 30 秒前的边，却读到了当前时刻的 scan 退化指标。
```

当前第二版做法：

```text
节点 AddTrajectoryNode() 时，按节点时间查询最近的退化指标。
把这个指标拷贝绑定到 NodeId。
以后 Solve() 优化历史边时，只读 NodeId 对应的稳定副本。
```

这就是：

```text
time-aligned metric binding
```

或者论文中可说：

```text
面向异步前后端的节点级退化指标固化机制。
```

### 12.9 动态权重注入后端 Ceres

真正影响地图的位置：

```text
optimization_problem_2d.cc:451-501
```

当前流程：

```text
451-454  SlipDetector::Update()
455-458  计算 dynamic_odometry_translation_weight 和 dynamic_odometry_rotation_weight
496-501  AddResidualBlock() 时使用动态权重
```

核心代码逻辑：

```cpp
dynamic_odometry_translation_weight =
    options_.odometry_translation_weight() * slip_result.weight_scale;

dynamic_odometry_rotation_weight =
    options_.odometry_rotation_weight() * slip_result.weight_scale;
```

然后：

```cpp
CreateAutoDiffSpaCostFunction(Constraint::Pose{
    *relative_odometry,
    dynamic_odometry_translation_weight,
    dynamic_odometry_rotation_weight})
```

注意：当前毕业版本保持**标量缩放**，没有做 3x3 各向异性后端信息矩阵。这是有意收缩工程范围。

论文里可以写：

> 为保证系统稳定性和工程可落地性，本文在后端 odometry 约束层采用边级标量动态缩放方式，同时保留 Cartographer 原有 SPA cost function 结构，不修改 Ceres 后端残差维度。

### 12.10 创新点二诊断指标

文件：

```text
src/cartographer/cartographer/mapping/slip_metric.h
```

结构体位置：

```text
slip_metric.h:18-40
```

字段含义：

```text
slipping
  当前是否处于一致性异常状态。

slip_score
  未门控的横向/航向综合异常分数。

gated_slip_score
  乘以 LiDAR reliability 后的异常分数。

lateral_error
  横向不一致残差。

yaw_error
  航向不一致残差。

weight_scale
  当前 odometry 权重缩放系数。

translation_weight / rotation_weight
  实际注入 Ceres 的 odometry 权重。

anomaly_trigger_count
  本次 Solve() 中异常状态上升沿次数。

minimum_weight_scale
  本次 Solve() 中最小 odometry 权重缩放。
```

ROS 发布位置：

```text
node.cc:304-335
```

话题：

```text
/slip_metric
/slip_state
/odom_weight_scale
/slip_lidar_reliability
/consistency_anomaly_trigger_count
/odom_min_weight_scale
```

### 12.11 创新点二配置参数

文件：

```text
src/my_navigation/config/cartographer_scout_2d.lua:95-113
```

参数解释：

```text
slip_adaptive_odometry_weight_enabled
  是否启用创新点二。

slip_lateral_error_weight
  横向残差权重。

slip_yaw_error_weight
  航向残差权重。

slip_high_threshold
  一致性异常触发高阈值。

slip_low_threshold
  正常恢复低阈值，必须低于 high threshold。

slip_min_weight_scale
  异常时 odometry 权重最小缩放。

slip_max_weight_scale
  正常时 odometry 权重最大缩放。

slip_recovery_alpha
  一阶低通恢复系数。

slip_min_motion_distance / slip_min_motion_angle
  运动太小时不更新状态，避免静止噪声触发异常。

slip_lidar_reliability_gate_enabled
  是否启用 LiDAR 可靠性门控。

slip_lidar_reliability_min
  门控可靠性阈值。

slip_degeneracy_metric_max_time_delta_sec
  后端节点查询创新点一退化指标时允许的最大时间差。

slip_unknown_keep_previous_weight
  指标未知时是否保持上一权重。
```

### 12.12 创新点二边界和局限

当前模块能证明：

```text
当 wheel odometry 与 LiDAR local SLAM 在横向/航向上明显不一致时，
后端 odometry 约束会被动态降低，
从而减少错误 odom 对地图和轨迹的拉扯。
```

当前模块不能证明：

```text
真实轮胎物理打滑一定发生。
没有 IMU，没有轮胎动力学模型，没有地面真值时，不能直接观测真实打滑。
```

所以论文中应写：

```text
本文检测的是 LiDAR-odometry 一致性异常，可作为无 IMU 条件下轮式里程计异常的代理指标。
```

未来工作可以写：

```text
引入 IMU 或 ground truth 验证真实打滑。
引入 3x3 odometry 信息矩阵区分纵向、横向和航向。
使用 Hessian/NIS/Mahalanobis 构造更严格的统计检验。
```

当前毕业版本不做这些复杂升级。

## 13. 两个创新点如何协同

创新点一在前端：

```text
每帧 scan
  -> 点云协方差
  -> 退化方向和 D_conf
  -> 前端 Ceres 平移先验 2x2 各向异性权重
  -> 发布 /degeneracy_metric 和 /degeneracy_direction
  -> 按时间缓存退化指标
```

创新点二在后端：

```text
相邻后端节点
  -> 读取 NodeId 绑定的退化指标
  -> 判断 LiDAR 可靠性
  -> 比较 T_scan_ij 和 T_odom_ij 的 lateral/yaw 残差
  -> 异常时降低 odometry residual 权重
  -> 发布 /odom_weight_scale 等诊断
```

二者关系：

```text
创新点一解决：长走廊中“哪个方向退化、权重怎么各向异性调整”。
创新点二解决：odometry 发生一致性异常时，后端不要继续强信 odom。
```

创新点二依赖创新点一的一部分输出作为门控，但创新点二不是创新点一的重复。前者发生在后端 odometry 约束，后者发生在前端 scan matcher 平移先验。

## 14. `/map` 是怎么来的

很多人误以为 `cartographer_node` 直接发布 `/map`。实际不是。

Cartographer 2D 的 `/map` 来源：

```text
cartographer_node
  -> 发布 submap_list
  -> 提供 submap_query 服务

cartographer_occupancy_grid_node
  -> 订阅 submap_list
  -> 调用 submap_query
  -> PaintSubmapSlices()
  -> CreateOccupancyGridMsg()
  -> 发布 /map
```

相关文件：

```text
src/cartographer_ros/cartographer_ros/cartographer_ros/occupancy_grid_node_main.cc
src/cartographer_ros/cartographer_ros/cartographer_ros/map_builder_bridge.cc
```

所以：

```text
/map 是优化后的多个 submap 拼接结果。
/local_occupancy_grid 是本项目 local_grid_mapper 用最新 /scan 单独生成的局部图。
```

二者不是同一个地图，也不是同一个算法。

## 15. 当前源码修改清单

### 15.1 创新点一相关

```text
src/cartographer/cartographer/mapping/directional_degeneracy_metric.h
  新增退化指标结构体和查询接口。

src/cartographer/cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.cc
  新增退化指标缓存、Sigmoid、协方差特征值分析、2x2 权重矩阵构造。
  在 CeresScanMatcher2D::Match() 中注入各向异性平移先验。

src/cartographer/cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h
  声明 ComputeAnisotropicTranslationSqrtInformation() 和平滑状态。

src/cartographer/cartographer/mapping/internal/2d/scan_matching/translation_delta_cost_functor_2d.h
  将平移先验从标量 scaling factor 扩展为 2x2 sqrt information。

src/cartographer/cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_2d.proto
  增加创新点一配置字段。

src/cartographer_ros/cartographer_ros/cartographer_ros/trajectory_options.cc
  use_odometry=false 时强制关闭创新点一，保护纯激光模式。

src/cartographer_ros/cartographer_ros/cartographer_ros/node.cc
  发布 /degeneracy_metric 和 /degeneracy_direction。

src/my_navigation/config/cartographer_scout_2d.lua
  增加 [Innovation 1] 参数区。
```

### 15.2 创新点二相关

```text
src/cartographer/cartographer/mapping/internal/optimization/slip_detector.h
  新增无 IMU 一致性异常检测器。

src/cartographer/cartographer/mapping/slip_metric.h
  新增后端诊断指标结构体。

src/cartographer/cartographer/mapping/internal/optimization/optimization_problem_2d.cc
  在 AddTrajectoryNode() 中把退化指标绑定到 NodeId。
  在 Solve() 中对每条 odometry 边计算动态权重。
  发布后端一致性异常诊断快照。

src/cartographer/cartographer/mapping/internal/optimization/optimization_problem_2d.h
  增加 directional_degeneracy_metrics_by_node_。

src/cartographer/cartographer/mapping/proto/pose_graph/optimization_problem_options.proto
  增加创新点二配置字段。

src/cartographer/cartographer/mapping/internal/optimization/optimization_problem_options.cc
  从 Lua 读取创新点二配置。

src/cartographer_ros/cartographer_ros/cartographer_ros/node.cc
  发布 /slip_metric、/slip_state、/odom_weight_scale、
  /slip_lidar_reliability、/consistency_anomaly_trigger_count、
  /odom_min_weight_scale。

src/my_navigation/config/cartographer_scout_2d.lua
  增加 [Innovation 2] 参数区。
```

## 16. 源码阅读顺序

建议不要从 Ceres 和 pose graph 直接开始。按下面顺序读最清楚。

第一层：项目入口和配置

```text
src/my_navigation/launch/scout_mini_mapping_local_grid.launch
src/my_navigation/config/cartographer_scout_2d.lua
```

读完要能回答：

```text
Cartographer 订阅 /scan 还是 /points2？
是否使用 /odom？
是否使用 IMU？
创新点参数在哪开关？
```

第二层：ROS 消息入口

```text
src/cartographer_ros/cartographer_ros/cartographer_ros/node.cc
src/cartographer_ros/cartographer_ros/cartographer_ros/sensor_bridge.cc
```

读完要能回答：

```text
/scan 进哪个函数？
/odom 进哪个函数？
TF 在哪里查？
创新点诊断话题在哪里发布？
```

第三层：前端 scan matching

```text
src/cartographer/cartographer/mapping/internal/global_trajectory_builder.cc
src/cartographer/cartographer/mapping/internal/2d/local_trajectory_builder_2d.cc
src/cartographer/cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.cc
src/cartographer/cartographer/mapping/internal/2d/scan_matching/translation_delta_cost_functor_2d.h
```

读完要能回答：

```text
pose_prediction 怎么来？
Ceres 优化变量是什么？
occupied space residual 是什么？
创新点一在哪里影响 Ceres？
```

第四层：submap 和地图

```text
src/cartographer/cartographer/mapping/2d/submap_2d.cc
src/cartographer/cartographer/mapping/2d/probability_grid_range_data_inserter_2d.cc
src/cartographer_ros/cartographer_ros/cartographer_ros/occupancy_grid_node_main.cc
```

读完要能回答：

```text
scan 怎么写进 submap？
hit/miss 怎么影响黑白地图？
/map 为什么不是单帧 scan？
```

第五层：后端优化

```text
src/cartographer/cartographer/mapping/internal/2d/pose_graph_2d.cc
src/cartographer/cartographer/mapping/internal/optimization/optimization_problem_2d.cc
src/cartographer/cartographer/mapping/internal/optimization/slip_detector.h
```

读完要能回答：

```text
节点怎么进入 pose graph？
odometry 约束在哪里加？
创新点二如何计算一致性异常？
动态权重在哪里乘到 Ceres residual？
```

## 17. 论文方法章节推荐写法

### 17.1 创新点一可写成

```text
针对长走廊场景中 2D 激光扫描沿走廊方向约束不足的问题，本文在 Cartographer 2D 前端 Ceres scan matcher 中引入方向性退化感知机制。首先对当前帧 2D 点云计算协方差矩阵，并通过特征值分解获得主方向和条件数；随后利用 Sigmoid 函数将条件数映射为连续退化置信度，并通过一阶低通滤波进行时间平滑。最后根据退化主方向构造 2x2 各向异性平移先验矩阵，使优化在退化方向降低激光匹配的相对影响，在非退化方向保持激光约束。
```

源码对应：

```text
ceres_scan_matcher_2d.cc:251-409
translation_delta_cost_functor_2d.h:39-68
ceres_scan_matcher_2d.cc:451-456
```

### 17.2 创新点二可写成

```text
针对无 IMU 条件下轮式里程计可能受打滑或侧滑影响的问题，本文在 Cartographer 2D 后端优化中引入 LiDAR-odometry 一致性异常检测。该模块比较相邻关键帧之间 LiDAR local SLAM 相对位姿与 wheel odometry 相对位姿，在激光可靠性门控通过时，根据横向残差和航向残差构造异常指标。系统采用双阈值迟滞状态机避免频繁切换；异常状态下快速降低 odometry 约束权重，正常状态下通过一阶低通滤波平滑恢复权重。该方法不依赖 IMU，也不修改 Cartographer 原有 SPA cost function 结构。
```

源码对应：

```text
slip_detector.h:74-151
optimization_problem_2d.cc:421-501
optimization_problem_2d.cc:252-271
```

## 18. 实验观察点

### 18.1 创新点一观察

运行 Proposed 时关注：

```text
/degeneracy_metric
/degeneracy_direction
Cartographer 日志 [Innovation1]
地图中走廊纵向重影是否减少
墙体横向连续性是否保持
```

长走廊中期望：

```text
/degeneracy_metric 升高
/degeneracy_direction 指向走廊纵向
longitudinal_scale 增大
lateral_scale 接近 1 或变化较小
```

### 18.2 创新点二观察

运行 odom 异常注入实验时关注：

```text
/slip_metric
/slip_state
/odom_weight_scale
/slip_lidar_reliability
/consistency_anomaly_trigger_count
/odom_min_weight_scale
Cartographer 日志 [Innovation2]
```

异常区间期望：

```text
/slip_metric 升高
/slip_state 变 true
/odom_weight_scale 快速下降到 slip_min_weight_scale 附近
/odom_min_weight_scale 小于 1
Proposed 地图比静态高 odom 权重更少被拉歪
```

### 18.3 论文主图建议

建议做一张时间轴对齐图：

```text
上半部分：2D 地图 + 轨迹 + 异常区间位置标注
下半部分：
  蓝线：/odom_weight_scale
  绿虚线：/degeneracy_metric 或 /slip_lidar_reliability
  橙线：/slip_metric
  红色阴影：注入异常区间或检测异常区间
```

这张图要表达：

```text
异常发生前后，算法检测到一致性异常，并及时降低 odometry 权重，所以地图没有被错误 odom 拉歪。
```

## 19. 答辩常见问题

### Q1：为什么创新点一用点云协方差？

因为 2D 点云在长走廊中呈明显长条分布。协方差最大特征值方向对应点云分布最长方向，通常就是走廊纵向；最小特征值方向对应横向墙距约束。条件数能量化这种方向不均衡。

### Q2：为什么不用 D_x、D_y 标量投影？

因为退化方向不一定与机器人 x/y 轴完全重合。使用：

```text
R * diag(w_long, w_lat) * R^T
```

可以在任意方向上表达各向异性权重，矩阵乘法本身已经完成方向解耦。

### Q3：创新点一有没有真正影响优化？

有。真正影响位置是：

```text
ceres_scan_matcher_2d.cc:451-456
```

这里把 2x2 `sqrt_information` 传入 `TranslationDeltaCostFunctor2D`，进入 Ceres residual。

### Q4：创新点二是不是物理打滑检测？

严格说不是。它是无 IMU 条件下的 LiDAR-odometry 一致性异常检测。它能反映 odometry 与 LiDAR local SLAM 在横向/航向上的不一致，但不能单独证明真实轮胎打滑发生。

### Q5：为什么创新点二不用纵向误差？

因为长走廊中 LiDAR 纵向本来退化，拿纵向误差判断 odometry 是否异常容易误判。横向和航向通常由两侧墙体提供更强约束，所以更适合作为一致性检查维度。

### Q6：为什么后端只做标量权重缩放，不做 3x3 信息矩阵？

这是毕业版本的工程收缩。标量缩放可以最小侵入地接入 Cartographer 原有 SPA cost function，稳定、可编译、易实验。3x3 各向异性后端信息矩阵属于未来工作。

### Q7：为什么要 NodeId 绑定退化指标？

因为前端和后端异步。如果后端优化历史边时读取最新 scan 指标，会时间错位。NodeId 绑定能让每条历史边使用它对应节点时间附近的退化指标。

### Q8：没有 IMU 是否符合要求？

符合。当前配置 `TRAJECTORY_BUILDER_2D.use_imu_data = false`，创新点一只使用 2D scan 点云协方差，创新点二只比较 LiDAR local SLAM 和 wheel odometry 相对运动，没有引入 IMU 逻辑。

## 20. 最小验证命令

检查 launch 不含导航模块：

```bash
roslaunch my_navigation scout_mini_mapping_local_grid.launch --nodes
```

不应出现：

```text
move_base
explorer_controller
DWA
TEB
Frontier
```

检查基础话题：

```bash
rostopic hz /scan
rostopic hz /odom
rostopic echo -n 1 /tf_static
```

检查创新点话题：

```bash
rostopic echo /degeneracy_metric
rostopic echo /degeneracy_direction
rostopic echo /slip_metric
rostopic echo /odom_weight_scale
rostopic echo /slip_lidar_reliability
```

录包后检查：

```bash
rosbag info bags/corridor_normal_complete.bag
```

至少应包含：

```text
/scan
/odom
/tf
/tf_static
```

## 21. 最终阅读心法

把项目分成三层，不要混在一起：

```text
ROS 层：
  负责话题、TF、Lua 参数、诊断话题发布。

2D 前端：
  负责 scan matching、submap 插入、局部位姿估计。
  创新点一在这里。

后端位姿图：
  负责 node/submap 约束、odometry residual、全局优化。
  创新点二在这里。
```

再把两种地图分清：

```text
/map：
  Cartographer 全局 submap 拼接图。

/local_occupancy_grid：
  本项目自定义局部实时栅格，只看当前车周围。
```

再把两个创新点分清：

```text
创新点一：
  前端，处理长走廊方向性退化，改 Ceres scan matcher 平移先验。

创新点二：
  后端，处理 LiDAR-odom 一致性异常，改 odometry 约束权重。
```

按这个结构读源码，你就不会把 `/map`、submap、local grid、scan matcher、pose graph 和两个创新点混在一起。
