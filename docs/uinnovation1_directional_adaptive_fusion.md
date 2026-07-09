# 创新点一：具备 2D 方向性感知的自适应激光-里程计融合模块

本文档用中文详细说明本项目“创新点一”的源码改动、数学原理、工程原因和验证方式。为了便于论文、答辩和后续调参，下面会同时使用通俗解释和专业名称。

## 1. 这个创新点解决什么问题

本项目当前传感器条件是：

- 2D 激光雷达，也就是 `/scan`
- 轮式里程计，也就是 `/odom`
- 没有 IMU

Cartographer 2D 原本可以使用 `/odom`，但它的前端 Ceres scan matcher 里，平移先验权重通常是一个全局静态标量：

```text
translation_weight = 10.
```

这意味着无论小车在开阔区域、普通房间，还是长走廊，Cartographer 都用同一个平移约束强度。

问题出现在长走廊场景：

- 走廊纵向，也就是沿着走廊前后方向，激光看到的几何结构变化很小。
- 走廊横向，也就是左右墙之间方向，激光约束很强。
- 如果仍然用一个统一权重，系统无法表达“纵向退化、横向不退化”这种事实。

本创新点的目标是：

```text
让 Cartographer 2D 能判断当前 scan 是否处于方向性退化场景，
并识别退化主方向，然后在 Ceres 优化中使用 2x2 各向异性权重矩阵。
```

专业名称：

- 方向性退化：Directional Degeneracy
- 各向异性权重：Anisotropic Weighting
- 激光-里程计自适应融合：Adaptive LiDAR-Odometry Fusion
- 条件数退化检测：Condition Number based Degeneracy Detection
- Ceres 平移先验残差：Ceres Translation Prior Residual

## 2. 整体实现思路

每次 Cartographer 2D 做 Ceres scan matching 时，都会拿到当前帧过滤后的 2D 点云。我们就在这个时刻做以下事情：

1. 用当前帧 2D 点云计算一个 2x2 协方差矩阵。
2. 对协方差矩阵做特征值分解。
3. 最大特征值对应的方向，认为是点云拉得最长的方向。
4. 如果最大特征值和最小特征值差距很大，说明点云形状很细长，常见于走廊。
5. 用 sigmoid 函数把“细长程度”变成 0 到 1 的退化置信度。
6. 对退化置信度和方向做时间平滑，避免权重一帧一帧乱跳。
7. 用完整 2x2 矩阵构造 Ceres 平移先验残差权重。
8. ROS 层发布 `/degeneracy_metric` 和 `/degeneracy_direction`，便于 RViz 或实验分析。

专业链路：

```text
PointCloud -> 2D Covariance Matrix -> Eigen Decomposition
-> Condition Number -> Sigmoid Degeneracy Confidence
-> Low-pass Filtering -> 2x2 Sqrt Information Matrix
-> Ceres TranslationDeltaCostFunctor2D
```

## 3. 为什么用 2D 点云协方差

当前帧 scan 中每个点都可以看成二维点：

```text
p_i = [x_i, y_i]^T
```

先计算均值：

```text
mean = average(p_i)
```

再计算协方差矩阵：

```text
Sigma = average((p_i - mean) * (p_i - mean)^T)
```

这个 `Sigma` 是一个 2x2 矩阵：

```text
Sigma = [ sigma_xx  sigma_xy
          sigma_yx  sigma_yy ]
```

通俗理解：

- 如果点云在某个方向拉得很长，这个方向的方差就大。
- 如果点云在另一个方向很窄，这个方向的方差就小。
- 长走廊中的点云通常会沿走廊方向形成细长分布。

专业解释：

协方差矩阵描述了点云的二阶几何分布。对称正定或半正定的 2x2 协方差矩阵可以通过特征值分解得到主轴方向：

```text
Sigma * v = lambda * v
```

其中：

- `lambda_max`：最大特征值
- `lambda_min`：最小特征值
- `v_long`：最大特征值对应的特征向量，也就是主方向
- `v_lat`：最小特征值对应的特征向量，也就是与主方向正交的方向

## 4. 为什么用条件数判断退化

条件数定义为：

```text
kappa = lambda_max / lambda_min
```

通俗理解：

- 如果 `kappa` 接近 1，说明点云比较均匀，不像走廊。
- 如果 `kappa` 很大，说明点云非常细长，存在明显方向性。
- 长走廊、长墙边、狭长通道通常会让 `kappa` 变大。

专业名称：

- 条件数：Condition Number
- 结构退化：Geometric Degeneracy
- 主方向：Principal Direction

代码中加入了数值保护：

```text
lambda_min = max(lambda_min, epsilon)
lambda_max = max(lambda_max, epsilon)
```

原因：

- 避免除以 0。
- 避免点数太少或点云几乎共线时产生数值爆炸。
- 保证每帧计算稳定。

## 5. 为什么不用硬阈值，而用 sigmoid

不能这样写：

```text
if kappa > threshold:
  confidence = 1
else:
  confidence = 0
```

原因是这会造成权重突然跳变。小车移动时，scan 点云每帧都有噪声，如果权重在 0 和 1 之间硬切换，Ceres 优化结果会抖动。

所以使用 sigmoid：

```text
D = sigmoid(slope * (kappa - threshold))
```

其中：

- `D` 是退化置信度，范围是 `[0, 1]`
- `threshold` 是开始认为退化明显的条件数中心
- `slope` 控制过渡快慢

通俗理解：

- `kappa` 小时，`D` 接近 0。
- `kappa` 很大时，`D` 接近 1。
- 中间区域是平滑过渡，不会突然变。

专业名称：

- Sigmoid 映射：Sigmoid Mapping
- 平滑置信度：Smooth Confidence
- 软阈值：Soft Thresholding

## 6. 为什么还要做时间平滑

即使使用 sigmoid，单帧 scan 仍然可能因为噪声、遮挡、动态障碍物导致方向和置信度跳动。

所以代码中又加了一阶低通滤波：

```text
smoothed = alpha * current + (1 - alpha) * previous
```

通俗理解：

- `alpha` 越大，系统越灵敏，但更容易抖。
- `alpha` 越小，系统更稳，但响应更慢。

专业名称：

- 一阶低通滤波：First-order Low-pass Filter
- 指数滑动平均：Exponential Moving Average, EMA
- 时间一致性：Temporal Consistency

另外，特征向量有一个工程细节：特征向量的正负号本身没有唯一性。

也就是说：

```text
v 和 -v 在数学上表示同一条方向轴
```

如果不处理，RViz 中方向箭头可能一帧朝前、一帧朝后。代码中通过和上一帧方向做点积来统一方向：

```text
if previous_direction dot current_direction < 0:
  current_direction = -current_direction
```

专业名称：

- Eigenvector Sign Ambiguity
- Direction Sign Alignment

## 7. 为什么删除 D_x、D_y，直接用完整 2x2 矩阵

早期讨论中曾考虑把退化置信度拆成：

```text
D_x
D_y
```

后来明确删除这种做法。

原因是方向解耦本来就应该由完整 2x2 矩阵完成：

```text
S = R * diag(w_long, w_lat) * R^T
```

其中：

```text
R = [v_long, v_lat]
```

通俗理解：

- `v_long` 是退化主方向，比如走廊纵向。
- `v_lat` 是非退化方向，比如走廊横向。
- `diag(w_long, w_lat)` 表示在这两个方向上分别给不同权重。
- 左右乘 `R` 和 `R^T` 后，矩阵会自动把这些方向权重转换回原来的 x/y 坐标系。

所以不需要再人为投影出 `D_x` 和 `D_y`。否则会重复计算方向信息，反而容易让逻辑混乱。

专业名称：

- 特征基坐标系：Eigenbasis
- 对角信息矩阵：Diagonal Information Matrix
- 坐标变换：Coordinate Transformation
- 各向异性信息矩阵：Anisotropic Information Matrix

## 8. Ceres 里具体改了什么

原始 Cartographer 2D 的平移先验残差大致是：

```text
r = w * (p - p0)
```

其中：

- `p` 是 Ceres 当前优化的位姿平移量
- `p0` 是外推器给出的预测平移
- `w` 是静态标量 `translation_weight`

本创新点改成：

```text
r = S * (p - p0)
```

其中：

- `S` 是 2x2 sqrt information matrix
- `S` 的构造方式是：

```text
S = R * diag(w_long, w_lat) * R^T
```

这里的 `S` 不是普通权重标量，而是一个矩阵。它可以表达：

- 沿走廊方向：更相信 odom/motion prior
- 垂直走廊方向：继续让 LiDAR scan matching 起主要作用

专业名称：

- 残差加权：Residual Weighting
- 平方根信息矩阵：Square-root Information Matrix
- 运动先验：Motion Prior
- 前端扫描匹配：Front-end Scan Matching

## 9. 为什么只改 TranslationDeltaCostFunctor2D，而不硬改 occupied-space cost

Cartographer 2D 的 Ceres scan matcher 里主要有三类残差：

1. `OccupiedSpaceCostFunction2D`
2. `TranslationDeltaCostFunctor2D`
3. `RotationDeltaCostFunctor2D`

其中 `OccupiedSpaceCostFunction2D` 是每个激光点落到栅格地图上的概率代价。它不是一个显式的二维平移误差：

```text
不是 e = [dx, dy]
```

而是类似：

```text
每个点落到地图占用概率上的标量代价
```

所以不能简单地给它乘一个 2x2 矩阵。硬改它会涉及插值栅格代价函数的 ABI 和残差维度，侵入性很大，也容易破坏 Cartographer 原有行为。

因此本实现采用更稳妥的方式：

```text
保持 occupied-space cost 原样，
通过 TranslationDeltaCostFunctor2D 的 2x2 各向异性先验改变 LiDAR 与 odom/motion prior 的相对影响。
```

通俗理解：

- 激光地图匹配项仍然负责贴墙。
- 平移先验项变成方向性约束。
- 走廊纵向退化时，平移先验变强，避免激光在走廊方向乱滑。
- 横向不退化时，平移先验不乱增强，保留激光对左右墙的校正能力。

专业名称：

- 最小侵入式修改：Minimally Invasive Modification
- ABI 兼容：Application Binary Interface Compatibility
- 相对约束权重：Relative Constraint Weighting

## 10. Ceres AutoDiff 为什么要特别小心

Ceres 使用 AutoDiff 时，优化变量不是普通 `double`，而可能是 Ceres 的 Jet 类型。

如果在 functor 的 `operator()` 里随便做矩阵类型转换，容易引发模板推导问题。

本实现遵守以下原则：

```text
sqrt_information_ 使用 Eigen::Matrix<double, 2, 2>
误差向量 error 使用 Eigen::Matrix<T, 2, 1>
用 double 矩阵直接乘 T 类型误差向量
```

代码逻辑对应：

```cpp
const Eigen::Matrix<T, 2, 1> error(pose[0] - T(x_), pose[1] - T(y_));
const Eigen::Matrix<T, 2, 1> weighted_error = sqrt_information_ * error;
```

为什么这样写：

- `sqrt_information_` 是每帧优化前预先算好的常量。
- 不需要在 AutoDiff 内部做特征值分解。
- 不需要把 2x2 矩阵强行转成模板类型 `T`。
- Ceres Jet 类型只存在于误差向量和残差传播中，推导最安全。

专业名称：

- Ceres AutoDiff
- Jet Type
- Template Scalar Type
- Constant Jacobian Weighting

## 11. 动态权重如何理解

基础权重来自 Lua：

```text
translation_weight = 10.
```

退化置信度为：

```text
D in [0, 1]
```

代码中构造了两个方向的实际权重：

```text
w_long = base_weight * odom_long_scale / scan_long_scale
w_lat  = base_weight * odom_lat_scale  / scan_lat_scale
```

其中：

- `long` 表示退化主方向
- `lat` 表示横向或非退化方向
- `odom_*_scale` 用于增强 motion prior
- `scan_*_scale` 用于表达相对降低 LiDAR 在对应方向的影响

注意：这里不是直接修改 occupied-space cost，而是通过增强或保持 translation prior 的方向性权重，改变 Ceres 最终优化时各方向的相对信任关系。

专业名称：

- Longitudinal Direction：纵向或退化主方向
- Lateral Direction：横向或非退化方向
- Relative Weight Scaling：相对权重缩放
- Adaptive Motion Prior：自适应运动先验

## 12. 修改了哪些文件

### 12.1 `ceres_scan_matcher_options_2d.proto`

文件：

```text
src/cartographer/cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_2d.proto
```

新增参数：

```text
directional_adaptive_fusion_enabled
directional_degeneracy_condition_number_threshold
directional_degeneracy_sigmoid_slope
directional_degeneracy_smoothing_alpha
directional_degeneracy_min_num_points
directional_degeneracy_eigenvalue_epsilon
directional_adaptive_odom_longitudinal_alpha
directional_adaptive_odom_lateral_alpha
directional_adaptive_scan_longitudinal_beta
directional_adaptive_scan_lateral_beta
directional_adaptive_min_scan_weight_scale
directional_adaptive_log_scale_change_threshold
```

为什么要改：

- Cartographer 的 Lua 参数最终会进入 proto options。
- 如果 proto 不加字段，C++ 层无法拿到这些配置。
- 新增字段可以让创新点一完全配置驱动。

专业名称：

- Protocol Buffers
- Runtime Configuration
- Parameter-driven Design

### 12.2 `translation_delta_cost_functor_2d.h`

文件：

```text
src/cartographer/cartographer/mapping/internal/2d/scan_matching/translation_delta_cost_functor_2d.h
```

改动：

- 保留原来的标量权重构造方式。
- 新增 2x2 `sqrt_information` 构造方式。
- `operator()` 中用矩阵乘误差向量。

为什么要改：

- 原来的标量权重只能表示各向同性约束。
- 2x2 矩阵才能表达方向性约束。
- 保留旧接口可以降低对原有代码和测试的影响。

专业名称：

- Cost Functor
- Residual Block
- Isotropic Weight
- Anisotropic Weight

### 12.3 `ceres_scan_matcher_2d.cc / .h`

文件：

```text
src/cartographer/cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.cc
src/cartographer/cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h
```

改动：

- 读取 Lua/proto 中新增参数。
- 新增 `ComputeAnisotropicTranslationSqrtInformation()`。
- 在 `Match()` 中调用该函数，生成 2x2 权重矩阵。
- 将 2x2 矩阵传给 `TranslationDeltaCostFunctor2D`。
- 保存最新退化指标。

为什么要改：

- CeresScanMatcher2D 是 Cartographer 2D 前端精匹配的核心。
- 当前 scan 点云、submap grid、初始位姿和 Ceres problem 都在这里汇合。
- 在这里注入方向性感知，路径最短、侵入最小。

专业名称：

- Local SLAM Front-end
- Ceres Scan Matcher
- Scan-to-Map Matching
- Optimization Problem Construction

### 12.4 `local_trajectory_builder_2d.cc`

文件：

```text
src/cartographer/cartographer/mapping/internal/2d/local_trajectory_builder_2d.cc
```

改动：

- 保存实时相关匹配分数 `real_time_correlative_score`。
- 调用 Ceres scan matcher 时把 score 传进去。

为什么要改：

- 用户需求中要求提取 Cartographer 内部 scan matcher 的实时匹配分数。
- 当前实现把该 score 放进退化指标结构中，便于后续日志、调参或扩展。

专业名称：

- Real-time Correlative Scan Matcher
- Scan Matching Score
- Coarse-to-Fine Matching

### 12.5 `directional_degeneracy_metric.h`

文件：

```text
src/cartographer/cartographer/mapping/directional_degeneracy_metric.h
```

改动：

- 新增公开指标结构 `DirectionalDegeneracyMetric`。
- 新增 `GetLatestDirectionalDegeneracyMetric()`。

为什么要改：

- `cartographer_ros` 不能直接 include Cartographer internal 目录下的头文件。
- 所以需要一个公开头文件，把 core 层指标安全暴露给 ROS 层。

专业名称：

- Public API Boundary
- Thread-safe Metric Snapshot
- Core-to-ROS Interface

### 12.6 `node.cc / node.h`

文件：

```text
src/cartographer_ros/cartographer_ros/cartographer_ros/node.cc
src/cartographer_ros/cartographer_ros/cartographer_ros/node.h
```

改动：

- 新增 ROS publisher：

```text
/degeneracy_metric
/degeneracy_direction
```

- 在 `PublishLocalTrajectoryData()` 中读取最新退化指标并发布。

为什么要改：

- 退化检测发生在 Cartographer core 的传感器处理线程。
- ROS 话题发布更适合放在 `cartographer_ros` 的定时发布逻辑里。
- 用 metric snapshot + mutex 读取，避免数据竞争。

专业名称：

- ROS Publisher
- Data Race
- Mutex Protection
- Timer Callback

### 12.7 `trajectory_options.cc`

文件：

```text
src/cartographer_ros/cartographer_ros/cartographer_ros/trajectory_options.cc
```

改动：

- 如果 `use_odometry = false`，强制关闭：

```text
directional_adaptive_fusion_enabled = false
```

为什么要改：

- 创新点一是激光-里程计融合模块。
- 如果没有启用 odom，就不应该改变纯激光 Cartographer 行为。
- 这样可以保证 backward compatibility。

专业名称：

- Backward Compatibility
- Pure LiDAR Mode
- Runtime Safety Guard

### 12.8 Lua 配置

文件：

```text
src/cartographer/configuration_files/trajectory_builder_2d.lua
src/my_navigation/config/cartographer_scout_2d.lua
```

改动：

- 默认 Cartographer 2D 配置中加入参数，但默认关闭。
- 项目配置 `cartographer_scout_2d.lua` 中开启创新点一。

为什么要改：

- 默认配置不能影响其他 Cartographer 用法。
- 项目主线需要启用创新点一用于实验。
- 所有参数写进 Lua，便于论文实验调参。

专业名称：

- Lua Configuration
- Experiment Reproducibility
- Parameter Tuning

## 13. 参数怎么调

项目配置入口：

```text
src/my_navigation/config/cartographer_scout_2d.lua
```

参数块标识：

```text
[Innovation 1: Directional Adaptive Fusion Parameters]
```

关键参数说明：

| 参数 | 通俗解释 | 专业含义 |
|---|---|---|
| `directional_adaptive_fusion_enabled` | 是否开启创新点一 | Module Enable Switch |
| `directional_degeneracy_condition_number_threshold` | 多细长才算退化明显 | Condition Number Threshold |
| `directional_degeneracy_sigmoid_slope` | 退化置信度变化快慢 | Sigmoid Slope |
| `directional_degeneracy_smoothing_alpha` | 时间平滑系数 | Low-pass Filter Alpha |
| `directional_degeneracy_min_num_points` | 最少点数保护 | Minimum Valid Points |
| `directional_degeneracy_eigenvalue_epsilon` | 防止除零的小量 | Numerical Epsilon |
| `directional_adaptive_odom_longitudinal_alpha` | 退化方向增强 odom 先验多少 | Longitudinal Prior Gain |
| `directional_adaptive_odom_lateral_alpha` | 横向增强 odom 先验多少 | Lateral Prior Gain |
| `directional_adaptive_scan_longitudinal_beta` | 退化方向相对降低激光影响多少 | Longitudinal LiDAR Downweight |
| `directional_adaptive_scan_lateral_beta` | 横向是否降低激光影响 | Lateral LiDAR Downweight |
| `directional_adaptive_min_scan_weight_scale` | 激光影响最低保留比例 | Minimum Scan Scale |
| `directional_adaptive_log_scale_change_threshold` | 权重变化多大才打印日志 | Log Threshold |

推荐初始值：

```text
threshold = 8.0
slope = 0.5
smoothing_alpha = 0.2
odom_longitudinal_alpha = 1.0
odom_lateral_alpha = 0.0
scan_longitudinal_beta = 0.5
scan_lateral_beta = 0.0
```

调参建议：

- 如果走廊中仍然纵向重影，可以适当增大 `odom_longitudinal_alpha`。
- 如果开阔区被 odom 拉偏，可以降低 `odom_longitudinal_alpha` 或增大 threshold。
- 如果退化指标跳动明显，可以降低 `smoothing_alpha`。
- 如果退化响应太慢，可以增大 `smoothing_alpha`。

## 14. ROS 输出怎么看

新增话题：

```text
/degeneracy_metric
/degeneracy_direction
```

`/degeneracy_metric`：

- 类型：`std_msgs/Float32`
- 含义：整体退化置信度
- 范围：理论上 `[0, 1]`

通俗看法：

- 接近 0：当前 scan 不太像退化场景。
- 接近 1：当前 scan 很可能处于方向性退化场景。

`/degeneracy_direction`：

- 类型：`geometry_msgs/Vector3`
- 含义：退化主方向向量
- z 固定为 0

通俗看法：

- 如果小车在长走廊中，这个方向应该大致沿走廊纵向。
- 如果方向乱跳，说明点云几何不稳定或滤波参数需要调整。

专业名称：

- Degeneracy Metric
- Principal Degeneracy Direction
- Runtime Diagnostics

## 15. 终端日志怎么看

当退化置信度较高，并且权重变化超过配置阈值时，会打印类似：

```text
[Innovation1] Directional Degeneracy detected! longitudinal weight scaled by ...
```

这个日志表示：

- 系统检测到明显方向性退化。
- 退化方向上的 translation prior 权重已经被动态调整。
- lateral 方向是否变化取决于参数。

注意：

- 没有日志不代表模块没运行，可能只是权重变化没超过日志阈值。
- 更可靠的在线观察方式是订阅 `/degeneracy_metric`。

## 16. 性能开销

每帧新增计算主要包括：

1. 遍历点云计算均值。
2. 再遍历点云计算 2x2 协方差。
3. 对 2x2 矩阵做特征值分解。

复杂度：

```text
O(N)
```

其中 `N` 是当前帧过滤后的点数。

2x2 特征值分解非常轻量，主要开销只是两次点云遍历。相比 Ceres 优化和栅格插值，这部分开销很小。

专业名称：

- Computational Complexity
- Eigen Decomposition
- Real-time Front-end Constraint

## 17. 和 IMU 的关系

本创新点没有加入任何 IMU 逻辑。

代码只使用：

- 当前 2D scan 点云
- Cartographer 已有的 odom / motion prior 路径
- Ceres scan matcher

没有使用：

- IMU acceleration
- IMU angular velocity
- Gravity alignment from IMU

这符合项目约束：

```text
仅 2D 激光 + 轮式里程计，无 IMU
```

## 18. 验证结果

已完成的最小验证：

```bash
source /opt/ros/noetic/setup.bash
catkin_make_isolated --install --use-ninja --pkg cartographer cartographer_ros my_navigation
```

结果：

```text
通过
```

运行 Cartographer 2D Ceres scan matcher 单元测试：

```bash
./build_isolated/cartographer/install/cartographer.mapping.internal.2d.scan_matching.ceres_scan_matcher_2d_test
```

结果：

```text
4 tests passed
```

检查主 launch 节点：

```bash
source /opt/ros/noetic/setup.bash
source install_isolated/setup.bash
roslaunch my_navigation scout_mini_mapping_local_grid.launch --nodes
```

输出中没有：

```text
move_base
DWA
TEB
Frontier
explorer_controller
```

说明创新点一没有引入自动导航、路径规划或探索模块。

## 19. 后续实验应该怎么做

建议按三组实验比较：

### Baseline A：低静态权重

特点：

- 原始 Cartographer
- `use_odometry=true`
- translation weight 较低

预期：

- 长走廊纵向容易重影。

### Baseline B：高静态权重

特点：

- 原始 Cartographer
- `use_odometry=true`
- translation weight 统一调高

预期：

- 长走廊可能变好。
- 但开阔区或打滑区可能被 odom 拉偏。

### Proposed：创新点一

特点：

- 启用方向性退化检测。
- 使用 2x2 各向异性 translation prior。
- 退化方向增强 odom/motion prior。
- 非退化方向保留 LiDAR scan matching 优势。

预期：

- 长走廊纵向重影减少。
- 横向墙面仍然清晰。
- 开阔区不会长期保持高 odom 权重。
- `/degeneracy_direction` 应大致指向走廊纵向。

## 20. 一句话总结

本创新点把 Cartographer 2D 原本“一个标量管所有方向”的平移权重，升级为“根据当前 scan 几何退化方向实时构造的 2x2 各向异性权重矩阵”。这样系统能在长走廊纵向更相信 odom，在横向继续相信 LiDAR，从而更符合 2D 激光 SLAM 在退化环境中的真实观测能力。
