# lidar_adaptive 实现方案

## 1. 研究定位

本目录是 Cartographer 2D 激光 SLAM 创新与实验的独立子工程。论文主线暂定为：

> 基于 16 线激光置信投影与信息驱动地图更新的 Cartographer 2D SLAM 方法研究

本方案追求硕士论文所需的完整工作量，不以修改 Cartographer 后端优化方程为目标。第一阶段只改动点云预处理和扫描选择接口；如实验结果稳定，再增加一个很小的前端桥接，使“关键帧选择”只影响地图插入，不影响每帧 scan matching。

## 2. 当前代码状态

当前工作区 `HEAD=511df70` 是清理后的 Cartographer 3.4.1 基线。历史的方向性退化融合和里程计异常动态调权代码不在当前源码中，历史提交和实验报告不能视为当前二进制功能。

仓库中已有的基础能力：

- `my_navigation/src/velodyne_deskewed_laserscan.cpp`：包含 ring 筛选、点时间字段读取、里程计插值、逐点去畸变、孤立回波过滤和同角度最近距离选择；目前未在 `my_navigation/CMakeLists.txt` 中编译成运行目标。
- Cartographer `LocalTrajectoryBuilder2D`：已经将用于 scan matching 的点云与用于地图插入的点云分开，并提供按角度保留最近回波的插入过滤选项。
- `my_navigation/local_grid_mapper`：发布车体坐标系局部栅格和 map 坐标系显示栅格，可作为动态/静态双地图实验基础。

本目录新增的当前实现：

1. `confidence_projection_node`：PointCloud2 到 LaserScan 的高度、ring、邻域一致性和同角度置信投影。
2. `adaptive_scan_selector_node`：计算扫描质量和信息量分数，输出关键帧选择诊断；默认转发全部扫描，避免误配置导致 Cartographer 丢帧。
3. `launch/lidar_adaptive_pipeline.launch`：只启动两个研究节点，不自动启动 Cartographer，便于先验证输入输出。
4. `launch/lidar_adaptive_cartographer.launch`：将研究输出接入 Cartographer 2D、占据栅格和局部地图；`start_sim:=true` 时可同时启动 Scout Mini Gazebo 数据源。

节点参数快照分为 `config/projection.yaml` 和 `config/selector.yaml`，避免将带有 `projection`/`selector` 顶层键的组合 YAML 误加载到 ROS 私有参数空间。`config/lidar_adaptive.yaml` 保留为论文和实验总览快照。

## 3. 系统数据流

```text
/velodyne_points
        |
        v
confidence_projection_node
        |  /scan_confidence
        |  /lidar_scan_quality
        |  /lidar_valid_beam_ratio
        v
adaptive_scan_selector_node
        |  /scan_selected
        |  /lidar_information_score
        |  /keyframe_selected
        v
Cartographer 2D (实验 launch 中显式 remap)
        |
        +-- /map
        +-- TF
        +-- /local_occupancy_grid
```

`adaptive_scan_selector_node` 有两个模式：

- `forward_selected_scans=false`：每帧都转发，只发布选择诊断，适合默认运行和定位稳定性验证。
- `forward_selected_scans=true`：只转发被选择的扫描，适合对比“扫描选择对地图和实时性的影响”。该模式暂时属于外部扫描选择实验，不宣称等价于 Cartographer 内部只插入关键帧。

## 4. 创新点一：16 线高度/线号/邻域置信投影

### 4.1 输入字段

节点优先读取 PointCloud2 中的：

- `x, y, z`：几何位置；
- `ring`：激光线号，可选；
- `time`：点级时间，可选，供后续去畸变桥接使用；
- `intensity`：当前版本不把强度作为硬判据，避免受材质和入射角影响。

当仿真点云没有 `ring` 或 `time` 时，节点使用兼容模式：保留全部几何点，并使用帧时间。实车实验应设置 `require_ring=true`，确保传感器字段错误不会静默进入正式结果。

### 4.2 高度筛选

先使用允许高度区间删除明显地面、车体和过高点：

```text
min_height <= z <= max_height
```

在允许区间内再定义稳定障碍高度区间：

```text
stable_min_height <= z <= stable_max_height
```

稳定区间内的点得到较高的高度置信度，其余允许点保留但降低置信度。高度阈值必须通过训练序列确定，并在验证/测试序列冻结。

### 4.3 同角度多回波处理

将水平角划分为固定角度 bin。每个 bin 保存：

- 最近有效距离；
- 高度置信度；
- 相近距离回波数量；
- 是否有邻域支持。

同一角度的多个回波不全部写入二维扫描，优先选择稳定高度区间内的最近返回。多个 ring 在距离容差内同时返回时，增加该 bin 的置信度。

### 4.4 孤立回波过滤

对每个候选 bin，在左右邻域搜索几何上相近的有效点。若邻域端点距离超过阈值，则认为该返回可能是噪声、飞点或动态短时伪点并删除。

邻域端点距离使用二维余弦定理计算：

```text
d^2 = r_i^2 + r_j^2 - 2 r_i r_j cos(delta_angle)
```

### 4.5 质量指标

输出的扫描质量为：

```text
Q = 0.50 * valid_ratio
  + 0.20 * support_ratio
  + 0.30 * mean_height_confidence
```

该分数是工程质量指标，不是位姿协方差，也不是严格的概率观测模型。

## 5. 创新点二：信息驱动扫描选择

### 5.1 信息量定义

当前外部选择节点计算：

```text
I_t = 0.35 * Q_projection
    + 0.25 * Q_valid
    + 0.20 * Q_coverage
    + 0.15 * Q_novelty
    + 0.05 * Q_motion
```

当前版本中 `Q_coverage` 使用有效角度覆盖率近似，`Q_novelty` 使用相邻扫描的距离变化比例近似。后续接入 Cartographer 局部栅格后，可把 `Q_novelty` 替换为“落入未知或状态变化栅格的端点比例”。

### 5.2 选择规则

- 第一帧必选；
- 信息量高于 `selection_score` 且投影质量高于 `min_quality` 时选择；
- 连续超过 `max_skip_seconds` 未选帧时强制选择；
- 低信息量直走廊帧可以跳过，转角和新区域帧优先保留。

### 5.3 重要边界

当前节点的筛选模式会减少送入 Cartographer 的 scan 数量，因此它只能作为“外部扫描选择实验”。正式算法版本应优先实现以下桥接：

```text
每帧 scan 都进入 scan matching
只有 selected=true 的帧进入地图插入和后端节点生成
```

该桥接只应修改 `LocalTrajectoryBuilder2D::InsertIntoSubmap` 附近的插入策略，不应修改 Ceres 残差和 Pose Graph 方程。桥接实现完成前，论文不能声称“关键帧选择不影响定位”。

## 6. 可选创新点三：动态/静态双地图

在 `local_grid_mapper` 基础上维护短期历史栅格：

- 连续多帧在相同位置出现的点标记为静态候选；
- 位置变化快或只出现一次的点标记为动态候选；
- 全局 Cartographer 输入只使用静态候选；
- `base_link` 局部地图保留当前动态障碍物。

该模块需要 Gazebo 行人或真实动态数据。由于多帧一致性依赖位姿估计，动态判断应设置最小运动和历史窗口，并把误删静态点作为失败案例报告。

## 7. 文件职责

```text
src/lidar_adaptive/
├── CMakeLists.txt
├── package.xml
├── IMPLEMENTATION_PLAN.md
├── config/lidar_adaptive.yaml
├── config/projection.yaml
├── config/selector.yaml
├── launch/lidar_adaptive_pipeline.launch
├── launch/lidar_adaptive_cartographer.launch
├── include/lidar_adaptive/       # 后续可复用策略头文件
├── src/
│   ├── confidence_projection_node.cpp
│   └── adaptive_scan_selector_node.cpp
├── scripts/                      # 离线指标和绘图脚本
├── test/                         # 单元测试
├── doc/                          # 实验协议、论文草稿、审计记录
└── experiments/                  # 每次实验独立目录和结果
```

当前已提供 `test/test_launch_structure.py`，检查研究入口不意外拉起导航节点，以及完整入口将 Cartographer 的 `scan` remap 到研究输出。它是结构回归，不代替真实 bag/Gazebo 指标实验。

`experiments/README.md` 提供实验目录模板。任何正式结果都必须复制该模板，填写数据清单、参数快照、源码版本、运行日志、失败原因和指标文件；禁止只保存最终截图。

以后所有创新算法、配置、实验脚本、原始日志、指标、图表、论文草稿和复现实验说明必须放在 `src/lidar_adaptive` 下。Cartographer 核心如需桥接修改，只允许做最小接口变更，并必须在本目录保存：

- 修改原因和文件清单；
- patch 或源码指纹；
- 对应消融实验；
- 回滚方式。

不得把创新配置散落到 `my_navigation`、`cartographer` 或导航包而不在本目录登记。

## 8. 实验协议

### 8.1 基线和消融

```text
C0  标准 pointcloud_to_laserscan + 原始 Cartographer
C1  置信投影 + 全扫描转发
C2  C1 + 外部信息驱动扫描选择
C3  C2 + 动态/静态双地图（可选）
```

横向算法对比至少包含：原始 Cartographer、参数冻结后的 Cartographer、GMapping。Hector、Karto、SLAM Toolbox 只有在 ROS1 接口和输入完全公平时才加入。

### 8.2 数据划分

```text
corridor_repeat_01：参数选择
corridor_repeat_02：验证
corridor_repeat_03：最终测试
Gazebo：连续真值和动态场景验证
```

同一 bag 的不同算法必须使用相同的点云、TF、odom、投影高度范围和回放倍速。每个关键配置至少独立运行 3 次，并报告均值和标准差。

### 8.3 指标

Gazebo 有连续真值时：

- ATE RMSE/P95；
- RPE 平移和旋转；
- 地图 IoU、Chamfer；
- 每帧处理时间、CPU、内存、丢帧数。

真实 bag 没有外部连续真值时：

- 起终点闭合差；
- 与干净参考的 Chamfer；
- 墙体 F1 和连续长度；
- 走廊宽度均值/P90；
- 占据连通分量和地图碎片；
- 关键帧数、scan 数、子图数和运行时间。

真实 bag 的结构指标不得写成 ATE/RPE。人工 odometry 扰动只能称为可控一致性异常，不能称为真实轮胎打滑。

## 9. 验收标准

### 第一阶段：投影节点

- `catkin_make` 或 `catkin_make_isolated` 能构建 `lidar_adaptive`；
- 仿真无 ring/time 点云可以兼容运行；
- 实车有 ring/time 字段时可严格筛选；
- 输出 `/scan_confidence` 频率不低于输入的 95%；
- 质量话题、有效束比例和有效束数量同步发布；
- 输入异常时不发布伪造的有效扫描。

### 第二阶段：选择节点

- 默认 `forward_selected_scans=false` 时不改变 scan 数量；
- `keyframe_selected` 和 `lidar_information_score` 每帧发布；
- 强制最大跳过时间生效；
- 选择比例和处理时间可离线统计；
- 低信息量筛选实验与全扫描基线可重复。

### 第三阶段：Cartographer 桥接

- 不修改 Ceres 和 Pose Graph 优化方程；
- 每帧 scan matching 与关键帧地图插入行为可分别统计；
- 关闭桥接后与标准 Cartographer 行为一致；
- 在 Gazebo 真值和真实 bag 上分别完成消融。

完整研究入口的当前行为是将 `/scan_selected` 作为 Cartographer 输入。因此 `forward_selected_scans=true` 的 C2 仍属于外部抽帧实验。只有完成内部桥接并逐帧统计 scan matching 与地图插入后，才能把 C2 命名为“仅地图更新自适应”。

## 10. 风险与处理

1. 高度阈值依赖安装姿态：先用静态地面标定，测试序列冻结阈值。
2. 置信投影误删障碍物：保留 C0/C1 图像和有效束统计，设置 `publish_empty_on_failure=false`。
3. 外部筛选导致定位丢失：默认全扫描转发，桥接阶段只控制地图插入。
4. 近距离动态点被误认为静态：动态模块作为可选扩展，并使用多帧和最小运动门限。
5. 真实数据没有连续真值：使用 Gazebo 真值支持 ATE/RPE，真实 bag 使用结构代理。
6. 当前历史创新代码不在 HEAD：禁止直接引用历史结果，重新移植后重新编译并保存源码指纹。

## 11. 推荐论文贡献

1. 提出面向 16 线 LiDAR 的高度、线号和邻域一致性置信投影方法。
2. 提出基于扫描质量和信息量的自适应关键帧/地图更新策略。
3. 构建 Scout Mini + Gazebo + Cartographer 2D 实验系统，并完成多数据集、多算法和消融验证。

动态/静态双地图作为第四章扩展，不要求作为主要创新点。强度直方图、固定搜索窗口和简单连续三帧闭环不单独作为核心创新。

## 12. 实施顺序和论文边界

1. 先冻结 C0 基线，记录原始 `/scan` 频率、地图、轨迹和处理时间。
2. 运行 C1，使用 `projection.yaml` 的训练序列确定高度与邻域阈值，再在验证/测试序列冻结。
3. 运行 C2 外部抽帧实验，扫描 `selection_score` 和 `max_skip_seconds`，同时记录输入/输出扫描数和定位失败。
4. 只有 C1/C2 结果稳定后，才考虑 Cartographer 内部桥接。桥接候选位置、修改清单、patch、回滚和消融必须先记录在 `doc/cartographer_bridge_design.md`。
5. 论文中将 C1 表述为“16 线 LiDAR 置信投影”，将当前 C2 表述为“外部信息驱动扫描选择”；完成内部桥接前，不使用“仅地图插入关键帧”或“选择不影响定位”的结论。
