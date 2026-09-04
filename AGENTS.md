## Cartographer 2D 与 lidar_adaptive 项目要求

### 1. 项目目标

当前项目基于 ROS Noetic、Gazebo、Scout Mini、16 线 LiDAR 和 Cartographer 2D，完成：

1. 实时二维建图与定位；
2. 以 `base_link` 为中心的实时局部占据栅格地图；
3. 面向 16 线 LiDAR 的置信投影和信息驱动地图更新研究。

当前研究主线不是自动路径规划、Frontier 探索、目标搜索或 move_base 调参。导航包可以保留，但建图创新实验不得依赖导航节点发布速度。

### 2. 主数据流

标准基线：

```text
/velodyne_points_raw
  -> pointcloud_to_pointcloud2.py
/velodyne_points
  -> pointcloud_to_laserscan
/scan
  -> Cartographer 2D
/map + map/odom/base_link TF
```

研究链路：

```text
/velodyne_points
  -> lidar_adaptive/confidence_projection_node
/scan_confidence + quality topics
  -> lidar_adaptive/adaptive_scan_selector_node
/scan_selected (只在显式实验模式使用)
  -> Cartographer 2D
```

`local_grid_mapper` 继续订阅 `/scan` 并发布 `/local_occupancy_grid`。研究链路必须通过 launch 参数显式切换，不能覆盖标准基线。

### 3. 强制目录归属

`src/lidar_adaptive` 是所有创新研究内容的唯一归档目录。以后新增的以下内容必须放在该目录下面：

- 创新算法源码；
- 头文件和可复用策略库；
- ROS 节点、消息、配置和 launch；
- 离线分析、绘图和指标脚本；
- 单元测试和回归测试；
- 实验 bag 清单、日志、地图、轨迹和指标；
- 论文方法说明、实验协议、失败案例和结果审计。

推荐结构：

```text
src/lidar_adaptive/
├── CMakeLists.txt
├── package.xml
├── IMPLEMENTATION_PLAN.md
├── config/
├── launch/
├── include/lidar_adaptive/
├── src/
├── scripts/
├── test/
├── doc/
└── experiments/<date>_<dataset>_<case>/
```

不允许把新的创新参数、脚本或实验结果零散放入 `my_navigation`、`cartographer`、`cartographer_ros` 或导航包而不在 `src/lidar_adaptive` 登记。

如果确实需要修改 Cartographer 核心，只允许做最小桥接修改，并且必须在 `src/lidar_adaptive/doc` 中保存：修改文件清单、原因、patch 或源码指纹、回滚方法和对应消融实验。不得直接重写 Ceres 或 Pose Graph 而不经过方案审查。

### 4. 当前创新范围

#### 4.1 16 线 LiDAR 置信投影

使用高度、ring、邻域支持和同角度最近回波生成可靠的 `/scan_confidence`。仿真点云缺少 ring/time 时允许兼容模式；实车正式实验应设置 `require_ring=true` 并检查字段类型。

强度只作为可选诊断量，不能把简单强度直方图写成核心创新。强度受距离、材质和入射角影响，必须经过序列划分和地点级验证。

#### 4.2 信息驱动扫描选择

信息量由投影质量、有效束比例、角度覆盖、新颖度和运动量组成。默认模式必须转发全部 scan，仅发布选择诊断。只有在实验明确设置 `forward_selected_scans=true` 时才允许减少送入 Cartographer 的 scan。

后续正式桥接应优先做到：每帧继续 scan matching，仅在选中帧进行地图插入和后端节点生成。桥接完成前，不得声称“关键帧选择不影响定位”。

#### 4.3 动态/静态双地图（可选）

可基于 `local_grid_mapper` 增加多帧静态一致性判断：全局图使用静态候选，局部图保留动态障碍物。没有动态场景数据时不得报告动态过滤收益。

### 5. 当前源码事实

- 当前 `HEAD=511df70` 是清理后的 Cartographer 3.4.1 基线；历史提交 `d26f18c`、`0440183` 中的方向性退化和里程计异常模块不属于当前源码功能。
- `src/my_navigation/src/velodyne_deskewed_laserscan.cpp` 是可复用基础文件，但当前 `my_navigation/CMakeLists.txt` 未将其编译为运行目标；标准 launch 仍使用 `pointcloud_to_laserscan`。
- 当前 Cartographer 已支持匹配点云和地图插入点云分离，以及按角度保留最近回波的地图插入过滤。相关核心修改必须以基线行为回归为前提。

### 6. 实验要求

#### 6.1 对比组

至少包含：

1. 标准 Cartographer；
2. 参数冻结后的 Cartographer；
3. 置信投影；
4. 置信投影 + 信息驱动选择；
5. 可选动态/静态双地图；
6. GMapping 作为 ROS1 横向基线。

Hector、Karto 和 SLAM Toolbox 只有在 ROS1 接口、TF、输入话题和参数公平时才加入。

#### 6.2 数据划分

```text
corridor_repeat_01：参数选择
corridor_repeat_02：验证
corridor_repeat_03：最终测试
Gazebo：连续真值、动态障碍和可控扰动
```

同一对比组必须固定输入 bag、点云投影、TF、odom、回放倍速和参数。每个主要配置至少运行 3 次，报告均值、标准差、失败数量和失败原因。

#### 6.3 指标口径

Gazebo 有连续真值时可以报告 ATE、RPE、地图 IoU、Chamfer 和实时性。真实 bag 没有外部连续真值时只能报告闭合差、墙体 F1、Chamfer 到干净参考、走廊宽度、连通分量和运行时延，不能把这些指标写成 ATE/RPE。

任何人工 odometry 扰动只能称为“可控一致性异常注入”，不能称为真实轮胎打滑。

### 7. 运行与验收

标准建图入口：

```bash
source /opt/ros/noetic/setup.bash
source install_isolated/setup.bash
roslaunch my_navigation scout_mini_mapping_local_grid.launch gui:=false rviz:=true
```

研究节点入口：

```bash
roslaunch lidar_adaptive lidar_adaptive_pipeline.launch
```

标准 launch 不得出现 `move_base`、`explorer_controller`、DWA、TEB 或 Frontier 节点。运行时应检查 `/scan`、`/map`、`/local_occupancy_grid` 和完整 TF 链。

每次修改后至少执行：

```bash
catkin_make_isolated --pkg lidar_adaptive --install --use-ninja -j2
roslaunch lidar_adaptive lidar_adaptive_pipeline.launch --nodes
git diff --check
```

若工作区已经确认可用普通 catkin 构建，也可以追加 `catkin_make --pkg lidar_adaptive`；当前工作区包含 plain-cmake/非标准包，隔离构建是可靠的最低验证命令。

涉及 Cartographer 或 my_navigation 的修改，还必须执行对应包的隔离构建和 launch `--nodes` 检查。

### 8. 修改纪律

1. 修改前先执行 `git status --short`，不得回退用户已有改动。
2. 删除文件、大范围重构或恢复历史版本前，先列清单并获得明确确认。
3. 不把成功截图当作精度证据，不隐瞒失败运行。
4. 新算法必须先有 baseline、消融和失败案例，再讨论收益。
5. 参数选择、验证和最终测试序列必须分离。
6. 所有实验结果、配置快照和源码指纹写入 `src/lidar_adaptive/experiments`。
7. 没有外部真值时，明确区分工程验证、机制验证、结构代理和绝对精度。

### 9. 学术责任

论文主张必须与证据一致。禁止使用“首次”“完全解决”“真实打滑识别”“所有指标最优”等未经证据支持的表述。文献中已有的强度回环、连续一致性、关键帧选择和自适应调度只能作为研究基础，创新点应落在面向当前 16 线 LiDAR/Scout Mini/Cartographer 2D 场景的具体融合设计和可复现实验上。

每次对数据集属性、真值、算法收益或统计结论做判断前，先核对源码、配置、原始数据和实验记录。发现旧结论与当前源码不一致时，必须主动更正并更新 `每日总结.md`。

### 10. 交互流程

需求不明确时，每轮只提出一个澄清问题，达到充分理解后先给出方案、风险和验收标准，等待用户明确授权再修改代码或运行长实验。用户已明确授权时，仍需保持修改范围、验证步骤和结果边界透明。

每次最终回复末尾附带“每日总结”，并将同样内容追加到根目录 `每日总结.md`。
