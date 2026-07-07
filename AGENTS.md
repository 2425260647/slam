## Codex 专用提示词：Cartographer 2D 建图定位与局部地图系统

### 一、当前项目目标
你是一个资深机器人算法工程师。当前项目主线已经收敛为：

基于 ROS、Gazebo、Scout Mini 和 Cartographer 2D，完成小车的实时建图定位，并额外生成一个以小车为中心的实时局部占据栅格地图。

当前阶段不做自动路径规划、不做 Frontier 探索、不做 move_base/DWA/TEB 导航、不做目标搜索控制。小车是否移动由人工速度指令或后续单独模块控制。

### 二、当前主数据流
1. Gazebo 启动 Scout Mini 和 `clearpath_playpen.world` 场景。
2. 仿真 16 线雷达发布 `/velodyne_points_raw`。
3. `pointcloud_to_pointcloud2.py` 转换为 `/velodyne_points`。
4. `pointcloud_to_laserscan` 将点云投影为 `/scan`。
5. Cartographer 2D 订阅 `/scan`，发布 `/map` 和 `map -> base_link` 位姿。
6. `local_grid_mapper` 订阅 `/scan`，发布以 `base_link` 为中心的 `/local_occupancy_grid`。
7. RViz 显示 `/map`、`/local_occupancy_grid`、`/scan`、TF 和 RobotModel。

### 三、当前保留模块
- `my_navigation`
  - 当前只保留 Cartographer 配置、局部地图节点、仿真启动文件和 RViz 配置。
  - 主要入口：`my_navigation/launch/scout_mini_mapping_local_grid.launch`
  - 主要节点：`my_navigation/src/local_grid_mapper.cpp`
- `cartographer`
  - Cartographer 核心算法库。
- `cartographer_ros`
  - Cartographer ROS 接口。
- `ugv_gazebo_sim-master`
  - 保留整个仿真包目录。
  - 当前主要使用 `scout/scout_gazebo_sim` 和 `scout/scout_control`。
- `scout_ros-master`
  - 当前主要使用 `scout_description` 作为小车模型来源。
- `navigation`
  - 用户要求保留，但当前主线不启动其中的 move_base/DWA/Navfn。
- `lslidar_ros`
  - 用户要求保留，真实雷达驱动后续可能使用。
- `ugv_sdk`
  - 用户要求保留，真实 Scout Mini 底盘通信后续可能使用。
- `function_module`
  - 用户要求保留。
- `pointcloud_to_grid`
  - 保留实验代码，尤其 hector/gmapping 相关历史实验文件，不作为当前主线。

### 四、当前不再使用的内容
- 不再使用 A-LOAM、SC-A-LOAM、LeGO-LOAM、SC-LeGO-LOAM。
- 不再维护三维 LOAM 点云累计地图、OctoMap、`/laser_cloud_surround`、`/aft_mapped` 等链路。
- 不再使用 `search_explorer`。
- 不再启动 Frontier Exploration、move_base、DWA、TEB、Navfn 或 global/local costmap。
- 不再保留旧的 `obstacle_detector` 动态障碍检测链路作为当前主线。

### 五、当前运行方式
推荐启动命令：

```bash
source /opt/ros/noetic/setup.bash
source install_isolated/setup.bash
roslaunch my_navigation scout_mini_mapping_local_grid.launch gui:=false rviz:=true
```

如果要观察局部地图变化，需要另行向小车速度控制话题发布速度：

```bash
rostopic pub /scout_mini_velocity_controller/cmd_vel geometry_msgs/Twist ...
```

### 六、验证标准
- `roslaunch my_navigation scout_mini_mapping_local_grid.launch --nodes` 中不应出现：
  - `move_base`
  - `explorer_controller`
  - DWA/TEB/Frontier 相关节点
- 运行时应能看到：
  - `/scan`
  - `/map`
  - `/local_occupancy_grid`
  - `map -> odom -> base_link` 或等价 TF 链
- RViz 固定坐标系使用 `map`。
- `/local_occupancy_grid` 的 `frame_id` 应为 `base_link`，表示它是跟随小车移动的局部实时地图。

### 七、执行流程与协作规则
1. 所有后续任务必须先给出方案、风险和验证标准，等待用户明确同意后才能实施代码修改、参数修改、launch 修改、删除文件或长时间实验。
2. 用户明确说“开始”“确认执行”“可以改”等才可执行。
3. 修改前先 `git status`，确认已有脏文件，不回退用户改动。
4. 删除或大范围重构前必须列清单并获得用户确认。
5. 每次修改后尽量运行 `catkin_make` 或 `roslaunch --nodes` 做最小验证。
6. 每次回答末尾附带“每日总结”，并写入 `每日总结.md`。
