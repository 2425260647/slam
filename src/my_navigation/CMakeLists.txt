cmake_minimum_required(VERSION 2.8.3)
project(my_navigation)

# 查找依赖项（仅保留本包需要的）
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
  visualization_msgs
)

# 如果使用了 PCL 或其他库，自行添加，但要注意是否真的需要
# find_package(PCL REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# ===== 障碍物检测节点 =====
add_executable(obstacle_detector
  src/obstacle_detector.cpp
)
target_link_libraries(obstacle_detector
  ${catkin_LIBRARIES}
)

# ===== A-LOAM 位姿桥接节点 =====
add_executable(aloam_pose_bridge
  src/aloam_pose_bridge.cpp
)
target_link_libraries(aloam_pose_bridge
  ${catkin_LIBRARIES}
)

# ===== 安装规则（务必给出 RUNTIME DESTINATION）=====
install(TARGETS obstacle_detector aloam_pose_bridge
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
