cmake_minimum_required(VERSION 2.8.3)
project(my_navigation)

# 查找依赖项（仅保留本包需要的）
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# ===== 实时局部栅格地图节点 =====
add_executable(local_grid_mapper
  src/local_grid_mapper.cpp
)
target_link_libraries(local_grid_mapper
  ${catkin_LIBRARIES}
)

# ===== 安装规则（务必给出 RUNTIME DESTINATION）=====
install(TARGETS local_grid_mapper
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
