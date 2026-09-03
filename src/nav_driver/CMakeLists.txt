cmake_minimum_required(VERSION 3.0.2)
project(nav_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  move_base_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  dynamic_reconfigure
  std_srvs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    actionlib
    move_base_msgs
    nav_msgs
    geometry_msgs
    visualization_msgs
    tf2
    std_msgs
    tf2_ros
    tf2_geometry_msgs
    dynamic_reconfigure
    std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(navigation_driver src/my_navigation.cpp)
target_link_libraries(navigation_driver ${catkin_LIBRARIES})

add_executable(sim_env src/sim_env.cpp)
target_link_libraries(sim_env ${catkin_LIBRARIES})

add_executable(cmd_vel_arbiter src/cmd_vel_arbiter.cpp)
target_link_libraries(cmd_vel_arbiter ${catkin_LIBRARIES})

install(TARGETS navigation_driver sim_env cmd_vel_arbiter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config include/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
