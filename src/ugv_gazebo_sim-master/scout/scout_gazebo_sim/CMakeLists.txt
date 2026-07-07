cmake_minimum_required(VERSION 2.8.3)
project(scout_gazebo_sim)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS    
    roslaunch 
    roslint 
    rospy
    roscpp 
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2 
    tf2_ros
)
catkin_package(
  INCLUDE_DIRS include
#   LIBRARIES 
  CATKIN_DEPENDS roscpp sensor_msgs
#   DEPENDS Boost
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(scout_gazebo STATIC src/scout_skid_steer.cpp)
target_link_libraries(scout_gazebo ${catkin_LIBRARIES})

add_executable(scout_skid_steer_controller src/scout_skid_steer_controller.cpp)
target_link_libraries(scout_skid_steer_controller scout_gazebo ${catkin_LIBRARIES})

catkin_install_python(
  PROGRAMS
    scripts/pointcloud_to_pointcloud2.py
    scripts/moving_obstacle_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Install ##
#############

roslaunch_add_file_check(launch)

install(
  DIRECTORY config launch urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
