cmake_minimum_required(VERSION 3.5)
project(search_explorer)

# ============== 关键：强制 C++14 ==============
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 删除任何 set(CMAKE_CXX_STANDARD 17) 或 add_compile_options(-std=c++17)
# =============================================

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    tf2_sensor_msgs
    angles
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        roscpp
        nav_msgs
        geometry_msgs
        sensor_msgs
        visualization_msgs
        tf2
        tf2_ros
        tf2_sensor_msgs
        angles
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(explorer_controller
    src/explorer_controller.cpp
)
target_link_libraries(explorer_controller ${catkin_LIBRARIES})
