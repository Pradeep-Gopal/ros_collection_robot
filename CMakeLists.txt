cmake_minimum_required(VERSION 3.0.2)
project(ros_collection_robot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_core
  moveit_ros_perception
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  gazebo_ros
)

add_executable(ros_collection_robot src/main.cpp src/order_manager.cpp)

include_directories(
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(ros_collection_robot ${catkin_LIBRARIES})
