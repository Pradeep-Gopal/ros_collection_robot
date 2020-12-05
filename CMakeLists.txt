cmake_minimum_required(VERSION 3.0.2)
project(ros_collection_robot)

add_compile_options(-std=c++11)

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

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/include
)


add_executable(navigator
        src/navigator.cpp
        src/line.cpp
        src/decoder.cpp
        src/manipulator.cpp
        src/polygon.cpp
        src/map.cpp
        src/order_manager.cpp
        )
target_link_libraries(navigator ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(
            main_test
            test/main_test.launch
            test/order_manager_test.cpp
            test/line_test.cpp
            test/main.cpp
            src/order_manager.cpp
            src/line.cpp
    )
    target_link_libraries(
            main_test
            ${catkin_LIBRARIES}
    )
    add_dependencies(
            main_test
            ${catkin_EXPORTED_TARGETS}
    )
endif()
