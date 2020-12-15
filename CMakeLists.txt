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
        tf2
        gazebo_ros
        cv_bridge
        )

#find_package(OpenCV REQUIRED)
find_package(OpenCV 3.2.0 REQUIRED
        CONFIG
        PATHS /usr/local /usr
        NO_DEFAULT_PATH
        )
#message("OpenCV version: ${OpenCV_VERSION}")

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/include
)


add_executable(navigator
        src/main.cpp
        src/navigator.cpp
        src/line.cpp
        src/polygon.cpp
        src/decoder.cpp
        src/manipulator.cpp
        src/map.cpp
        src/order_manager.cpp
        src/controller.cpp
        src/path_planner.cpp
        src/node.cpp

        lib/clipper.cpp
        src/main.cpp)

target_link_libraries(navigator ${catkin_LIBRARIES})
target_link_libraries(navigator yaml-cpp)
target_link_libraries(navigator ${OpenCV_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(
            main_test
            test/main_test.launch

            test/order_manager_test.cpp
            test/line_test.cpp
            test/main.cpp
            test/polygon_test.cpp
            test/map_test.cpp
            test/path_planner_test.cpp
            test/node_test.cpp
            test/decoder_test.cpp

            src/order_manager.cpp
            src/line.cpp
            src/polygon.cpp
            src/map.cpp
            src/path_planner.cpp
            src/node.cpp
            src/decoder.cpp
            src/navigator.cpp

            lib/clipper.cpp
    )
    target_link_libraries(
            main_test
            ${catkin_LIBRARIES}
            ${OpenCV_LIBRARIES}
    )
    target_link_libraries(main_test
            yaml-cpp)
    add_dependencies(
            navigator
            main_test
            ${catkin_EXPORTED_TARGETS}
    )
endif()