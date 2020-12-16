/**
 * @file navigator.h
 * @author Pradeep Gopal, Justin Albrecht Govind Ajith Kumar
 * @copyright MIT License
 * @brief Header for the Navigator Class
 * This is the header file for the Navigator class.
 * Sends navigation commands to the robot
 */

/**
 *MIT License
 *Copyright (c) 2020 Pradeep Gopal, Justin Albrecht, Govind Ajith Kumar
 *Permission is hereby granted, free of charge, to any person obtaining a copy
 *of this software and associated documentation files (the "Software"), to deal
 *in the Software without restriction, including without limitation the rights
 *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *copies of the Software, and to permit persons to whom the Software is
 *furnished to do so, subject to the following conditions:
 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.
 */

#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <sensor_msgs/Image.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/LaserEcho.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "../include/path_planner.h"
#include "../include/order_manager.h"
#include "../include/node.h"
#include "../include/decoder.h"
#include "../include/line.h"
#include "../include/polygon.h"
#include <ros_collection_robot/Cube.h>

/**
 * @brief      This class helps the robot navigate inside the warehouse world
 */
class Navigator{

private:

    // -- Attributes

    bool found_object_;
    int object_detector_count;
    double lidar_min_front_;
    PathPlanner path_planner_;
    geometry_msgs::Point cube_position_;
    ros::NodeHandle nh_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher vel_pub_;
    ros::Publisher collector_pub_;
    geometry_msgs::Point robot_location_;
    double robot_rotation_; // rotation about z in radians (0 - 2pi)
    tf::Transform transform_;
    tf::TransformBroadcaster br_;
    std::vector<char> cubes_;
    PathPlanner planner;
    Decoder decoder;

public:

    // -- Attributes

    bool determined_pose;
    bool cube_detected_;
    bool approaching_cube_;
    int current_waypoint_;
    std::vector<char> order_;
    std::vector<geometry_msgs::Point> waypoints;

    // -- Methods

    /**
     * @brief      Constructor for the Navigator class
     */
    Navigator();

    /**
     * @brief      Callback to the /scan topic from LIDAR
     * @param[in]  msg  Vector containing the message from /scan topic
     * @return     None
     */
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr&);

    /**
     * @brief      Callback to the /odom topic
     * @param[in]  msg  Message containing the odometry info of the robot 
     * @return     None
     */
    void odomCallback(const nav_msgs::Odometry::ConstPtr&);

    /**
     * @brief      Parse the waypoint YAML file
     *
     * @param[in]  fname  path to the location of the waypoint yaml file
     * 
     * @return     None
     */
    void parseWaypoints(std::string);

    /**
     * @brief      Rotates the robot to face a waypoint
     *
     * @param[in]  Point  The point to which the robot has to face
     * 
     * @return     None
     */
    void facePoint(geometry_msgs::Point);

    /**
     * @brief      Drives the robot to a waypoint
     *
     * @param[in]  Point  The point to which the robot has to drive to
     *
     * @return     int -1 if fails
     */
    int driveToPoint(geometry_msgs::Point);

    /**
     * @brief      Sets the linear and angular velocity of the robot to zero
     * @return     None
     */
    void stop();

    /**
     * @brief      Reverses the robot
     *
     * @param[in]  distance  Distance to which the robot has to be reversed
     * 
     * @return     None
     */
    void reverse(double);

    /**
     * @brief      check if the detected cube is in the order or not
     *
     * @param[in]  vector containing the cube id  
     * 
     * @return None
     */
    void checkCollectionObject(std::vector<double>);

    /**
     * @brief      Navigates the robot between the predefined waypoints
     *
     * @return     True if success else False
     */
    bool navigate();

    /**
     * @brief      Moves the robot towards an object which needs to be collected
     * 
     * @return     None
     */
    void goToCollectionObject();
};
