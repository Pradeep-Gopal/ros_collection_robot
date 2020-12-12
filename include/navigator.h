#pragma once
#include <vector>
#include <cmath>
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

class Navigator{

private:
    bool determined_pose;
    bool found_object_;
    PathPlanner path_planner_;
    std::vector<geometry_msgs::Point>euler_waypoints_;
    int current_waypoint_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher vel_pub_;
    geometry_msgs::Point robot_location_;
    double robot_rotation_; // rotation about z in radians (0 - 2pi)
    tf::Transform transform_;
    tf::TransformBroadcaster br_;
    ros::NodeHandle nh_;
    PathPlanner planner;

public:
    std::vector<geometry_msgs::Point> waypoints;
    Navigator();
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr&);
    void odomCallback(const nav_msgs::Odometry::ConstPtr&);
    void facePoint(geometry_msgs::Point);
    void driveToPoint(geometry_msgs::Point);
    void stop();
    void checkCollectionObject(std::vector<double>);
    void followWayPoints(std::string);
    void goToCollectionObject();
    void driveToDropOff();
    void returnToEulerPath();
    ros::NodeHandle getNodeHandle();
};
