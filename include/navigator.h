#pragma once
#include <vector>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include <sensor_msgs/Image.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/LaserEcho.h"
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "../include/path_planner.h"

class Navigator{

private:
    bool found_object_;
    PathPlanner path_planner_;
    std::vector<geometry_msgs::Point>euler_waypoints_;
    int current_waypoint_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber odom_sub_;
    tf::Transform transform_;
    tf::TransformBroadcaster br_;
    ros::NodeHandle nh_;
public:
    void lidarCallback(sensor_msgs :: LaserScan& );
    void checkCollectionObject(std::vector<double>);
    void followEulerPath();
    void goToCollectionObject();
    void driveToDropOff();
    void returnToEulerPath();
};
