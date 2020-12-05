#pragma once
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "ros/ros.h"

class Controller{
private:
    ros::Publisher cmd_vel_pub;
public:
    void driveToWaypoint(geometry_msgs::Pose);
};
