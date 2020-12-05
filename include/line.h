#pragma once
#include <vector>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "ros/ros.h"

class Line{

private:
    geometry_msgs::Point point_1_;
    geometry_msgs::Point point_2_;
    geometry_msgs::Point test_point_;
public:
    double a;
    double b;
    double c;
    Line(geometry_msgs::Point , geometry_msgs::Point , geometry_msgs::Point );
    void calculateCoefficients();
};
