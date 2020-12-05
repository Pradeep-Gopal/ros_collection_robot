#pragma once
#include <vector>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"

class Line{

private:
    geometry_msgs::Point end_;
    double a;
    double b;
    double c;
public:
    //edited this in the uml diagram as well, constructor cant have a void return type
    Line(geometry_msgs::Point a, geometry_msgs::Point b, geometry_msgs::Point c);
    //changed name from coefficients to getCoefficients, made more sense
    void getCoefficients();
};
