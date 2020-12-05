#pragma once
#include <vector>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "ros/ros.h"
#include "../include/polygon.h"

class Map{
private:
    std::vector<Polygon> obstacles_;
    std::vector<Polygon> offset_obstacles_;
    double clearance_;
public:
    bool insideObstacle(geometry_msgs::Point);
    Polygon polyFromRect(std::map<std::string,double>);
    Polygon polyFromCircle(std::map<std::string,double>);
    Polygon offsetPolygon(Polygon);
};
