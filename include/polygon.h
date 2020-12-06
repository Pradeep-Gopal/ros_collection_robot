#pragma once
#include <vector>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "../include/line.h"

class Polygon{

private:
    std::vector<geometry_msgs::Point> vertices_;
    int n_;
    geometry_msgs::Point centroid_;
    std::vector<Line> lines_;
public:
    std::vector<geometry_msgs::Point> getVertices();
    Polygon(std::vector<geometry_msgs::Point>);
    void calculateCentroid();
    bool insideObject(geometry_msgs::Point);
    geometry_msgs::Point getCentroid();
};