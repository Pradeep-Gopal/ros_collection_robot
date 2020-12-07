#pragma once
#include <vector>
#include <utility>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "../include/map.h"
#include "ros/ros.h"
#include "../include/node.h"

class PathPlanner{

private:
    Map map_;

public:
    std::vector<geometry_msgs::Point> checkNeighbors(Node&);
    std::vector<geometry_msgs::Point> AStar(geometry_msgs::Point,geometry_msgs::Point);
    std::vector<geometry_msgs::Point> EulerPath();
};
