#pragma once
#include <vector>
#include <utility>
#include <queue>
#include <unordered_map>
#include "geometry_msgs/Point.h"
#include "../include/map.h"
#include "../include/node.h"
#include "ros/ros.h"

class PathPlanner{

private:
    double grid_size;

public:
    Map map_;
    PathPlanner();
    std::vector<Node> checkNeighbors(Node&, Node&);
    std::vector<geometry_msgs::Point> AStar(geometry_msgs::Point, geometry_msgs::Point);
};
