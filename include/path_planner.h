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
    double clearance_ = 0.3;

public:
    double grid_size;
    int height;
    int width;
    Map map;
    PathPlanner();
    std::vector<Node> checkNeighbors(Node&, Node&);
    std::string generate_node_id(geometry_msgs::Point);
    std::vector<geometry_msgs::Point> AStar(geometry_msgs::Point, geometry_msgs::Point);
};
