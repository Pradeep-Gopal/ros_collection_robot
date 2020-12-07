#pragma once
#include <vector>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "ros/ros.h"

class Node{
private:
public:
    Node(geometry_msgs::Point);
    geometry_msgs::Point position;
    geometry_msgs::Point parent;
    double cost_so_far; // cost to the position
    double heuristic_cost;// cost to the goal
    double total_cost;// cost_so_far + heuristic_cost

    //operator overloading ==
    bool operator==(const Node&);
};