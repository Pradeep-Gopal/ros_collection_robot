#pragma once
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "ros/ros.h"

class Node{
private:
public:
    Node();
    geometry_msgs::Point position;
    geometry_msgs::Point parent;
    double g; // cost to the position
    double h;// cost to the goal
    double f;// total cost (g+h)
    std::string id;

    void generate_id();
    bool operator==(const Node&); //operator overloading ==
};

struct CompareNodeCosts {
    bool operator()(Node& n1, Node& n2){
        if (n1.f > n2.f)
            return true;
        return false;
    }
};