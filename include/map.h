#pragma once
#include <vector>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "ros/ros.h"
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include "../include/polygon.h"
#include "../lib/clipper.hpp"

class Map{
    private:
        std::vector<Polygon> obstacles_;
        std::vector<Polygon> offset_obstacles_;
        double clearance_;

    public:
        Map();
        void parseYAML(std::string);
        bool insideObstacle(geometry_msgs::Point);
        Polygon polyFromRect(double, double, double, double, double);
        Polygon polyFromCircle(double, double, double);
        Polygon offsetPolygon(Polygon);
};

