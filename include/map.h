/**
 * @file map.h
 * @author Pradeep Gopal, Justin Albrecht Govind Ajith Kumar
 * @copyright MIT License
 * @brief Header for the map Class
 */

/**
 *MIT License
 *Copyright (c) 2020 Pradeep Gopal, Justin Albrecht, Govind Ajith Kumar
 *Permission is hereby granted, free of charge, to any person obtaining a copy
 *of this software and associated documentation files (the "Software"), to deal
 *in the Software without restriction, including without limitation the rights
 *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *copies of the Software, and to permit persons to whom the Software is
 *furnished to do so, subject to the following conditions:
 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.
 */

#pragma once
#include <vector>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "ros/ros.h"
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include "../include/polygon.h"
#include "../lib/clipper.hpp"

/**
 * @brief      This class initiates and formalizes the map of the warehouse
 */
class Map{
    private:
        // -- Attributes
        std::vector<Polygon> obstacles_;
        std::vector<Polygon> offset_obstacles_;
        double clearance_;

    public:
        // -- Methods
        
        /**
         * @brief      Constructs a new instance.
         * @param[in]  clearance
         */
        explicit Map(double);

        /**
         * @brief      Reads the YAML file
         * @param[in]  fname path to the YAML file
         */
        void parseYAML(std::string);

        /**
         * @brief      Checks if a point is inside the obstacle or not
         *
         * @param[in]  Point  Geometry_msgs::Point
         *
         * @return     True if point is present inside obstacle space
         */
        bool insideObstacle(geometry_msgs::Point);

        /**
         * @brief      Creates the rectangle shaped obstacles from the Warehouse
         * @return     Polygon
         */
        Polygon polyFromRect(double, double, double, double, double);

        /**
         * @brief      Creates the circular obstacles from the Warehouse
         * @return     Polygon
         */
        Polygon polyFromCircle(double, double, double);

        /**
         * @brief     Offsets the polygon by some distance
         *
         * @param[in]  Polygon
         *
         * @return     Offsetted polygon
         */
        Polygon offsetPolygon(Polygon);

        /**
         * @brief      Adds an obstacle to the existing obstacle space
         *
         * @param[in]  Polygon
         */
        void addObstacle(Polygon);
};

