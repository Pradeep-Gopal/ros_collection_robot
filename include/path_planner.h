/**
 * @file path_planner.h
 * @author Pradeep Gopal, Justin Albrecht Govind Ajith Kumar
 * @copyright MIT License
 * @brief Header for the Path planner Class
 * This is the header file for the path_planner class.
 * Path planning using A star algorithm
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
#include <utility>
#include <queue>
#include <unordered_map>
#include <string>
#include "geometry_msgs/Point.h"
#include "../include/map.h"
#include "../include/node.h"
#include "ros/ros.h"

/**
 * @brief      This class describes a path planner.
 */
class PathPlanner{
 private:
    double clearance_ = 0.4;

 public:
    double grid_size;
    int height;
    int width;
    Map map;
	/**
	 * @brief      User defined path planner constructor
	*/
    PathPlanner();
	/**
	 * @brief      Returns list of neighbors for a particular node
	 *  in the geometry_msgs::Point format
	 *
	 * @param      curr_node  The curr node
	 * @param      end_node   The end node
	 *
	 * @return     Returns a vector of geometry_msgs::Point that is 
	 * neigbouring to each node in the map where the robot is present
	 */
    std::vector<Node> checkNeighbors(Node&, Node&);
	/**
	 * @brief      Generates an unique string node ID for 
	 * every single node in the map
	 *
	 * @param[in]  position  The position
	 *
	 * @return     The node ID
	 */
    std::string generate_node_id(geometry_msgs::Point);
	/**
	 * @brief      A-star path planner for the robot
	 *
	 * @param[in]  start  The start node
	 * @param[in]  end    The end node
	 *
	 * @return     A vector of geometry_msgs::Point that the
	 * robot should traverse as per the optimal solution
	 */
    std::vector<geometry_msgs::Point>
    AStar(geometry_msgs::Point, geometry_msgs::Point);
};
