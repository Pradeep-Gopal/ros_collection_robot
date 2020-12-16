/**
 * @file node.h
 * @author Pradeep Gopal, Justin Albrecht Govind Ajith Kumar
 * @copyright MIT License
 * @brief Header for the Line Class
 * This is the header file for the node class.
 * Used to define the structure for nodes used in A*
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
#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "ros/ros.h"

/**
 * @brief      This class provides template for all nodes
 */
class Node{
private:
public:
    
    // -- Attributes
    geometry_msgs::Point position;
    geometry_msgs::Point parent;
    double g; // cost to the position
    double h;// cost to the goal
    double f;// total cost (g+h)
    std::string id;

    // -- Methods

    /**
     * @brief      Constructor for the Node class
     */
    Node();

    /**
     * @brief      Equality operator.

     * @return     The result of the equality
     */
    bool operator==(const Node&); //operator overloading ==
};

/**
 * @brief      Struct used for puhsing nodes in the priority queue based on cost.
 */
struct CompareNodeCosts {
    bool operator()(Node& n1, Node& n2){
        if (n1.f > n2.f)
            return true;
        return false;
    }
};