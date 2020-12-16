/**
 * @file polygon.h
 * @author Pradeep Gopal, Justin Albrecht Govind Ajith Kumar
 * @copyright MIT License
 * @brief Header for the Polygon Class
 * This is the header file for the polygon class.
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
#include "../include/line.h"

/**
 * @brief      This class describes a polygon 
 * using a set of lines
 */
class Polygon{
 private:
    std::vector<geometry_msgs::Point> vertices_;
    int n_;
    geometry_msgs::Point centroid_;
    std::vector<Line> lines_;

 public:
	/**
	 * @brief      User defined constructor
	 *
	 * @param[in]  vertices  The vertices of the polygon
	 */
    std::vector<geometry_msgs::Point> getVertices();
	/**
	 * @brief      Gets the vertices.
	 *
	 * @return     The vertices.
	 */
    explicit Polygon(std::vector<geometry_msgs::Point>);
	/**
	 * @brief      Calculates the centroid.
	 */
    void calculateCentroid();
	/**
	 * @brief      Checks if a coordinate is inside the polygon
	 *
	 * @param[in]  coord  The coordinate
	 *
	 * @return     Boolean of the presence (true or false)
	 */
    bool insideObject(geometry_msgs::Point);
	/**
	 * @brief      Gets the centroid.
	 *
	 * @return     The centroid.
	 */
    geometry_msgs::Point getCentroid();
};
