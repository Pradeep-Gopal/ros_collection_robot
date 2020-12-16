/**
 * @file line.h
 * @author Pradeep Gopal, Justin Albrecht Govind Ajith Kumar
 * @copyright MIT License
 * @brief Header for the Line Class
 * This is the header file for the line class.
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

/**
 * @brief      This class describes the half plane equation between 
 * two points
 */
class Line{

private:
    geometry_msgs::Point point_1_;
    geometry_msgs::Point point_2_;
    geometry_msgs::Point test_point_;
public:
    double a;
    double b;
    double c;
    /**
	 * @brief      User defined constructor of the line class
	 *
	 * @param[in]  point_1     The first point
	 * @param[in]  point_2     The second point
	 * @param[in]  test_point  The point whose position with 
	 * respect to the line is to be determined
     */
    Line(geometry_msgs::Point , geometry_msgs::Point , geometry_msgs::Point );
	/**
	 * @brief      Calculates line coefficients
	 */
    void calculateCoefficients();
};
