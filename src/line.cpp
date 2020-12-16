/**
 * @file line.cpp
 * @author Pradeep Gopal, Justin Albrecht Govind Ajith Kumar
 * @copyright MIT License
 * @brief Source File for the Line Class
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

#include "../include/line.h"

/**
 * @brief      User defined constructor of the line class
 *
 * @param[in]  point_1     The first point
 * @param[in]  point_2     The second point
 * @param[in]  test_point  The point whose position with 
 * respect to the line is to be determined
 */
Line::Line(geometry_msgs::Point point_1,
           geometry_msgs::Point point_2, geometry_msgs::Point test_point) {
    point_1_ = point_1;
    point_2_ = point_2;
    test_point_ = test_point;
    calculateCoefficients();
}

/**
 * @brief      Calculates line coefficients
 */
void Line::calculateCoefficients() {
    a = point_1_.y - point_2_.y;
    b = point_2_.x - point_1_.x;
    c = (point_1_.x*point_2_.y) - (point_2_.x *point_1_.y);

    if (((a*test_point_.x) + (b*test_point_.y) + c) < 0) {
        a *= -1;
        b *= -1;
        c *= -1;
    }
}
