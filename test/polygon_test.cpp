/**
 * @file polygon_test.cpp
 * @author Pradeep Gopal, Justin Albrecht Govind Ajith Kumar
 * @copyright MIT License
 * @brief Test for polygon Class
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

#include <gtest/gtest.h>
#include "../include/polygon.h"

/**
 * @brief Test case for the calculate Centroid Function
 */
TEST(calculateCentroid, simpleCentroidTest) {
    std::vector <geometry_msgs::Point> vertices;
    geometry_msgs::Point point1;
    point1.x = 0;
    point1.y = 0;
    geometry_msgs::Point point2;
    point2.x = 2;
    point2.y = 0;
    geometry_msgs::Point point3;
    point3.x = 1;
    point3.y = 2;
    vertices.push_back(point1);
    vertices.push_back(point2);
    vertices.push_back(point3);
    Polygon polygon(vertices);

    EXPECT_NEAR(polygon.getCentroid().x, 1, 0.01);
    EXPECT_NEAR(polygon.getCentroid().y, 0.66, 0.05);
}

/**
 * @brief Test case for the inside Object Function
 */
TEST(insideObject, testPointEnclosure) {
    std::vector <geometry_msgs::Point> vertices;
    geometry_msgs::Point point1;
    point1.x = 0;
    point1.y = 0;
    geometry_msgs::Point point2;
    point2.x = 2;
    point2.y = 0;
    geometry_msgs::Point point3;
    point3.x = 1;
    point3.y = 2;
    vertices.push_back(point1);
    vertices.push_back(point2);
    vertices.push_back(point3);
    Polygon polygon(vertices);

    geometry_msgs::Point check_point;
    check_point.x = 1;
    check_point.y = 0.1;

    EXPECT_TRUE(polygon.insideObject(check_point));

    check_point.x = 3;
    check_point.y = 2;
    EXPECT_FALSE(polygon.insideObject(check_point));
}
