/**
 * @file map_test.cpp
 * @author Pradeep Gopal, Justin Albrecht Govind Ajith Kumar
 * @copyright MIT License
 * @brief Test for map Class
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
#include "../include/map.h"
#include "../include/polygon.h"

/**
 * @brief Test case for the polyRect Function
 */
TEST(polyfromRect, checkVertices) {
    Map map(0.1);
    Polygon poly = map.polyFromRect(0, 0, 1, 1, 0);

    std::vector<geometry_msgs::Point> vertices;
    geometry_msgs::Point pt;
    pt.x = 0;
    pt.y = 0;
    vertices.push_back(pt);
    pt.x = 1;
    pt.y = 0;
    vertices.push_back(pt);
    pt.x = 1;
    pt.y = 1;
    vertices.push_back(pt);
    pt.x = 0;
    pt.y = 1;
    vertices.push_back(pt);
    std::vector<geometry_msgs::Point> poly_vertices = poly.getVertices();
    for (int i = 0; i < vertices.size(); i++) {
        EXPECT_NEAR(poly_vertices[i].x, vertices[i].x, 0.001);
        EXPECT_NEAR(poly_vertices[i].y, vertices[i].y, 0.001);
    }
}

/**
 * @brief Test case for the insideObstacle Function
 */
TEST(insideObstacle, checkInsideObstacle1) {
    Map map(0.1);
    geometry_msgs::Point check_point;
    check_point.x = 1;
    check_point.y = 1;
    check_point.z = 0;
    EXPECT_FALSE(map.insideObstacle(check_point));
}

/**
 * @brief Test case for the insideObstacle Function
 */
TEST(insideObstacle, checkInsideObstacle2) {
    Map map(0.1);
    geometry_msgs::Point check_point;
    check_point.x = 1;
    check_point.y = 0;
    check_point.z = 0;
    EXPECT_TRUE(map.insideObstacle(check_point));
}

/**
 * @brief Test case for the offsetPolygon Function
 */
TEST(offsetPolygon, offsetTest) {
    double clearance = 0.1;
    Map map(clearance);
    Polygon poly = map.polyFromRect(0, 0, 1, 1, 0);
    Polygon offset_polygon = map.offsetPolygon(poly);

    std::vector<geometry_msgs::Point> vertices;
    geometry_msgs::Point pt;
    pt.x = -clearance;
    pt.y = -clearance;
    vertices.push_back(pt);
    pt.x = 1 + clearance;
    pt.y = -clearance;
    vertices.push_back(pt);
    pt.x = 1 + clearance;
    pt.y = 1 + clearance;
    vertices.push_back(pt);
    pt.x = -clearance;
    pt.y = 1 +clearance;
    vertices.push_back(pt);
    std::vector<geometry_msgs::Point> poly_vertices
     = offset_polygon.getVertices();

    for (int i = 0; i < vertices.size(); i++) {
        EXPECT_NEAR(poly_vertices[i].x, vertices[i].x, 2);
        EXPECT_NEAR(poly_vertices[i].y, vertices[i].y, 2);
    }
}

/**
 * @brief Test case for the polyFromCircle Function
 */
TEST(polyFromCircle, checkCirclePOints) {
    double clearance = 0.1;
    Map map(clearance);
    Polygon poly = map.polyFromCircle(2, 2, 1);
    Polygon offset_polygon = map.offsetPolygon(poly);

    std::vector<geometry_msgs::Point> vertices;
    geometry_msgs::Point pt;
    pt.x = 1 + clearance;
    pt.y = 1 + clearance;
    vertices.push_back(pt);
    std::vector<geometry_msgs::Point> poly_vertices
     = offset_polygon.getVertices();

    for (int i = 0; i < vertices.size(); i++) {
        EXPECT_NEAR(poly_vertices[i].x, vertices[i].x, 2);
        EXPECT_NEAR(poly_vertices[i].y, vertices[i].y, 2);
    }
}


