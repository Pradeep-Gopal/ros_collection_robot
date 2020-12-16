/**
 * @file navigator_test.cpp
 * @author Pradeep Gopal, Justin Albrecht Govind Ajith Kumar
 * @copyright MIT License
 * @brief Test for navigator Class
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
#include "../include/navigator.h"

/**
 * @brief Test case for the navigator Constructor
 */
TEST(navigatorConstructor, navigatorConstructorTest) {
    OrderManager order_manager;
    order_manager.generateOrder();
    order_manager.spawnCubes();
    Navigator nav;
    ASSERT_FALSE(nav.cube_detected_);
    ASSERT_FALSE(nav.cube_detected_);
    ASSERT_FALSE(nav.determined_pose);
    EXPECT_EQ(nav.current_waypoint_, 0);
}

/**
 * @brief Test case for the navigate Function
 */
TEST(navigate, navigateTest1) {
    Navigator nav;
    std::vector<geometry_msgs::Point> waypoints_nav;
    waypoints_nav = nav.waypoints;
    EXPECT_EQ(waypoints_nav.size(), 19);
}
