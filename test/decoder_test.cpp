/**
 * @file decoder_test.cpp
 * @author Pradeep Gopal, Justin Albrecht Govind Ajith Kumar
 * @copyright MIT License
 * @brief Test for decoder Class
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
#include "../include/decoder.h"
#include "../include/navigator.h"

/**
 * @brief Test case for the decoder Function
 */
TEST(decoder, testDecoder) {
    ros::NodeHandle nh;
    Decoder decoder(nh);
    Navigator nav;
    EXPECT_FALSE(decoder.determined_camera_params);
}

/**
 * @brief Test case for the getCube Function
 */
TEST(getCube, testGetCube) {
    geometry_msgs::Point location;
    location.x = 1;
    location.y = 1;
    location.z = 0;
    ros::NodeHandle nh;
    Decoder decoder(nh);
    EXPECT_EQ(decoder.getCube(location).id, -1);
}


