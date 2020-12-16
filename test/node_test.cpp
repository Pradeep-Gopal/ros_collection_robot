/**
 * @file node_test.cpp
 * @author Pradeep Gopal, Justin Albrecht Govind Ajith Kumar
 * @copyright MIT License
 * @brief Test for node Class
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
#include "../include/node.h"

/**
 * @brief Test case for the node Constructor 
 */
TEST(nodeConstructor, ghf_test) {
    Node node;
    EXPECT_EQ(node.g, 0);
    EXPECT_EQ(node.g, 0);
    EXPECT_EQ(node.g, 0);
}

/**
 * @brief Test case for the operator test Function
 */
TEST(OperatorTest, equals_operator_test) {
    Node node1;
    Node node2;
    node1.position.x = 5;
    node1.position.y = 6;
    node2.position.x = 5.0001;
    node2.position.y = 6.0001;
    EXPECT_EQ(node1 == node2, 1);
}

/**
 * @brief Test case for the operator test Function
 */
TEST(OperatorTest2, equals_operator_test2) {
Node node1;
Node node2;
node1.position.x = 5;
node1.position.y = 6;
node2.position.x = 9.01;
node2.position.y = 11.001;
EXPECT_EQ(node1 == node2, 0);
}


