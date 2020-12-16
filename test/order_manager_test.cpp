/**
 * @file order_manager_test.cpp
 * @author Pradeep Gopal, Justin Albrecht Govind Ajith Kumar
 * @copyright MIT License
 * @brief Test for order_manager Class
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
#include "../include/order_manager.h"

/**
 * @brief Test case for the generate Order Function
 */
TEST(generateOrder, orderSizeCheck) {
    OrderManager manager;
    manager.generateOrder();
    EXPECT_EQ(manager.getOrder().size(), manager.getOrderSize());
    EXPECT_EQ(manager.getCubes().size(), manager.getTotalCubes());
}

/**
 * @brief Test case for the get Total Cubes Function
 */
TEST(getTotalCubes, totalCubeTest) {
    OrderManager manager;
    EXPECT_EQ(manager.getTotalCubes(), 8);
}

/**
 * @brief Test case for the get ORder Size Function
 */
TEST(getOrderSize, OrderSizeTest) {
    OrderManager manager;
    EXPECT_EQ(manager.getOrderSize(), 4);
}

/**
 * @brief Test case for the get Order Function
 */
TEST(getOrder, spawnandOrderCheck) {
    OrderManager manager;
    manager.generateOrder();
    manager.spawnCubes();
    std::vector<char> cubes;
    cubes = manager.getCubes();
    EXPECT_EQ(cubes.size(), manager.getTotalCubes());
}

/**
 * @brief Test case for the get Cubes Function
 */
TEST(getCubes, spawnandCubeCheck) {
    OrderManager manager;
    manager.generateOrder();
    manager.spawnCubes();
    std::vector<char> orders;
    orders = manager.getOrder();
    EXPECT_EQ(orders.size(), manager.getOrderSize());
}
