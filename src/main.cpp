/**
 * @file main.cpp
 * @author Pradeep Gopal, Justin Albrecht Govind Ajith Kumar
 * @copyright MIT License
 * @brief main file where the code starts executing
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

#include "../include/path_planner.h"
#include "../include/order_manager.h"
#include "../include/node.h"
#include "../include/decoder.h"
#include "../include/line.h"
#include "../include/polygon.h"
#include "../include/navigator.h"
#include "../include/map.h"

/**
 * @brief      Main function that runs the navigator node
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "navigator");

    OrderManager manager;
    manager.generateOrder();
    manager.spawnCubes();

    Navigator nav;
    bool success = nav.navigate();

    if (success)
        ROS_INFO_STREAM("The robot completed the order");
    else
        ROS_INFO_STREAM("The robot was unable to complete the order");
}
