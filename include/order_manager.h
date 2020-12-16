/**
 * @file order_manager.h
 * @author Pradeep Gopal, Justin Albrecht Govind Ajith Kumar
 * @copyright MIT License
 * @brief Header for the Line Class
 * This is the header file for the order_manager class.
 * Takes care of managing orders to be fulfilled by the robot
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
#include<cstdlib>
#include<ctime>
#include<fstream>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SpawnModelRequest.h>
#include <gazebo_msgs/SpawnModelResponse.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/DeleteModelRequest.h>
#include <gazebo_msgs/DeleteModelResponse.h>
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "../include/map.h"
#include <ros/package.h>
#include <ros_collection_robot/Cube.h>

/**
 * @brief      This class takes care of managing the orders 
 * given to the robot
 */
class OrderManager{

private:

	// -- Attributes

	std::vector<char>cubes_;
	std::vector<char>order_;
	ros::NodeHandle nh_;
    ros::Subscriber collection_sub_;
	int total_cubes_ = 8;
	int order_size_ = 4;
	int max_x_ = 1500; // x_max to spawn cubes in mm
	int max_y_ = 1500; // y_max to spawn cubes in mm
	double clearance_ = 1;
    Map map_object_;
    std::vector<std::string> cube_names_;
    std::vector<std::string> delete_cubes_;
    std::vector<geometry_msgs::Point> cube_locations_;

public:

	// -- Methods

	/**
	 * @brief      Constructor to the order manager class
	 */
    OrderManager();
    
    /**
     * @brief      Callback to the collection objects
     * @param 	   Input to the callback
     * @return None
     */
    void collectionCallback(const ros_collection_robot::Cube::ConstPtr&);

    /**
     * @brief      Generates a random order with the cubes spawned inside 
     * the warehouse
     * 
     * @return None
     */
	void generateOrder();

	/**
	 * @brief      Spwans the cubes at random locations inside the map
	 * 
	 * @return None
	 */
	void spawnCubes();

	/**
	 * @brief      Deletes the cubes from the world once its collected 
	 * by the robot
	 *
	 * @param[in]  id Id of the cube
	 */
	void deleteCube(std::string);

	/**
	 * @brief      Gets the total cubes.
	 *
	 * @return     The total cubes.
	 */
	int getTotalCubes();

	/**
	 * @brief      Gets the order size.
	 *
	 * @return     The order size.
	 */
	int getOrderSize();

	/**
	 * @brief      Gets the order.
	 *
	 * @return     The order.
	 */
	std::vector<char> getOrder();

	/**
	 * @brief      Gets the cubes.
	 *
	 * @return     The cubes.
	 */
	std::vector<char> getCubes();
};
