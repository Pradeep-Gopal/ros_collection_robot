#pragma once
#include <vector>
#include<cstdlib>
#include<ctime>
#include<fstream>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SpawnModelRequest.h>
#include <gazebo_msgs/SpawnModelResponse.h>
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "../include/map.h"
#include <ros/package.h>

class OrderManager{

private:
	std::vector<char>cubes_;
	std::vector<char>order_;
	ros::NodeHandle nh_;
	ros::Publisher order_pub_;
	int total_cubes_ = 8;
	int order_size_ = 4;
	double clearance_ = 1;
    Map map_object_;

public:
    OrderManager();
	void generateOrder();
	void spawnCubes();

	// getters
	int getTotalCubes();
	int getOrderSize();
	std::vector<char> getOrder();
	std::vector<char> getCubes();
};
