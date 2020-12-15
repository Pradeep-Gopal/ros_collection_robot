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

class OrderManager{

private:
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
    OrderManager();
    void collectionCallback(const ros_collection_robot::Cube::ConstPtr&);
	void generateOrder();
	void spawnCubes();
	void deleteCube(std::string);

	// getters
	int getTotalCubes();
	int getOrderSize();
	std::vector<char> getOrder();
	std::vector<char> getCubes();
};
