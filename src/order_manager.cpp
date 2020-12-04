#include "ros/ros.h"
#include "../include/order_manager.h"

void OrderManager::generateOrder(){
	ROS_INFO_STREAM("generateOrderCalled");
}

void OrderManager::spawnCubes(){
	ROS_INFO_STREAM("spawncubes");	
}

int OrderManager::getTotalCubes(){
	return total_cubes_;
}

int OrderManager::getOrderSize(){
	return order_size_;
}
std::vector<char> OrderManager::getOrder(){
	return order_;
}
std::vector<char> OrderManager::getCubes(){
	return cubes_;
}

