#include<cstdlib>
#include <ctime>
#include "ros/ros.h"
#include "../include/order_manager.h"

void OrderManager::generateOrder(){
	ROS_INFO_STREAM("Generating Order ...");
	std::vector<char>available_cubes = {'A','B','C','D','E','F','G','H'};
    int random_cube_idx;
    srand(time(NULL));// to generate a truly random number every time
    for(int i = 0; i < total_cubes_; i++){
        random_cube_idx = (rand() % (available_cubes.size()));
        cubes_.push_back(available_cubes[random_cube_idx]);
    }
    for(auto cube:cubes_){
        ROS_INFO_STREAM("CUBE : "<<cube);
    }
    int order_idx;
    std::vector<int>cubes_selected;
    while(order_.size()<4){
        order_idx = (rand() % (cubes_.size()));
        if (std::count(cubes_selected.begin(), cubes_selected.end(), order_idx)){
        }
        else{
            cubes_selected.push_back(order_idx);
            order_.push_back(cubes_[order_idx]);
        }
    }
    for(auto order:order_){
        ROS_INFO_STREAM("ORDER : "<<order);
    }
    ROS_INFO_STREAM("Random Cubes generated ...");
}

void OrderManager::spawnCubes(){
	ROS_INFO_STREAM("spawn_cubes");
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

int main(int argc, char **argv){
    ros::init(argc, argv, "order_manager");
    OrderManager manager;
    manager.generateOrder();
}