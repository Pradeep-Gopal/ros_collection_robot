#include<cstdlib>
#include<ctime>
#include<fstream>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SpawnModelRequest.h>
#include <gazebo_msgs/SpawnModelResponse.h>
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
    std::vector<geometry_msgs::Point> random_locations;
    geometry_msgs::Point random_point;
    random_point.x = 1;
    random_point.y = 2;
    random_point.z = 0;
    random_locations.push_back(random_point);
    ROS_INFO_STREAM(random_point);
    ros::ServiceClient spawn_object = nh_.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel spawn;
    spawn.request.model_name="box";

    std::string path = ros::package::getPath("ros_collection_robot");
    std::string model_path = path + "/models/cube_A/model.sdf";
    std::ifstream ifs(model_path);

    std::string xml = "";
    for(std::string line; std::getline(ifs,line); )
    {
        xml += line;
    }

    spawn.request.model_xml = xml;
    geometry_msgs::Pose pose;

    pose.position.x=random_point.x;
    pose.position.y=random_point.y;
    pose.position.z=random_point.z;
    pose.orientation.x=0;
    pose.orientation.y=0;
    pose.orientation.z=0;
    pose.orientation.w=1;

    spawn.request.initial_pose = pose;

    if (!spawn_object.call(spawn)) {
        ROS_INFO_STREAM("Failed to call service %s");
    }
    ROS_INFO_STREAM(spawn.response.status_message);
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
