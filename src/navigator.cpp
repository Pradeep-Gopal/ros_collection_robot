#include "../include/line.h"
#include "../include/polygon.h"
#include "../include/navigator.h"
#include "../include/order_manager.h"

void Navigator::lidarCallback(sensor_msgs :: LaserScan &msg){

}

void Navigator::checkCollectionObject(std::vector<double> check_object){

}

void Navigator::followEulerPath(){

}

void Navigator::goToCollectionObject(){

}

void Navigator::driveToDropOff(){

}

void Navigator::returnToEulerPath(){

}

int main(int argc, char **argv){
    ros::init(argc, argv, "navigator");
    OrderManager order_manager;
    order_manager.spawnCubes();
}