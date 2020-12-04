#include "ros/ros.h"
#include "../include/order_manager.h"


int main(int argc, char **argv){
    ros::init(argc, argv, "order_manager");
    OrderManager manager;
    manager.generateOrder();
}