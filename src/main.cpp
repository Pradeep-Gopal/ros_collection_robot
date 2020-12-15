#include "../include/path_planner.h"
#include "../include/order_manager.h"
#include "../include/node.h"
#include "../include/decoder.h"
#include "../include/line.h"
#include "../include/polygon.h"
#include "../include/navigator.h"
#include "../include/map.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "navigator");

    OrderManager order_manager;
    order_manager.generateOrder();
    order_manager.spawnCubes();

    Navigator nav;
    int success = nav.navigate();

    if (success)
        ROS_INFO_STREAM("The robot finished searching the space");
    else
        ROS_INFO_STREAM("The robot was unable to search the space");
}