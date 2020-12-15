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

    Navigator nav;

    int success = nav.navigate();

    if (success)
        ROS_INFO_STREAM("The robot completed the order");
    else
        ROS_INFO_STREAM("The robot was unable to complete the order");
}