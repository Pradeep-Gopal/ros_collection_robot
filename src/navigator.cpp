#include "../include/line.h"
#include "../include/polygon.h"
#include "../include/navigator.h"
#include "../include/order_manager.h"
#include "../include/node.h"
#include "../include/decoder.h"

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
    ros::NodeHandle nh;
    OrderManager order_manager;
    order_manager.generateOrder();
    order_manager.spawnCubes();

    Decoder decoder(nh);
    ros::Rate r(1);

    while(ros::ok()){
        std::vector<int> ids = decoder.detectTags();
//        if (ids.size() == 1) {
//            ROS_INFO_STREAM("Marker id: " << ids[0]);
//        }
        ros::spin();
        r.sleep();
    }

//    PathPlanner planner;
//
//    geometry_msgs::Point start;
//    start.x = 1;
//    start.y = 1;
//
//    geometry_msgs::Point end;
//    end.x = 5;
//    end.y = 4.5;
//
//    if (planner.map_.insideObstacle(end)){
//        ROS_WARN_STREAM("Invalid goal position");
//    } else {
//        std::vector <geometry_msgs::Point> path;
//        path = planner.AStar(start, end);
//
//        ROS_INFO_STREAM("Length of path: " << path.size());
//
//        for (geometry_msgs::Point pt:path) {
//            ROS_INFO_STREAM("(" << pt.x << "," << pt.y << ")");
//        }
//    }
}