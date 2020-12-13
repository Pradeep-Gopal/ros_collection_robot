#include "../include/line.h"
#include "../include/polygon.h"
#include "../include/navigator.h"
#include "../include/order_manager.h"
#include "../include/node.h"
#include "../include/decoder.h"

Navigator::Navigator() {
    odom_sub_ = nh_.subscribe("/odom", 10,
                                &Navigator::odomCallback, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
    lidar_sub_ = nh_.subscribe("/scan",10,
                               &Navigator::lidarCallback, this);
    determined_pose = false;
    cube_detected_ = false;
    approaching_cube_ = false;
    current_waypoint_ = 0;
    std::string fname = ros::package::getPath("ros_collection_robot") + "/config/waypoints.yaml";
    parseWaypoints(fname);
}

void Navigator::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    geometry_msgs::Point point;
    for (int i = 0; i < msg->ranges.size(); ++i) {
        if (msg->ranges[i] < 0.5 || msg->ranges[i] > msg->range_max)
            continue;

        double theta = (i*3.14)/180;
        double phi = theta+robot_rotation_;
        if (phi > 2*3.14)
            phi -= 2*3.14;

        point.x = msg->ranges[i]*cos(phi)+robot_location_.x-0.024;
        point.y = msg->ranges[i]*sin(phi)+robot_location_.y;

        if (!planner.map.insideObstacle(point)){
            cube_detected_ = true;
            cube_position_ = point;
            break;
        }
    }
}

void Navigator::odomCallback(const nav_msgs::Odometry::ConstPtr &msg){
    geometry_msgs::Pose robot_pose = msg->pose.pose;

    robot_location_ = robot_pose.position;
    geometry_msgs::Quaternion quat = robot_pose.orientation;

    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    if (yaw < 0)
        yaw += 2*3.14;

    robot_rotation_ = yaw;
    determined_pose = true;
}

void Navigator::facePoint(geometry_msgs::Point goal_pt) {
    // determine heading necessary to face goal
    while (!determined_pose){
        ros::spinOnce();
    }

    double pi = 3.14159;
    double goal_heading = atan2(goal_pt.y - robot_location_.y,
                                goal_pt.x - robot_location_.x);

    if (goal_heading < 0)
        goal_heading += 2 * pi;

    // determine cw and ccw angles necessary to reach heading
    double diff = goal_heading - robot_rotation_;

    int direction = 1; //-1 for cw 1 for ccw
    if (diff < 0)
        direction = -1;

    if (abs(diff) > pi)
        direction *= -1;

    // rotate until heading is correct
    geometry_msgs::Twist vel;
    vel.angular.z = 0.25*direction;

    ros::Rate r(30);
    while(ros::ok()) {
        ros::spinOnce();
        diff = goal_heading - robot_rotation_;
        if (abs(diff)<0.05)
            break;

        vel_pub_.publish(vel);
        r.sleep();
    }

    stop();
}

int Navigator::driveToPoint(geometry_msgs::Point goal) {
    double pi = 3.14159;
    double heading = atan2(goal.y - robot_location_.y,
                           goal.x - robot_location_.x);
    if (heading < 0)
        heading += 2 * pi;

    double diff = heading - robot_rotation_;

    if (abs(diff) > pi)
        diff = 2*pi - abs(diff);

    if (abs(diff) > 0.5){
        stop();
        ros::Duration(0.5).sleep();
        facePoint(goal);
    }

    geometry_msgs::Twist vel;
    vel.linear.x = 0.3;
    vel_pub_.publish(vel);

    ros::Rate r(30);
    double kp = 0.2;
    double max_speed = 0.3;

    while(ros::ok()) {
        ros::spinOnce();
        double distance = sqrt(pow(goal.x-robot_location_.x,2)+
                    pow(goal.y-robot_location_.y,2));

        double heading = atan2(goal.y - robot_location_.y,
                                              goal.x - robot_location_.x);
        if (heading < 0)
            heading += 2 * pi;

        double diff = heading - robot_rotation_;

        int direction = 1; // -1 for cw 1 for ccw
        if (diff < 0)
            direction = -1;

        if (abs(diff) > pi) {
            direction *= -1;
            diff = 2*pi - abs(diff);
        }

        double speed = abs(diff) * kp;
        if (speed > max_speed)
            speed = max_speed;

        vel.angular.z = direction*speed;

        vel_pub_.publish(vel);

        if (distance<0.1)
            return 1;

        if (cube_detected_ && !approaching_cube_)
            return 0;

        r.sleep();
    }
}

void Navigator::stop(){
    geometry_msgs::Twist vel;
    vel.linear.x = 0;
    vel.angular.z = 0;
    vel_pub_.publish(vel);
}

void Navigator::parseWaypoints(std::string fname) {
    YAML::Node data = YAML::LoadFile(fname);
    YAML::Node waypoint_data = data["waypoints"];
    geometry_msgs::Point point;
    for (int i = 0; i < waypoint_data.size(); ++i) {
        point.x = waypoint_data[i][0].as<double>();
        point.y = waypoint_data[i][1].as<double>();

        waypoints.push_back(point);
    }
}

void Navigator::navigate() {
    while (ros::ok()) {
        ROS_INFO_STREAM("Driving to point (" << waypoints[current_waypoint_].x << ","
            << waypoints[current_waypoint_].y << ")");
        bool success = driveToPoint(waypoints[current_waypoint_]);

        if (success) {
            current_waypoint_++;
        } else if (cube_detected_) {
            stop();
            ROS_INFO_STREAM("Detected collection object");
            goToCollectionObject();
            ROS_INFO_STREAM("Reached collection object");
            break;
        }
    }
}

void Navigator::goToCollectionObject(){
    approaching_cube_ = true;

    // find point 0.5 m away from cube in direction that is closest and not in map
    double approach_radius = 0.5;
    std::vector<geometry_msgs::Point> approach_candidates;
    std::vector<double> distances;
    std::vector<std::pair<double,double>> directions = {std::pair<double,double>(0,-approach_radius),
                                                        std::pair<double,double>(0,approach_radius),
                                                        std::pair<double,double>(-approach_radius,0),
                                                        std::pair<double,double>(approach_radius,0)};
    for (std::pair<double,double> dir:directions) {
        geometry_msgs::Point approach;
        approach.x = cube_position_.x + dir.first;
        approach.y = cube_position_.y + dir.second;

        if (!planner.map.insideObstacle(approach)) {
            approach_candidates.push_back(approach);
            distances.push_back(sqrt(pow(approach.x - robot_location_.x, 2) + pow(approach.y - robot_location_.y, 2)));
        }
    }

    auto min_dist = std::min_element(distances.begin(), distances.end());
    geometry_msgs::Point approach_point = approach_candidates[std::distance(distances.begin(), min_dist)];

    // insert cube into map as a circle at approximate position
    Polygon new_poly = planner.map.polyFromCircle(cube_position_.x,cube_position_.y,0.05);
    planner.map.addObstacle(new_poly);

    // generate path to approach point
    std::vector<geometry_msgs::Point> path = planner.AStar(robot_location_,approach_point);

    for (geometry_msgs::Point pt:path) {
        driveToPoint(pt);
    }

    stop();
    facePoint(cube_position_);
}

void Navigator::driveToDropOff(){

}

void Navigator::returnToEulerPath(){

}

ros::NodeHandle Navigator::getNodeHandle(){
    return nh_;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "navigator");

//    OrderManager order_manager;
//    order_manager.generateOrder();
//    order_manager.spawnCubes();

    Navigator nav;
    nav.navigate();
}