#include "../include/navigator.h"

Navigator::Navigator()
    : decoder(nh_){
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

    cubes_ = {'A','B','C','D','E','F','G','H'};

    std::string order_str;
    nh_.getParam("order",order_str);

    for (char const &c: order_str){
        order_.push_back(c);
    }
}

void Navigator::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    geometry_msgs::Point point;
    for (int i = 0; i < msg->ranges.size(); ++i) {
        if (msg->ranges[i] < 0.5 || msg->ranges[i] > 2)
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

    int cone_angle = 20;
    std::vector<double> front_cone;
    std::vector<double> b;

    b = std::vector<double>(msg->ranges.begin(),
                            msg->ranges.begin()+cone_angle);
    front_cone = std::vector<double>(msg->ranges.end()-cone_angle,
                                     msg->ranges.end());

    front_cone.insert(front_cone.end(), b.begin(), b.end());

    lidar_min_front_ = *std::min_element(front_cone.begin(),front_cone.end());
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

    double angular_speed;
    if (abs(diff) > 0.5)
        angular_speed = .3;
    else
        angular_speed = 0.1;

    vel.angular.z = angular_speed*direction;

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
    vel.linear.x = 0.2;
    vel_pub_.publish(vel);

    ros::Rate r(30);
    double kp = 0.2;
    double max_speed = 0.3;

    double start_distance = sqrt(pow(goal.x-robot_location_.x,2)+
                           pow(goal.y-robot_location_.y,2));

    while(ros::ok()) {
        ros::spinOnce();
        double distance = sqrt(pow(goal.x-robot_location_.x,2)+
                    pow(goal.y-robot_location_.y,2));

        if (distance > start_distance) {
            stop();
            facePoint(goal);
        }

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

void Navigator::reverse(double time){
    geometry_msgs::Twist vel;
    vel.linear.x = -.1;
    vel.angular.z = 0;
    vel_pub_.publish(vel);
    ros::Time start_time = ros::Time::now();
    while (ros::ok()){
        ros::spinOnce();
        if (ros::Time::now() - start_time >= ros::Duration(time))
            break;
    }
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

int Navigator::navigate() {
    while (ros::ok()) {
        if(determined_pose) {
            if (current_waypoint_ == waypoints.size()){
                stop();
                return 1;
            }

            if (planner.map.insideObstacle(waypoints[current_waypoint_]))
                current_waypoint_++;

            ROS_INFO_STREAM("Driving to point (" << waypoints[current_waypoint_].x << ","
                                                 << waypoints[current_waypoint_].y << ")");

            std::vector <geometry_msgs::Point> path = planner.AStar(robot_location_, waypoints[current_waypoint_]);

            if (path.size() == 0) { // path planning failed
                stop();
                return 0;
            }

            bool reached_waypoint = true;
            for (geometry_msgs::Point pt:path) {
                reached_waypoint = driveToPoint(pt);
                if (cube_detected_) {
                    stop();
                    goToCollectionObject();
                    Cube cube = decoder.getCube(robot_location_);

                    if (cube.id == -1)
                        ROS_WARN_STREAM("Cube could not be decoded");
                    else {
                        ROS_INFO_STREAM("The cube type is " << cubes_[cube.id]);
                        ROS_INFO_STREAM(cube.pose);
                    }
                    ros::Duration(1).sleep();
                    reverse(2);
                    break;
                }
            }

            if (reached_waypoint)
                current_waypoint_++;
        }
        ros::spinOnce();
    }
}

void Navigator::goToCollectionObject(){
    approaching_cube_ = true;
    ROS_INFO_STREAM("Detected collection object");

    // find point approach radius away from cube in direction that is closest and not in map
    double approach_radius = 0.7;
    std::vector<geometry_msgs::Point> approach_candidates;
    std::vector<double> distances;
    std::vector<std::pair<double,double>> directions = {std::pair<double,double>(0,-approach_radius),
                                                        std::pair<double,double>(0,approach_radius),
                                                        std::pair<double,double>(-approach_radius,0),
                                                        std::pair<double,double>(approach_radius,0)};

    geometry_msgs::Point approx_position; //saving current cube position
    approx_position.x = cube_position_.x;
    approx_position.y = cube_position_.y;

    for (std::pair<double,double> dir:directions) {
        geometry_msgs::Point approach;
        approach.x = approx_position.x + dir.first;
        approach.y = approx_position.y + dir.second;

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
    facePoint(approx_position);

    geometry_msgs::Twist vel;
    vel.linear.x = 0.1;
    vel_pub_.publish(vel);

    while (ros::ok()) {
        ros::spinOnce();
        if (lidar_min_front_ <= 0.3)
            break;
    }

    stop();
    ROS_INFO_STREAM("Reached collection object");
    cube_detected_ = false;
    approaching_cube_ = false;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "navigator");

    OrderManager order_manager;
    order_manager.generateOrder();

//    order_manager.spawnCubes();
    Navigator nav;

    for (auto c:nav.order_){
        ROS_INFO_STREAM(c);
    }

    ros::spin();
//    int success = nav.navigate();
//
//    if (success)
//        ROS_INFO_STREAM("The robot finished searching the space");
//    else
//        ROS_INFO_STREAM("The robot was unable to search the space");
}