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
    determined_pose = false;
}

void Navigator::lidarCallback(sensor_msgs::LaserScan &msg){

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
    vel.angular.z = .25*direction;
    vel_pub_.publish(vel);
    ROS_INFO_STREAM("Turning to face goal.");

    ros::Rate r(30);
    while(ros::ok()){
        ros::spinOnce();
        diff = goal_heading - robot_rotation_;
        if (abs(diff)<0.05) {
            ROS_INFO_STREAM("Finished Turning.");
            break;
        }
        r.sleep();
    }

    // stop
    vel.angular.z = 0;
    vel.linear.x = 0;
    vel_pub_.publish(vel);
}

void Navigator::driveToPoint(geometry_msgs::Point goal) {
    facePoint(goal);

    double pi = 3.14159;

    ros::Duration(0.5).sleep();

    geometry_msgs::Twist vel;
    vel.linear.x = .2;
    vel_pub_.publish(vel);
    ROS_INFO_STREAM("Driving to Goal.");

    ros::Rate r(30);
    double distance;
    double kp = .4;

    while(ros::ok()){
        ros::spinOnce();
        distance = sqrt(pow(goal.x-robot_location_.x,2)+
                    pow(goal.y-robot_location_.y,2));

        double heading = atan2(goal.y - robot_location_.y,
                                              goal.x - robot_location_.x);
        if (heading < 0)
            heading += 2 * pi;

        double diff = heading - robot_rotation_;

        if (diff > 0)
            vel.angular.z = -diff*kp;
        else
            vel.angular.z = diff*kp;

        if (abs(diff) > 0.5) {
            ROS_WARN_STREAM("Robot veered too far off course");
            break;
        }

        vel_pub_.publish(vel);

        if (distance<0.05) {
            ROS_INFO_STREAM("Reached goal");
            break;
        }
        r.sleep();
    }

    vel.linear.x = 0;
    vel.angular.z = 0;
    vel_pub_.publish(vel);
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
    Navigator nav;

    geometry_msgs::Point goal;
    goal.x = 1;
    goal.y = 1;

    nav.driveToPoint(goal);
//    OrderManager order_manager;
//    order_manager.generateOrder();
//    order_manager.spawnCubes();
//
//    Decoder decoder(nh);
//    ros::Rate r(1);
//
//    while(ros::ok()){
//
//        ros::spin();
//    }

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