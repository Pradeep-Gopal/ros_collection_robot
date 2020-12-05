#pragma once
#include <vector>
// #include <geometry_msgs/PointStamped.h>
class Manipulator{

private:

	std::vector<geometry_msgs::Pose>pick_waypoints_;
	std::vector<geometry_msgs::Pose>place_waypoints_;
	ros::Publisher end_effector_publisher_;

public:
	
	void placePart();
	void pickPart();

};
