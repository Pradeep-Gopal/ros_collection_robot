#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>

int main(int argc,
         char **argv) {
	ros::init(argc, argv, "order_manager_test");
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}