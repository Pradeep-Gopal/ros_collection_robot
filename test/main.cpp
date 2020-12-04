#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc,
         char **argv) {
	ros::init(argc, argv, "order_manager_test");
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}