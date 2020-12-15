#include <gtest/gtest.h>
#include "../include/navigator.h"

TEST(navigatorConstructor,navigatorConstructorTest) {
    OrderManager order_manager;
    order_manager.generateOrder();
    order_manager.spawnCubes();
    Navigator nav;
    ASSERT_FALSE(nav.cube_detected_);
    ASSERT_FALSE(nav.cube_detected_);
    ASSERT_FALSE(nav.determined_pose);
    EXPECT_EQ(nav.current_waypoint_,0);
}

TEST(navigate,navigateTest1) {
    Navigator nav;
    std::vector<geometry_msgs::Point> waypoints_nav;
    waypoints_nav = nav.waypoints;
    EXPECT_EQ(waypoints_nav.size(),19);
}
