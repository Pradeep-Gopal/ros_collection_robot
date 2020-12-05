#include <gtest/gtest.h>
#include "../include/line.h"

TEST(coefficient,verticalLineTest) {
    geometry_msgs::Point point1;
    point1.x = 2;
    point1.y = 1;
    geometry_msgs::Point point2;
    point2.x = 2;
    point2.y = 3;
    geometry_msgs::Point test_point;
    test_point.x = 3;
    test_point.y = 2;
    Line line(point1,point2,test_point);
    EXPECT_EQ(line.a,2 );
    EXPECT_EQ(line.b,0 );
    EXPECT_EQ(line.c,-4 );
}
