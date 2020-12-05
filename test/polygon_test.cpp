#include <gtest/gtest.h>
#include "../include/polygon.h"

TEST(calculateCentroid,simpleCentroidTest) {
    std::vector <geometry_msgs::Point> vertices;
    geometry_msgs::Point point1;
    point1.x = 0;
    point1.y = 0;
    geometry_msgs::Point point2;
    point2.x = 2;
    point2.y = 0;
    geometry_msgs::Point point3;
    point3.x = 1;
    point3.y = 2;
    vertices.push_back(point1);
    vertices.push_back(point2);
    vertices.push_back(point3);
    Polygon polygon(vertices);

    EXPECT_NEAR(polygon.getCentroid().x,1,0.01);
    EXPECT_NEAR(polygon.getCentroid().y,0.66,0.05);
}