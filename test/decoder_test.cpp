#include <gtest/gtest.h>
#include "../include/map.h"
#include "../include/polygon.h"

TEST(polyfromRect,checkVertices) {
Map map(0.1);
Polygon poly = map.polyFromRect(0,0,1,1,0);

std::vector<geometry_msgs::Point> vertices;
geometry_msgs::Point pt;
pt.x = 0;
pt.y = 0;
vertices.push_back(pt);
pt.x = 1;
pt.y = 0;
vertices.push_back(pt);
pt.x = 1;
pt.y = 1;
vertices.push_back(pt);
pt.x = 0;
pt.y = 1;
vertices.push_back(pt);
std::vector<geometry_msgs::Point> poly_vertices = poly.getVertices();
for (int i = 0; i<vertices.size();i++){
EXPECT_NEAR(poly_vertices[i].x,vertices[i].x,0.001);
EXPECT_NEAR(poly_vertices[i].y,vertices[i].y,0.001);
}
}

TEST(insideObstacle, checkInsideObstacle1) {
Map map(0.1);
geometry_msgs::Point check_point;
check_point.x = 1;
check_point.y = 1;
check_point.z = 0;
EXPECT_FALSE(map.insideObstacle(check_point));
}

TEST(insideObstacle, checkInsideObstacle2) {
Map map(0.1);
geometry_msgs::Point check_point;
check_point.x = 1;
check_point.y = 0;
check_point.z = 0;
EXPECT_TRUE(map.insideObstacle(check_point));
}

TEST(offsetPolygon,offsetTest) {
double clearance = 0.1;
Map map(clearance);
Polygon poly = map.polyFromRect(0,0,1,1,0);
Polygon offset_polygon = map.offsetPolygon(poly);

std::vector<geometry_msgs::Point> vertices;
geometry_msgs::Point pt;
pt.x = -clearance;
pt.y = -clearance;
vertices.push_back(pt);
pt.x = 1 + clearance;
pt.y = -clearance;
vertices.push_back(pt);
pt.x = 1 + clearance;
pt.y = 1 + clearance;
vertices.push_back(pt);
pt.x = -clearance;
pt.y = 1 +clearance;
vertices.push_back(pt);
std::vector<geometry_msgs::Point> poly_vertices = offset_polygon.getVertices();

for (int i = 0; i<vertices.size();i++){
EXPECT_NEAR(poly_vertices[i].x,vertices[i].x,2);
EXPECT_NEAR(poly_vertices[i].y,vertices[i].y,2);
}
}

TEST(polyFromCircle,checkCirclePOints) {

double clearance = 0.1;
Map map(clearance);
Polygon poly = map.polyFromCircle(2,2,1);
Polygon offset_polygon = map.offsetPolygon(poly);

std::vector<geometry_msgs::Point> vertices;
geometry_msgs::Point pt;
pt.x = 1 + clearance;
pt.y = 1 + clearance;
vertices.push_back(pt);
std::vector<geometry_msgs::Point> poly_vertices = offset_polygon.getVertices();

for (int i = 0; i<vertices.size();i++){
EXPECT_NEAR(poly_vertices[i].x,vertices[i].x,2);
EXPECT_NEAR(poly_vertices[i].y,vertices[i].y,2);
}
}


