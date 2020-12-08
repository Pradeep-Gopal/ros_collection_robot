#include <gtest/gtest.h>
#include "../include/map.h"
#include "../include/polygon.h"

TEST(polyfromRect,checkVertices) {
    std::string path = ros::package::getPath("ros_collection_robot");
    ROS_INFO_STREAM("/test/test_config/test.yaml");
    Map map(path+"/test/test_config/test.yaml",0.1);
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

//TEST(offsetPolygon,offsetTest) {
//    std::string path = ros::package::getPath("ros_collection_robot");
//    ROS_INFO_STREAM("/test/test_config/test.yaml");
//    double clearance = 0.1;
//    Map map(path+"/test/test_config/test.yaml",clearance);
//    Polygon poly = map.polyFromRect(0,0,1,1,0);
//
//    Polygon offset_polygon = map.offsetPolygon(poly);
//
//    std::vector<geometry_msgs::Point> vertices;
//    geometry_msgs::Point pt;
//    pt.x = -clearance;
//    pt.y = -clearance;
//    vertices.push_back(pt);
//    pt.x = 1 + clearance;
//    pt.y = -clearance;
//    vertices.push_back(pt);
//    pt.x = 1 + clearance;
//    pt.y = 1 + clearance;
//    vertices.push_back(pt);
//    pt.x = -clearance;
//    pt.y = 1 +clearance;
//    vertices.push_back(pt);
//    std::vector<geometry_msgs::Point> poly_vertices = offset_polygon.getVertices();
//
//    for (int i = 0; i<vertices.size();i++){
//    EXPECT_NEAR(poly_vertices[i].x,vertices[i].x,0.001);
//    EXPECT_NEAR(poly_vertices[i].y,vertices[i].y,0.001);
//    }
//}



