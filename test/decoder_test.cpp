#include <gtest/gtest.h>
#include "../include/decoder.h"
#include "../include/navigator.h"
#include <ros/ros.h>
#include <ros/service_client.h>

TEST(decoder, testDecoder) {
    ros::NodeHandle nh;
    Decoder decoder(nh);
    Navigator nav;
    EXPECT_FALSE(decoder.determined_camera_params);
}

TEST(getCube, testGetCube) {
    geometry_msgs::Point location;
    location.x = 1;
    location.y = 1;
    location.z = 0;
    ros::NodeHandle nh;
    Decoder decoder(nh);
    EXPECT_EQ(decoder.getCube(location).id, -1);
}


