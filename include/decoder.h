#pragma once
#include <sensor_msgs/Image.h>

class Decoder{

private:

    ros::Subscriber camera_sub_;
    ros::Publisher order_sub_;

public:
    
    void cameraCallbacks(const sensor_msgs::ImageConstPtr& msg);
    std::string decodeQr(const sensor_msgs::ImageConstPtr& msg);
    bool inOrder(std::string order_string);

};
