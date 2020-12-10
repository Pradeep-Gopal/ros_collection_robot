#pragma once
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <image_transport/image_transport.h>
#include "ros/ros.h"

class Decoder{
private:
    image_transport::ImageTransport it_;
    image_transport::Subscriber camera_sub_;
    image_transport::Publisher image_pub_;
    cv::Mat frame_;
    std::vector<int> marker_ids_;

public:
    Decoder(ros::NodeHandle&);
    void cameraCallback(const sensor_msgs::ImageConstPtr&);
    std::vector<int> detectTags();
};
