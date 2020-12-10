#include "../include/decoder.h"

Decoder::Decoder(ros::NodeHandle& nh)
    : it_(nh)
    {
            // Subscrive to input video feed and publish output video feed
    camera_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
                                &Decoder::cameraCallback, this);
    image_pub_ = it_.advertise("/decoder/output_video", 1);
    }


void Decoder::cameraCallback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        frame_ = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    image_pub_.publish(cv_ptr->toImageMsg());
}

std::vector<int> Decoder::detectTags() {

}
