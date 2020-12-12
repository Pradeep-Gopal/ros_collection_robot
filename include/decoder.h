#pragma once
#include "geometry_msgs/PoseStamped.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <image_transport/image_transport.h>
#include "ros/ros.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class Decoder{
private:
    bool determined_camera_params;
    image_transport::ImageTransport it_;
    image_transport::Subscriber camera_sub_;
    ros::Subscriber camera_info_sub_;
    image_transport::Publisher image_pub_;
    cv::Mat frame_;
    std::vector<int> marker_ids_;

public:
    cv::Mat k_matrix_;
    cv::Mat d_matrix_;
    std::string camera_frame_;
    Decoder(ros::NodeHandle&);
    void cameraCallback(const sensor_msgs::ImageConstPtr&);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr&);
    geometry_msgs::Pose determineCubePose();
    std::vector<int> getMarkerIDs();
};
