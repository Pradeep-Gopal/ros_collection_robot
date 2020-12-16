/**
 * @file decoder.h
 * @author Pradeep Gopal, Justin Albrecht Govind Ajith Kumar
 * @copyright MIT License
 * @brief Header file for the decoder class
 * Header file for the decoder class which does the AR tag decoding
 */

/**
 *MIT License
 *Copyright (c) 2020 Pradeep Gopal, Justin Albrecht, Govind Ajith Kumar
 *Permission is hereby granted, free of charge, to any person obtaining a copy
 *of this software and associated documentation files (the "Software"), to deal
 *in the Software without restriction, including without limitation the rights
 *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *copies of the Software, and to permit persons to whom the Software is
 *furnished to do so, subject to the following conditions:
 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.
 */

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

struct Cube{
    geometry_msgs::Pose pose;
    int id;
};

/**
 * @brief This class does the AR code decoding
 * and checks if cube exists in order
 */
class Decoder{
private:
    // -- Attributes
    image_transport::ImageTransport it_;
    image_transport::Subscriber camera_sub_;
    ros::Subscriber camera_info_sub_;
    image_transport::Publisher image_pub_;
    cv::Mat k_matrix_;
    cv::Mat d_matrix_;
    std::string camera_frame_;
    std::vector<Cube> cubes_;

public:
    // -- Attributes
    bool determined_camera_params;

    // -- Methods
    explicit Decoder(ros::NodeHandle&);

    /**
     * @brief Camera callback which reads from the camera ROS topic
     * @param[in] camera message from the rostopic
     */
    void cameraCallback(const sensor_msgs::ImageConstPtr&);

    /**
     * @brief Callback to store the information of the camera
     *
     * @param[in] message containing the camera info
     */
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr&);
    Cube getCube(geometry_msgs::Point);
};

