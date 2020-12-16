/**
 * @file decoder.cpp
 * @author Pradeep Gopal, Justin Albrecht Govind Ajith Kumar
 * @copyright MIT License
 * @brief Source file for the decoder class
 * Source file for the decoder class which does the AR tag decoding
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

#include "../include/decoder.h"

/**
 * @brief      Constructs a new instance.
 * Initializes the callback functions
 * @param      nh    Node Handle
 * @return None
 */
Decoder::Decoder(ros::NodeHandle& nh)
    : it_(nh) {
        // Subscribe to input video feed and publish output video feed
        camera_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
                                    &Decoder::cameraCallback, this);
        camera_info_sub_ = nh.subscribe("/camera/rgb/camera_info", 10,
                                        &Decoder::cameraInfoCallback, this);
        image_pub_ = it_.advertise("/decoder/output_video", 1);
        determined_camera_params = false;
    }

/**
 * @brief Callback for the Camera Info Callback
 * @param[in]  camInfo_msg  The camera information message
 * @return None
 */
void Decoder::cameraInfoCallback(const
        sensor_msgs::CameraInfoConstPtr& camInfo_msg) {
    cv::Mat K(3, 3, CV_64FC1, (void *) camInfo_msg->K.data());
    k_matrix_ = K;
    cv::Mat D = cv::Mat::zeros(1, 5, CV_64FC1);
    d_matrix_ = D;
    camera_frame_ = camInfo_msg->header.frame_id;
    determined_camera_params = true;
}

/**
 * @brief      Callback for the Camera
 * Also detects the presence of an AR tag in the frame
 * @param[in]  msg   The message
 * @return None
 */
void Decoder::cameraCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::
            DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::
            getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::detectMarkers(cv_ptr->image, dictionary, markerCorners,
                             marker_ids, parameters, rejectedCandidates);

    if (determined_camera_params) {
        std::vector <cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(
                markerCorners, 0.04, k_matrix_, d_matrix_, rvecs, tvecs);

        for (int i = 0; i < rvecs.size(); ++i) {
            auto rvec = rvecs[i];
            auto tvec = tvecs[i];

            // Build identity rotation matrix as a cv::Mat
            cv::Mat rot(3, 3, CV_64FC1);
            cv::Rodrigues(rvec, rot);

            // Convert to a tf2::Matrix3x3
            tf2::Matrix3x3 tf2_rot(rot.at<double>(0, 0),
                    rot.at<double>(0, 1), rot.at<double>(0, 2),
                           rot.at<double>(1, 0),
                                   rot.at<double>(1, 1), rot.at<double>(1, 2),
                           rot.at<double>(2, 0),
                                   rot.at<double>(2, 1), rot.at<double>(2, 2));

            // Create a transform and convert to a Pose
            tf2::Transform tf2_transform(tf2_rot,
                                 tf2::Vector3(tvec[0], tvec[1], tvec[2]+0.2));
            geometry_msgs::Pose relative_cube_pose;
            tf2::toMsg(tf2_transform, relative_cube_pose);

            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tfListener(tfBuffer);
            geometry_msgs::TransformStamped transformStamped;
            geometry_msgs::PoseStamped world_pose;
            geometry_msgs::PoseStamped relative_pose;
            relative_pose.pose = relative_cube_pose;

            ros::Duration timeout(2.0);
            try {
                transformStamped = tfBuffer.lookupTransform("odom",
                                                        camera_frame_,
                                                        ros::Time(0), timeout);
            } catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
            }
            tf2::doTransform(relative_pose, world_pose, transformStamped);

            bool new_cube = true;
            Cube cube;
            cube.pose = world_pose.pose;
            cube.id = marker_ids[i];

            for (int j = 0; j < cubes_.size(); ++j) {
                double distance = sqrt(pow(cubes_[j].pose.position.x
                        - world_pose.pose.position.x, 2) +
                        pow(cubes_[j].pose.position.y
                        - world_pose.pose.position.y, 2));
                if (distance < 1) {
                    new_cube = false;
                    cubes_[i] = cube;  // replace existing pose with new one
                }
            }

            if (new_cube) {
                cubes_.push_back(cube);
            }
        }
    }

    cv::aruco::drawDetectedMarkers(cv_ptr->image, markerCorners, marker_ids);
    image_pub_.publish(cv_ptr->toImageMsg());
}

/**
 * @brief      Gets the cube.
 * @param[in]  location  The location
 * @return     The cube.
 */
Cube Decoder::getCube(geometry_msgs::Point location) {
    for (Cube cube : cubes_) {
        double distance = sqrt(pow(cube.pose.position.x - location.x, 2) +
                               pow(cube.pose.position.y - location.y, 2));
        if (distance < 1)
            return cube;
    }
    Cube blank_cube;
    blank_cube.id = -1;
    return blank_cube;
}
