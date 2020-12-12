#include "../include/decoder.h"

Decoder::Decoder(ros::NodeHandle& nh)
    : it_(nh){
        // Subscribe to input video feed and publish output video feed
        camera_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
                                    &Decoder::cameraCallback, this);
        camera_info_sub_ = nh.subscribe("/camera/rgb/camera_info",1,
                                        &Decoder::cameraInfoCallback, this);
        image_pub_ = it_.advertise("/decoder/output_video", 1);
        determined_camera_params = false;
    }

void Decoder::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camInfo_msg) {
    cv::Mat K(3, 3, CV_64FC1, (void *) camInfo_msg->K.data());
    k_matrix_ = K;
//    cv::Mat D(1, 5, CV_64FC1, (void *) camInfo_msg->D.data());
    cv::Mat D = cv::Mat::zeros(1,5, CV_64FC1);
    d_matrix_ = D;
    camera_frame_ = camInfo_msg->header.frame_id;
    determined_camera_params = true;
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
//    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
//        cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    marker_ids_.clear();
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    cv::aruco::detectMarkers(cv_ptr->image, dictionary, markerCorners, marker_ids_, parameters, rejectedCandidates);
    cv::aruco::drawDetectedMarkers(cv_ptr->image, markerCorners, marker_ids_);

    if (determined_camera_params){
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.04, k_matrix_, d_matrix_, rvecs, tvecs);

        for (int i = 0; i < rvecs.size(); ++i) {
            auto rvec = rvecs[i];
            auto tvec = tvecs[i];

            cv::aruco::drawAxis(cv_ptr->image, k_matrix_, d_matrix_, rvec, tvec, 0.1);

            // Build identity rotation matrix as a cv::Mat
            cv::Mat rot(3, 3, CV_64FC1);
            cv::Rodrigues(rvec, rot);

            // Convert to a tf2::Matrix3x3
            tf2::Matrix3x3 tf2_rot(rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2),
                                   rot.at<double>(1, 0), rot.at<double>(1, 1), rot.at<double>(1, 2),
                                   rot.at<double>(2, 0), rot.at<double>(2, 1), rot.at<double>(2, 2));

            // Create a transform and convert to a Pose
            tf2::Transform tf2_transform(tf2_rot, tf2::Vector3(tvec[0],tvec[1],tvec[2]));
            geometry_msgs::Pose cube_pose;
            tf2::toMsg(tf2_transform, cube_pose);

            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tfListener(tfBuffer);
            geometry_msgs::TransformStamped transformStamped;

            geometry_msgs::PoseStamped world_pose, relative_pose;

            relative_pose.pose = cube_pose;

            ros::Duration timeout(2.0);

            try {
                transformStamped = tfBuffer.lookupTransform("odom",
                                                            camera_frame_,
                                                            ros::Time(0), timeout);
            } catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                continue;
            }

            tf2::doTransform(relative_pose, world_pose, transformStamped);

            ROS_INFO_STREAM(world_pose);
        }

    }

    image_pub_.publish(cv_ptr->toImageMsg());
}

geometry_msgs::Pose Decoder::determineCubePose() {

}

std::vector<int> Decoder::getMarkerIDs() {
    return marker_ids_;
}
