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

//            geometry_msgs::Pose cube_pose;

            cv::aruco::drawAxis(cv_ptr->image, k_matrix_, d_matrix_, rvec, tvec, 0.1);

//
//            cube_pose.position.x = tvec[0];
//            cube_pose.position.y = tvec[1];
//            cube_pose.position.z = tvec[2];
//            cube_pose.orientation.x = q_tf.getX();
//            cube_pose.orientation.y = q_tf.getY();
//            cube_pose.orientation.z = q_tf.getZ();
//            cube_pose.orientation.w = q_tf.getW();
//
//            ROS_INFO_STREAM(cube_pose);

            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(tvec[0],tvec[1],tvec[2]));

            tf::Quaternion q;
            q.setRPY(rvec[0], rvec[1], rvec[2]);
            transform.setRotation(q);

            br.sendTransform(tf::StampedTransform(transform,
                                                  ros::Time::now(), "world", camera_frame_));
        }
    }

    image_pub_.publish(cv_ptr->toImageMsg());
}

geometry_msgs::Pose Decoder::determineCubePose() {

}

std::vector<int> Decoder::getMarkerIDs() {
    return marker_ids_;
}
