#ifndef ARUCO_DETECTION_H
#define ARUCO_DETECTION_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include "geometry_msgs/Pose.h"


class ArUco_Detection {

    public:

        ArUco_Detection();
        ~ArUco_Detection();

        void spinner (void);

    private:

        ros::NodeHandle nh;

        image_transport::ImageTransport it;
        image_transport::Subscriber it_subscriber;
        image_transport::Publisher it_publisher;
        ros::Publisher aruco_real_pose_publisher, aruco_measured_pose_publisher;

        const std::string OPENCV_WINDOW = "OpenCV Image";
        const std::string ARUCO_DETECTION_WINDOW = "ArUco Detection";
        const std::string ARUCO_POSE_EXTIMATION_WINDOW = "ArUco Pose Extimation";

        const int ARUCO_ID = 11;
        const float ARUCO_DIMENSION = 10; //dimension in centimeters
        // const float ARUCO_DIMENSION = 0.010; //dimension in meters

        // Define a Dictionary type variable for marker detection
        cv::Ptr<cv::aruco::Dictionary>markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_ARUCO_ORIGINAL);

        bool im_proc = false;
        int image_width, image_height;
        double image_width_f, image_height_f;
        double SCALE_X, SCALE_Y, SCALE_Z;
        std::string camera_name, distortion_model;
        std::vector<float> camera_matrix_data, camera_matrix_data_default;
        std::vector<float> distortion_coefficients_data, distortion_coefficients_data_default;
        std::vector<float> rectification_matrix_data, rectification_matrix_data_default;
        std::vector<float> projection_matrix_data, projection_matrix_data_default;

        cv_bridge::CvImagePtr cv_ptr;

        cv::Mat camera_matrix, distortion_coefficients;
        cv::Mat rectification_matrix, projection_matrix;

        std::vector<cv::Vec3d> rvecs, tvecs;
        geometry_msgs::Pose aruco_real_pose, aruco_measured_pose;
        
        bool init = false;
        
        // Define variables to store the output of marker detection
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

        void Default_Parameters (void);
        void Compute_Parameter_Matrix (void);

        void Image_Callback (const sensor_msgs::ImageConstPtr &);

        void ArUco_Detect (cv_bridge::CvImagePtr &image_ptr, const std::string window_name);
        void ArUco_Pose_Estimation (cv_bridge::CvImagePtr &image_ptr, cv::Mat cameraMat, cv::Mat distCoeffs, const std::string window_name);
        
        void Publish_ArUco_Pose (std::vector<cv::Vec3d> rvecs, std::vector<cv::Vec3d> tvecs);
        
        void Update_GUI (cv_bridge::CvImagePtr &image_ptr, const std::string window_name);

        void clear (void);


};

#endif /* ARUCO_DETECTION_H */
