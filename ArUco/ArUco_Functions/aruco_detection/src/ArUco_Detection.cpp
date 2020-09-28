#include "ArUco_Detection/ArUco_Detection.h"

ArUco_Detection::ArUco_Detection() : it(nh) {

    Default_Parameters();

    nh.param("/ArUco_Detection_Node/im_proc", im_proc, false);

    nh.param("/ArUco_Detection_Node/image_width", image_width_f, 1920.00);
    nh.param("/ArUco_Detection_Node/image_height", image_height_f, 1080.00);
    nh.param("/ArUco_Detection_Node/camera_name", camera_name, std::string("C920"));
    nh.param("/ArUco_Detection_Node/camera_matrix/data", camera_matrix_data, camera_matrix_data_default);
    nh.param("/ArUco_Detection_Node/distortion_model", distortion_model, std::string("plumb_bob"));
    nh.param("/ArUco_Detection_Node/distortion_coefficients/data", distortion_coefficients_data, distortion_coefficients_data_default);
    nh.param("/ArUco_Detection_Node/rectification_matrix/data", rectification_matrix_data, rectification_matrix_data_default);
    nh.param("/ArUco_Detection_Node/projection_matrix/data", projection_matrix_data, projection_matrix_data_default);

    nh.param("/ArUco_Detection_Node/scale_factors/scale_x", SCALE_X, 1.0);   //1.5
    nh.param("/ArUco_Detection_Node/scale_factors/scale_y", SCALE_Y, 1.0);   //2.7
    nh.param("/ArUco_Detection_Node/scale_factors/scale_z", SCALE_Z, 1.0);   //1.96

    // nh.getParam("/ArUco_Detection_Node/im_proc", im_proc);

    // nh.getParam("/ArUco_Detection_Node/image_width", image_width_f);
    // nh.getParam("/ArUco_Detection_Node/image_height", image_height_f);
    // nh.getParam("/ArUco_Detection_Node/camera_name", camera_name);
    // nh.getParam("/ArUco_Detection_Node/camera_matrix/data", camera_matrix_data);
    // nh.getParam("/ArUco_Detection_Node/distortion_model", distortion_model);
    // nh.getParam("/ArUco_Detection_Node/distortion_coefficients/data", distortion_coefficients_data);
    // nh.getParam("/ArUco_Detection_Node/rectification_matrix/data", rectification_matrix_data);
    // nh.getParam("/ArUco_Detection_Node/projection_matrix/data", projection_matrix_data);

    // nh.getParam("/ArUco_Detection_Node/scale_factors/scale_x", SCALE_X);
    // nh.getParam("/ArUco_Detection_Node/scale_factors/scale_y", SCALE_Y);
    // nh.getParam("/ArUco_Detection_Node/scale_factors/scale_z", SCALE_Z);

    Compute_Parameter_Matrix();

    // Subscriber and publisher
    it_publisher = it.advertise("/aruco_detection/output_video", 1000);
    aruco_measured_pose_publisher = nh.advertise<geometry_msgs::Pose>("/aruco_detection/aruco_measured_pose", 1);
    aruco_real_pose_publisher = nh.advertise<geometry_msgs::Pose>("/aruco_detection/aruco_real_pose", 1);

    if (im_proc) {it_subscriber = it.subscribe("/C920/image_rect", 1000, &ArUco_Detection::Image_Callback, this);}
    else {it_subscriber = it.subscribe("/C920/image_raw", 1000, &ArUco_Detection::Image_Callback, this);}


}


ArUco_Detection::~ArUco_Detection() {

}


void ArUco_Detection::Default_Parameters (void) {

    camera_matrix_data_default = {1402.740493, 0.000000, 980.962060, 0.000000, 1400.827205, 577.031025, 0.000000, 0.000000, 1.000000};
    distortion_coefficients_data_default = {0.105196, -0.167871, -0.001255, -0.000956, 0.000000};
    rectification_matrix_data_default = {1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000};
    projection_matrix_data_default = {1421.073730, 0.000000, 979.995217, 0.000000, 0.000000, 1424.568115, 575.907298, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000};
    
}


void ArUco_Detection::Compute_Parameter_Matrix (void) {

    std::cout << std::endl << std::endl;
    ROS_INFO("Camera Name \t\t=\t%s", camera_name.c_str());
    ROS_INFO("Image Width \t\t=\t%d", (int)image_width_f);
    ROS_INFO("Image Height \t\t=\t%d", (int)image_height_f);

    std::cout << std::endl;
    ROS_INFO("Distortion Model \t=\t%s", distortion_model.c_str());
    if(im_proc) {ROS_INFO("Image Proc \t\t=\ttrue");} else {ROS_INFO("Image Proc \t\t=\tfalse");};
    
    std::cout << std::endl;
    ROS_INFO("Scale Factor X \t\t=\t%.2f", SCALE_X);
    ROS_INFO("Scale Factor Y \t\t=\t%.2f", SCALE_Y);
    ROS_INFO("Scale Factor Z \t\t=\t%.2f", SCALE_Z);

    std::cout << std::endl;

    image_width = (int)image_width_f;
    image_height = (int)image_height_f;


    camera_matrix = (cv::Mat1d(3,3) << camera_matrix_data[0], camera_matrix_data[1], camera_matrix_data[2],
                                       camera_matrix_data[3], camera_matrix_data[4], camera_matrix_data[5], 
                                       camera_matrix_data[6], camera_matrix_data[7], camera_matrix_data[8]);

    std::cout << "\n\t\t\t\t" << camera_matrix.at<double>(0,0) << "\t      " << camera_matrix.at<double>(0,1) << "\t\t" << camera_matrix.at<double>(0,2) << std::endl;
    std::cout << "Camera Matrix \t\t=\t   " << camera_matrix.at<double>(1,0) << "\t   " << camera_matrix.at<double>(1,1) << "\t" << camera_matrix.at<double>(1,2) << std::endl;
    std::cout << "\t\t\t\t   " << camera_matrix.at<double>(2,0) << "\t      " << camera_matrix.at<double>(2,1) << "\t\t   " << camera_matrix.at<double>(2,2) << std::endl << std::endl;


    distortion_coefficients = (cv::Mat1d(1,5) << distortion_coefficients_data[0], distortion_coefficients_data[1], distortion_coefficients_data[2],
                                                 distortion_coefficients_data[3], distortion_coefficients_data[4]);

    std::cout << "\nDistortion Coeffs \t=\t" << distortion_coefficients.at<double>(0,0) << "  " << distortion_coefficients.at<double>(0,1) << "  " << distortion_coefficients.at<double>(0,2)  << "  " << distortion_coefficients.at<double>(0,3) << "  " << distortion_coefficients.at<double>(0,4) << std::endl << std::endl;


    rectification_matrix = (cv::Mat1d(3,3) << rectification_matrix_data[0], rectification_matrix_data[1], rectification_matrix_data[2],
                                              rectification_matrix_data[3], rectification_matrix_data[4], rectification_matrix_data[5], 
                                              rectification_matrix_data[6], rectification_matrix_data[7], rectification_matrix_data[8]);

    std::cout << "\n\t\t\t\t" << rectification_matrix.at<double>(0,0) << "\t" << rectification_matrix.at<double>(0,1) << "\t" << rectification_matrix.at<double>(0,2) << std::endl;
    std::cout << "Rectification Matrix \t=\t" << rectification_matrix.at<double>(1,0) << "\t" << rectification_matrix.at<double>(1,1) << "\t" << rectification_matrix.at<double>(1,2) << std::endl;
    std::cout << "\t\t\t\t" << rectification_matrix.at<double>(2,0) << "\t" << rectification_matrix.at<double>(2,1) << "\t" << rectification_matrix.at<double>(2,2) << std::endl << std::endl;


    projection_matrix = (cv::Mat1d(3,4) << projection_matrix_data[0], projection_matrix_data[1], projection_matrix_data[2] , projection_matrix_data[3],
                                           projection_matrix_data[4], projection_matrix_data[5], projection_matrix_data[6] , projection_matrix_data[7],
                                           projection_matrix_data[8], projection_matrix_data[9], projection_matrix_data[10], projection_matrix_data[11]);

    std::cout << "\n\t\t\t\t" << projection_matrix.at<double>(0,0) << "\t      " << projection_matrix.at<double>(0,1) << "\t\t" << projection_matrix.at<double>(0,2) << "\t      " << projection_matrix.at<double>(0,3) << std::endl;
    std::cout << "Projection Matrix \t=\t   " << projection_matrix.at<double>(1,0) << "\t   " << projection_matrix.at<double>(1,1) << "\t" << projection_matrix.at<double>(1,2) << "\t      " << projection_matrix.at<double>(1,3) << std::endl;
    std::cout << "\t\t\t\t   " << projection_matrix.at<double>(2,0) << "\t      " << projection_matrix.at<double>(2,1) << "\t\t   " << projection_matrix.at<double>(2,2) << "\t      " << projection_matrix.at<double>(2,3) << std::endl;
    std::cout << std::endl << std::endl;

}


void ArUco_Detection::Image_Callback (const sensor_msgs::ImageConstPtr &msg) {
    
    try {cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); init = true;}

    catch (cv_bridge::Exception& e) {

      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    
    }

}


void ArUco_Detection::ArUco_Detect (cv_bridge::CvImagePtr &image_ptr, const std::string window_name) {

    cv::Mat imageCopy;

    image_ptr->image.copyTo(imageCopy);

    // Detect markers
    cv::aruco::detectMarkers(imageCopy, markerDictionary, markerCorners, markerIds);

    // Display the image
    cv::namedWindow(window_name, CV_WINDOW_NORMAL); //CV_WINDOW_AUTOSIZE in alternativa

    // Draw detected markers on the displayed image
    cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);

    // Show the image with the detected marker
    cv::imshow(window_name, imageCopy);

    // If a marker was identified, show its ID
    for (int i = 0; i < markerIds.size(); i++) {std::cout << "marker " << (i + 1) << " ID is:  " << markerIds[i] << std::endl;}

    cv::waitKey(1);

}


void ArUco_Detection::ArUco_Pose_Estimation (cv_bridge::CvImagePtr &image_ptr, cv::Mat cameraMat, cv::Mat distCoeffs, const std::string window_name) {
    
    cv::Mat imageCopy;

    image_ptr->image.copyTo(imageCopy);

    clear();

    //Detect Markers
    cv::aruco::detectMarkers(imageCopy, markerDictionary, markerCorners, markerIds);

    if (markerIds.size() > 0) {

        if (markerIds[0] == ARUCO_ID) {

            // Draw detected markers on the displayed image
            cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);

        /****************************************************************************************************************
         *                                                                                                              *
         * - The markerCorners parameter is the vector of marker corners returned by the detectMarkers() function       *
         *                                                                                                              *
         * - The second parameter is the size of the marker side in meters or in any other unit; the translation        *
         *   vectors of the estimated poses will be in the same unit.                                                   *
         *                                                                                                              *
         * - cameraMatrix and distCoeffs are the camera calibration parameters (camera calibration process)             *
         *                                                                                                              *
         * - rvecs and tvecs (output parameters) are the rotation and translation vectors respectively, for each        *
         *   of the markers in markerCorners.                                                                           *
         *                                                                                                              *
         * *************************************************************************************************************/

            //distance returned in centimeters (ARUCO_DIMENSION is in cm)
            cv::aruco::estimatePoseSingleMarkers(markerCorners, ARUCO_DIMENSION, cameraMat, distCoeffs, rvecs, tvecs);

            // Draw axis for each marker
            for(int i = 0; i < markerIds.size(); i++) {cv::aruco::drawAxis(imageCopy, cameraMat, distCoeffs, rvecs[i], tvecs[i], ARUCO_DIMENSION);}

            // Publish Position of the ArUco Marker (only the first ArUco[0])
            Publish_ArUco_Pose(rvecs,tvecs);
            
        }
    }

    // Display the image
    cv::namedWindow(window_name, CV_WINDOW_NORMAL);
    cv::resizeWindow(window_name, 720, 480);
    // cv::resizeWindow(window_name, 800, 800);
    cv::imshow(window_name, imageCopy);

    cv::waitKey(1);

}


void ArUco_Detection::Publish_ArUco_Pose (std::vector<cv::Vec3d> rvecs, std::vector<cv::Vec3d> tvecs) {

    //tvecs is the translation vector between camera and aruco reference system   ->   x,y,z
    //rvecs is the rotation vector between camera and aruco reference system      ->   alpha(x),beta(y),gamma(z)

    /****************************************************************************************************************
     *                                                                                                              *
     * - x = orizzontale (+ = dx), y = verticale (+ = basso), z = profondità (distanza)                             *
     *                                                                                                              *
     * - z misurato = z reale * 1.96 (sperimentalmente con real camera)                                             *
     *                                                                                                              *
     * - x,y (posizione 2D nell'immagine) misurati rispetto all'angolo in alto a destra (real camera) o rispetto    *
     *   all'origine (virtual camera);                                                                              *
     *   dipendono dalla dimensione dell'aruco nell'immagine, quindi sono proporzionali alla distanza z             *
     *                                                                                                              *
     * *************************************************************************************************************/

    //Aruco Measured Pose from tvec and rvec

    aruco_measured_pose.position.x = tvecs[0][0];
    aruco_measured_pose.position.y = tvecs[0][1];
    aruco_measured_pose.position.z = tvecs[0][2];

    aruco_measured_pose.orientation.x = rvecs[0][0];
    aruco_measured_pose.orientation.y = rvecs[0][1];
    aruco_measured_pose.orientation.z = rvecs[0][2];
    aruco_measured_pose.orientation.w = 1;

    if ((aruco_measured_pose.position.x == 0)  && (aruco_measured_pose.position.y == 0) || (aruco_measured_pose.position.z == 0)) {/*NO ARUCO*/}
    else {aruco_measured_pose_publisher.publish(aruco_measured_pose);}

    if (camera_name == "C920") {

        //Compute Aruco Real Pose (Only with Real Camera)
        //x,y totali dell'immagine sono proporzionali a z

        float x_tot = aruco_measured_pose.position.z / SCALE_X;
        float y_tot = aruco_measured_pose.position.z / SCALE_Y;

        //centro dell'immagine (valori negativi perchè origine in alto a dx) -> solo camera laboratorio !

        float x_0 = -(x_tot / 2);
        float y_0 = -(y_tot / 2);

        aruco_real_pose.position.x = aruco_measured_pose.position.x - x_0;
        aruco_real_pose.position.y = -(aruco_measured_pose.position.y - y_0);
        aruco_real_pose.position.z = aruco_measured_pose.position.z / SCALE_Z;

        aruco_real_pose.orientation.x = aruco_measured_pose.orientation.x;
        aruco_real_pose.orientation.y = aruco_measured_pose.orientation.y;
        aruco_real_pose.orientation.z = aruco_measured_pose.orientation.z;
        aruco_real_pose.orientation.w = aruco_measured_pose.orientation.w;

    } else {aruco_real_pose = aruco_measured_pose;}

    if ((aruco_real_pose.position.x == 0)  && (aruco_real_pose.position.y == 0) && (aruco_real_pose.position.z == 0)) {/*NO ARUCO*/}
    else {aruco_real_pose_publisher.publish(aruco_real_pose);}

}


void ArUco_Detection::Update_GUI (cv_bridge::CvImagePtr &image_ptr, const std::string window_name) {

    // Output modified video stream
    it_publisher.publish(cv_ptr->toImageMsg());

    //Update GUI Window
    cv::namedWindow(window_name, CV_WINDOW_AUTOSIZE);
    cv::imshow(window_name, cv_ptr->image);
    cv::waitKey(1);

    // Output modified video stream
    it_publisher.publish(image_ptr->toImageMsg());

}

void ArUco_Detection::clear (void) {

    aruco_real_pose.position.x = 0;
    aruco_real_pose.position.y = 0;
    aruco_real_pose.position.z = 0;

    aruco_measured_pose.position.x = 0;
    aruco_measured_pose.position.y = 0;
    aruco_measured_pose.position.z = 0;

    markerIds.clear();
    markerCorners.clear();
    rvecs.clear();
    tvecs.clear();

}


void ArUco_Detection::spinner (void) {

    ros::spinOnce();

    if (init) {

        // ArUco_Detect (cv_ptr, ARUCO_DETECTION_WINDOW);
        ArUco_Pose_Estimation (cv_ptr, camera_matrix, distortion_coefficients, ARUCO_POSE_EXTIMATION_WINDOW);
        // Update_GUI (cv_ptr, OPENCV_WINDOW);
        
    }

}
