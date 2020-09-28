#include "aruco_approaching.h"


//----------------------------------------------------- CONSTRUCTOR -----------------------------------------------------//


aruco_approaching::aruco_approaching() {

    // scale factor camera parameters
    nh.param("/ArUco_Detection_Node/scale_factors/scale_x", SCALE_X, 1.0);   //1.5
    nh.param("/ArUco_Detection_Node/scale_factors/scale_y", SCALE_Y, 1.0);   //2.7
    nh.param("/ArUco_Detection_Node/scale_factors/scale_z", SCALE_Z, 1.0);   //1.96

    //Publisher & Subscriber

    aruco_real_pose_subscriber = nh.subscribe("/aruco_detection/aruco_real_pose", 1000, &aruco_approaching::Real_Pose_Callback, this);
    aruco_measured_pose_subscriber = nh.subscribe("/aruco_detection/aruco_measured_pose", 1000, &aruco_approaching::Measured_Pose_Callback, this);
    
    manipulator_realtime_position_subscriber =  nh.subscribe("/Robot_Bridge/prbt_Current_State_Position", 1000, &aruco_approaching::Manipulator_Pose_Callback, this);

    mobile_base_velocity_publisher = nh.advertise<geometry_msgs::Twist>("/Robot_Bridge/mpo_500_Planned_Velocity", 1000);
        
    //Service Clients
    
    plan_joint_space_client = nh.serviceClient<prbt_planner::PlanJointSpace>("/prbt_planner/plan_joint_space_service");
    plan_cartesian_space_client = nh.serviceClient<prbt_planner::PlanCartesianSpace>("/prbt_planner/plan_cartesian_space_service");
    set_planner_parameters_client = nh.serviceClient<prbt_planner::SetPlannerParameters>("/prbt_planner/set_planner_parameters_service");
    set_planner_dynamics_client = nh.serviceClient<prbt_planner::SetPlannerDynamics>("/prbt_planner/set_planner_dynamics_service");
    compute_end_effector_position_client = nh.serviceClient<prbt_planner::ComputeEndEffectorPosition>("/prbt_planner/compute_end_effector_position_service");
    get_manipulator_info_client = nh.serviceClient<prbt_planner::GetManipulatorInfo>("/prbt_planner/get_manipulator_info_service");
    get_manipulator_link_names_client = nh.serviceClient<prbt_planner::GetManipulatorLinkNames>("/prbt_planner/get_manipulator_link_names_service");
    get_manipulator_trajectory_client = nh.serviceClient<prbt_planner::GetManipulatorTrajectory>("/prbt_planner/get_manipulator_trajectory_service");

}


aruco_approaching::~aruco_approaching() {

    // delete planner;
    
}


//------------------------------------------------------ CALLBACK ------------------------------------------------------//


void aruco_approaching::Real_Pose_Callback (const geometry_msgs::Pose::ConstPtr &msg) {

    aruco_real_pose = *msg;
    
    if ((aruco_real_pose.position.x == 0)  && (aruco_real_pose.position.y == 0) && (aruco_real_pose.position.z == 0)) {aruco_detected = false;}
    else {aruco_detected = true;}

}


void aruco_approaching::Measured_Pose_Callback (const geometry_msgs::Pose::ConstPtr &msg) {

    aruco_measured_pose = *msg;

}


void aruco_approaching::Manipulator_Pose_Callback (const control_msgs::JointTrajectoryControllerState::ConstPtr &msg) {

    manipulator_current_position = *msg;

}


//------------------------------------------------ ARUCO FUNCTIONS ------------------------------------------------//


bool aruco_approaching::ArUco_Check (void) {

    aruco_detected = false;

    ros::Duration(1).sleep();

    ros::spinOnce();

    if (aruco_detected) {aruco_found = true; return true;}
    else return false;

}


bool aruco_approaching::ArUco_Alignment_Check (float tol_percentage) {

    ros::Duration(1).sleep();

    ros::spinOnce();

    //compute normalized pose of ArUco Marker (normalized on distance z)
    // geometry_msgs::Pose norm_pose = ArUco_Pose_Normalization(aruco_real_pose, aruco_measured_pose);

    //compute x position tolerance in function of z distance
    float position_tolerance = fabs((tol_percentage / 100) * aruco_measured_pose.position.z);

    if (fabs(aruco_real_pose.position.x) >= position_tolerance) {return false;}
    else {aruco_aligned = true; return true;}

}


/* bool aruco_approaching::ArUco_Alignment_Check (float tol) {

    ros::Duration(1).sleep();

    ros::spinOnce();

    //compute normalized pose of ArUco Marker (normalized on distance z)
    geometry_msgs::Pose norm_pose = ArUco_Pose_Normalization(aruco_real_pose, aruco_measured_pose);

    if ((norm_pose.position.x >= fabs(tol)) || (norm_pose.position.x <= -fabs(tol))) {return false;}
    else {aruco_aligned = true; return true;}

}


geometry_msgs::Pose aruco_approaching::ArUco_Pose_Normalization (geometry_msgs::Pose real_pose, geometry_msgs::Pose measured_pose) {

    //returns normalized pose in x and y (z unchanged)

    geometry_msgs::Pose normalized_pose;

    // const float SCALE_X = 1.5, SCALE_Y = 2.7;

    float x_tot = measured_pose.position.z / SCALE_X;
    float y_tot = measured_pose.position.z / SCALE_Y;

    normalized_pose = real_pose;

    normalized_pose.position.x = real_pose.position.x / fabs(x_tot / 2);
    normalized_pose.position.y = real_pose.position.y / fabs(y_tot / 2);

    return (normalized_pose);

} */


geometry_msgs::Pose aruco_approaching::Compute_ArUco_Position_Respect_Mobile_Base (void) {

    ros::spinOnce();

/************************************************************************************************************************
 *                                                                                                                      *
 *     Mobile Base (b)     Manipulator (m)     End Effector (e)     Camera Image (i)     ArUco Marker (a)               *
 *                                                                                                                      *
 *     Transformation Matrix     ->     Tᵇᵢ = Tᵇₘ * Tᵐₑ * Tᵉᵢ                                                           *
 *                                                                                                                      *
 *  To do the picking experiment, the ArUco Marker must be inside the manipulator workspace.                            *
 *  To do this, move the base towards the aruco until it is in an accessible point, therefore within a certain          *
 *  distance from the base of the manipulator.                                                                          *
 *  It is therefore useless to calculate the translation between the basic and manipulator reference systems            *
 *  because the distance must be calculated with respect to the position of the manipulator; on the other hand,         *
 *  the situation is different with respect to the 180 ° rotation of the reference system between the mobile base       *
 *  and the manipulator that must be considered.     //TODO:modifica                                                    *
 *       //TODO: y center SR mobile base = y center SR manipulator, x change                                            *
 *     Transformation Matrix     ->     Tᵇᵢ = Rᵇₘ * Tᵐₑ * Tᵉᵢ                                                           *
 *     aruco respect manipulator = Tbi * aruco respect camera;                                                          *
 *                                                                                                                      *
 ***********************************************************************************************************************/
    
    //compute manipulator base rotation  ->  x_man = -x_mbase | y_man = -y_mbase | z_man = z_mbase  ->  phi rotation around z
    Eigen::Matrix3d bm_rotation_matrix = Rotation_Matrix(cos(M_PI),-sin(M_PI),0, sin(M_PI),cos(M_PI),0, 0,0,1);
    Eigen::Quaterniond bm_rotation_quaternion(bm_rotation_matrix);
    std::vector<double> bm_rotation = Quaternion_TO_Vector(bm_rotation_quaternion);
    std::vector<double> bm_translation = {0.5, 0, 0}; //TODO: x distance manipulator - mobile base

    Eigen::Matrix4d Tbm = Transformation_Matrix(bm_translation, bm_rotation);
    // Transformation_Matrix_Cout("Tbm",Tbm);

    geometry_msgs::Pose end_effector_pose;
    
    //compute end-effector position in respect to the manipulator base (meters)
    if (compute_end_effector_position_client.call(compute_end_effector_position_srv)) {
        end_effector_pose = compute_end_effector_position_srv.response.ee_position;}
    else {ROS_ERROR("Failed to Call Service: \"ComputeEndEffectorPosition\"");}
    
    ROS_INFO("End-Effector Position: %.2f,  %.2f,  %.2f", end_effector_pose.position.x, end_effector_pose.position.y, end_effector_pose.position.z);
    ROS_INFO("End-Effector Orientation: %.2f,  %.2f,  %.2f,  %.2f", end_effector_pose.orientation.x, end_effector_pose.orientation.y, end_effector_pose.orientation.z, end_effector_pose.orientation.w);
    
    Eigen::Matrix4d Tme = Transformation_Matrix(end_effector_pose);
    // Transformation_Matrix_Cout("Tme",Tme);
    
    //compute camera translation in respect to end-effector  ->  //TODO: real measures 
    //camera position (x,y,z)cm is (0,10,10)
    //x -> camera rimane centrata      ->  oggetto non si sposta            ->  0cm;
    //y -> camera più in alto di 10cm  ->  oggetto trasla in basso di 10cm  ->  +10cm (y verso il basso)
    //z -> camera più lontana di 10cm  ->  oggetto si allontana di 10cm     ->  +10cm
    float camera_position_x = 0.0, camera_position_y = 0.10, camera_position_z = 0.10;
    std::vector<double> ei_translation = {camera_position_x, camera_position_y, camera_position_z};

    //compute real camera rotation  ->  x_camera = -x_ee | y_camera = -y_ee | z_camera = z_ee  ->  phi rotation around z
    /* Eigen::Matrix3d ei_rotation_matrix = Rotation_Matrix(cos(M_PI),-sin(M_PI),0, sin(M_PI),cos(M_PI),0, 0,0,1);
    Eigen::Quaterniond ei_rotation_quaternion(ei_rotation_matrix);
    std::vector<double> ei_rotation = Quaternion_TO_Vector(ei_rotation_quaternion); */

    //compute virtual camera rotation  ->  x_camera = y_ee | y_camera = -x_ee | z_camera = z_ee  ->  phi/2 rotation around z
    Eigen::Matrix3d ei_rotation_matrix = Rotation_Matrix(cos(M_PI/2),-sin(M_PI/2),0, sin(M_PI/2),cos(M_PI/2),0, 0,0,1);
    Eigen::Quaterniond ei_rotation_quaternion(ei_rotation_matrix);
    std::vector<double> ei_rotation = Quaternion_TO_Vector(ei_rotation_quaternion);

    Eigen::Matrix4d Tei = Transformation_Matrix(ei_translation, ei_rotation);
    // Transformation_Matrix_Cout("Tei",Tei);

    //aruco position respect camera (aruco_real_pose is in centimeters)
    Eigen::Vector3d aruco_pose_respect_camera;
    aruco_pose_respect_camera.x() = (aruco_real_pose.position.x)/100;
    aruco_pose_respect_camera.y() = (aruco_real_pose.position.y)/100;
    aruco_pose_respect_camera.z() = (aruco_real_pose.position.z)/100;
    ROS_WARN("ArUco Position Respect Camera: %.2f,  %.2f,  %.2f", aruco_pose_respect_camera.x(), aruco_pose_respect_camera.y(), aruco_pose_respect_camera.z());

    //compute transformation between mobile base and camera  ->  Tᵇᵢ = Tᵇₘ * Tᵐₑ * Tᵉᵢ
    //compute also the intermediate steps -> Tᵐᵢ = Tᵐₑ * Tᵉᵢ
    Eigen::Affine3d Tbi_(Tbm * Tme * Tei), Tmi_(Tme * Tei), Tei_(Tei);

    //compute aruco position respect end-effector, manipulator and mobile base
    Eigen::Vector3d aruco_pose_respect_end_effector = Tei_ * aruco_pose_respect_camera;
    Eigen::Vector3d aruco_pose_respect_manipulator = Tmi_ * aruco_pose_respect_camera;
    Eigen::Vector3d aruco_pose_respect_mobile_base = Tbi_ * aruco_pose_respect_camera;

    //print the trasformations results
    ROS_WARN("ArUco Position Respect End Effector: %.2f, %.2f, %.2f", aruco_pose_respect_end_effector.x(), aruco_pose_respect_end_effector.y(), aruco_pose_respect_end_effector.z());
    ROS_WARN("ArUco Position Respect Manipulator: %.2f, %.2f, %.2f", aruco_pose_respect_manipulator.x(), aruco_pose_respect_manipulator.y(), aruco_pose_respect_manipulator.z());
    ROS_WARN("ArUco Position Respect Mobile Base: %.2f, %.2f, %.2f", aruco_pose_respect_mobile_base.x(), aruco_pose_respect_mobile_base.y(), aruco_pose_respect_mobile_base.z());

    geometry_msgs::Pose aruco_respect_mobile_base;
    aruco_respect_mobile_base.orientation = end_effector_pose.orientation;
    aruco_respect_mobile_base.position.x = aruco_pose_respect_mobile_base.x();
    aruco_respect_mobile_base.position.y = aruco_pose_respect_mobile_base.y();
    aruco_respect_mobile_base.position.z = aruco_pose_respect_mobile_base.z();

    //measured in meters
    return aruco_respect_mobile_base;

}


//------------------------------------------ MOBILE BASE MOVEMENT FUNCTIONS ------------------------------------------//


void aruco_approaching::Mobile_Base_Linear_Movement(float x_velocity, float y_velocity, float z_twist, float movement_time) {

    geometry_msgs::Twist cmd_vel;

    //publish cmd_vel
    cmd_vel_linear(&cmd_vel, x_velocity, y_velocity, 0.0);
    cmd_vel_angular(&cmd_vel, 0.0, 0.0, z_twist);
    mobile_base_velocity_publisher.publish(cmd_vel);
    ROS_INFO("Vel_X = %.2f, Vel_Y = %.2f, Twist_Z = %.2f", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
    ROS_INFO("Movement Time = %.2f", movement_time);

    //wait movement time
    ros::Duration(movement_time).sleep();

    //stop the movement
    cmd_vel_linear(&cmd_vel, 0.0, 0.0, 0.0);
    cmd_vel_angular(&cmd_vel, 0.0, 0.0, 0.0);
    mobile_base_velocity_publisher.publish(cmd_vel);

}


void aruco_approaching::Mobile_Base_GOTO_ArUco (float x_vel) {

    //compute distance between mobile base and aruco marker (in meters)
    geometry_msgs::Pose aruco_pose = Compute_ArUco_Position_Respect_Mobile_Base();

    float x_dis_mb_and_man = 0.50; //TODO: x distance between mobile base and manipulator
    float manipulator_workspace = 0.50;

    //compute the distance I want to move the mobile base, considering manipulator workspace
    //50cm is the maximum workspace of manipulator -> +30cm to get closer the aruco
    //TODO: extra space needed = -40cm (move less), disattivato ora
    float distance = aruco_pose.position.x -x_dis_mb_and_man -manipulator_workspace +0.30;

    //fixed velocity (velocity) -> change time (distance = vel * time)
    float translation_time = fabs(distance / x_vel);

    ROS_INFO("Distance = %.2f", fabs(distance));
    ROS_INFO("Movement Time: %.2f", translation_time);

    Mobile_Base_Linear_Movement(fabs(x_vel), 0.0, 0.0, translation_time);

    ROS_WARN("Mobile Base ARRIVED");
    
}


//------------------------------------------------------- MAIN --------------------------------------------------------//


void aruco_approaching::approach (void) {

    ros::spinOnce();
    
    if (!ArUco_Check()) {ROS_ERROR("ArUco Marker NOT-DETECTED");}
    else if (ArUco_Check()) {

        if(!ArUco_Alignment_Check(2)) {ROS_ERROR("ArUco Marker NOT-ALIGNED");}
        else if(ArUco_Alignment_Check(2)) {

            ROS_INFO("ArUco Marker Aligned -> Start Approaching");
            Mobile_Base_GOTO_ArUco(0.5);

            aruco_approached = true;

        }
    }
}
