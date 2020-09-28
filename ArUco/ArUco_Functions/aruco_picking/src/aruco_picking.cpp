#include "aruco_picking.h"


//----------------------------------------------------- CONSTRUCTOR -----------------------------------------------------//


aruco_picking::aruco_picking() {

    //Publisher & Subscriber

    aruco_real_pose_subscriber = nh.subscribe("/aruco_detection/aruco_real_pose", 1000, &aruco_picking::Real_Pose_Callback, this);
    
    manipulator_position_reached_subscriber = nh.subscribe("/Robot_Bridge/prbt_Position_Reached", 1000, &aruco_picking::Manipulator_Position_Reached_Callback, this);
    manipulator_realtime_position_subscriber =  nh.subscribe("/Robot_Bridge/prbt_Current_State_Position", 1000, &aruco_picking::Manipulator_Pose_Callback, this);
    
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


aruco_picking::~aruco_picking() {

    // delete planner;
    
}


//------------------------------------------------------ CALLBACK ------------------------------------------------------//


void aruco_picking::Real_Pose_Callback (const geometry_msgs::Pose::ConstPtr &msg) {

    aruco_real_pose = *msg;
    
    if ((aruco_real_pose.position.x == 0)  && (aruco_real_pose.position.y == 0) && (aruco_real_pose.position.z == 0)) {aruco_detected = false;}
    else {aruco_detected = true;}

}


void aruco_picking::Manipulator_Pose_Callback (const control_msgs::JointTrajectoryControllerState::ConstPtr &msg) {

    manipulator_current_position = *msg;

}


void aruco_picking::Manipulator_Position_Reached_Callback (const std_msgs::Bool::ConstPtr &msg) {

    position_reached = *msg;

}


//------------------------------------------------ ARUCO FUNCTIONS ------------------------------------------------//


void aruco_picking::Home_Position (void) {

    //manipulator to home position

    if (get_manipulator_info_client.call(get_manipulator_info_srv)) {final_position = get_manipulator_info_srv.response.home_position;}
    else {ROS_ERROR("Failed to Call Service: \"GetManipulatorInfo\"");}
    
    plan_joint_space_srv.request.final_position = final_position;
    plan_joint_space_srv.request.send = true;
    
    if (plan_joint_space_client.call(plan_joint_space_srv)) {bool planning_succesfull = plan_joint_space_srv.response.response;}
    else {ROS_ERROR("Failed to Call Service: \"PlanJointSpace\"");}

    Wait_For_Position_Reached();

}


bool aruco_picking::ArUco_Check (void) {

    aruco_detected = false;

    ros::Duration(1).sleep();

    ros::spinOnce();

    if (aruco_detected) {aruco_found = true; return true;}
    else return false;

}


geometry_msgs::Pose aruco_picking::Compute_ArUco_Position_Respect_Manipulator (void) {

    ros::spinOnce();

/************************************************************************************************************************
 *                                                                                                                      *
 *     Mobile Base (b)     Manipulator (m)     End Effector (e)     Camera Image (i)     ArUco Marker (a)               *
 *                                                                                                                      *
 *     Transformation Matrix     ->     Tᵐᵢ = Tᵐₑ * Tᵉᵢ                                                                 *
 *                                                                                                                      *
 ***********************************************************************************************************************/

    //compute end-effector position in respect to the manipulator base (meters)
    geometry_msgs::Pose end_effector_pose;

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

    //compute transformation between mobile base and camera  ->  Tᵐᵢ = Tᵐₑ * Tᵉᵢ
    //Eigen::Matrix4d Tmi = Tme * Tei;
    Eigen::Affine3d Tmi(Tme * Tei);

    //aruco position respect camera (aruco_real_pose is in centimeters)
    Eigen::Vector3d aruco_pose_respect_camera;
    aruco_pose_respect_camera.x() = (aruco_real_pose.position.x)/100;
    aruco_pose_respect_camera.y() = (aruco_real_pose.position.y)/100;
    aruco_pose_respect_camera.z() = (aruco_real_pose.position.z)/100;
    ROS_INFO("ArUco Position Respect Camera: %.2f,  %.2f,  %.2f", aruco_pose_respect_camera.x(), aruco_pose_respect_camera.y(), aruco_pose_respect_camera.z());
    
    //compute aruco position respect manipulator
    Eigen::Vector3d aruco_pose_respect_manipulator = Tmi * aruco_pose_respect_camera;
    ROS_WARN("ArUco Position Respect Manipulator: %.2f, %.2f, %.2f", aruco_pose_respect_manipulator.x(), aruco_pose_respect_manipulator.y(), aruco_pose_respect_manipulator.z());

    geometry_msgs::Pose aruco_respect_manipulator;
    aruco_respect_manipulator.orientation = end_effector_pose.orientation;
    aruco_respect_manipulator.position.x = aruco_pose_respect_manipulator.x();
    aruco_respect_manipulator.position.y = aruco_pose_respect_manipulator.y();
    aruco_respect_manipulator.position.z = aruco_pose_respect_manipulator.z();

    //measured in meters
    return aruco_respect_manipulator;

}


//----------------------------------------- MANIPULATOR PICKING FUNCTIONS -----------------------------------------//


void aruco_picking::Wait_For_Position_Reached (void) {

    position_reached.data = false;
    while (!position_reached.data) {ros::spinOnce(); /* WAIT TO REACH DESIRED POSITION */}

}


bool aruco_picking::Check_Manipulator_Workspace (geometry_msgs::Pose position) {

    bool position_reachable = false;

    float distance = sqrt(pow(position.position.x,2) + pow(position.position.y,2) + pow(position.position.z,2));

    //max manipulator excursion = 1m, safety = 90cm
    if (distance < 0.90) {position_reachable = true;}

    if (position_reachable) return true;
    else return false;

}


void aruco_picking::Manipulator_Picking (void) {

    geometry_msgs::PoseStamped aruco_position;
    std::string end_effector_link;

    if (get_manipulator_link_names_client.call(get_manipulator_link_names_srv)) {
        aruco_position.header.frame_id = get_manipulator_link_names_srv.response.world;
        end_effector_link = get_manipulator_link_names_srv.response.end_effector;
    } else {ROS_ERROR("Failed to Call Service: \"GetManipulatorLinkNames\"");}

    aruco_position.pose = Compute_ArUco_Position_Respect_Manipulator();

    if (Check_Manipulator_Workspace(aruco_position.pose)) {

        set_planner_parameters_srv.request.planner_id = "RRTconnect";
        set_planner_parameters_srv.request.attempts = 20;
        set_planner_parameters_srv.request.time = 3.0;
        set_planner_parameters_srv.request.v_factor = 0.2;
        set_planner_parameters_srv.request.a_factor = 0.2;

        if (set_planner_parameters_client.call(set_planner_parameters_srv)) {}
        else {ROS_ERROR("Failed to Call Service: \"SetPlannerParameters\"");}

        //move manipulator to aruco marker

        plan_cartesian_space_srv.request.final_pose = aruco_position;
        plan_cartesian_space_srv.request.link_name = end_effector_link;
        plan_cartesian_space_srv.request.send = true;
    
        if (plan_cartesian_space_client.call(plan_cartesian_space_srv)) {bool planning_succesfull = plan_cartesian_space_srv.response.response;}
        else {ROS_ERROR("Failed to Call Service: \"PlanCartesianSpace\"");}

        Wait_For_Position_Reached();

        ROS_WARN("Manipulator ARRIVED to ArUco Marker");

    } else {ROS_ERROR("ArUco Not Reachable");}

}


//------------------------------------------------------- MAIN --------------------------------------------------------//


void aruco_picking::pick (void) {

    ros::spinOnce();
    
    if (!ArUco_Check()) {ROS_ERROR("ArUco Marker NOT-DETECTED");}
    else if (ArUco_Check()) {

        ROS_INFO("ArUco Marker DETECTED -> Start Picking");
        Manipulator_Picking();

        //wait gripping time
        ros::Duration(2).sleep();

        Home_Position();

        picking_complete = true;

    }
}
