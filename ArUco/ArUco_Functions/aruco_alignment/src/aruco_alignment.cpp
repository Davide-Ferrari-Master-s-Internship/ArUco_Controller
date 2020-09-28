#include "aruco_alignment.h"


//----------------------------------------------------- CONSTRUCTOR -----------------------------------------------------//

aruco_alignment::aruco_alignment() {

    // scale factor camera parameters
    nh.param("/ArUco_Detection_Node/scale_factors/scale_x", SCALE_X, 1.0);   //1.5
    nh.param("/ArUco_Detection_Node/scale_factors/scale_y", SCALE_Y, 1.0);   //2.7
    nh.param("/ArUco_Detection_Node/scale_factors/scale_z", SCALE_Z, 1.0);   //1.96

    //Publisher & Subscriber

    aruco_real_pose_subscriber = nh.subscribe("/aruco_detection/aruco_real_pose", 1000, &aruco_alignment::Real_Pose_Callback, this);
    aruco_measured_pose_subscriber = nh.subscribe("/aruco_detection/aruco_measured_pose", 1000, &aruco_alignment::Measured_Pose_Callback, this);

    manipulator_position_reached_subscriber = nh.subscribe("/Robot_Bridge/prbt_Position_Reached", 1000, &aruco_alignment::Manipulator_Position_Reached_Callback, this);
    manipulator_realtime_position_subscriber =  nh.subscribe("/Robot_Bridge/prbt_Current_State_Position", 1000, &aruco_alignment::Manipulator_Pose_Callback, this);

    manipulator_trajectory_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("/Robot_Bridge/prbt_Planned_Trajectory", 1000);
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


aruco_alignment::~aruco_alignment() {

    // delete planner;

}


//------------------------------------------------------ CALLBACK ------------------------------------------------------//


void aruco_alignment::Real_Pose_Callback (const geometry_msgs::Pose::ConstPtr &msg) {

    aruco_real_pose = *msg;
    
    if ((aruco_real_pose.position.x == 0)  && (aruco_real_pose.position.y == 0) && (aruco_real_pose.position.z == 0)) {aruco_detected = false;}
    else {aruco_detected = true;}

}


void aruco_alignment::Measured_Pose_Callback (const geometry_msgs::Pose::ConstPtr &msg) {

    aruco_measured_pose = *msg;

}


void aruco_alignment::Manipulator_Pose_Callback (const control_msgs::JointTrajectoryControllerState::ConstPtr &msg) {

    manipulator_current_position = *msg;

}


void aruco_alignment::Manipulator_Position_Reached_Callback (const std_msgs::Bool::ConstPtr &msg) {

    position_reached = *msg;

}


//---------------------------------------------- MANIPULATOR FUNCTIONS ----------------------------------------------//


void aruco_alignment::Manipulator_Stop (void) {

    trajectory_msgs::JointTrajectory actual_position;
    actual_position.points.resize(1);

    actual_position.joint_names = manipulator_current_position.joint_names;
    actual_position.points[0].positions = manipulator_current_position.desired.positions;
    actual_position.points[0].time_from_start = ros::Duration(1);

    manipulator_trajectory_publisher.publish(actual_position);

    final_position = actual_position.points[0].positions;
    plan_joint_space_srv.request.final_position = final_position;
    plan_joint_space_srv.request.send = false;
    
    if (plan_joint_space_client.call(plan_joint_space_srv)) {bool planning_succesfull = plan_joint_space_srv.response.response;}
    else {ROS_ERROR("Failed to Call Service: \"PlanJointSpace\"");}

}


void aruco_alignment::Wait_For_Position_Reached (void) {

    position_reached.data = false;
    while (!position_reached.data) {ros::spinOnce(); /* WAIT TO REACH DESIRED POSITION */}

}


void aruco_alignment::Wait_For_ArUco_Centering (float centring_tol, std::string robot_name) {

    position_reached.data = false;

    ros::spinOnce();

    while (!aruco_centered) {

        aruco_centered = ArUco_Alignment_Check(centring_tol);

        if (robot_name == "manipulator") {if (position_reached.data) break;}
        else if (robot_name == "mobile_base") {if ((ros::Time::now() - timer).toSec() >= max_moving_time) break;}
    
    }

    if (aruco_centered) {

        if (robot_name == "manipulator") {Manipulator_Stop(); ROS_INFO("Joint 1 Position = %.2f", manipulator_current_position.actual.positions[0]);}
        else if (robot_name == "mobile_base") {Mobile_Base_Stop();}

        ROS_WARN("ArUco Marker CENTERED");
        
    } else {ROS_ERROR("ArUco Marker NOT-CENTERED");}

}


//------------------------------------------------ ARUCO FUNCTIONS ------------------------------------------------//


bool aruco_alignment::ArUco_Check (void) {

    aruco_detected = false;

    ros::Duration(1).sleep();

    ros::spinOnce();

    if (aruco_detected) {aruco_found = true; return true;}
    else return false;

}


bool aruco_alignment::ArUco_Alignment_Check (float tol_percentage) {

    ros::spinOnce();

    //compute normalized pose of ArUco Marker (normalized on distance z)
    // geometry_msgs::Pose norm_pose = ArUco_Pose_Normalization(aruco_real_pose, aruco_measured_pose);

    //compute x position tolerance in function of z distance
    float position_tolerance = fabs((tol_percentage / 100) * aruco_measured_pose.position.z);

    if (fabs(aruco_real_pose.position.x) >= position_tolerance) {return false;}
    else {return true;}

}


/* geometry_msgs::Pose aruco_alignment::ArUco_Pose_Normalization (geometry_msgs::Pose real_pose, geometry_msgs::Pose measured_pose) {

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


//-------------------------------------------- ARUCO ALIGNMENT FUNCTIONS ---------------------------------------------//


void aruco_alignment::ArUco_Manipulator_Centering (float centring_tolerance) {

    ros::spinOnce();

    aruco_centered = false;
    
    final_position = manipulator_current_position.actual.positions;
    ROS_INFO("Joints Position = [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", final_position[0], final_position[1], final_position[2], final_position[3], final_position[4], final_position[5]);

    //compute normalized pose of ArUco Marker (normalized on distance z)
    /* geometry_msgs::Pose normalized_pose;
    normalized_pose = ArUco_Pose_Normalization(aruco_real_pose, aruco_measured_pose);
    ROS_INFO("Normalized Pose X: %.2f", normalized_pose.position.x);

    //compute rotation of joint 1 in respect to ArUco Marker normalized position
    if (normalized_pose.position.x > centring_tolerance) {
        
        //aruco dx -> rotate dx (negative)
        if (get_manipulator_info_client.call(get_manipulator_info_srv)) {
            std::vector<float> prbt_joint_limits = get_manipulator_info_srv.response.prbt_joint_limits;
            final_position[0] = -prbt_joint_limits[0];
        } else {ROS_ERROR("Failed to Call Service: \"GetManipulatorInfo\"");}
        
    } else if (normalized_pose.position.x < -centring_tolerance) {

        //aruco sx -> rotate sx (positive)  
        if (get_manipulator_info_client.call(get_manipulator_info_srv)) {
            std::vector<float> prbt_joint_limits = get_manipulator_info_srv.response.prbt_joint_limits;
            final_position[0] = prbt_joint_limits[0];
        } else {ROS_ERROR("Failed to Call Service: \"GetManipulatorInfo\"");}
        
    } else {aruco_centered = true;} */

    //compute x position tolerance in function of z distance
    float position_tolerance = fabs((centring_tolerance / 100) * aruco_measured_pose.position.z);

    //compute rotation of joint 1 in respect to ArUco Marker position
    if (aruco_measured_pose.position.x > position_tolerance) {
        
        //aruco dx respect origin -> rotate dx (negative)
        if (get_manipulator_info_client.call(get_manipulator_info_srv)) {
            std::vector<float> prbt_joint_limits = get_manipulator_info_srv.response.prbt_joint_limits;
            final_position[0] = -prbt_joint_limits[0];
        } else {ROS_ERROR("Failed to Call Service: \"GetManipulatorInfo\"");}
        
    } else if (aruco_measured_pose.position.x < -position_tolerance) {

        //aruco sx respect origin -> rotate sx (positive)  
        if (get_manipulator_info_client.call(get_manipulator_info_srv)) {
            std::vector<float> prbt_joint_limits = get_manipulator_info_srv.response.prbt_joint_limits;
            final_position[0] = prbt_joint_limits[0];
        } else {ROS_ERROR("Failed to Call Service: \"GetManipulatorInfo\"");}
        
    } else {aruco_centered = true;}

    //plan and move the manipulator

    set_planner_dynamics_srv.request.v_factor = 0.05;
    set_planner_dynamics_srv.request.a_factor = 0.1;    

    if (set_planner_dynamics_client.call(set_planner_dynamics_srv)) {bool planning_succesfull = set_planner_dynamics_srv.response.response;}
    else {ROS_ERROR("Failed to Call Service: \"SetPlannerDynamic\"");}

    plan_joint_space_srv.request.final_position = final_position;
    plan_joint_space_srv.request.send = true;

    if (plan_joint_space_client.call(plan_joint_space_srv)) {bool planning_succesfull = plan_joint_space_srv.response.response;}
    else {ROS_ERROR("Failed to Call Service: \"PlanJointSpace\"");}
    
    Wait_For_ArUco_Centering(centring_tolerance, "manipulator");

}


//------------------------------------------ MOBILE BASE MOVEMENT FUNCTIONS ------------------------------------------//


void aruco_alignment::Mobile_Base_Linear_Movement(float x_velocity, float y_velocity, float z_twist, float movement_time) {

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


void aruco_alignment::Mobile_Base_Stop (void) {

    geometry_msgs::Twist cmd_vel;

    //stop the movement
    cmd_vel_linear(&cmd_vel, 0.0, 0.0, 0.0);
    cmd_vel_angular(&cmd_vel, 0.0, 0.0, 0.0);
    mobile_base_velocity_publisher.publish(cmd_vel);

}


void aruco_alignment::Mobile_Base_Rotate_To_ArUco (void) {

    //rotate mobile base in front of the aruco marker

    ros::spinOnce();

    float z_twist_vel = 0.5;

    std::vector<double> home_position; // {0.08, 0.75, 2.35, 1.55, 0.00, -0.241};
    if (get_manipulator_info_client.call(get_manipulator_info_srv)) {home_position = get_manipulator_info_srv.response.home_position;}
    else {ROS_ERROR("Failed to Call Service: \"GetManipulatorInfo\"");}

    //get joint 1 position
    float joint1_actual_position = manipulator_current_position.actual.positions[0];
    float joint1_effective_rotation = joint1_actual_position - home_position[0];

    //how many degrees the prbt rotates (Joint 1 Limits = +- 2.967 = +- 170°)
    // 2.967 : 170 = joint1_effective_rotation : rotation_degree°

    // float rotation_degree = fabs(joint1_effective_rotation * 170 / 2.967);
    // float rotation_rad = Degree_TO_Radiant(rotation_degree);
    float rotation_rad = joint1_effective_rotation;

    ROS_INFO("Rotation Radiants: %.2f", rotation_rad);   

    //joint1_effective_rotation > 0 -> manipulator sx rotated     ->     mobile base has to sx rotate (z_twist_vel > 0)
    //joint1_effective_rotation < 0 -> manipulator dx rotated     ->     mobile base has to dx rotate (z_twist_vel < 0)

    //change z_twist_vel sign
    z_twist_vel = z_twist_vel * Sign(joint1_effective_rotation);

    //fixed velocity (z_twist_vel) -> change time (angle = w * time)
    float rotation_time = fabs(rotation_rad / z_twist_vel);
    rotation_time *= 1.8;
    ROS_INFO("Rotation Time: %.2f", rotation_time);

    //rotation of prbt to 0 of joint1
    
    set_planner_dynamics_srv.request.v_factor = 0.5;
    set_planner_dynamics_srv.request.a_factor = 0.5;    

    if (set_planner_dynamics_client.call(set_planner_dynamics_srv)) {bool planning_succesfull = set_planner_dynamics_srv.response.response;}
    else {ROS_ERROR("Failed to Call Service: \"SetPlannerDynamic\"");}

    if (get_manipulator_info_client.call(get_manipulator_info_srv)) {
        final_position = get_manipulator_info_srv.response.home_position;
    } else {ROS_ERROR("Failed to Call Service: \"GetManipulatorInfo\"");}
    
    trajectory_msgs::JointTrajectory prbt_trajectory;

    if (get_manipulator_trajectory_client.call(get_manipulator_trajectory_srv)) {
        prbt_trajectory = get_manipulator_trajectory_srv.response.prbt_trajectory;
    } else {ROS_ERROR("Failed to Call Service: \"GetManipulatorTrajectory\"");}
   
    // prbt_trajectory.points.resize(1);
    prbt_trajectory.points[0].positions = final_position;
    prbt_trajectory.points[0].time_from_start = ros::Duration(rotation_time);
    
    plan_joint_space_srv.request.final_position = final_position;
    plan_joint_space_srv.request.send = false;

    // fake trajectory to the moveit model
    if (plan_joint_space_client.call(plan_joint_space_srv)) {bool planning_succesfull = plan_joint_space_srv.response.response;}
    else {ROS_ERROR("Failed to Call Service: \"PlanJointSpace\"");}

    //publish manipulator trajectory
    manipulator_trajectory_publisher.publish(prbt_trajectory);

    ROS_INFO("Rotation Velocity Z = %.2f", z_twist_vel);

    Mobile_Base_Linear_Movement(0.0, 0.0, z_twist_vel, rotation_time);

    ROS_WARN("Mobile Base Rotated"); 
    
}


void aruco_alignment::Mobile_Base_Translate_To_ArUco (float centring_tolerance) {
    
    //translate mobile base in front of the aruco marker

    float mobile_base_position = 0;
    float y_vel = 0.05;
    aruco_centered = false;

    final_position = manipulator_current_position.actual.positions;
 
    //compute normalized pose of ArUco Marker (normalized on distance z)
    /* geometry_msgs::Pose normalized_pose;
    normalized_pose = ArUco_Pose_Normalization(aruco_real_pose, aruco_measured_pose);
    ROS_INFO("Normalized Pose: %.2f", normalized_pose.position.x);

    //compute rotation of joint 1 in respect to ArUco Marker normalized position
    if (normalized_pose.position.x > centring_tolerance) {mobile_base_position += - 1;}        //aruco dx -> translate dx (negative)
    else if (normalized_pose.position.x < -centring_tolerance) {mobile_base_position += + 1;}  //aruco sx -> translate sx (positive)
    else {aruco_centered = true;} */

    //compute x position tolerance in function of z distance
    float position_tolerance = fabs((centring_tolerance / 100) * aruco_measured_pose.position.z);

    //compute rotation of joint 1 in respect to ArUco Marker normalized position
    if (aruco_measured_pose.position.x > centring_tolerance) {mobile_base_position += - 1;}        //aruco dx -> translate dx (negative)
    else if (aruco_measured_pose.position.x < -centring_tolerance) {mobile_base_position += + 1;}  //aruco sx -> translate sx (positive)
    else {aruco_centered = true;}

    //change y_vel sign
    y_vel = fabs(y_vel) * Sign(mobile_base_position);

    //max movement allowd = 1m
    max_moving_time = fabs(1 / y_vel);
    timer = ros::Time::now();

    geometry_msgs::Twist cmd_vel;

    //publish cmd_vel
    cmd_vel_linear(&cmd_vel, 0.0, y_vel, 0.0);
    cmd_vel_angular(&cmd_vel, 0.0, 0.0, 0.0);
    mobile_base_velocity_publisher.publish(cmd_vel);
    
    Wait_For_ArUco_Centering(centring_tolerance, "mobile_base");

}


//------------------------------------------------------- MAIN --------------------------------------------------------//


void aruco_alignment::align (void) {

    ros::spinOnce();
    
    if (!ArUco_Check()) {ROS_ERROR("ArUco Marker NOT-DETECTED"); }
    else if (ArUco_Check()) {

        ROS_INFO("ArUco Marker Detected -> Start Alignment");
        
        ROS_INFO("Start ArUco Centering");
        // ArUco_Manipulator_Centering(0.05);
        ArUco_Manipulator_Centering(2);
      
        ROS_INFO("Start Mobile Base Rotation to ArUco");
        Mobile_Base_Rotate_To_ArUco();

        //if ArUco Marker is not aligned -> translate mobile base
        if (!ArUco_Alignment_Check(2)) {
            
            ROS_INFO("Start Translation to ArUco");
            Mobile_Base_Translate_To_ArUco(2);
        
        }

        aruco_aligned = true;

    }
}


void aruco_alignment::align_manipulator_only (void) {

    ros::spinOnce();
    
    if (!ArUco_Check()) {ROS_ERROR("ArUco Marker NOT-DETECTED"); }
    else if (ArUco_Check()) {

        ROS_INFO("ArUco Marker Detected -> Start Alignment");
        
        ROS_INFO("Start ArUco Centering");
        ArUco_Manipulator_Centering(2);

        aruco_aligned = true;

    }
}