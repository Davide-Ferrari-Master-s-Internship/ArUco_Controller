#include "aruco_search.h"


//----------------------------------------------------- CONSTRUCTOR -----------------------------------------------------//

aruco_search::aruco_search() {

    //Publisher & Subscriber

    aruco_real_pose_subscriber = nh.subscribe("/aruco_detection/aruco_real_pose", 1000, &aruco_search::Real_Pose_Callback, this);

    prbt_position_reached_subscriber = nh.subscribe("/Robot_Bridge/prbt_Position_Reached", 1000, &aruco_search::Manipulator_Position_Reached_Callback, this);
    prbt_realtime_position_subscriber =  nh.subscribe("/Robot_Bridge/prbt_Current_State_Position", 1000, &aruco_search::Manipulator_Pose_Callback, this);

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


aruco_search::~aruco_search() {

    // delete planner;
    
}


//------------------------------------------------------ CALLBACK ------------------------------------------------------//


void aruco_search::Real_Pose_Callback (const geometry_msgs::Pose::ConstPtr &msg) {

    aruco_real_pose = *msg;
    
    if ((aruco_real_pose.position.x == 0)  && (aruco_real_pose.position.y == 0) && (aruco_real_pose.position.z == 0)) {aruco_detected = false;}
    else {aruco_detected = true;}

}


void aruco_search::Manipulator_Pose_Callback (const control_msgs::JointTrajectoryControllerState::ConstPtr &msg) {

    manipulator_current_position = *msg;

}


void aruco_search::Manipulator_Position_Reached_Callback (const std_msgs::Bool::ConstPtr &msg) {

    position_reached = *msg;

}


//------------------------------------------------ PLANNER FUNCTIONS ------------------------------------------------//


void aruco_search::Home_Position (void) {

    //manipulator to home position
    
    if (get_manipulator_info_client.call(get_manipulator_info_srv)) {final_position = get_manipulator_info_srv.response.home_position;}
    else {ROS_ERROR("Failed to Call Service: \"GetManipulatorInfo\"");}
    
    plan_joint_space_srv.request.final_position = final_position;
    plan_joint_space_srv.request.send = true;
    
    if (plan_joint_space_client.call(plan_joint_space_srv)) {bool planning_succesfull = plan_joint_space_srv.response.response;}
    else {ROS_ERROR("Failed to Call Service: \"PlanJointSpace\"");}

    Wait_For_Position_Reached();

}


void aruco_search::Manipulator_Stop (void) {

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


//------------------------------------------------ INITIAL FUNCTIONS ------------------------------------------------//


void aruco_search::Wait_For_Position_Reached (void) {

    position_reached.data = false;
    while (!position_reached.data) {ros::spinOnce(); /* WAIT TO REACH DESIRED POSITION */}

}


void aruco_search::Wait_For_ArUco_Detection (void) {

    aruco_detected = false;
    position_reached.data = false;

    while (!aruco_detected) {
        
        ros::spinOnce();

        if (position_reached.data) break;

    }

    if (aruco_detected) {

        Manipulator_Stop();
               
        ROS_WARN("ArUco Marker DETECTED");
        aruco_found = true;
        
        ros::spinOnce();

        std::vector<double> actual_position = manipulator_current_position.desired.positions;

        ROS_INFO("Joints Position = [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", actual_position[0], actual_position[1], actual_position[2], actual_position[3], actual_position[4], actual_position[5]);
 
    }

}


void aruco_search::Mobile_Base_Linear_Movement(float x_velocity, float y_velocity, float z_twist, float movement_time) {

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


//-------------------------------------------- ARUCO DETECTION FUNCTIONS ---------------------------------------------//


void aruco_search::ArUco_Manipulator_Search (void) {

    //search ArUco Marker

    ros::spinOnce();
    
    if (get_manipulator_info_client.call(get_manipulator_info_srv)) {final_position = get_manipulator_info_srv.response.home_position;}
    else {ROS_ERROR("Failed to Call Service: \"GetManipulatorInfo\"");}
        
    set_planner_dynamics_srv.request.v_factor = 0.1;
    set_planner_dynamics_srv.request.a_factor = 0.1;    

    if (set_planner_dynamics_client.call(set_planner_dynamics_srv)) {bool planning_succesfull = set_planner_dynamics_srv.response.response;}
    else {ROS_ERROR("Failed to Call Service: \"SetPlannerDynamic\"");}

    int sign = -1;
    int step = 0;
    aruco_detected = false;

    ros::spinOnce();

    while (!aruco_detected) {

        //rotation of webcam using manipulator
        step++;
        sign = sign * -1;

        //go to +- joint1 limits
        if (get_manipulator_info_client.call(get_manipulator_info_srv)) {
            std::vector<float> prbt_joint_limits = get_manipulator_info_srv.response.prbt_joint_limits;
            final_position[0] = prbt_joint_limits[0] * sign;
        } else {ROS_ERROR("Failed to Call Service: \"GetManipulatorInfo\"");}
        
        plan_joint_space_srv.request.final_position = final_position;
        plan_joint_space_srv.request.send = true;
        
        if (plan_joint_space_client.call(plan_joint_space_srv)) {bool planning_succesfull = plan_joint_space_srv.response.response;}
        else {ROS_ERROR("Failed to Call Service: \"PlanJointSpace\"");}

        Wait_For_ArUco_Detection ();

        if ((step == 2) && (!aruco_detected)) {

            //after firs two cycle -> checked left and right one times
            //change joint 2 position (up) and try again

            final_position[1] = 1.2;
            plan_joint_space_srv.request.final_position = final_position;
            plan_joint_space_srv.request.send = true;
        
            if (plan_joint_space_client.call(plan_joint_space_srv)) {bool planning_succesfull = plan_joint_space_srv.response.response;}
            else {ROS_ERROR("Failed to Call Service: \"PlanJointSpace\"");}

            Wait_For_Position_Reached();

        } else if ((step == 3) && (!aruco_detected)) {

            //change joint 2 position (down) and try again

            final_position[1] = 0.2;
            plan_joint_space_srv.request.final_position = final_position;
            plan_joint_space_srv.request.send = true;
        
            if (plan_joint_space_client.call(plan_joint_space_srv)) {bool planning_succesfull = plan_joint_space_srv.response.response;}
            else {ROS_ERROR("Failed to Call Service: \"PlanJointSpace\"");}

            Wait_For_Position_Reached();

        } else if ((step == 4) && (!aruco_detected)) {
    
            ROS_ERROR("ArUco Marker NOT-DETECTED");
            break;

        }

    }

}


//------------------------------------------------------- MAIN --------------------------------------------------------//


void aruco_search::search (void) {

    ros::Duration(2).sleep();

    ros::spinOnce();

    Home_Position();

    // ros::Duration(2).sleep();
    // ros::Duration(10).sleep();

    ArUco_Manipulator_Search();

}


void aruco_search::move_to_search_point (void) {

    ros::spinOnce();

    Home_Position();

    ros::Duration(2).sleep();

    Mobile_Base_Linear_Movement(0.5,0,0,3);
    Mobile_Base_Linear_Movement(0.5,0,1,3);
    Mobile_Base_Linear_Movement(0.5,0,0,2);

}
