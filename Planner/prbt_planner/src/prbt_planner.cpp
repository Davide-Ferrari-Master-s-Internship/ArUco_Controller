#include "prbt_planner.h"

//----------------------------------------------------- CONSTRUCTOR -----------------------------------------------------//

prbt_planner_class::prbt_planner_class () {

  //define manipulator parameters (dynamic planner)

    manipulator_name = "manipulator";
    manipulator_joint_names = {"prbt_joint_1", "prbt_joint_2", "prbt_joint_3", "prbt_joint_4", "prbt_joint_5", "prbt_joint_6"};
    prbt_trajectory.joint_names = {"prbt_joint_1", "prbt_joint_2", "prbt_joint_3", "prbt_joint_4", "prbt_joint_5", "prbt_joint_6"};
    prbt_trajectory.points.resize(1);

    //LINK BASE      ->   "prbt_base_link" = "prbt_base" = "world"
    //END-EFFECTOR   ->   "prbt_flange" = "prbt_tcp" = "prbt_tool0"

    prbt_link_names.world = "world";
    prbt_link_names.base = "prbt_base_link";
    prbt_link_names.links = {"prbt_link_1", "prbt_link_2", "prbt_link_3", "prbt_link_4", "prbt_link_5"};
    prbt_link_names.end_effector = "prbt_tcp";
    
    dyn_planner = new dynamic_planner(manipulator_name, manipulator_joint_names, 0.5, 0.5);

  //define manipulator obstacles (dynamic planner)

    //prbt position on mobile base      ->   x = 24.75, y = 0, z = 0 [cm]
    //pose of the center of mobile base in respect of prbt_base [m]
    float mobile_base_pose[3] = {0.2475, 0, -0.1925}, mobile_base_orientation[4] = {0,0,0,1};

    //MPO_500 base dimension            ->   x = 77,  y = 52, z = 38.5 (from floor) [cm]
    //MPO_500 base dimension [SAFETY]   ->   x = 100, y = 80, z = 38.5 (from floor) [cm]
    Obstacle_Pose(&mobile_base, prbt_link_names.base, "mobile_base", mobile_base_pose, mobile_base_orientation);
    Obstacle_Dimension(&mobile_base, 1, 0.8, 0.384);

    //pose of the center of floor in respect of prbt_base [m]
    float floor_pose[3] = {0, 0, -0.384 - 0.04}, floor_orientation[4] = {0,0,0,1};

    //FLOOR dimension [SAFETY]   ->   x = 400, y = 400, z = 8 [cm]
    Obstacle_Pose(&floor, prbt_link_names.base, "floor", floor_pose, floor_orientation);
    Obstacle_Dimension(&floor, 4, 4, 0.08);

    //pose of the center of contoller cabinet in respect of prbt_base [m]
    //float cabinet_pose[3] = {0.4325, 0, 0.35}, cabinet_orientation[4] = {0,0,0,1};

    //CABINET dimension [SAFETY]   ->   x = 40, y = 60, z = 70 [cm]
    // Obstacle_Pose(&controller_cabinet, prbt_link_names.base, "controller_cabinet", cabinet_pose, cabinet_orientation);
    // Obstacle_Dimension(&controller_cabinet, 0.4, 0.6, 0.7);

    dyn_planner -> collision_objects.resize(2);
    dyn_planner -> collision_objects[0] = mobile_base;
    dyn_planner -> collision_objects[1] = floor;
    // dyn_planner -> collision_objects[2] = controller_cabinet;

    dyn_planner -> planning_scene_interface.applyCollisionObjects(dyn_planner -> collision_objects);

    for(int i = 0; i < dyn_planner -> collision_objects.size(); i++) {
        dyn_planner -> planning_scene -> processCollisionObjectMsg(dyn_planner -> collision_objects[i]);
    }

  //Service Servers

    plan_joint_space_server = nh.advertiseService("/prbt_planner/plan_joint_space_service", &prbt_planner_class::Plan_Joint_Space_Callback, this);
    plan_cartesian_space_server = nh.advertiseService("/prbt_planner/plan_cartesian_space_service", &prbt_planner_class::Plan_Cartesian_Space_Callback, this);
    set_planner_parameters_server = nh.advertiseService("/prbt_planner/set_planner_parameters_service", &prbt_planner_class::Set_Planner_Parameters_Callback, this);
    set_planner_dynamics_server = nh.advertiseService("/prbt_planner/set_planner_dynamics_service", &prbt_planner_class::Set_Planner_Dynamics_Callback, this);
    compute_end_effector_position_server = nh.advertiseService("/prbt_planner/compute_end_effector_position_service", &prbt_planner_class::Compute_End_Effector_Position_Callback, this);
    get_manipulator_info_server = nh.advertiseService("/prbt_planner/get_manipulator_info_service", &prbt_planner_class::Get_Manipulator_Info_Callback, this);
    get_manipulator_link_names_server = nh.advertiseService("/prbt_planner/get_manipulator_link_names_service", &prbt_planner_class::Get_Manipulator_Link_Names_Callback, this);
    get_manipulator_trajectory_server = nh.advertiseService("/prbt_planner/get_manipulator_trajectory_service", &prbt_planner_class::Get_Manipulator_Trajectory_Callback, this);

}


prbt_planner_class::~prbt_planner_class() {

    delete dyn_planner;

}


//----------------------------------------------------- SERVICES CALLBACK -----------------------------------------------------//


bool prbt_planner_class::Plan_Joint_Space_Callback(prbt_planner::PlanJointSpace::Request &req, prbt_planner::PlanJointSpace::Response &res) {

    std::vector<double> final_position = req.final_position;
    ROS_INFO("Final Position: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f",final_position[0],final_position[1],final_position[2],final_position[3],final_position[4],final_position[5]);
    bool send = req.send;

    plan_joint_space(final_position, send);

    res.response = true;

    return true;

}


bool prbt_planner_class::Plan_Cartesian_Space_Callback(prbt_planner::PlanCartesianSpace::Request &req, prbt_planner::PlanCartesianSpace::Response &res) {

    geometry_msgs::PoseStamped final_pose = req.final_pose;
    std::string link_name = req.link_name;
    bool send = req.send;

    plan_cartesian_space(final_pose, link_name, send);

    res.response = true;

    return true;

}


bool prbt_planner_class::Set_Planner_Parameters_Callback(prbt_planner::SetPlannerParameters::Request &req, prbt_planner::SetPlannerParameters::Response &res) {

    std::string planner_id = req.planner_id;
    int attempts = req.attempts;
    double time = req.time;
    double v_factor = req.v_factor;
    double a_factor = req.a_factor;

    set_planner_parameters(planner_id, attempts, time, v_factor, a_factor);

    res.response = true;

    return true;

}


bool prbt_planner_class::Set_Planner_Dynamics_Callback(prbt_planner::SetPlannerDynamics::Request &req, prbt_planner::SetPlannerDynamics::Response &res) {
    
    double v_factor = req.v_factor;
    double a_factor = req.a_factor;

    set_planner_dynamic(v_factor, a_factor);

    res.response = true;

    return true;

}


bool prbt_planner_class::Compute_End_Effector_Position_Callback(prbt_planner::ComputeEndEffectorPosition::Request &req, prbt_planner::ComputeEndEffectorPosition::Response &res) {

    bool request = req.request;

    geometry_msgs::Pose ee_position = compute_end_effector_position();

    res.ee_position = ee_position;

    return true;

}


bool prbt_planner_class::Get_Manipulator_Info_Callback(prbt_planner::GetManipulatorInfo::Request &req, prbt_planner::GetManipulatorInfo::Response &res) {

    bool request = req.request;
    
    res.manipulator_name = manipulator_name;
    res.manipulator_joint_names = manipulator_joint_names;
    res.home_position = home_position;
    res.prbt_joint_limits = prbt_joint_limits;

    return true;
}


bool prbt_planner_class::Get_Manipulator_Link_Names_Callback(prbt_planner::GetManipulatorLinkNames::Request &req, prbt_planner::GetManipulatorLinkNames::Response &res) {

    bool request = req.request;

    res.base = prbt_link_names.base;
    res.end_effector = prbt_link_names.end_effector;
    res.links = prbt_link_names.links;
    res.world = prbt_link_names.world;

    return true;

}


bool prbt_planner_class::Get_Manipulator_Trajectory_Callback(prbt_planner::GetManipulatorTrajectory::Request &req, prbt_planner::GetManipulatorTrajectory::Response &res) {

    bool request = req.request;

    res.prbt_trajectory = prbt_trajectory;

    return true;
    
}


//------------------------------------------------ OBSTACLE CREATION FUNCTIONS ------------------------------------------------//


void prbt_planner_class::Obstacle_Pose (moveit_msgs::CollisionObject *object, std::string reference_id, std::string object_id, float position[3], float orientation[4]) {

    object -> header.frame_id = reference_id;
    object -> id = object_id;
    object -> primitives.resize(1);
    object -> primitives[0].type = 1;
    
    object -> primitive_poses.resize(1);
    object -> primitive_poses[0].position.x = position[0];
    object -> primitive_poses[0].position.y = position[1];
    object -> primitive_poses[0].position.z = position[2];  
    
    object -> primitive_poses[0].orientation.x = orientation[0];   
    object -> primitive_poses[0].orientation.y = orientation[1];   
    object -> primitive_poses[0].orientation.z = orientation[2];   
    object -> primitive_poses[0].orientation.w = orientation[3];  
 
    object -> operation = 0;
}


void prbt_planner_class::Obstacle_Dimension (moveit_msgs::CollisionObject *object, float x, float y, float z) {

    object -> primitives[0].dimensions.resize(3);
    object -> primitives[0].dimensions[0] = x;
    object -> primitives[0].dimensions[1] = y;
    object -> primitives[0].dimensions[2] = z;

}


//------------------------------------------------ DYNAMIC PLANNER FUNCTIONS ------------------------------------------------//


void prbt_planner_class::plan_joint_space(std::vector<double> final_position, bool send) {

    dyn_planner -> plan (final_position, send);

}


void prbt_planner_class::plan_cartesian_space(geometry_msgs::PoseStamped final_pose, std::string link_name, bool send) {

    dyn_planner -> plan (final_pose, link_name, send);

}


void prbt_planner_class::set_planner_parameters(std::string planner_id, int attempts, double time, double v_factor, double a_factor) {

    dyn_planner -> set_planner_param(planner_id, attempts, time, v_factor, a_factor);

}


void prbt_planner_class::set_planner_dynamic(double v_factor, double a_factor) {

    dyn_planner -> set_planner_dyn(v_factor, a_factor);

}


geometry_msgs::Pose prbt_planner_class::compute_end_effector_position (void) {

    geometry_msgs::Pose ee_position = dyn_planner -> compute_end_effector_pose();

    return (ee_position);

}


void prbt_planner_class::spinner (void) {

    ros::spinOnce();

}
