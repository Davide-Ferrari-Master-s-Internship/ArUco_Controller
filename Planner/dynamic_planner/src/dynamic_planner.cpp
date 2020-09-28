#include "dynamic_planner/dynamic_planner.h"

dynamic_planner::dynamic_planner(std::string manipulator_name, std::vector<std::string> joints, std::vector<double> initial, double v_f, double a_f): n("~"), planning_group(manipulator_name){

    visual_tools = moveit_visual_tools::MoveItVisualToolsPtr(new moveit_visual_tools::MoveItVisualTools("world"));

    visual_tools->deleteAllMarkers();
    visual_tools->loadRemoteControl();
    visual_tools->trigger();

    joint_pub =  n.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states", 1000, true);
    trajectory_pub = n.advertise<trajectory_msgs::JointTrajectory>("/Robot_Bridge/prbt_Planned_Trajectory", 1000, true);
    stop_pub = n.advertise<std_msgs::Bool>("/stop", 1, true);
    
    trajpoint_sub =  n.subscribe("/Robot_Bridge/prbt_Trajectory_Counter", 2, &dynamic_planner::TrajPointCallback, this);
    joint_names = joints;
    
    //This is necessary to move the robot in an initial vertical configuration like the one in V-Rep
    initial_pose.name = joint_names;
    initial_pose.position.resize(initial.size());
    
    for(int i = 0; i < initial.size(); i++){
        initial_pose.position[i] = initial[i];
    }

    move_robot(initial_pose);
    
    robot_model_loader = robot_model_loader::RobotModelLoader("robot_description");
    robot_model = robot_model_loader.getModel();
    robot_state = robot_state::RobotStatePtr(new robot_state::RobotState(robot_model));
    joint_model_group = robot_state->getJointModelGroup(planning_group);

    robot_state->setJointGroupPositions(joint_model_group, initial_pose.position);
    
    planning_scene = planning_scene::PlanningScenePtr(new planning_scene::PlanningScene(robot_model));
    planning_pipeline = planning_pipeline::PlanningPipelinePtr(new planning_pipeline::PlanningPipeline(robot_model, n, "planning_plugin", "request_adapters"));

    request.group_name = planning_group;
    request.goal_constraints.resize(1);
    robot_state::robotStateToRobotStateMsg(*robot_state, request.start_state);

    vel_factor = v_f;
    acc_factor = a_f;
    num_attempts = 20;
    planning_time = 5;
    
    request.max_velocity_scaling_factor = vel_factor;
    request.max_acceleration_scaling_factor = acc_factor;    

}

dynamic_planner::dynamic_planner(std::string manipulator_name, std::vector<std::string> joints, double v_f, double a_f): n("~"), planning_group(manipulator_name){

    visual_tools = moveit_visual_tools::MoveItVisualToolsPtr(new moveit_visual_tools::MoveItVisualTools("world"));

    visual_tools->deleteAllMarkers();
    visual_tools->loadRemoteControl();
    visual_tools->trigger();

    joint_pub =  n.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states", 1000, true);
    trajectory_pub = n.advertise<trajectory_msgs::JointTrajectory>("/Robot_Bridge/prbt_Planned_Trajectory", 1000, true);
    stop_pub = n.advertise<std_msgs::Bool>("/stop", 1, true);
    
    joint_sub =  n.subscribe("/Robot_Bridge/prbt_Current_State_Position", 2, &dynamic_planner::InitialPoseCallback, this);
    trajpoint_sub =  n.subscribe("/Robot_Bridge/prbt_Trajectory_Counter", 2, &dynamic_planner::TrajPointCallback, this);
    initialized = false;

    joint_names = joints;
    
    //This is necessary to move the robot in an initial vertical configuration like the one in V-Rep
    initial_pose.name = joint_names;
    initial_pose.position.resize(joints.size());
   
    ros::Duration(1).sleep();
    ros::spinOnce();

    move_robot(initial_pose);
    
    robot_model_loader = robot_model_loader::RobotModelLoader("robot_description");
    robot_model = robot_model_loader.getModel();
    robot_state = robot_state::RobotStatePtr(new robot_state::RobotState(robot_model));
    joint_model_group = robot_state->getJointModelGroup(planning_group);

    kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(robot_model));
    kinematic_state->setToDefaultValues();

    robot_state->setJointGroupPositions(joint_model_group, initial_pose.position);

    planning_scene = planning_scene::PlanningScenePtr(new planning_scene::PlanningScene(robot_model));
    planning_pipeline = planning_pipeline::PlanningPipelinePtr(new planning_pipeline::PlanningPipeline(robot_model, n, "planning_plugin", "request_adapters"));

    request.group_name = planning_group;
    request.goal_constraints.resize(1);
    robot_state::robotStateToRobotStateMsg(*robot_state, request.start_state);

    vel_factor = v_f;
    acc_factor = a_f;

    request.max_velocity_scaling_factor = vel_factor;
    request.max_acceleration_scaling_factor = acc_factor;

}

dynamic_planner::~dynamic_planner(){

    // delete visual_tools.get();
    // delete robot_state.get();
    // delete kinematic_state.get();
    // delete planning_scene.get();
    // delete planning_pipeline.get();

    robot_model.reset();
    robot_state.reset();
    kinematic_state.reset();
    planning_scene.reset();

}

void dynamic_planner::InitialPoseCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& joint_state){
    if (!initialized){
        for(int i = 0; i < joint_state->actual.positions.size(); i++){
            initial_pose.position[i] = joint_state->actual.positions[i];
        }
        initialized = true;
    }
}

void dynamic_planner::TrajPointCallback(const std_msgs::Int32::ConstPtr& traj_point){
    trajpoint = traj_point->data;
}

void dynamic_planner::set_planner_param(std::string planner_id, int attempts, double time, double v_factor, double a_factor){
    
    planner_name = planner_id;
    num_attempts = attempts;
    planning_time = time;
    
    if(v_factor != 99.0){
        vel_factor = v_factor;
    }

    if(a_factor != 99.0){
        acc_factor = a_factor;    
    }
}

void dynamic_planner::set_planner_dyn(double v_factor, double a_factor){
    
    if(v_factor != 99.0){
        vel_factor = v_factor;
    }

    if(a_factor != 99.0){
        acc_factor = a_factor;    
    }
}

void dynamic_planner::plan(std::vector<double> final_posit, bool send){

    final_position = final_posit;

    int num_tries = 0;

    request.planner_id = planner_name;
    request.allowed_planning_time = planning_time;
    request.num_planning_attempts = num_attempts;
    request.max_velocity_scaling_factor = vel_factor;
    request.max_acceleration_scaling_factor = acc_factor;        

    robot_state::robotStateToRobotStateMsg(*robot_state, request.start_state);

    robot_state->setJointGroupPositions(joint_model_group, final_position);

    goal = kinematic_constraints::constructGoalConstraints(*robot_state, joint_model_group, 0.01, 0.01);
    
    request.goal_constraints[0] = goal;

    planning_scene::PlanningScenePtr new_planning_scene(new planning_scene::PlanningScene(robot_model));
    for(int i = 0; i < collision_objects.size(); ++i){
        new_planning_scene->processCollisionObjectMsg(collision_objects[i]);
    }

    while(num_tries < 5){

        planning_pipeline->generatePlan(new_planning_scene, request, result);

        if (result.error_code_.val != result.error_code_.SUCCESS){

            ROS_ERROR("Could not compute plan successfully");
            ++num_tries;
            if(num_tries == 5){
                ROS_ERROR("TROPPI TENTATIVIIII");
                success = false;
                std_msgs::Bool stop;
                stop.data = true;
                stop_pub.publish(stop);
            }

        } else {

            result.trajectory_->getRobotTrajectoryMsg(trajectory);

            if(send)
            {
                trajectory_pub.publish(trajectory.joint_trajectory);
            }

            visual_tools->deleteAllMarkers();
            visual_tools->publishTrajectoryLine(result.trajectory_, joint_model_group);
            visual_tools->trigger();
            num_tries = 0;
            success = true;
            break;
        }
    } 

}

void dynamic_planner::plan(geometry_msgs::PoseStamped final_pos, std::string link_name, bool send){

    if(final_pos.header.frame_id == ""){
        final_pos.header.frame_id = "world";
    }

    final_pose = final_pos;

    int num_tries = 0;

    request.planner_id = planner_name;
    request.allowed_planning_time = planning_time;
    request.num_planning_attempts = num_attempts;
    request.max_velocity_scaling_factor = vel_factor;
    request.max_acceleration_scaling_factor = acc_factor;

    robot_state::robotStateToRobotStateMsg(*robot_state, request.start_state);

    goal = kinematic_constraints::constructGoalConstraints(link_name, final_pose, 0.001, 0.001);
    
    request.goal_constraints[0] = goal;

    planning_scene::PlanningScenePtr new_planning_scene(new planning_scene::PlanningScene(robot_model));
    for(int i = 0; i < collision_objects.size(); ++i){
        new_planning_scene->processCollisionObjectMsg(collision_objects[i]);
    }

    while(num_tries < 5){

        planning_pipeline->generatePlan(new_planning_scene, request, result);

        if (result.error_code_.val != result.error_code_.SUCCESS){

            ROS_ERROR("Could not compute plan successfully");
            ++num_tries;
            if(num_tries == 5){
                ROS_ERROR("TROPPI TENTATIVIIII");
                success = false;
            }

        } else {

            result.trajectory_->getRobotTrajectoryMsg(trajectory);
            robot_state->setJointGroupPositions(joint_model_group, trajectory.joint_trajectory.points.back().positions);

            if(send)
            {
                trajectory_pub.publish(trajectory.joint_trajectory);
            }

            visual_tools->deleteAllMarkers();
            visual_tools->publishTrajectoryLine(result.trajectory_, joint_model_group);
            visual_tools->trigger();
            num_tries = 0;
            success = true;
            break;
        }
    }

    check_trajectory(trajectory, trajectory.joint_trajectory.points.size() - 2); 

}

void dynamic_planner::move_robot(sensor_msgs::JointState joint_states){

    joint_pub.publish(joint_states);
}

void dynamic_planner::move_robot(moveit_msgs::RobotTrajectory robot_trajectory){

    sensor_msgs::JointState trajectory_pose;

    trajectory_pose.name = robot_trajectory.joint_trajectory.joint_names;

    for(int i = 0; i < robot_trajectory.joint_trajectory.points.size(); ++i)
    {
        trajectory_pose.position = robot_trajectory.joint_trajectory.points[i].positions;
        move_robot(trajectory_pose);
        ros::Duration(0.02).sleep();
    }
}

void dynamic_planner::check_trajectory(moveit_msgs::RobotTrajectory robot_trajectory, int trajstate){

    int size = robot_trajectory.joint_trajectory.points.size();
    int bound = std::min(size/3, size-trajstate);
    
    for (int i = trajstate; i < trajstate+bound; ++i)
    {
        robot_state->setJointGroupPositions(joint_model_group, robot_trajectory.joint_trajectory.points[i].positions);
        if(planning_scene->isStateColliding(*robot_state, planning_group, false))
        {
            invalid_state = i;
            int k = std::min(invalid_state-trajstate, 30);
            robot_trajectory.joint_trajectory.points.erase(robot_trajectory.joint_trajectory.points.begin()+invalid_state-k, robot_trajectory.joint_trajectory.points.end());
            robot_state->setJointGroupPositions(joint_model_group, robot_trajectory.joint_trajectory.points[robot_trajectory.joint_trajectory.points.size()-1].positions);
            success = false;
            plan(final_position, false);
            if(success){
                merge(robot_trajectory); 
            }
            break;
        }
    }
}

void dynamic_planner::check_trajectory(moveit_msgs::RobotTrajectory robot_trajectory, int trajstate, std::string link_name){

    int size = robot_trajectory.joint_trajectory.points.size();
    int bound = std::min(size/3, size-trajstate);
    
    for (int i = trajstate; i < trajstate+bound; ++i)
    {
        robot_state->setJointGroupPositions(joint_model_group, robot_trajectory.joint_trajectory.points[i].positions);
        if(planning_scene->isStateColliding(*robot_state, planning_group, false))
        {
            invalid_state = i;
            int k = std::min(invalid_state-trajstate, 30);
            robot_trajectory.joint_trajectory.points.erase(robot_trajectory.joint_trajectory.points.begin()+invalid_state-k, robot_trajectory.joint_trajectory.points.end());
            robot_state->setJointGroupPositions(joint_model_group, robot_trajectory.joint_trajectory.points[robot_trajectory.joint_trajectory.points.size()-1].positions);
            success = false;
            plan(final_pose, link_name, false);
            if(success){
                merge(robot_trajectory); 
            }
            break;

        }
    }
}

void dynamic_planner::merge(moveit_msgs::RobotTrajectory robot_trajectory){

    if(robot_trajectory.joint_trajectory.points.size() < trajectory.joint_trajectory.points.size())
    {
        for(int k = 0; k < robot_trajectory.joint_trajectory.points.size(); ++k)
        {
            trajectory.joint_trajectory.points.insert(trajectory.joint_trajectory.points.begin(),robot_trajectory.joint_trajectory.points[robot_trajectory.joint_trajectory.points.size()-(1+k)]);
        }
    } else {
        for(int k = 0; k < trajectory.joint_trajectory.points.size(); ++k)
        {
            robot_trajectory.joint_trajectory.points.push_back(trajectory.joint_trajectory.points[k]);
        }
        trajectory = robot_trajectory;
    }
    trajectory_pub.publish(trajectory.joint_trajectory);

    visual_tools->deleteAllMarkers();
    visual_tools->publishTrajectoryLine(trajectory, joint_model_group);
    visual_tools->trigger(); 
}

geometry_msgs::Pose dynamic_planner::compute_end_effector_pose (void) {

    //joint_names, joint_model_group, robot_model -> già nel planner

    geometry_msgs::Pose ee_position;
    std::vector<std::string> prbt_link_names = {"prbt_link_1", "prbt_link_2", "prbt_link_3", "prbt_link_4", "prbt_link_5", "prbt_flange"};

    //link names
    std::vector<std::string> model_link_names = joint_model_group->getLinkModelNames();
    // for(std::size_t i = 0; i < model_link_names.size(); ++i) {ROS_INFO("Joint %d: %s", i + 1, model_link_names[i].c_str());}

    //get the current joint state
    robot_state->copyJointGroupPositions(joint_model_group, joint_kinematic_values);
    // kinematic_state->getJointPositions();
    //print joint values
    //for(std::size_t i = 0; i < joint_names.size(); ++i) {ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_kinematic_values[i]);}
    
    //forward kinematic
    const Eigen::Affine3d &end_effector_state = robot_state->getGlobalLinkTransform(model_link_names[5]);

    /* Print end-effector pose. Remember that this is in the model frame */
    // ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
    // ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());

    Eigen::Matrix3d rot = end_effector_state.rotation();
    Eigen::Quaterniond quat(rot);

    //position in meters (robot lenght = 1m)
    ee_position.position.x = end_effector_state.translation().x();
    ee_position.position.y = end_effector_state.translation().y();
    ee_position.position.z = end_effector_state.translation().z();
    
    ee_position.orientation.x = quat.x();
    ee_position.orientation.y = quat.y();
    ee_position.orientation.z = quat.z();
    ee_position.orientation.w = quat.w();

    return ee_position;

}

void dynamic_planner::spinner(){

    ros::spinOnce();
    return;

}

