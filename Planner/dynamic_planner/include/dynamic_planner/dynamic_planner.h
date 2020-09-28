#ifndef MOTION_PLANNING_H
#define MOTION_PLANNING_H

#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <control_msgs/JointTrajectoryControllerState.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/collision_detection/collision_tools.h>
#include "moveit/trajectory_processing/iterative_spline_parameterization.h"


class dynamic_planner
{
	public:
		dynamic_planner(std::string manipulator_name, std::vector<std::string> joints, std::vector<double> initial, double vel_factor = 0.2, double acc_factor = 0.2);
		dynamic_planner(std::string manipulator_name, std::vector<std::string> joints, double vel_factor = 0.2, double acc_factor = 0.2);
		~dynamic_planner();
		void spinner(void);
		void plan(std::vector<double> final_position, bool send = true);
		void plan(geometry_msgs::PoseStamped final_pose, std::string link_name, bool send = true);
		void move_robot(sensor_msgs::JointState joint_states);
		void move_robot(moveit_msgs::RobotTrajectory robot_trajectory);
		void check_trajectory(moveit_msgs::RobotTrajectory robot_trajectory, int trajstate);
		void check_trajectory(moveit_msgs::RobotTrajectory robot_trajectory, int trajstate, std::string link_name);
		void set_planner_param(std::string planner_id = "", int attempts = 0, double time = 0, double v_factor = 99.0, double a_factor = 99.0);
		void set_planner_dyn(double v_factor = 99.0, double a_factor = 99.0);

		geometry_msgs::Pose compute_end_effector_pose (void);

		moveit_visual_tools::MoveItVisualToolsPtr visual_tools;
		moveit_msgs::RobotTrajectory trajectory;

		std::vector<moveit_msgs::CollisionObject> collision_objects;
		moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
		planning_scene::PlanningScenePtr planning_scene;
		robot_model::RobotModelPtr robot_model;
		int trajpoint;		

	private:
		ros::NodeHandle n;
		ros::Publisher joint_pub;
		ros::Publisher trajectory_pub;
		ros::Publisher stop_pub;
		ros::Subscriber joint_sub;
		ros::Subscriber trajpoint_sub;

		void InitialPoseCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& joint_state);
		void TrajPointCallback(const std_msgs::Int32::ConstPtr& traj_point);

		sensor_msgs::JointState initial_pose;
		std::vector<std::string> joint_names;
		
		void check_all_trajectory(moveit_msgs::RobotTrajectory robot_trajectory);
		void merge(moveit_msgs::RobotTrajectory robot_trajectory);

		moveit_msgs::CollisionObject table;
		moveit_msgs::CollisionObject obstacle;
		
		robot_model_loader::RobotModelLoader robot_model_loader;		
		robot_state::RobotStatePtr robot_state, kinematic_state;
		const robot_model::JointModelGroup* joint_model_group;	
		std::vector<double> joint_kinematic_values;	
		
		std::string planning_group;

		std::vector<double> final_position;
		geometry_msgs::PoseStamped final_pose;

		planning_pipeline::PlanningPipelinePtr planning_pipeline;
		planning_interface::MotionPlanRequest request;
		planning_interface::MotionPlanResponse result;
		moveit_msgs::Constraints goal;

		std::string planner_name;
		int num_attempts;
		double planning_time;
		double vel_factor;
		double acc_factor;

		int invalid_state;
		bool success;
		bool initialized;

};

#endif /* MOTION_PLANNING_H */
