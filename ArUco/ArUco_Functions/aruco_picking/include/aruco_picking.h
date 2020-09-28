#ifndef ARUCO_PICKING_H
#define ARUCO_PICKING_H

#include "ros/ros.h"

#include "prbt_planner.h"
#include "useful_functions.h"

#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "trajectory_msgs/JointTrajectory.h"

#include "prbt_planner/PlanJointSpace.h"
#include "prbt_planner/PlanCartesianSpace.h"
#include "prbt_planner/SetPlannerParameters.h"
#include "prbt_planner/SetPlannerDynamics.h"
#include "prbt_planner/ComputeEndEffectorPosition.h"
#include "prbt_planner/GetManipulatorInfo.h"
#include "prbt_planner/GetManipulatorLinkNames.h"
#include "prbt_planner/GetManipulatorTrajectory.h"

class aruco_picking {

    public:

        aruco_picking();
        ~aruco_picking();

        bool aruco_found = false;
        bool picking_complete = false;
        void pick (void);

    private:

        ros::NodeHandle nh;

        ros::Subscriber aruco_real_pose_subscriber, manipulator_position_reached_subscriber, 
                        manipulator_realtime_position_subscriber;

        ros::ServiceClient plan_joint_space_client;
        ros::ServiceClient plan_cartesian_space_client;
        ros::ServiceClient set_planner_parameters_client;
        ros::ServiceClient set_planner_dynamics_client;
        ros::ServiceClient compute_end_effector_position_client;
        ros::ServiceClient get_manipulator_info_client;
        ros::ServiceClient get_manipulator_link_names_client;
        ros::ServiceClient get_manipulator_trajectory_client;
        
        prbt_planner::PlanJointSpace plan_joint_space_srv;
        prbt_planner::PlanCartesianSpace plan_cartesian_space_srv;
        prbt_planner::SetPlannerParameters set_planner_parameters_srv;
        prbt_planner::SetPlannerDynamics set_planner_dynamics_srv;
        prbt_planner::ComputeEndEffectorPosition compute_end_effector_position_srv;
        prbt_planner::GetManipulatorInfo get_manipulator_info_srv;
        prbt_planner::GetManipulatorLinkNames get_manipulator_link_names_srv;
        prbt_planner::GetManipulatorTrajectory get_manipulator_trajectory_srv;

        geometry_msgs::Pose aruco_real_pose;
        control_msgs::JointTrajectoryControllerState manipulator_current_position;

        bool aruco_detected = false;

        std_msgs::Bool position_reached;
        std::vector<double> final_position;

		// prbt_planner *planner;

        void Real_Pose_Callback (const geometry_msgs::Pose::ConstPtr &);
        void Manipulator_Pose_Callback (const control_msgs::JointTrajectoryControllerState::ConstPtr &);
        void Manipulator_Position_Reached_Callback (const std_msgs::Bool::ConstPtr &);

        void Home_Position (void);
        bool ArUco_Check (void);
        void Wait_For_Position_Reached (void);

        geometry_msgs::Pose Compute_ArUco_Position_Respect_Manipulator (void);
        bool Check_Manipulator_Workspace (geometry_msgs::Pose position);
        void Manipulator_Picking (void);

};

#endif /* ARUCO_PICKING_H */