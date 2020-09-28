#ifndef ARUCO_ALIGNMENT_H
#define ARUCO_ALIGNMENT_H

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


class aruco_alignment {

    public:

        aruco_alignment();
        ~aruco_alignment();

        bool aruco_found = false;
        bool aruco_aligned = false;
        void align (void);
        void align_manipulator_only (void);

    private:

        ros::NodeHandle nh;

        ros::Publisher  manipulator_trajectory_publisher, mobile_base_velocity_publisher;
        ros::Subscriber aruco_real_pose_subscriber, aruco_measured_pose_subscriber,
                        manipulator_position_reached_subscriber, manipulator_realtime_position_subscriber;

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

        geometry_msgs::Pose aruco_real_pose, aruco_measured_pose;
        control_msgs::JointTrajectoryControllerState manipulator_current_position;

        bool aruco_detected = false;
        bool aruco_centered = false;

        double SCALE_X, SCALE_Y, SCALE_Z;

        float max_moving_time;
        ros::Time timer;

        std_msgs::Bool position_reached;
        std::vector<double> final_position;

		// prbt_planner *planner;

        void Real_Pose_Callback (const geometry_msgs::Pose::ConstPtr &);
        void Measured_Pose_Callback (const geometry_msgs::Pose::ConstPtr &);
        void Manipulator_Pose_Callback (const control_msgs::JointTrajectoryControllerState::ConstPtr &);
        void Manipulator_Position_Reached_Callback (const std_msgs::Bool::ConstPtr &);

        void Manipulator_Stop (void);
        void Wait_For_Position_Reached (void);
        void Wait_For_ArUco_Centering (float centring_tol, std::string robot_name);

        bool ArUco_Check (void);
        bool ArUco_Alignment_Check (float tol_percentage = 2);

        // bool ArUco_Alignment_Check (float tol = 0.1);
        // geometry_msgs::Pose ArUco_Pose_Normalization (geometry_msgs::Pose real_pose, geometry_msgs::Pose measured_pose);
        
        void ArUco_Manipulator_Centering (float centring_tolerance = 2);

        void Mobile_Base_Linear_Movement(float x_vel, float y_vel, float z_twist, float movement_time);
        void Mobile_Base_Stop (void);
        void Mobile_Base_Rotate_To_ArUco (void);
        void Mobile_Base_Translate_To_ArUco (float centring_tolerance = 2);

};

#endif /* ARUCO_ALIGNMENT_H */